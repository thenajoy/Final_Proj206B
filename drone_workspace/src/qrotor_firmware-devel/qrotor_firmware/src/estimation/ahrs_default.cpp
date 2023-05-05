#include "estimation/ahrs_default.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AHRSDefault::AHRSDefault(FlightController &_flightcontroller) :
    firmware_(_flightcontroller) {

}

void AHRSDefault::reset_state() {
  state_.reset();
}

void AHRSDefault::gyro_filters(bool lpf, bool notch) {
  GYRO_LPF = lpf;
  GYRO_NOTCH = notch;
  Logger::STATUS(utils::Cat("gyro filters: lpf: ", GYRO_LPF, " notch: ", GYRO_NOTCH));
}
void AHRSDefault::gyro_rate_filters(bool lpf, bool notch) {
  GYRO_RATE_LPF = lpf;
  GYRO_RATE_NOTCH = notch;
  Logger::STATUS(utils::Cat("gyro-rate filters: lpf: ", GYRO_RATE_LPF, " notch: ", GYRO_RATE_NOTCH));
}
void AHRSDefault::accel_filters(bool lpf, bool notch) {
  ACCEL_LPF = lpf;
  ACCEL_NOTCH = notch;
  Logger::STATUS(utils::Cat("accel filters: lpf: ", ACCEL_LPF, " notch: ", ACCEL_NOTCH));
}
void AHRSDefault::set_gyro_lpf_cutoff(float fc) {
  Logger::STATUS(utils::Cat("Setting gyro: lpf freq: ", fc));
  gyro_low_pass_.set_cutoff_freq(firmware_.loop_rate(), fc);
}
void AHRSDefault::set_gyro_notch_freq(float fc, float w) {
  Logger::STATUS(utils::Cat("Setting gyro: notch freq: ", fc, " notch width ", w));
  gyro_notch_.set_notch_frequency(firmware_.loop_rate(), fc, w);
}
void AHRSDefault::set_gyro_rate_lpf_cutoff(float fc) {
  Logger::STATUS(utils::Cat("Setting gyro rate: lpf freq: ", fc));
  gyro_rate_low_pass_.set_cutoff_freq(firmware_.loop_rate(), fc);
}
void AHRSDefault::set_gyro_rate_notch_freq(float fc, float w) {
  Logger::STATUS(utils::Cat("Setting gyro rate: notch freq: ", fc, " notch width ", w));
  gyro_rate_notch_.set_notch_frequency(firmware_.loop_rate(), fc, w);
}
void AHRSDefault::set_accel_lpf_cutoff(float fc) {
  Logger::STATUS(utils::Cat("Setting accel: lpf freq: ", fc));
  accel_low_pass_.set_cutoff_freq(firmware_.loop_rate(), fc);
}
void AHRSDefault::set_accel_notch_freq(float fc, float w) {
  Logger::STATUS(utils::Cat("Setting accel: notch freq: ", fc, " notch width ", w));
  accel_notch_.set_notch_frequency(firmware_.loop_rate(), fc, w);
}

void AHRSDefault::init() {
  //---------------------- Calculate the offset -----------------------------

  float offset[3] = {0.0, 0.0, 0.0};
  float ax, ay, az, gx, gy, gz;


  //-------------------------------------------------------------------------

//  printf("Beginning Gyro calibration...\n");
//  for (int i = 0; i < 100; i++) {
//    firmware_.board_.read_accel_gyro(ax, ay, az, gx, gy, gz);
//    gx *= 180 / M_PI;
//    gy *= 180 / M_PI;
//    gz *= 180 / M_PI;
//
//    offset[0] += gx * 0.0175; // Note, find out why I did this!
//    offset[1] += gy * 0.0175;
//    offset[2] += gz * 0.0175;
//
//    usleep(10000);
//
//  }
//  offset[0] /= 100.0;
//  offset[1] /= 100.0;
//  offset[2] /= 100.0;
//
//  printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
//  gyro_offset_ = {offset[0], offset[1], offset[2]};
  reset_state();
  Logger::STATUS(std::string("AttitudeEstimator initialized!"));
}

void AHRSDefault::filtering(const float dt) {
  // Todo add different frequencies for different axes

  // scaling the accelerometer readings
  accel_ = firmware_.sensors_.imu().accel / (float) G_SI; // original readings in m/s^2
//  accel_ -= accel_offset_;
  // Applying low-pass on accel data
  if (ACCEL_NOTCH)
    accel_ = accel_notch_.apply(accel_);
  if (ACCEL_LPF)
    accel_ = accel_low_pass_.apply(accel_);

  // removing bias from gyro-scope readings
  gyro_ = firmware_.sensors_.imu().gyro * (180 / M_PI) * 0.0175; // [rad/s]
//  gyro_ -= gyro_offset_;

  // Applying filters to gyro data
  if (GYRO_NOTCH)
    gyro_ = gyro_notch_.apply(gyro_);
  if (GYRO_LPF)
    gyro_ = gyro_low_pass_.apply(gyro_);

  // Compute gyro rates and filter it
  if (dt > 1e-4) {
    gyro_rate_ = (gyro_ - gyro_prev_) / dt;
  }
  if (GYRO_RATE_NOTCH)
    gyro_rate_ = gyro_rate_notch_.apply(gyro_rate_);
  if (GYRO_RATE_LPF)
    gyro_rate_ = gyro_rate_low_pass_.apply(gyro_rate_);
  gyro_prev_ = gyro_;

}

void AHRSDefault::run(const float dt) {
  // processing the IMU sensor readings
  float ax, ay, az;
  float gx, gy, gz;

  // filtering the sensor readings
  filtering(dt);

  // assigning angular velocities
  state_.ang_vel = gyro_;
  state_.ang_vel_rates = gyro_rate_;
  state_.linear_accel = accel_;

//  matrix::Quatf q = firmware_.ext_pose_handler_->pose().quat;
//  state_.set_quat(q);
//  state_.refreshQuat();

  ax = accel_(0);
  ay = accel_(1);
  az = accel_(2);
  gx = gyro_(0);
  gy = gyro_(1);
  gz = gyro_(2);

  // Run Complementary Filter
  update(dt, ax, ay, az, gx, gy, gz);

  // assigning quaternion values
  state_.set_quat(q0, q1, q2, q3);

  // Get Euler angles from the estimated quaternion
  float roll, pitch, y_;
  getEuler(&roll, &pitch, &y_);

  // Get yaw value from Pose Estimator
  float yaw;
  matrix::Eulerf eul_from_pose(firmware_.pos_estimator_->pose().quat);
  yaw = eul_from_pose(2);
  if (std::isnan(yaw)) {
    Logger::ERROR(utils::Cat("quat ", firmware_.pos_estimator_->pose().quat(0),
                             " ", firmware_.pos_estimator_->pose().quat(1),
                             " ", firmware_.pos_estimator_->pose().quat(2),
                             " ", firmware_.pos_estimator_->pose().quat(3)));
    Logger::ERROR("yaw is NAN");
    yaw = 0;
  }

  // store the computed Euler Angles
  state_.set_euler(roll, pitch, yaw);
  state_.refreshEuler();
}

void AHRSDefault::update(float dt, float ax, float ay, float az, float gx, float gy, float gz) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;


  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;    // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;    // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f;    // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);        // pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRSDefault::getEuler(float *roll, float *pitch, float *yaw) {
  *roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)); // rads
  *pitch = asin(2 * (q0 * q2 - q3 * q1)); // rads
  *yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)); // rads
}

float AHRSDefault::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *) &y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *) &i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float AHRSDefault::getW() {
  return q0;
}

float AHRSDefault::getX() {
  return q1;
}

float AHRSDefault::getY() {
  return q2;
}

float AHRSDefault::getZ() {
  return q3;
}

}