#include "estimation/attitude_estimator.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeEstimator::AttitudeEstimator(FlightController &_flightcontroller)
    : firmware_(_flightcontroller) {
  reset_state();
}

void AttitudeEstimator::external_yaw_fusion(matrix::Quatf &_q) const {
  matrix::Eulerf euler_(_q);
  float yaw;
  matrix::Eulerf eul_from_pose(firmware_.pos_estimator_->pose().quat);
  yaw = eul_from_pose(2);
  if (std::isnan(yaw)) {
    Logger::ERROR(utils::Cat("quat ", firmware_.pos_estimator_->pose().quat(0),
                             " ", firmware_.pos_estimator_->pose().quat(1), " ",
                             firmware_.pos_estimator_->pose().quat(2), " ",
                             firmware_.pos_estimator_->pose().quat(3)));
    Logger::ERROR("yaw is NAN");
    yaw = 0;
  }
  euler_(2) = yaw;
  _q = matrix::Quatf(euler_);
}

void AttitudeEstimator::external_quat_fusion(matrix::Quatf &_q,
                                             matrix::Quatf &_qm) const {
  float kp = firmware_.params_->get(Params::ATT_EST_INTERNAL_ATT_COVAR);
  float km = firmware_.params_->get(Params::ATT_EST_EXTERNAL_ATT_COVAR);
  matrix::Matrix3f K;
  K.setIdentity();
  K = kp / (kp + km) * K;

  matrix::Dcmf R = _q.to_dcm();
  matrix::Dcmf Rm = _qm.to_dcm();
  matrix::Dcmf Re = Rm * R.T();

  // converting Rerror to angle-axis
  matrix::AxisAnglef eta = matrix::AxisAnglef(Re);

  // Correction step: using (bad) Kalman update
  matrix::AxisAnglef delta = K * eta;
  matrix::Dcmf R_correction(delta);

  // final update
  _q = matrix::Quatf(R_correction * R);
}

void AttitudeEstimator::gyro_filters(bool lpf, bool notch) {
  GYRO_LPF = lpf;
  GYRO_NOTCH = notch;
  Logger::STATUS(
      utils::Cat("gyro filters: lpf: ", GYRO_LPF, " notch: ", GYRO_NOTCH));
}
void AttitudeEstimator::gyro_rate_filters(bool lpf, bool notch) {
  GYRO_RATE_LPF = lpf;
  GYRO_RATE_NOTCH = notch;
  Logger::STATUS(utils::Cat("gyro-rate filters: lpf: ", GYRO_RATE_LPF,
                            " notch: ", GYRO_RATE_NOTCH));
}
void AttitudeEstimator::accel_filters(bool lpf, bool notch) {
  ACCEL_LPF = lpf;
  ACCEL_NOTCH = notch;
  Logger::STATUS(
      utils::Cat("accel filters: lpf: ", ACCEL_LPF, " notch: ", ACCEL_NOTCH));
}
void AttitudeEstimator::set_gyro_lpf_cutoff(float fc) {
  Logger::STATUS(utils::Cat("Setting gyro: lpf freq: ", fc));
  gyro_low_pass_.set_cutoff_freq(firmware_.loop_rate(), fc);
}
void AttitudeEstimator::set_gyro_notch_freq(float fc, float w) {
  Logger::STATUS(
      utils::Cat("Setting gyro: notch freq: ", fc, " notch width ", w));
  gyro_notch_.set_notch_frequency(firmware_.loop_rate(), fc, w);
}
void AttitudeEstimator::set_gyro_rate_lpf_cutoff(float fc) {
  Logger::STATUS(utils::Cat("Setting gyro rate: lpf freq: ", fc));
  gyro_rate_low_pass_.set_cutoff_freq(firmware_.loop_rate(), fc);
}
void AttitudeEstimator::set_gyro_rate_notch_freq(float fc, float w) {
  Logger::STATUS(
      utils::Cat("Setting gyro rate: notch freq: ", fc, " notch width ", w));
  gyro_rate_notch_.set_notch_frequency(firmware_.loop_rate(), fc, w);
}
void AttitudeEstimator::set_accel_lpf_cutoff(float fc) {
  Logger::STATUS(utils::Cat("Setting accel: lpf freq: ", fc));
  accel_low_pass_.set_cutoff_freq(firmware_.loop_rate(), fc);
}
void AttitudeEstimator::set_accel_notch_freq(float fc, float w) {
  Logger::STATUS(
      utils::Cat("Setting accel: notch freq: ", fc, " notch width ", w));
  accel_notch_.set_notch_frequency(firmware_.loop_rate(), fc, w);
}

void AttitudeEstimator::filtering(const float dt) {
  // Todo add different frequencies for different axes

  // scaling the accelerometer readings
  accel_ = firmware_.sensors_.imu().accel /
      (float) G_SI; // original readings in m/s^2
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

void AttitudeEstimator::getEuler(float &roll, float &pitch, float &yaw) {
  roll = atan2(2 * (quat(0) * quat(1) + quat(2) * quat(3)),
               1 - 2 * (quat(1) * quat(1) + quat(2) * quat(2))); // rads
  pitch = asin(2 * (quat(0) * quat(2) - quat(3) * quat(1)));     // rads
  yaw = atan2(2 * (quat(0) * quat(3) + quat(1) * quat(2)),
              1 - 2 * (quat(2) * quat(2) + quat(3) * quat(3))); // rads
}

} // namespace qrotor_firmware
