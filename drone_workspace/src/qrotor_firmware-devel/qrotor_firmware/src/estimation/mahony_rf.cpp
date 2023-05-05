//
// Created by kotaru on 3/10/21.
//
#include "estimation/mahony_rf.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

MahonyRF::MahonyRF(FlightController &_flightcontroller)
    : AttitudeEstimator(_flightcontroller) {}

MahonyRF::~MahonyRF() = default;

void MahonyRF::run_LPF() {
  const matrix::Vector3f raw_accel = {
      static_cast<float>(firmware_.sensors_.imu().accel(0) / G_SI),
      static_cast<float>(firmware_.sensors_.imu().accel(1) / G_SI),
      static_cast<float>(firmware_.sensors_.imu().accel(2) / G_SI)};
  accel_LPF_ = (1.0f - alpha_acc) * raw_accel + alpha_acc * accel_LPF_;

  const matrix::Vector3f raw_gyro = {firmware_.sensors_.imu().gyro(0),
                                     firmware_.sensors_.imu().gyro(1),
                                     firmware_.sensors_.imu().gyro(2)};
  gyro_LPF_ = {
      (1.0f - alpha_gyro_xy) * raw_gyro(0) + alpha_gyro_xy * gyro_LPF_(0),
      (1.0f - alpha_gyro_xy) * raw_gyro(1) + alpha_gyro_xy * gyro_LPF_(1),
      (1.0f - alpha_gyro_z) * raw_gyro(2) + alpha_gyro_z * gyro_LPF_(2)};
}

void MahonyRF::set_external_attitude_update() {
  extatt_update_next_run_ = true;
  q_extatt_ = firmware_.ext_pose_handler_->pose().quat;
}

bool MahonyRF::can_use_extatt() const {
  return firmware_.ext_pose_handler_->new_pose_obtained();
}

void MahonyRF::init() {

  last_time_ = 0;
  last_acc_update_us_ = 0;
  last_extatt_update_us_ = 0;
  reset_state();
  start_us = utils::get_current_time();
  Logger::STATUS("Mahony from Rosflight initialized!");
}

void MahonyRF::run(float dt) {

  //
  // Timing Setup
  //
  const unsigned long now_us = firmware_.sensors_.imu().timestamp_us;
  if (last_time_ == 0) {
    last_time_ = now_us;
    last_acc_update_us_ = now_us;
    last_extatt_update_us_ = now_us;
    return;
  } else if (now_us < last_time_) {
    // this shouldn't happen
    last_time_ = now_us;
    return;
  }
  //  printf("ros_dt: %f, ", dt_s);
  // TODO: find the right "dt_s" the ros-time or system clock
  dt = float(now_us - last_time_) * 1e-6f;
//  printf(" imu_board_dt: %f\n", dt_s);
  last_time_ = now_us;
  state_.timestamp_us = now_us;

  // Low-pass filter accel and gyro measurements
  run_LPF();

  //
  // Gyro Correction Term (werr)
  //
  float kp = 0.0f;
  float ki = firmware_.params_->get(Params::MAHONY_FILTER_KI);
  matrix::Vector3f w_err;

  // Get error estimated by accelerometer measurement
  w_err = accel_correction();
//  printf("werr: %f %f %f", w_err(0), w_err(1), w_err(2));
  kp = firmware_.params_->get(Params::MAHONY_FILTER_KP_ACC);
  last_acc_update_us_ = now_us;

//  if (can_use_extatt()) {
//    // Get error estimated by external attitude measurement. Overwrite any
//    // correction based on the accelerometer (assumption: extatt is better).
//    w_err = extatt_correction();
//    kp = firmware_.params_reconfig_[Params::MAHONY_FILTER_KP_EXT];
//
//    // the angular rate correction from external attitude updates occur at a
//    // different rate than IMU updates, so it needs to be integrated with a
//    // different dt_s. The following scales the correction term by the timestep
//    // ratio so that it is integrated correctly.
//    const float extAttDt = float(now_us - last_extatt_update_us_) * 1e-6f;
//    const float scaleDt = (dt_s > 0) ? (extAttDt / dt_s) : 0.0f;
//    w_err *= scaleDt;
//
//    last_extatt_update_us_ = now_us;
//    extatt_update_next_run_ = false;
//  }

  // Crank up the gains for the first few seconds for quick convergence
  if ((now_us - start_us) < static_cast<uint64_t>(PARAM_INIT_TIME) * 1000) {
    kp = firmware_.params_->get(Params::MAHONY_FILTER_KP_ACC) * 10.0f;
    ki = firmware_.params_->get(Params::MAHONY_FILTER_KI) * 10.0f;
  }

  //
  // Composite Bias-Free Angular Rate (wfinal)
  //
  //  std::cout << "kp: " << kp << " ki: " << ki << std::endl;
  //  firmware_.params_reconfig_.debug_print();

  // Integrate biases driven by measured angular error
  // eq 47b Mahony Paper, using correction term w_err found above
  gyro_bias_ -= ki * w_err * dt;

  // Build the composite omega vector for kinematic propagation
  // This the stuff inside the p function in eq. 47a - Mahony Paper
  matrix::Vector3f wbar = smoothed_gyro_measurement();
  matrix::Vector3f wfinal = wbar - gyro_bias_ + kp * w_err;

  //
  // Propagate Dynamics
  //
  integrate_angular_rate(q, wfinal, dt);
//  std::cout << " mahony_rf: q: " << q(0) << " " << q(1) << " " << q(2) << " "
//            << q(3) << std::endl;
  state_.set_quat(q);
  state_.refreshQuat();

  //
  // Post-Processing
  //
  // Extract Euler Angles for controller
  //  state_.attitude.get_RPY(&state_.roll, &state_.pitch, &state_.yaw);

  // Save off adjust gyro measurements with estimated biases for control
  state_.ang_vel = gyro_LPF_ - gyro_bias_;

  // If it has been more than 0.5 seconds since the accel update ran and we
  // are supposed to be getting them then trigger an unhealthy estimator error.
  // TODO
}

matrix::Vector3f MahonyRF::accel_correction() const {

  // turn measurement into a unit vector
  matrix::Vector3f a = accel_LPF_.normalized();

  // Get the quaternion from accelerometer (low-frequency measure q)
  // (Not in either paper)
  matrix::Quaternionf q_acc_inv(g_, a);

  // Get the error quaternion between observer and low-freq q
  // Below Eq. 45 Mahony Paper
  matrix::Quaternionf q_tilde = q_acc_inv * state_.q();

  // Correction Term of Eq. 47a and 47b Mahony Paper
  // w_acc = 2*s_tilde*v_tilde
  matrix::Vector3f w_acc;
  w_acc = {-2.0f * q_tilde(0) * q_tilde(1), -2.0f * q_tilde(0) * q_tilde(2),
           0.0f};
  // Don't correct z, because it's unobservable from the accelerometer

  return w_acc;
}

matrix::Vector3f MahonyRF::extatt_correction() const {

  // DCM rows of attitude estimate and external measurement (world w.r.t body).
  // These are the world axes from the perspective of the body frame.
  // Note: If we extracted cols it would be body w.r.t world.
  matrix::Vector3f xhat_BW, yhat_BW, zhat_BW;
  matrix::Vector3f xext_BW, yext_BW, zext_BW;

  // extract rows of rotation matrix from quaternion attitude estimate
  quaternion_to_dcm(state_.q(), xhat_BW, yhat_BW, zhat_BW);

  // extract rows of rotation matrix from quaternion external attitude
  quaternion_to_dcm(q_extatt_, xext_BW, yext_BW, zext_BW);

  // calculate cross products of corresponding axes as an error metric. For
  // example, if vehicle were level but extatt had a different yaw angle than
  // the internal estimate, xext_BW.cross(xhat_BW) would be a measure of how the
  // filter needs to update in order to the internal estimate's yaw to closer to
  // the extatt measurement. This is done for each axis.
  matrix::Vector3f w_ext =
      xext_BW.cross(xhat_BW) + yext_BW.cross(yhat_BW) + zext_BW.cross(zhat_BW);

  return w_ext;
}

matrix::Vector3f MahonyRF::smoothed_gyro_measurement() {
  matrix::Vector3f wbar;
  if (PARAM_FILTER_USE_QUAD_INT) {
    // Quadratic Interpolation (Eq. 14 Casey Paper)
    // this step adds 12 us on the STM32F10x chips
    wbar = (w2_ / -12.0f) + w1_ * (8.0f / 12.0f) + gyro_LPF_ * (5.0f / 12.0f);
    w2_ = w1_;
    w1_ = gyro_LPF_;
  } else {
    wbar = gyro_LPF_;
  }

  return wbar;
}

void MahonyRF::integrate_angular_rate(matrix::Quaternionf &quat,
                                      const matrix::Vector3f &omega,
                                      const float dt) const {
  // only propagate if we've moved
  // TODO[PCL]: Will this ever be true? We should add a margin to this
  const float sqrd_norm_w =
      omega(0) * omega(0) + omega(1) * omega(1) + omega(2) * omega(2);
  if (sqrd_norm_w == 0.0f)
    return;

  // for convenience
  const float &p = omega(0), &q = omega(1), &r = omega(2);

  if (PARAM_FILTER_USE_MAT_EXP) {
    // Matrix Exponential Approximation (From Attitude Representation and
    // Kinematic Propagation for Low-Cost UAVs by Robert T. Casey) (Eq. 12 Casey
    // Paper) This adds 90 us on STM32F10x chips
    float norm_w = sqrtf(sqrd_norm_w);
    float t1 = cosf((norm_w * dt) / 2.0f);
    float t2 = 1.0f / norm_w * sinf((norm_w * dt) / 2.0f);
    quat(0) = t1 * quat(0) + t2 * (-p * quat(1) - q * quat(2) - r * quat(3));
    quat(1) = t1 * quat(1) + t2 * (p * quat(0) + r * quat(2) - q * quat(3));
    quat(2) = t1 * quat(2) + t2 * (q * quat(0) - r * quat(1) + p * quat(3));
    quat(3) = t1 * quat(3) + t2 * (r * quat(0) + q * quat(1) - p * quat(2));
    quat.normalize();
  } else {
    // Euler Integration
    // (Eq. 47a Mahony Paper)
    matrix::Quaternionf qdot(0.5f * (-p * quat(1) - q * quat(2) - r * quat(3)),
                             0.5f * (p * quat(0) + r * quat(2) - q * quat(3)),
                             0.5f * (q * quat(0) - r * quat(1) + p * quat(3)),
                             0.5f * (r * quat(0) + q * quat(1) - p * quat(2)));
    quat += qdot * dt;
    quat.normalize();
  }
}

void MahonyRF::quaternion_to_dcm(const matrix::Quatf &q, matrix::Vector3f &X,
                                 matrix::Vector3f &Y,
                                 matrix::Vector3f &Z) {
  // R(q) = [X.x X.y X.z]
  //        [Y.x Y.y Y.z]
  //        [Z.x Z.y Z.z]

  const float &w = q(0), &x = q(1), &y = q(2), &z = q(3);
  X = {1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - z * w),
       2.0f * (x * z + y * w)};
  Y = {2.0f * (x * y + z * w), 1.0f - 2.0f * (x * x + z * z),
       2.0f * (y * z - x * w)};
  Z = {2.0f * (x * z - y * w), 2.0f * (y * z + x * w),
       1.0f - 2.0f * (x * x + y * y)};
}

} // namespace qrotor_firmware
