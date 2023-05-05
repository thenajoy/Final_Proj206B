//
// Created by kotaru on 3/16/21.
//
//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

#include "estimation/madgwick_ahrs.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

MadgwickAHRS::MadgwickAHRS(FlightController &_flightcontroller)
    : AttitudeEstimator(_flightcontroller) {}

MadgwickAHRS::~MadgwickAHRS() = default;

void MadgwickAHRS::init() { Logger::STATUS("Madgwick AHRS initialized!"); }

void MadgwickAHRS::run(float dt) {

  //
  // sensor readings
  //
  // scaling the accelerometer readings original readings in m/s^2
  accel_ = firmware_.sensors_.imu().accel / (float) G_SI;
  gyro_ = firmware_.sensors_.imu().gyro * (180 / M_PI) * 0.0175; // [rad/s]
  //  std::cout << "dt_s " << dt_s << " accel: " << accel_(0) << " " << accel_(1) <<
  //  " " << accel_(2) << std::endl; std::cout << "gyro: " << gyro_(0) << " " <<
  //  gyro_(1) << " " << gyro_(2) << std::endl;

  //
  // low pass filtering the sensor readings
  //
  accel_ = accel_low_pass_.apply(accel_);
  gyro_ = gyro_low_pass_.apply(gyro_);
  if (dt > 1e-4) {
    gyro_rate_ = (gyro_ - gyro_prev_) / dt;
  }
  gyro_rate_ = gyro_rate_low_pass_.apply(gyro_rate_);
  gyro_prev_ = gyro_;

  // assigning angular velocities
  state_.ang_vel = gyro_;
  state_.ang_vel_rates = gyro_rate_;
  state_.linear_accel = accel_;

  //
  // perform attitude update
  //
  update(dt);

  //  std::cout << "estimated quat " << quat(0) << " "  << quat(1) << " " << quat(2) << " " << quat(3) << std::endl;

  //
  // Brute-force fuse yaw value from external pose estimator
  //
  external_yaw_fusion(quat);

  //
  // Fuse external pose to the attitude estimation
  //
  if (firmware_.params_->get_bool(Params::ATT_EST_USE_EXTERNAL_ATT)) {
    //    Logger::INFO("Running external q fusion");
    matrix::Quatf qm = firmware_.pos_estimator_->pose().quat;
    this->external_quat_fusion(quat, qm);
  }

  //
  // Update the estimated quaternion to the state
  //
  state_.set_quat(quat);
  state_.refreshQuat();
}

void MadgwickAHRS::update(float dt) {
  matrix::Quatf s;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  //
  // Rate of change of quaternion from gyroscope
  //
  matrix::Quatf qdot = omega2qdot(quat, gyro_);

  //
  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  //
  if (accel_.norm() != 0) {

    // Normalise accelerometer measurement
    accel_.normalize();

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * quat(0);
    _2q1 = 2.0f * quat(1);
    _2q2 = 2.0f * quat(2);
    _2q3 = 2.0f * quat(3);
    _4q0 = 4.0f * quat(0);
    _4q1 = 4.0f * quat(1);
    _4q2 = 4.0f * quat(2);
    _8q1 = 8.0f * quat(1);
    _8q2 = 8.0f * quat(2);
    q0q0 = quat(0) * quat(0);
    q1q1 = quat(1) * quat(1);
    q2q2 = quat(2) * quat(2);
    q3q3 = quat(3) * quat(3);

    // Gradient decent algorithm corrective step
    s = {_4q0 * q2q2 + _2q2 * accel_(0) + _4q0 * q1q1 - _2q1 * accel_(1),
         _4q1 * q3q3 - _2q3 * accel_(0) + 4.0f * q0q0 * quat(1) -
             _2q0 * accel_(1) - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 +
             _4q1 * accel_(2),
         4.0f * q0q0 * quat(2) + _2q0 * accel_(0) + _4q2 * q3q3 -
             _2q3 * accel_(1) - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 +
             _4q2 * accel_(2),
         4.0f * q1q1 * quat(3) - _2q1 * accel_(0) + 4.0f * q2q2 * quat(3) -
             _2q2 * accel_(1)};
    s.normalize();

    // Apply feedback step
    qdot -= beta * s;
  }

  // Integrate rate of change of quaternion to yield quaternion
  quat += qdot * dt;

  // Normalise quaternion
  quat.normalize();

  if (std::isnan(quat.norm())) {
    Logger::ERROR("quaternion is NaN!");
    quat = {1.0, 0., 0., 0.};
  }
}

} // namespace qrotor_firmware
