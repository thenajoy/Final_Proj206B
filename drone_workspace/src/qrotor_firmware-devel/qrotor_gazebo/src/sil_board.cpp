/*
 * Copyright (c) 2017 Daniel Koch, James Jackson and Gary Ellingson, BYU MAGICC
 * Lab. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <qrotor_gazebo/sil_board.h>
#include <ros/ros.h>

#include <fstream>
#include <iostream>

namespace qrotor_gazebo {

constexpr double rad2Deg(double x) { return 180.0 / M_PI * x; }
constexpr double deg2Rad(double x) { return M_PI / 180.0 * x; }

SILBoard::SILBoard() {
  y_acc = GazeboVector(0.0, 0.0, 9.8);
  y_acc_prev_ = GazeboVector(0.0, 0.0, 9.8);

  y_gyro = GazeboVector(0.0, 0.0, 0.0);
  y_gyro_prev_ = GazeboVector(0.0, 0.0, 0.0);
  
  latestRC_.values[0] = 1500;
  latestRC_.values[1] = 1500;
  latestRC_.values[2] = 1000;
  latestRC_.values[3] = 1500;
  latestRC_.values[4] = 2000;
  latestRC_.values[5] = 1000;
  latestRC_.values[6] = 2000;
  latestRC_.values[7] = 1000;
}

SILBoard::~SILBoard() = default;

void SILBoard::init() { boot_time_ = GZ_COMPAT_GET_SIM_TIME(world_); }

void SILBoard::gazebo_setup(gazebo::physics::LinkPtr link,
                            gazebo::physics::WorldPtr world,
                            gazebo::physics::ModelPtr model,
                            ros::NodeHandle *nh, std::string mav_type) {
  link_ = link;
  world_ = world;
  model_ = model;
  nh_ = nh;
  mav_type_ = mav_type;

  // Get Sensor Parameters
  gyro_stdev_ = nh->param<double>("gyro_stdev", 0.13);
  gyro_bias_range_ = nh->param<double>("gyro_bias_range", 0.15);
  gyro_bias_walk_stdev_ = nh->param<double>("gyro_bias_walk_stdev", 0.001);

  acc_stdev_ = nh->param<double>("acc_stdev", 1.15);
  acc_bias_range_ = nh->param<double>("acc_bias_range", 0.15);
  acc_bias_walk_stdev_ = nh->param<double>("acc_bias_walk_stdev", 0.001);

  ROS_INFO(" noise params: gyro_stdev: %f, gyro_bias_range: %f, "
           "gyro_bias_walk_stdev %f",
           gyro_stdev_, gyro_bias_range_, gyro_bias_walk_stdev_);
  ROS_INFO(" noise params: acc_stdev: %f, acc_bias_range: %f, "
           "acc_bias_walk_stdev %f",
           acc_stdev_, acc_bias_range_, acc_bias_walk_stdev_);

  mag_stdev_ = nh->param<double>("mag_stdev", 1.15);
  mag_bias_range_ = nh->param<double>("mag_bias_range", 0.15);
  mag_bias_walk_stdev_ = nh->param<double>("mag_bias_walk_stdev", 0.001);

  baro_stdev_ = nh->param<double>("baro_stdev", 1.15);
  baro_bias_range_ = nh->param<double>("baro_bias_range", 0.15);
  baro_bias_walk_stdev_ = nh->param<double>("baro_bias_walk_stdev", 0.001);

  airspeed_stdev_ = nh_->param<double>("airspeed_stdev", 1.15);
  airspeed_bias_range_ = nh_->param<double>("airspeed_bias_range", 0.15);
  airspeed_bias_walk_stdev_ =
      nh_->param<double>("airspeed_bias_walk_stdev", 0.001);

  sonar_stdev_ = nh_->param<double>("sonar_stdev", 1.15);
  sonar_min_range_ = nh_->param<double>("sonar_min_range", 0.25);
  sonar_max_range_ = nh_->param<double>("sonar_max_range", 8.0);

  imu_update_rate_ = nh_->param<double>("imu_update_rate", 500.0);
  imu_update_period_us_ = (uint64_t)(1e6 / imu_update_rate_);

  // Calculate Magnetic Field Vector (for mag simulation)
  auto inclination = nh_->param<double>("inclination", 1.14316156541);
  auto declination = nh_->param<double>("declination", 0.198584539676);
  GZ_COMPAT_SET_Z(inertial_magnetic_field_, sin(-inclination));
  GZ_COMPAT_SET_X(inertial_magnetic_field_,
                  cos(-inclination) * cos(-declination));
  GZ_COMPAT_SET_Y(inertial_magnetic_field_,
                  cos(-inclination) * sin(-declination));

  // Get the desired altitude at the ground (for baro and LLA)

  origin_altitude_ = nh->param<double>("origin_altitude", 1387.0);
  origin_latitude_ = nh->param<double>("origin_latitude", 40.2463724);
  origin_longitude_ = nh->param<double>("origin_longitude", -111.6474138);

  horizontal_gps_stdev_ = nh->param<double>("horizontal_gps_stdev", 1.0);
  vertical_gps_stdev_ = nh->param<double>("vertical_gps_stdev", 3.0);
  gps_velocity_stdev_ = nh->param<double>("gps_velocity_stdev", 0.1);

  // Configure Noise
  random_generator_ = std::default_random_engine(
      std::chrono::system_clock::now().time_since_epoch().count());
  normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  uniform_distribution_ = std::uniform_real_distribution<double>(-1.0, 1.0);

  gravity_ = GZ_COMPAT_GET_GRAVITY(world_);

  // Initialize the Sensor Biases
  GZ_COMPAT_SET_X(gyro_bias_,
                  gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_,
                  gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_,
                  gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_X(acc_bias_,
                  acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(acc_bias_,
                  acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(acc_bias_,
                  acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_X(mag_bias_,
                  mag_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(mag_bias_,
                  mag_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(mag_bias_,
                  mag_bias_range_ * uniform_distribution_(random_generator_));
  baro_bias_ = baro_bias_range_ * uniform_distribution_(random_generator_);
  airspeed_bias_ =
      airspeed_bias_range_ * uniform_distribution_(random_generator_);

  prev_vel_1_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_2_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  prev_vel_3_ = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  last_time_ = GZ_COMPAT_GET_SIM_TIME(world_);
  next_imu_update_time_us_ = 0;

  rc_sub_ = nh_->subscribe("RC", 1, &SILBoard::RCCallback, this);
}

void SILBoard::RCCallback(const qrotor_firmware::RCRaw &msg) {
  rc_received_ = true;
  last_rc_message_ = ros::Time::now();
  latestRC_ = msg;
  for (int i = 0; i < 8; i++) {
    //    printf("pwm[%d]: %d", i, latestRC_.values[i]);
  }
  //  printf("\n");
}

int SILBoard::read_channel(int _ch) { return latestRC_.values[_ch]; }

// sensors
/// TODO these sensors have noise, no bias
/// noise params are hard coded
void SILBoard::sensors_init() {
  // Initialize the Biases
  GZ_COMPAT_SET_X(gyro_bias_,
                  gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_,
                  gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_,
                  gyro_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_X(acc_bias_,
                  acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(acc_bias_,
                  acc_bias_range_ * uniform_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(acc_bias_,
                  acc_bias_range_ * uniform_distribution_(random_generator_));

  // Gazebo coordinates is NWU and Earth's magnetic field is defined in NED,
  // hence the negative signs
  double inclination_ = 1.14316156541;
  double declination_ = 0.198584539676;
  GZ_COMPAT_SET_Z(inertial_magnetic_field_, sin(-inclination_));
  GZ_COMPAT_SET_X(inertial_magnetic_field_,
                  cos(-inclination_) * cos(-declination_));
  GZ_COMPAT_SET_Y(inertial_magnetic_field_,
                  cos(-inclination_) * sin(-declination_));

#if GAZEBO_MAJOR_VERSION >= 9
  using SC = gazebo::common::SphericalCoordinates;
  using Ang = ignition::math::Angle;
  sph_coord_.SetSurfaceType(SC::SurfaceType::EARTH_WGS84);
  sph_coord_.SetLatitudeReference(Ang(deg2Rad(origin_latitude_)));
  sph_coord_.SetLongitudeReference(Ang(deg2Rad(origin_longitude_)));
  sph_coord_.SetElevationReference(origin_altitude_);
  // Force x-axis to be north-aligned. I promise, I will change everything to
  // ENU in the next commit
  sph_coord_.SetHeadingOffset(Ang(-M_PI / 2.0));
#endif
}

bool SILBoard::read_accel_gyro(float &ax, float &ay, float &az, float &gx,
                               float &gy, float &gz) {
  q_I_NWU = GZ_COMPAT_GET_ROT(GZ_COMPAT_GET_WORLD_POSE(link_));
  current_vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);

  // this is James' egregious hack to overcome wild imu while sitting on the
  // ground
  if (GZ_COMPAT_GET_LENGTH(current_vel) < 0.05)
    y_acc = q_I_NWU.RotateVectorReverse(-gravity_);
  else
    y_acc = q_I_NWU.RotateVectorReverse(
        GZ_COMPAT_GET_WORLD_LINEAR_ACCEL(link_) - gravity_);

  // Apply normal noise (only if armed, because most of the noise comes from
  // motors
  if (motors_spinning()) {
    GZ_COMPAT_SET_X(y_acc,
                    GZ_COMPAT_GET_X(y_acc) +
                        acc_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Y(y_acc,
                    GZ_COMPAT_GET_Y(y_acc) +
                        acc_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Z(y_acc,
                    GZ_COMPAT_GET_Z(y_acc) +
                        acc_stdev_ * normal_distribution_(random_generator_));
  }

  // Perform Random Walk for biases
  GZ_COMPAT_SET_X(acc_bias_, GZ_COMPAT_GET_X(acc_bias_) +
                                 acc_bias_walk_stdev_ *
                                     normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(acc_bias_, GZ_COMPAT_GET_Y(acc_bias_) +
                                 acc_bias_walk_stdev_ *
                                     normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(acc_bias_, GZ_COMPAT_GET_Z(acc_bias_) +
                                 acc_bias_walk_stdev_ *
                                     normal_distribution_(random_generator_));

  // Add constant Bias to measurement
  GZ_COMPAT_SET_X(y_acc, GZ_COMPAT_GET_X(y_acc) + GZ_COMPAT_GET_X(acc_bias_));
  GZ_COMPAT_SET_Y(y_acc, GZ_COMPAT_GET_Y(y_acc) + GZ_COMPAT_GET_Y(acc_bias_));
  GZ_COMPAT_SET_Z(y_acc, GZ_COMPAT_GET_Z(y_acc) + GZ_COMPAT_GET_Z(acc_bias_));

  if (sqrt(y_acc.X() * y_acc.X() + y_acc.Y() * y_acc.Y() +
           y_acc.Z() * y_acc.Z()) <= 30) {
    ax = y_acc.X();
    ay = y_acc.Y();
    az = y_acc.Z();
    y_acc_prev_ = y_acc;
  } else {
    gzerr << "[BAD] accelerometer reading (ax): " << y_acc.X()
          << " (ay): " << y_acc.Y() << " (az): " << y_acc.Z() << std::endl;
    gzerr << "[Prev] accelerometer reading (ax): " << y_acc_prev_.X()
          << " (ay): " << y_acc_prev_.Y() << " (az): " << y_acc_prev_.Z()
          << std::endl;
    ax = y_acc_prev_.X();
    ay = y_acc_prev_.Y();
    az = y_acc_prev_.Z();
  }

  y_gyro = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Normal Noise from motors
  if (motors_spinning()) {
    GZ_COMPAT_SET_X(y_gyro,
                    GZ_COMPAT_GET_X(y_gyro) +
                        gyro_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Y(y_gyro,
                    GZ_COMPAT_GET_Y(y_gyro) +
                        gyro_stdev_ * normal_distribution_(random_generator_));
    GZ_COMPAT_SET_Z(y_gyro,
                    GZ_COMPAT_GET_Z(y_gyro) +
                        gyro_stdev_ * normal_distribution_(random_generator_));
  }

  // Random Walk for bias
  GZ_COMPAT_SET_X(gyro_bias_, GZ_COMPAT_GET_X(gyro_bias_) +
                                  gyro_bias_walk_stdev_ *
                                      normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(gyro_bias_, GZ_COMPAT_GET_Y(gyro_bias_) +
                                  gyro_bias_walk_stdev_ *
                                      normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(gyro_bias_, GZ_COMPAT_GET_Z(gyro_bias_) +
                                  gyro_bias_walk_stdev_ *
                                      normal_distribution_(random_generator_));

  // Apply Constant Bias
  GZ_COMPAT_SET_X(y_gyro,
                  GZ_COMPAT_GET_X(y_gyro) + GZ_COMPAT_GET_X(gyro_bias_));
  GZ_COMPAT_SET_Y(y_gyro,
                  GZ_COMPAT_GET_Y(y_gyro) + GZ_COMPAT_GET_Y(gyro_bias_));
  GZ_COMPAT_SET_Z(y_gyro,
                  GZ_COMPAT_GET_Z(y_gyro) + GZ_COMPAT_GET_Z(gyro_bias_));

  bool bad_gyro_reading = false;
  bad_gyro_reading = std::isnan(y_gyro.X()) || std::isnan(y_gyro.Y()) ||
                     std::isnan(y_gyro.Z());

  bad_gyro_reading = bad_gyro_reading ||
                     (sqrt(y_gyro.X() * y_gyro.X() + y_gyro.Y() * y_gyro.Y() +
                           y_gyro.Z() * y_gyro.Z()) > 100);

  if (bad_gyro_reading) {
    gzerr << "[BAD] gyroscope reading: "
          << "gx: " << y_gyro.X() << " gy: " << y_gyro.Y()
          << " gz: " << y_gyro.Z() << std::endl;
    gzerr << "[Prev] gyroscope reading: "
          << "gx: " << y_gyro_prev_.X() << " gy: " << y_gyro_prev_.Y()
          << " gz: " << y_gyro_prev_.Z() << std::endl;
    // Convert to NWU for output
    gx = GZ_COMPAT_GET_X(y_gyro_prev_);
    gy = GZ_COMPAT_GET_Y(y_gyro_prev_);
    gz = GZ_COMPAT_GET_Z(y_gyro_prev_);
  } else {
    // Convert to NWU for output
    gx = GZ_COMPAT_GET_X(y_gyro);
    gy = GZ_COMPAT_GET_Y(y_gyro);
    gz = GZ_COMPAT_GET_Z(y_gyro);
    y_gyro_prev_ = y_gyro;
  }
  return true;
}

bool SILBoard::read_accel_gyro_mag(float &ax, float &ay, float &az, float &gx,
                                   float &gy, float &gz, float &mx, float &my,
                                   float &mz) {
  SILBoard::read_accel_gyro(ax, ay, az, gx, gy, gz);

  GazeboPose I_to_B = GZ_COMPAT_GET_WORLD_POSE(link_);
  GazeboVector noise;
  GZ_COMPAT_SET_X(noise, mag_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(noise, mag_stdev_ * normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(noise, mag_stdev_ * normal_distribution_(random_generator_));

  // Random Walk for bias
  GZ_COMPAT_SET_X(mag_bias_, GZ_COMPAT_GET_X(mag_bias_) +
                                 mag_bias_walk_stdev_ *
                                     normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Y(mag_bias_, GZ_COMPAT_GET_Y(mag_bias_) +
                                 mag_bias_walk_stdev_ *
                                     normal_distribution_(random_generator_));
  GZ_COMPAT_SET_Z(mag_bias_, GZ_COMPAT_GET_Z(mag_bias_) +
                                 mag_bias_walk_stdev_ *
                                     normal_distribution_(random_generator_));

  // combine parts to create a measurement
  GazeboVector y_mag =
      GZ_COMPAT_GET_ROT(I_to_B).RotateVectorReverse(inertial_magnetic_field_) +
      mag_bias_ + noise;

  // Convert measurement to NWU
  mx = GZ_COMPAT_GET_X(y_mag);
  my = GZ_COMPAT_GET_Y(y_mag);
  mz = GZ_COMPAT_GET_Z(y_mag);

  return true;
}

void SILBoard::imu_not_responding_error(void) {
  ROS_ERROR("[gazebo_rosflight_sil] imu not responding");
}

// PWM
void SILBoard::write_pwms(const float *pwm) {
  for (int i = 1; i < 4; i++) {
    pwm_outputs_[i] = pwm[i];
  }
}

void SILBoard::arm_motors() {
  for (int i = 1; i < 4; i++) {
    pwm_outputs_[i] = 1100;
  }
}
void SILBoard::disarm_motors() {
  for (int i = 1; i < 4; i++) {
    pwm_outputs_[i] = 1000;
  }
}

bool SILBoard::motors_spinning() {
  if (pwm_outputs_[0] > 1100 || pwm_outputs_[1] > 1100 ||
      pwm_outputs_[2] > 1100 || pwm_outputs_[3] > 1100)
    return true;
  else
    return false;
}

} // namespace qrotor_gazebo
