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

#ifndef QROTOR_GAZEBO_SIL_BOARD_H
#define QROTOR_GAZEBO_SIL_BOARD_H

#include <ros/ros.h>

#include <cmath>
#include <cstdbool>
#include <cstddef>
#include <cstdint>

#include "board.h"
#include <qrotor_firmware/RCRaw.h>
#include <qrotor_gazebo/common.h>

namespace qrotor_gazebo {
class SILBoard : public qrotor_firmware::Board {
private:
  GazeboVector inertial_magnetic_field_;

  double imu_update_rate_;

  double gyro_stdev_;
  double gyro_bias_walk_stdev_;
  double gyro_bias_range_;

  double acc_stdev_;
  double acc_bias_range_;
  double acc_bias_walk_stdev_;

  double baro_bias_walk_stdev_;
  double baro_stdev_;
  double baro_bias_range_;

  double mag_bias_walk_stdev_;
  double mag_stdev_;
  double mag_bias_range_;

  double airspeed_bias_walk_stdev_;
  double airspeed_stdev_;
  double airspeed_bias_range_;

  double sonar_stdev_;
  double sonar_max_range_;
  double sonar_min_range_;

  double horizontal_gps_stdev_;
  double vertical_gps_stdev_;
  double gps_velocity_stdev_;

  GazeboVector gyro_bias_;
  GazeboVector acc_bias_;
  GazeboVector mag_bias_;
  double baro_bias_;
  double airspeed_bias_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  GazeboVector gravity_;
  double origin_latitude_;
  double origin_longitude_;
  double origin_altitude_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;

  ros::NodeHandle *nh_;
  ros::Subscriber rc_sub_;
  qrotor_firmware::RCRaw latestRC_;
  bool rc_received_;
  ros::Time last_rc_message_;

  std::string mav_type_;

  // Time variables
  gazebo::common::Time boot_time_;
  uint64_t next_imu_update_time_us_;
  uint64_t imu_update_period_us_;

  void RCCallback(const qrotor_firmware::RCRaw &msg);
  bool motors_spinning();

  GazeboVector prev_vel_1_;
  GazeboVector prev_vel_2_;
  GazeboVector prev_vel_3_;
  gazebo::common::Time last_time_;

  float battery_voltage_multiplier{1.0};
  float battery_current_multiplier{1.0};
  static constexpr size_t BACKUP_SRAM_SIZE{1024};
  uint8_t backup_memory_[BACKUP_SRAM_SIZE];

  GazeboQuaternion q_I_NWU;
  GazeboVector current_vel;
  GazeboVector y_acc, y_acc_prev_;
  GazeboVector y_gyro, y_gyro_prev_;

public:
  SILBoard();
  ~SILBoard();

  // setup
  void init() override;

  // sensors
  void sensors_init() override;

  // IMU
  bool read_accel_gyro(float &ax, float &ay, float &az, float &gx, float &gy,
                       float &gz) override;
  bool read_accel_gyro_mag(float &ax, float &ay, float &az, float &gx,
                           float &gy, float &gz, float &mx, float &my,
                           float &mz) override;
  void imu_not_responding_error();

  // Radio
  int read_channel(int _ch) override;

  // Motors
  int pwm_outputs_[14]; // assumes maximum of 14 channels
  void write_pwms(const float *pwm) override;
  void arm_motors() override;
  void disarm_motors() override;

  // get noise params
  const double accel_stddev() const { return acc_stdev_; }
  const double gyro_stddev() const { return gyro_stdev_; }

  // Gazebo stuff
  void gazebo_setup(gazebo::physics::LinkPtr link,
                    gazebo::physics::WorldPtr world,
                    gazebo::physics::ModelPtr model, ros::NodeHandle *nh,
                    std::string mav_type);
  inline const int *get_outputs() const { return pwm_outputs_; }
#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::common::SphericalCoordinates sph_coord_;
#endif
};

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_SIL_BOARD_H
