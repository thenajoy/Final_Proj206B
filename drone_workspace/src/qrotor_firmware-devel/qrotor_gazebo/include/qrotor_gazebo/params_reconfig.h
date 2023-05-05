//
// Created by kotaru on 2/1/21.
//
#ifndef __QROTOR_FIRMWARE_PARAMS_RECONFIG_H__
#define __QROTOR_FIRMWARE_PARAMS_RECONFIG_H__

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "qrotor_flight.h"

namespace qrotor_gazebo {

class ParamsReconfig {
 protected:
  ros::NodeHandle nh_;

  // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> ddynrec_;
  qrotor_firmware::FlightController firmware_;

  // gains
  std::vector<matrix::Vector3f> pos_gains_, euler_gains_, ang_vel_gains_;
  char gain_chars[3] = {'p', 'd', 'i'};
  char axes_chars[3] = {'x', 'y', 'z'};

  bool GYRO_LPF{true}, GYRO_NOTCH{true};
  bool GYRO_RATE_LPF{true}, GYRO_RATE_NOTCH{true};
  bool ACCEL_LPF{true}, ACCEL_NOTCH{true};
  float gyro_lpf_freq{60}, gyro_notch_freq{80}, gyro_notch_width{0.55};
  float accel_lpf_freq{100}, accel_notch_freq{80}, accel_notch_width{0.75};
  float gyro_rate_lpf_freq{80}, gyro_rate_notch_freq{80}, gyro_rate_notch_width{0.75};

  float eta2 = 100.0;
  float epsilon2 = 4.0;
  float c2 = 20.0;

  void set_pid_gains();
  void set_noise_params();
  void set_att_clf_params();

 public:
  ParamsReconfig(ros::NodeHandle &_nh, qrotor_firmware::FlightController &fcu_);
  void init();
};
}
#endif //__QROTOR_FIRMWARE_PARAMS_RECONFIG_H__
