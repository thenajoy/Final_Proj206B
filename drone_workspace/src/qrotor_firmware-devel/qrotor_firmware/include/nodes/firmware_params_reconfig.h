//
// Created by kotaru on 2/1/21.
//
#ifndef __QROTOR_FIRMWARE_PARAMS_RECONFIG_H__
#define __QROTOR_FIRMWARE_PARAMS_RECONFIG_H__

#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include "qrotor_flight.h"
#include <boost/bind.hpp>

namespace qrotor_firmware {

class ParamsReconfig {
protected:
  ros::NodeHandle nh_;

  // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> ddynrec_;

  /// \brief flight controller
  FlightController firmware_;

  // gains
  std::vector<matrix::Vector3f> pos_gains_, vel_gains_, euler_gains_, ang_vel_gains_;
  char gain_chars[3] = {'p', 'd', 'i'};
  char axes_chars[3] = {'x', 'y', 'z'};

  // Noise/Sensor filters
  bool GYRO_LPF{true}, GYRO_NOTCH{false};
  bool GYRO_RATE_LPF{true}, GYRO_RATE_NOTCH{false};
  bool ACCEL_LPF{true}, ACCEL_NOTCH{false};
  float gyro_lpf_freq{60}, gyro_notch_freq{120}, gyro_notch_width{0.55};
  float accel_lpf_freq{100}, accel_notch_freq{120}, accel_notch_width{0.55};
  float gyro_rate_lpf_freq{80}, gyro_rate_notch_freq{80}, gyro_rate_notch_width{0.75};

  // Attitude Clf control
  float eta2 = 100.0;
  float epsilon2 = 4.0;
  float c2 = 20.0;

  matrix::Vector3f _pos_sp{0.f, 0.f, 1.0f};

  void set_pid_gains();
  void set_noise_params();
  void set_att_clf_params();
  void ros_param_read();
  void set_param_server();
  void set_setpoint();
  void firmware_param_cb(double new_value, Params p) ;

public:
  ParamsReconfig(ros::NodeHandle &_nh, FlightController &fcu_);
  void init();
};
}
#endif //__QROTOR_FIRMWARE_PARAMS_RECONFIG_H__
