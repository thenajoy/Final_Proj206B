#ifndef __QROTOR_DISPALY_H__
#define __QROTOR_DISPLAY_H__

// system
#include <ros/ros.h>
#include <sys/time.h>

// cpp
#include <cstdio>
#include <eigen3/Eigen/Dense> // For use of matrix and vector operations
#include <iomanip>            // std::setw
#include <iostream>
#include <math.h>
#include <string>
#include <unistd.h>
#include <vector>

#include <tf/tf.h>

#include "nav_msgs/Odometry.h"
#include "qrotor_firmware/Log.h"

namespace qrotor_ground {

class DisplayInterface {

private:
  /// ros handle
  ros::NodeHandle nh_;

  // Subscribers
//  ros::Subscriber _sub_qrotor_log, _sub_qrotor_posevel, _sub_load_posevel,
//      _sub_cmd_traj, _sub_qrotor_odom, _sub_qrotor_cmd_input;
//  void callback_sub_qrotor_log(const qrotor_msgs::QuadrotorLog::ConstPtr &msg);
//  void callback_sub_qrotor_posevel(
//      const vrpn_client_ros::PoseVelStamped::ConstPtr &msg);
//  void callback_sub_load_posevel(
//      const vrpn_client_ros::PoseVelStamped::ConstPtr &msg);
//  void
//  callback_sub_cmd_traj(const qrotor_msgs::CommandTrajectory::ConstPtr &msg);
//  void callback_sub_odom(const nav_msgs::Odometry::ConstPtr &msg);
//  void callback_sub_cmd_input(
//      const qrotor_msgs::CommandThrustVectorMomentff::ConstPtr &msg);

  // display string
  std::string str;
  std::string c;

  // variables
  int width_ = 100;
  int height_ = 2;
  int cx, cy; // cursor x and y pos

  // time variables
  double current_time_ = 0.0;
  double dt_ = 0.0;
  double prev_time_ = 0.0;
  double state_time_ = 0.0;
  double t_ = 0.0;

  // state
  Eigen::Vector3d position_, velocity_;
  Eigen::Vector3d euler_, body_rates_;
  double thrust_;
  Eigen::Vector3d moment_;
  Eigen::Vector3d cmd_thrust_;

  float onboard_loop_freq_, voltage;
  std::string isArmed, voltageString, modeString, quadNameString,
      loadNameString;

  uint8_t mode;
  Eigen::Vector3d cmd_position_;
  float traj_t_ = 0.0;

  // colors
  std::string RED, GREEN, YELLOW, BLUE;

  // loads
  bool is_load_suspended = false;
  double cable_length;
  Eigen::Vector3d cmd_load_position_, load_position_, load_velocity_;

public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DisplayInterface(ros::NodeHandle &nh, std::string _vehicle_name);
  ~DisplayInterface();

  void addToLine(std::string c_);
  void addNextLine();
  void initialize();
  void updateTime();

  void displayScreen();
  void update();
};

} // namespace qrotor_ground

#endif /* __QROTOR_DISPLAY_H__ */
