//
// Created by kotaru on 5/7/21.
//
#ifndef QROTOR_FIRMWARE_ROSSERVICE_IO_H_
#define QROTOR_FIRMWARE_ROSSERVICE_IO_H_

#include <ros/ros.h>

// ros msgs
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <qrotor_firmware/Log.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <qrotor_firmware/Setpoint.h>
#include <qrotor_firmware/FlatTrajectory.h>
#include <qrotor_firmware/RCRaw.h>
#include <std_msgs/Bool.h>

#include <tf2_ros/transform_broadcaster.h>

#include "firmware_publisher.hpp"
#include "qrotor_flight.h"

namespace qrotor_firmware {

class FirmwareIO {
private:
  /// \brief Ros node handle
  ros::NodeHandle &nh_;
  /// \brief Flightcontroller handle
  FlightController &firmware_;

  /// \brief List of ROS Service Servers
  ros::ServiceServer imu_calibrate_srv_;
  ros::ServiceServer motors_kill_srv_;
  ros::ServiceServer motors_arm_srv_;
  ros::ServiceServer motors_disarm_srv_;
  ros::ServiceServer event_go_srv_;
  ros::ServiceServer event_nogo_srv_;
  ros::ServiceServer takeoff_srv_;
  ros::ServiceServer landing_srv_;
  ros::ServiceServer control_mode_srv_;
  ros::ServiceServer move_srv_;
  ros::ServiceServer setpoint_srv_;
  ros::ServiceServer flat_traj_srv_;
  ros::ServiceServer offb_ctrl_srv_;

  /// \brief List of ROS Service callbacks
  bool calibrateImuSrvCB(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res);
  bool killMotorsSrvCB(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res);
  bool armMotorsSrvCB(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res);
  bool disarmMotorsSrvCB(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res);
  bool eventGoSrvCB(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);
  bool eventNoGoSrvCB(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res);
  bool takeoffSrvCB(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);
  bool landingSrvCB(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);
  bool controlModeSrvCB(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
  bool offbCtrlSrvCB(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
  bool moveSrvCB(qrotor_firmware::Setpoint::Request &req,
                 qrotor_firmware::Setpoint::Response &res);
  bool setpointSrvCB(qrotor_firmware::Setpoint::Request &req,
                     qrotor_firmware::Setpoint::Response &res);
  bool flatTrajSrvCB(qrotor_firmware::FlatTrajectory::Request &req,
                     qrotor_firmware::FlatTrajectory::Response &res);

  /// \brief RosPublisher to publish quad-odom from tracking camera
  FirmwarePublisher<nav_msgs::Odometry> pub_odom_t265_{nh_, "odom/t265", 10};
  /// \brief RosPublisher to publish the odometer estimate of the quadrotor
  FirmwarePublisher<nav_msgs::Odometry> pub_odom_est_{nh_, "odom/estimate", 10};
  /// \brief RosPublisher to publish the setpoint odometry of the quadrotor
  FirmwarePublisher<nav_msgs::Odometry> pub_odom_sp_{nh_, "odom/setpoint", 10};
  /// \brief RosPublisher to publish quadrotor log
  FirmwarePublisher<qrotor_firmware::Log> pub_log_{nh_, "log", 10};
  /// \brief RosPublisher to publish quadrotor imu
  FirmwarePublisher<sensor_msgs::Imu> pub_imu_{nh_, "imu", 10};
  /// \brief Transform Broadcaster
  tf2_ros::TransformBroadcaster br;

  // subscriber
  ros::Subscriber sub_mocap_;
  ros::Subscriber sub_rc_;
  int rc_from_ros[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  void callback_sub_mocap(const nav_msgs::Odometry::ConstPtr &msg) const;
  void callback_sub_rc(const qrotor_firmware::RCRaw::ConstPtr &msg);

  ros::Subscriber subThrustVector;

  //
  ros::Subscriber sub_kill_all_, sub_takeoff_all_, sub_land_all_, sub_mission_cmds_all_, sub_move_all_;
  void killAllCb(const std_msgs::Bool::ConstPtr &msg);
  void takeOffAllCb(const std_msgs::Bool::ConstPtr &msg);
  void landAllCb(const std_msgs::Bool::ConstPtr &msg);
  void missionStartCb(const std_msgs::Bool::ConstPtr &msg);
  void thrustVectorCb(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void moveAllCb(const geometry_msgs::Vector3::ConstPtr &msg);

public:
  FirmwareIO(ros::NodeHandle &_nh, FlightController &_fcu);
  ~FirmwareIO() = default;

  /**
   * IO interface initialization
   */
  void init() {}
  /**
   * IO interface run
   */
  void run();
  /**
   * publish ros topics
   */
  void publish();
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_ROSSERVICE_IO_H_
