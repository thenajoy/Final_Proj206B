#ifndef __QROTOR_GROUND_DRONE_MANAGER_H__
#define __QROTOR_GROUND_DRONE_MANAGER_H__
// system
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include <sys/time.h>

#include "qrotor_ground/qrotor_states.h"
#include "eigen_conversions/eigen_msg.h"

#include <nav_msgs/Odometry.h>
#include <qrotor_firmware/Log.h>

namespace qrotor_ground {
class GroundStation;
class DroneManager {
private:
  /// \brief ros handler
  ros::NodeHandle *nh_;
  /// \brief ground station handler
  GroundStation &station_;

  /// \brief bird name
  std::string bird_name_;
  /// \brief vehicle state
  qrotor_ground::RigidbodyState state_;
  /// \brief last state update
  ros::Time last_state_update_;
  /// \brief state time out
  double state_timeout_{2}; // seconds

  /// \brief vehicle log subscriber
  ros::Subscriber sub_log_;
  /// \brief vehicle odom from mocap subscriber
  ros::Subscriber sub_odom_;

  /// \brief callback function for vehicle log
  void logCb(const qrotor_firmware::Log::ConstPtr &msg);
  /// \brief callback function for vehicle odom
  void odomCb(const nav_msgs::Odometry::ConstPtr &msg);

public:
  DroneManager(std::string _bird_name, GroundStation &_gnd_station);
  ~DroneManager();

  void init();
  void run();

  /**
   * Returns the current state of the vehicle
   * @return RigidbodyState
   */
  inline const RigidbodyState &state() const { return state_; }
};

} // namespace qrotor_ground

#endif /* __QROTOR_GROUND_DRONE_MANAGER_H__ */