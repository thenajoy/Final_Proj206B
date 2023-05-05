#include <utility>

#include "qrotor_ground/drone_manager.h"
#include "qrotor_ground/ground_station.h"
namespace qrotor_ground {

DroneManager::DroneManager(std::string _bird_name, GroundStation &_gnd_station)
    : bird_name_(std::move(_bird_name)), station_(_gnd_station) {
  nh_ = new ros::NodeHandle(bird_name_);

  /* ros subscribers */
  sub_odom_ = nh_->subscribe("odometry/mocap", 1, &DroneManager::odomCb, this);
  sub_log_ = nh_->subscribe("log", 1, &DroneManager::logCb, this);


  /* init */
  init();
}

DroneManager::~DroneManager() = default;

/**************** callback functions ***************/
void DroneManager::logCb(const qrotor_firmware::Log::ConstPtr &msg) {}

void DroneManager::odomCb(const nav_msgs::Odometry::ConstPtr &msg) {
  last_state_update_ = ros::Time::now();
  tf::pointMsgToEigen(msg->pose.pose.position, state_.position);
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, state_.quaternion);
  tf::vectorMsgToEigen(msg->twist.twist.linear, state_.velocity);
}

void DroneManager::init() {}

void DroneManager::run() {

  if (ros::Time::now().toSec() - last_state_update_.toSec() > state_timeout_) {
    ROS_ERROR("Odometry/mocap timed out!");
  }

}

} // namespace qrotor_ground
