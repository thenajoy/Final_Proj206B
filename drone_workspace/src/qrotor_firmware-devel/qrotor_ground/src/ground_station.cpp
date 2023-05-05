#include <utility>

#include "qrotor_ground/ground_station.h"

namespace qrotor_ground {

GroundStation::GroundStation(ros::NodeHandle &nh, std::vector<std::string> vehicle_names)
    : nh_(nh), vehicle_names_(std::move(vehicle_names)) {
  /* ros params */
  num_of_vehicles_ = (int) vehicle_names_.size();
  for (int i = 0; i < num_of_vehicles_; i++) {
    auto *drone = new qrotor_ground::DroneManager(vehicle_names_.at(i), *this);
    vehicles_.push_back(drone);
  }

  /* init */
  init();
  /* run */
  run();
}

GroundStation::~GroundStation() {
  for (auto &vehicle : vehicles_)
    delete vehicle;
}

void GroundStation::clock() {
  current_time_ = ros::Time::now().toSec();
  dt_ = current_time_ - prev_time_;
  prev_time_ = current_time_;
}

void GroundStation::init() {}

void GroundStation::run() {
  clock();
  ros::Rate run_ros_loop_at(loop_rate_);

  while (ros::ok()) {
    ros::spinOnce();
    clock();

    run_ros_loop_at.sleep();
  }
  ros::shutdown();
}

} // namespace qrotor_ground