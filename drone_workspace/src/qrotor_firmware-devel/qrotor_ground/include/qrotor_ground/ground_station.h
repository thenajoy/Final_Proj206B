#ifndef _QROTOR_GROUND_H__
#define _QROTOR_GROUND_H__

#include <ros/ros.h>
#include <sys/time.h>
#include <vector>

// system
#include "qrotor_ground/drone_manager.h"

namespace qrotor_ground {

class GroundStation {
private:
  /// \brief RosHandler
  ros::NodeHandle nh_;
  /// \brief loop rate
  double loop_rate_ = 200;
  /// \brief time-variables
  double prev_time_ = ros::Time::now().toSec();
  double current_time_ = ros::Time::now().toSec();
  double dt_ = 0.0;
  double dt_sum_ = 0.0;

  /// \brief updates the clock
  void clock();
  /// \brief number of vehicles
  int num_of_vehicles_ = 0;
  /// \brief vehicle names
  std::vector<std::string> vehicle_names_;
  /// \brief list of vehicle managers
  std::vector<qrotor_ground::DroneManager *> vehicles_;

public:
  GroundStation(ros::NodeHandle &nh, std::vector<std::string> vehicle_names);
  ~GroundStation();

  void init();
  void run();
};

} // namespace qrotor_ground

#endif /* __QROTOR_GROUND_H__ */