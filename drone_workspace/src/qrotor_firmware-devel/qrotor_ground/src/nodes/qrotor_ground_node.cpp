
#include <ros/ros.h>
#include "qrotor_ground/ground_station.h"

int main(int argc, char **argv) {
  ROS_INFO("Initializing ground_station_node");
  // initializing ros node
  ros::init(argc, argv, "ground_station_node");
  ros::NodeHandle nh_;

  // get vehicle names
  std::vector<std::string> vehicle_names;
  if (nh_.hasParam("/vehicles")) {
    std::string s;
    nh_.getParam("/vehicles", s);
    std::istringstream iss(s);
    for (std::string s; iss >> s;)
      vehicle_names.push_back(s);
    for (const auto &v: vehicle_names) {
      std::cout << v << "\n";
    }
  }

  // ground station
  qrotor_ground::GroundStation station_(nh_, vehicle_names);
  return 0;
}
