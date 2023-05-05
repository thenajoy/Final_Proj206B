
// qrotor_hil_interface
#include "qrotor_ground/qrotor_display.h"

int main(int argc, char **argv) {
  int exit_val = 0;

  ROS_INFO("Initialing qrotor_display");
  // initializing ros node
  ros::init(argc, argv, "display_node");
  ros::NodeHandle nh;

  std::string vehicle_name;
  nh.param<std::string>("/vehicle/name", vehicle_name, "qrotor2");
  // std::cout << "vehicle name: " <<  vehicle_name << std::endl;

  qrotor_ground::DisplayInterface ds(nh, vehicle_name);

  ds.updateTime();
  ros::Rate run_ros_loop_at(24);
  while (ros::ok()) {
    ros::spinOnce();

    /*************/
    // std::cout <<
    // "-----------------------------------------------------------------------------------------"
    // << std::endl; std::cout << "ROLL\t\t| PITCH \t| YAW\t\t| gx\t\t| gy\t\t|
    // gz\t\t|" << std::endl; std::cout << "\033[2J\033[1;1H";
    system("clear");
    ds.update();

    run_ros_loop_at.sleep();
  }

  ros::shutdown();
  return 0;
}
