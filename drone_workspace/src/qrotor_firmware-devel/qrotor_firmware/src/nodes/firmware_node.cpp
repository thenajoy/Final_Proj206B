#include <ros/ros.h>

#include "nodes/firmware_interface.h"

int main(int argc, char** argv) {
    ROS_INFO("Initializing firmware_node!");
    ros::init(argc, argv, "firmware_node");
    ros::NodeHandle nh_;

    qrotor_firmware::FirmwareInterface qrotor_interface_(nh_);

    std::cout << "\033[31;1mEntering qrotor_interface_ init()\033[0m" << std::endl;
    qrotor_interface_.init();
    std::cout << "\033[31;1mEntering qrotor_interface_ run()\033[0m" << std::endl;
    qrotor_interface_.run();
}
