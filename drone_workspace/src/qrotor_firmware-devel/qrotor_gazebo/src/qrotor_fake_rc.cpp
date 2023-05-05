#include "qrotor_gazebo/fake_rc_transmitter.h"

int main(int argc, char** argv) {
    ROS_INFO("Initializing fake rc transmitter node!");
    ros::init(argc, argv, "qrotor_fake_rc_node");
    ros::NodeHandle nh_;

    qrotor_gazebo::FakeRCTransmitter qrotor_rc(nh_);
    ROS_INFO("Initializing fake rc setting up ddyn reconfiguration");
    qrotor_rc.setup_ddynrec();
    ROS_INFO("Initializing fake rc run");
    qrotor_rc.run();
    return 1;
}
