#include "qrotor_gazebo/multi_fake_rc_transmitters.h"

int main(int argc, char** argv) {
    ROS_INFO("Initializing fake rc transmitter node!");
    ros::init(argc, argv, "multi_qrotor_fake_rc_node");
    ros::NodeHandle nh_;

    qrotor_gazebo::MultipleFakeRCTransmitters multi_qrotor_rc(nh_, 4);
    multi_qrotor_rc.setup_ddynrec();
    multi_qrotor_rc.run();
    return 1;
}
