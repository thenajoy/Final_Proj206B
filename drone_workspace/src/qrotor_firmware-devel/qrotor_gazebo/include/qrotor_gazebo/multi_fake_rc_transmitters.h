#ifndef QROTOR_GAZEBO_MULTIPLE_FAKE_RC_H
#define QROTOR_GAZEBO_MULTIPLE_FAKE_RC_H

#include "qrotor_gazebo/fake_rc_transmitter.h"
#include "vector"
#include "string"

namespace qrotor_gazebo {

class MultipleFakeRCTransmitters {
  private:
    // ros
    ros::NodeHandle nh_;

    int num_qrotors;
    std::vector<std::shared_ptr<FakeRCTransmitter>> rc_transmitters_;
    // DDynamic reconfigure
    ddynamic_reconfigure::DDynamicReconfigure ddynrec_;

  public:
    MultipleFakeRCTransmitters(ros::NodeHandle& _nh, int n): nh_(_nh) {
        num_qrotors = n;
        for (int i = 0; i < num_qrotors; i++) {

            ros::NodeHandle nh(nh_, "quad" + std::to_string(i + 1));
            std::shared_ptr<FakeRCTransmitter> qrc_ =
                std::make_shared<FakeRCTransmitter>(nh);

            //            FakeRCTransmitter qrc_(nh);
            rc_transmitters_.push_back(qrc_);
        }
    }
    ~MultipleFakeRCTransmitters() {}


    void setup_ddynrec() {
        /// kill switch
        ddynrec_.registerVariable<bool>(
        "KILL_SWITCH", false, [this](bool new_value) {
            for (int i = 0; i < num_qrotors; i++) {
                rc_transmitters_.at(i)->kill_switch(new_value);
            }
        }, "channel 5");

        /// mode switch
        ddynrec_.registerVariable<int>(
        "MODE_SWITCH", 0, [this](int new_value) {
            for (int i = 0; i < num_qrotors; i++) {
                rc_transmitters_.at(i)->mode_switch(new_value);
            }
        }, "channel 7", 0, 2);

        /// arm/disarm
        ddynrec_.registerVariable<bool>(
        "ARM_DISARM", false, [this](bool new_value) {
            for (int i = 0; i < num_qrotors; i++) {
                rc_transmitters_.at(i)->arm_disarm(new_value);
            }
        }, "arm_disarm");
        ddynrec_.publishServicesTopics();
    }


    void run() {
        ros::Rate loop_rate(50);
        while (ros::ok()) {
            ros::spinOnce();
            for (int i = 0; i < num_qrotors; i++) {
                rc_transmitters_.at(i)->publish_rc_msg();
            }
            loop_rate.sleep();
        }
        ros::shutdown();
    }
};

} // qrotor_gazebo

#endif // QROTOR_GAZEBO_MULTIPLE_FAKE_RC_H
