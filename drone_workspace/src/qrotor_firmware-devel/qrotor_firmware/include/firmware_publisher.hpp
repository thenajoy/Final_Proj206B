#ifndef QROTOR_FIRMWARE_PUBLISHER_HPP
#define QROTOR_FIRMWARE_PUBLISHER_HPP

// system
#include <ros/ros.h>
#include <sys/time.h>

#include <iostream>

namespace qrotor_firmware {

template <class T>
class FirmwarePublisher {
  private:
    int publish_rate_;
    ros::NodeHandle &nh_;

  public:
    ros::Publisher _ros_msg;
    T msg_;
    int counter_;
    std::string name_;

    FirmwarePublisher(ros::NodeHandle& nh, std::string _name, int _publish_rate = 0) : nh_(nh), name_(_name), publish_rate_(publish_rate_) {
        counter_ = 0;
        _ros_msg = nh_.advertise<T>(name_, 1);
    }

    void publish() {
        //if (publish_rate_  <= counter_) {
        //    printf("tada! publishing....!\n");
        _ros_msg.publish(msg_);
        //counter_ = 0;
        //}
        //else {
        //    counter_++;
        //}
    }
};

}  // namespace qrotor_firmware

#endif  // QROTOR_FIRMWARE_PUBLISHER_HPP
