#ifndef QROTOR_GAZEBO_FAKE_RC_TRANSMITTER_H
#define QROTOR_GAZEBO_FAKE_RC_TRANSMITTER_H

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ros/ros.h>
#include <sys/time.h>
#include <tf/tf.h>

#include <string>
#include <qrotor_firmware/RCRaw.h>

namespace qrotor_gazebo {

class FakeRCTransmitter {
private:
  // ros
  ros::NodeHandle nh_;

  // RC publisher
  ros::Publisher pub_rc_raw;
  qrotor_firmware::RCRaw msg_rc_raw;
  int rc_sticks[4] = {1500, 1500, 1000, 1500};
  int rc_switches[4] = {1000, 1000, 1000, 1000};
  bool arm_disarm_requested = false, arm_status = false, arm_it = false;
  int arm_counter = 0;

  // DDynamic reconfigure
  ddynamic_reconfigure::DDynamicReconfigure ddynrec_;

public:
  FakeRCTransmitter(ros::NodeHandle &_nh) : nh_(_nh) {
    pub_rc_raw = nh_.advertise<qrotor_firmware::RCRaw>("RC", 3);
  }
  ~FakeRCTransmitter() {}

  void setup_ddynrec() {
    /// Channel parameters
    for (int i = 0; i < 4; i++) {
      ddynrec_.registerVariable<int>(
          "channel_" + std::to_string(i + 1), rc_sticks[i],
          [this, i](int new_value) {
            this->rc_sticks[i] = new_value;
          }, "rc channels", 1000,
          2000);
    }

    /// kill switch
    ddynrec_.registerVariable<bool>(
        "KILL_SWITCH", false, [this](bool new_value) {
          this->kill_switch(new_value);
        }, "channel 5");

    /// mode switch
    ddynrec_.registerVariable<int>(
        "MODE_SWITCH", 0, [this](int new_value) {
          this->mode_switch(new_value);
        }, "channel 7", 0, 2);

    /// arm/disarm
    ddynrec_.registerVariable<bool>(
        "ARM_DISARM", false, [this](bool new_value) {
          this->arm_disarm(new_value);
        }, "arm_disarm");

    /// mode switch
    ddynrec_.registerVariable<bool>(
        "MISSION_SWITCH", false, [this](bool new_value) {
          this->mission_switch(new_value);
        }, "channel 5");


    /// publish
    ddynrec_.publishServicesTopics();
  }

  void arm_disarm(bool ARM) {
    this->arm_it = ARM;
    this->arm_disarm_requested = true;
    if (this->arm_it)
      ROS_WARN("Arm requested!");
    else
      ROS_WARN("Disarm requested!");
  }

  void kill_switch(bool KILL) {
    if (!KILL) {
      this->rc_switches[0] = 1000;
    } else {
      this->rc_switches[0] = 2000;
    }
  }

  void mode_switch(int MODE) {
    switch (MODE) {
    case 0:this->rc_switches[2] = 1000;
      break;
    case 1:this->rc_switches[2] = 1500;
      break;
    case 2:this->rc_switches[2] = 2000;
      break;
    default:this->rc_switches[2] = 1000;
      break;
    }
  }

  void mission_switch(bool flag) {
    if (flag) {
      this->rc_switches[1] = 2000;
    } else {
      this->rc_switches[1] = 1000;
    }
  }

  void publish_rc_msg() {
    msg_rc_raw.header.stamp = ros::Time::now();
    if (arm_disarm_requested) {
      if (arm_it) {
        //                ROS_INFO("Sending fake arm signal!");
        msg_rc_raw.values[0] = 1500;
        msg_rc_raw.values[1] = 1500;
        msg_rc_raw.values[2] = 1000;  // throttle to low
        msg_rc_raw.values[3] = 2000;  // yaw to the right
      } else {
        //                ROS_INFO("Sending fake disarm signal!");
        msg_rc_raw.values[0] = 1500;
        msg_rc_raw.values[1] = 1500;
        msg_rc_raw.values[2] = 1000;  // throttle to low
        msg_rc_raw.values[3] = 1000;  // yaw to the left
      }
      msg_rc_raw.values[4] = rc_switches[0];  // Kill switch
      msg_rc_raw.values[5] = rc_switches[1];
      msg_rc_raw.values[6] = rc_switches[2];
      msg_rc_raw.values[7] = rc_switches[3];
      if (arm_counter > 100) {
        arm_disarm_requested = false;
        arm_counter = 0;
      }
      arm_counter++;
    } else {
      for (int i = 0; i < 4; i++) {
        msg_rc_raw.values[i] = rc_sticks[i];
      }
      for (int i = 0; i < 4; i++) {
        msg_rc_raw.values[i + 4] = rc_switches[i];
      }
    }
    pub_rc_raw.publish(msg_rc_raw);
  }

  void run() {
    ros::Rate loop_rate(50);
    while (ros::ok()) {
      ros::spinOnce();
      publish_rc_msg();
      loop_rate.sleep();
    }
    ros::shutdown();
  }
};

} // namespace qrotor_gazebo
#endif // QROTOR_GAZEBO_FAKE_RC_TRANSMITTER_H
