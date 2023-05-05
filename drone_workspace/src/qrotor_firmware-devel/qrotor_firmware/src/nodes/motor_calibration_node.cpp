// system
#include <ros/ros.h>
#include <sys/time.h>
#include <dynamic_reconfigure/server.h>
#include "qrotor_firmware/motor_calibConfig.h"

#include "navio_utils.h"

float pwm, script_pwm;
int channel, DIR;
bool activate = false;
bool duty_on = false;
bool run_script = false;
bool new_channel_activated = false;
qrotor_firmware::BrushlessMotor *rotor;

void cfg_callback(qrotor_firmware::motor_calibConfig &config, uint32_t level) {

  ROS_INFO("Reconfigure Request: Activate: %s Channel: %d pwm in us: %f",
           config.activate ? "True" : "False", config.channel,
           config.pwm);
  activate = config.activate;
  channel = config.channel;
  pwm = config.pwm;
  duty_on = config.duty_on;
  run_script = config.run_script;
  if (run_script) {
    script_pwm = 1000;
    DIR = 1;
  }

  if (activate && !new_channel_activated) {
    new_channel_activated = true;
    ROS_INFO("Activating new channel %d", channel);
    rotor = new qrotor_firmware::BrushlessMotor(channel);
  }

  if (!activate) {
    new_channel_activated = false;
  }

}

int main(int argc, char **argv) {

  ROS_INFO("Initialing motor_calibration_node");
  ros::init(argc, argv, "motor_calibration_node");
  ros::NodeHandle nh;

  //qrotor_firmware::BrushlessMotor rotor(0);

  // dynamic reconfigure
  dynamic_reconfigure::Server<qrotor_firmware::motor_calibConfig> server_cfg;
  dynamic_reconfigure::Server<qrotor_firmware::motor_calibConfig>::CallbackType cfg_hndl;

  cfg_hndl = boost::bind(&cfg_callback, _1, _2);
  server_cfg.setCallback(cfg_hndl);

  double tprev = ros::Time::now().toSec();
  double tnow = ros::Time::now().toSec();
  double dt = tnow - tprev;
  double dtsum = 0.0;

  //int DIR = 1;

  ros::Rate run_ros_loop_at(500);
  while (ros::ok()) {
    ros::spinOnce();
    /**************/
    tnow = ros::Time::now().toSec();
    dt = -tprev + tnow;
    tprev = tnow;
    dtsum += dt;
    //ROS_INFO("freq: %f",1/dt_s);
    /**************/

    if (run_script) {

      if (dtsum >= 4) {
        dtsum = 0;
        script_pwm += 100 * DIR;
        script_pwm = script_pwm > 1000 ? script_pwm : 1000;
        script_pwm = script_pwm < 2000 ? script_pwm : 2000;
        ROS_INFO("script_pwm %f", script_pwm);
      }

      if (DIR == 1 && script_pwm >= 2000) {
        DIR = -1;
      }
      if (DIR == -1 && script_pwm <= 1000) {
        DIR = 0;
      }

      rotor->set_duty_cycle(script_pwm);
      //ROS_INFO("script_pwm %f", script_pwm);
    } else {
      dtsum = 0;
      if (duty_on) {
        rotor->set_duty_cycle(pwm);
      }
    }

    /**************/
    run_ros_loop_at.sleep();
  }

  return 0;
}
