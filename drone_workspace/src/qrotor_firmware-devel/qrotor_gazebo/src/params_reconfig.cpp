//
// Created by kotaru on 2/1/21.
//

#include "qrotor_gazebo/params_reconfig.h"

namespace qrotor_gazebo {

ParamsReconfig::ParamsReconfig(ros::NodeHandle &_nh, qrotor_firmware::FlightController &fcu_) : nh_(_nh), firmware_(fcu_) {

  pos_gains_.push_back(matrix::Vector3f(4.0, 4.0, 8.0));    // kp
  pos_gains_.push_back(matrix::Vector3f(3.0, 3.0, 6.0));    // kd
  pos_gains_.push_back(matrix::Vector3f(0.15, 0.12, 0.25)); // ki

  euler_gains_.push_back(matrix::Vector3f(8.0, 8.0, 3.0));   // kp
  euler_gains_.push_back(matrix::Vector3f(0.3, 0.3, 0.225)); // kd
  euler_gains_.push_back(matrix::Vector3f(0.0, 0.0, 0.0));   // ki

  ang_vel_gains_.push_back(matrix::Vector3f(0.15, 0.15, 0.05));    // kp
  ang_vel_gains_.push_back(matrix::Vector3f(0.0001, 0.0001, 0.0)); // kd
  ang_vel_gains_.push_back(matrix::Vector3f(0.0, 0.0, 0.0));       // ki
}

void ParamsReconfig::init() {

  set_pid_gains();
  set_noise_params();
  set_att_clf_params();

}

void ParamsReconfig::set_pid_gains() {
  ros::NodeHandle nh3(nh_, "position_gains");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec3 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh3);

  ddynrec3->registerVariable<bool>(
      "position_PID", false,
      [this](bool new_value) {
        this->firmware_.pos_controller_->set_pid_flag(new_value);
      },
      "position PID flag");

// ddynamic reconfigure setting position gains
  for (int g = 0; g < 3; ++g) {
    for (int i = 0; i < 3; ++i) {
      ddynrec3->registerVariable<double>(
          std::string("pos_k")
              .append(1, gain_chars[g])
              .append("_")
              .append(1, axes_chars[i]),
          pos_gains_.at(g)(i),
          [this, g, i](double new_value) {
            this->pos_gains_.at(g)(i) = (float) new_value;
            this->firmware_.pos_controller_->set_pos_gains(
                pos_gains_.at(0), pos_gains_.at(1), pos_gains_.at(2));
            ROS_INFO("position k%c_%c updated to %f", this->gain_chars[g], this->axes_chars[i], new_value);
          },
          "parameter description", 0, 100);
    }
  }
  ddynrec3->publishServicesTopics();
  ddynrec_.push_back(ddynrec3);

  ros::NodeHandle nh_att(nh_, "attitude_gains");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec4 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_att);
  // ddynamic reconfigure setting attitude gains
  for (int g = 0; g < 3; ++g) {
    for (int i = 0; i < 3; ++i) {
      ddynrec4->registerVariable<double>(
          std::string("attitude_k")
              .append(1, gain_chars[g])
              .append("_")
              .append(1, axes_chars[i]),
          euler_gains_.at(g)(i),
          [this, g, i](double new_value) {
            this->euler_gains_.at(g)(i) = (float) new_value;
            this->firmware_.att_controller_->set_pos_gains(
                euler_gains_.at(0), euler_gains_.at(1), euler_gains_.at(2));
            ROS_INFO("position k%c_%c updated to %f", this->gain_chars[g], this->axes_chars[i], new_value);
          },
          "parameter description", 0, 100);
    }
  }
  ddynrec4->publishServicesTopics();
  ddynrec_.push_back(ddynrec4);

  ros::NodeHandle nh_ang_vel(nh_, "ang_vel_gains");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec5 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_ang_vel);
  // ddynamic reconfigure setting ang_vel gains
  for (int g = 0; g < 3; ++g) {
    for (int i = 0; i < 3; ++i) {
      ddynrec5->registerVariable<double>(
          std::string("ang_vel_k")
              .append(1, gain_chars[g])
              .append("_")
              .append(1, axes_chars[i]),
          ang_vel_gains_.at(g)(i),
          [this, g, i](double new_value) {
            this->ang_vel_gains_.at(g)(i) = (float) new_value;
            this->firmware_.att_controller_->set_vel_gains(
                ang_vel_gains_.at(0), ang_vel_gains_.at(1),
                ang_vel_gains_.at(2));
            ROS_INFO("position k%c_%c updated to %f", this->gain_chars[g], this->axes_chars[i], new_value);
          },
          "parameter description", 0, 100);
    }
  }
  ddynrec5->publishServicesTopics();
  ddynrec_.push_back(ddynrec5);
}

void ParamsReconfig::set_noise_params() {

  ros::NodeHandle nh_noise(nh_, "NOISE_PARAMS");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec6 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_noise);
  // ddynamic reconfigure setting noise-freq gains
  ddynrec6->registerVariable<bool>("GYRO_LPF", GYRO_LPF, [this](bool new_value) {
    this->GYRO_LPF = new_value;
    ROS_INFO("GYRO_LPF %d", new_value);
  }, "GYRO_LPF");
  ddynrec6->registerVariable<double>("gyro_lpf_freq", gyro_lpf_freq, [this](double new_value) {
                                       this->gyro_lpf_freq = (float) new_value;
                                       ROS_INFO("gyro_lpf_freq updated to %f", new_value);
                                     },
                                     "gyro low-pass filter cutoff freq", 0, 300);
  ddynrec6->registerVariable<bool>("GYRO_NOTCH", GYRO_NOTCH, [this](bool new_value) {
    this->GYRO_NOTCH = new_value;
    ROS_INFO("GYRO_NOTCH %d", new_value);
  }, "GYRO_NOTCH");
  ddynrec6->registerVariable<double>("gyro_notch_freq", gyro_notch_freq, [this](double new_value) {
                                       this->gyro_notch_freq = (float) new_value;
                                       ROS_INFO("gyro_notch_freq updated to %f", new_value);
                                     },
                                     "gyro notch filter frequency", 0, 300);
  ddynrec6->registerVariable<double>("gyro_notch_width", gyro_notch_width, [this](double new_value) {
                                       this->gyro_notch_width = (float) new_value;
                                       ROS_INFO("gyro_notch_width updated to %f", new_value);
                                     },
                                     "gyro notch filter width", 0, 1);

  ddynrec6->registerVariable<bool>("GYRO_RATE_LPF", GYRO_RATE_LPF, [this](bool new_value) {
    this->GYRO_RATE_LPF = new_value;
    ROS_INFO("GYRO_RATE_LPF %d", new_value);
  }, "GYRO_RATE_LPF");
  ddynrec6->registerVariable<double>("gyro_rate_lpf_freq", gyro_rate_lpf_freq, [this](double new_value) {
                                       this->gyro_rate_lpf_freq = (float) new_value;
                                       ROS_INFO("gyro_rate_lpf_freq updated to %f", new_value);
                                     },
                                     "gyro-rate low-pass filter cutoff freq", 0, 300);
  ddynrec6->registerVariable<bool>("GYRO_RATE_NOTCH", GYRO_RATE_NOTCH, [this](bool new_value) {
    this->GYRO_RATE_NOTCH = new_value;
    ROS_INFO("GYRO_RATE_NOTCH %d", new_value);
  }, "GYRO_RATE_NOTCH");
  ddynrec6->registerVariable<double>("gyro_rate_notch_freq", gyro_rate_notch_freq, [this](double new_value) {
                                       this->gyro_rate_notch_freq = (float) new_value;
                                       ROS_INFO("gyro_rate_notch_freq updated to %f", new_value);
                                     },
                                     "gyro-rate notch filter frequency", 0, 300);
  ddynrec6->registerVariable<double>("gyro_rate_notch_width", gyro_rate_notch_width, [this](double new_value) {
                                       this->gyro_rate_notch_width = (float) new_value;
                                       ROS_INFO("gyro_rate_notch_width updated to %f", new_value);
                                     },
                                     "gyro-rate notch filter width", 0, 1);

  ddynrec6->registerVariable<bool>("ACCEL_LPF", ACCEL_LPF, [this](bool new_value) {
    this->ACCEL_LPF = new_value;
    ROS_INFO("ACCEL_LPF %d", new_value);
  }, "ACCEL_LPF");
  ddynrec6->registerVariable<double>("accel_lpf_freq", accel_lpf_freq, [this](double new_value) {
                                       this->accel_lpf_freq = (float) new_value;
                                       ROS_INFO("accel_lpf_freq updated to %f", new_value);
                                     },
                                     "accel low-pass filter cutoff freq", 0, 300);
  ddynrec6->registerVariable<bool>("ACCEL_NOTCH", ACCEL_NOTCH, [this](bool new_value) {
    this->ACCEL_NOTCH = new_value;
    ROS_INFO("ACCEL_NOTCH %d", new_value);
  }, "ACCEL_NOTCH");
  ddynrec6->registerVariable<double>("accel_notch_freq", accel_notch_freq, [this](double new_value) {
                                       this->accel_notch_freq = (float) new_value;
                                       ROS_INFO("accel_notch_freq updated to %f", new_value);
                                     },
                                     "accel notch filter frequency", 0, 300);
  ddynrec6->registerVariable<double>("accel_notch_width", accel_notch_width, [this](double new_value) {
                                       this->accel_notch_width = (float) new_value;
                                       ROS_INFO("accel_notch_width updated to %f", new_value);
                                     },
                                     "accel notch filter width", 0, 1);

  ddynrec6->publishServicesTopics();
  ddynrec_.push_back(ddynrec6);
}

void ParamsReconfig::set_att_clf_params() {

  ros::NodeHandle nh_clf(nh_, "attitude_clf");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec7 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_clf);
  ddynrec7->registerVariable<double>("eta", eta2, [this](double new_value) {
    this->eta2 = (float) new_value;
    ROS_INFO("eta2 updated to %f", new_value);
  }, "eta2", 0, 200);
  ddynrec7->registerVariable<double>("epsilon2", epsilon2, [this](double new_value) {
    this->epsilon2 = (float) new_value;
    ROS_INFO("epsilon2 updated to %f", new_value);
  }, "epsilon2", 0, 50);
  ddynrec7->registerVariable<double>("c2", c2, [this](double new_value) {
    this->c2 = (float) new_value;
    ROS_INFO("c2 updated to %f", new_value);
  }, "c2", 0, 50);

}

}
