#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-emplace"
//
// Created by kotaru on 2/1/21.
//

#include "nodes/firmware_params_reconfig.h"

namespace qrotor_firmware {

ParamsReconfig::ParamsReconfig(ros::NodeHandle &_nh, FlightController &fcu_)
    : nh_(_nh), firmware_(fcu_) {

  std::cout << "ParamsReconfigConstructor" << std::endl;
  pos_gains_.push_back(matrix::Vector3f(4.0, 4.0, 8.0));    // kp
  pos_gains_.push_back(matrix::Vector3f(3.0, 3.0, 6.0));    // kd
  pos_gains_.push_back(matrix::Vector3f(0.15, 0.12, 0.25)); // ki

  vel_gains_.push_back(matrix::Vector3f(2.0, 2.0, 4.0));    // kp
  vel_gains_.push_back(matrix::Vector3f(0.30, 0.30, 0.10)); // kd
  vel_gains_.push_back(matrix::Vector3f(0.15, 0.12, 0.25)); // ki

  euler_gains_.push_back(matrix::Vector3f(8.0, 8.0, 3.0));   // kp
  euler_gains_.push_back(matrix::Vector3f(0.3, 0.3, 0.225)); // kd
  euler_gains_.push_back(matrix::Vector3f(0.0, 0.0, 0.0));   // ki

  ang_vel_gains_.push_back(matrix::Vector3f(0.15, 0.15, 0.05));    // kp
  ang_vel_gains_.push_back(matrix::Vector3f(0.0001, 0.0001, 0.0)); // kd
  ang_vel_gains_.push_back(matrix::Vector3f(0.0, 0.0, 0.0));       // ki
}

void ParamsReconfig::init() {
  ros_param_read();
  set_pid_gains();
  set_noise_params();
  set_att_clf_params();
  set_param_server();
  set_setpoint();
}

void ParamsReconfig::ros_param_read() {

  ROS_INFO("reading ros parameters");

  // mass properties
  firmware_.vehicle_params_.mass_ = (float) nh_.param<double>("mass", 1.0);
  std::vector<double> inertia(9);
  if (nh_.getParam("inertia", inertia)) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j)
        firmware_.vehicle_params_.inertia_(i, j) = (float) inertia.at(3 * i + j);
    }
  }
  std::vector<double> euler_offset(3), imu_accel_offset(3), imu_gyro_offset(3);
  if (nh_.getParam("euler_offset", euler_offset)) {
    firmware_.vehicle_params_.euler_offset =
        matrix::Vector3f((float) euler_offset.at(0), (float) euler_offset.at(1), (float) euler_offset.at(2));
  }
  if (nh_.getParam("imu_offset/accel", imu_accel_offset)) {
    firmware_.sensors_.set_accel_offset((float) imu_accel_offset.at(0),
                                        (float) imu_accel_offset.at(1),
                                        (float) imu_accel_offset.at(2));
  }
  if (nh_.getParam("imu_offset/gyro", imu_gyro_offset)) {
    firmware_.sensors_.set_gyro_offset((float) imu_gyro_offset.at(0),
                                       (float) imu_gyro_offset.at(1),
                                       (float) imu_gyro_offset.at(2));
  }

  float arm_length = (float) nh_.param<double>("arm_length", 0.1725);

  // temporary rotor parameters
  bool post_process = true;
  int num_motors = (int) nh_.param("num_rotors", 4);
  firmware_.vehicle_params_.num_rotors_ = num_motors;
  std::vector<double> rotor_positions(num_motors * 3),
      rotor_normals(num_motors * 3);
  std::vector<double> rotor_angles(num_motors);
  std::vector<int> rotor_directions(num_motors);

  // reading rotor parameters
  // TODO convert nh_.param to nh_.getParam
  firmware_.vehicle_params_.rotor_force_constant_ =
      (float) nh_.param<double>("force_constant", 4.104890333e-6);
  firmware_.vehicle_params_.rotor_force_offset_ =
      (float) nh_.param<double>("force_offset", 0);
  firmware_.vehicle_params_.rotor_moment_constant_ =
      (float) nh_.param<double>("torque_constant", 1.026e-07);
  firmware_.vehicle_params_.rotor_moment_offset_ =
      (float) nh_.param<double>("torque_offset", 7.68e-04);

  firmware_.vehicle_params_.rotorspeed2pwm_scale_ =
      (float) nh_.param("rotorspeed2pwm_scale", 0);
  firmware_.vehicle_params_.rotorspeed2pwm_offset_ =
      (float) nh_.param("rotorspeed2pwm_offset", 1000);
  firmware_.vehicle_params_.max_thrust_ = (float) nh_.param("max_thrust", 10.0);
  //
  std::vector<double> cam_xyz, cam_rpy;
  if (nh_.getParam("camera/offset", cam_xyz)) {
    ROS_WARN("CAMERA/OFFSET FOUND: cam_xyz size: %zu, values: %f, %f, %f",
             cam_xyz.size(), float(cam_xyz[0]), float(cam_xyz[1]),
             float(cam_xyz[2]));
    firmware_.ext_pose_handler_->set_cam_offset(
        float(cam_xyz[0]), float(cam_xyz[1]), float(cam_xyz[2]));
  }
  if (nh_.getParam("camera/euler", cam_rpy)) {
    ROS_WARN("CAMERA/EULER FOUND: cam_rpy size: %zu, values: %f, %f, %f",
             cam_rpy.size(), float(cam_rpy[0]), float(cam_rpy[1]),
             float(cam_rpy[2]));
    firmware_.ext_pose_handler_->set_cam_eul(float(cam_rpy[0] * M_PI / 180),
                                             float(cam_rpy[1] * M_PI / 180),
                                             float(cam_rpy[2] * M_PI / 180));
  }

  if (!nh_.getParam("rotor_positions", rotor_positions))
    post_process = false;
  if (!nh_.getParam("rotor_normals", rotor_normals))
    post_process = false;
  if (!nh_.getParam("rotor_directions", rotor_directions))
    post_process = false;
  if (!nh_.getParam("rotor_angles", rotor_angles))
    post_process = false;

  // post-processing rotor parameters
  if (post_process) {
    firmware_.vehicle_params_.rotors_.clear();
    firmware_.vehicle_params_.rotors_.resize(num_motors);
    for (int i = 0; i < num_motors; ++i) {
      firmware_.vehicle_params_.rotors_.at(i).angle = rotor_angles[i];
      firmware_.vehicle_params_.rotors_.at(i).arm_length = arm_length;
      firmware_.vehicle_params_.rotors_.at(i).rotor_force_constant =
          firmware_.vehicle_params_.rotor_force_constant_;
      firmware_.vehicle_params_.rotors_.at(i).rotor_force_offset =
          firmware_.vehicle_params_.rotor_force_offset_;
      firmware_.vehicle_params_.rotors_.at(i).rotor_moment_constant =
          firmware_.vehicle_params_.rotor_moment_constant_;
      firmware_.vehicle_params_.rotors_.at(i).rotor_moment_offset =
          firmware_.vehicle_params_.rotor_moment_offset_;
      firmware_.vehicle_params_.rotors_.at(i).direction = rotor_directions[i];
      for (int j = 0; j < 3; ++j) {
        firmware_.vehicle_params_.rotors_.at(i).position(j) =
            rotor_positions[3 * i + j];
        firmware_.vehicle_params_.rotors_.at(i).normal(j) =
            rotor_normals[3 * i + j];
      }
    }
    ROS_WARN("New vehicle parameters found! re-initializing mixer");
    firmware_.mixer_.init();
  } else {
    ROS_WARN("Using default rotor configuration");
  }
  firmware_.vehicle_params_.print();
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
            ROS_INFO("position k%c_%c updated to %f", this->gain_chars[g],
                     this->axes_chars[i], new_value);
          },
          "parameter description", 0, 100);
    }
  }
  ddynrec3->publishServicesTopics();
  ddynrec_.push_back(ddynrec3);

  ros::NodeHandle nh3b(nh_, "velocity_gains");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec3b =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh3b);

  // ddynamic reconfigure setting position gains
  for (int g = 0; g < 3; ++g) {
    for (int i = 0; i < 3; ++i) {
      ddynrec3b->registerVariable<double>(
          std::string("vel_k")
              .append(1, gain_chars[g])
              .append("_")
              .append(1, axes_chars[i]),
          vel_gains_.at(g)(i),
          [this, g, i](double new_value) {
            this->vel_gains_.at(g)(i) = (float) new_value;
            this->firmware_.pos_controller_->set_vel_gains(
                vel_gains_.at(0), vel_gains_.at(1), vel_gains_.at(2));
            ROS_INFO("velocity k%c_%c updated to %f", this->gain_chars[g],
                     this->axes_chars[i], new_value);
          },
          "parameter description", 0, 100);
    }
  }
  ddynrec3b->publishServicesTopics();
  ddynrec_.push_back(ddynrec3b);

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
            ROS_INFO("position k%c_%c updated to %f", this->gain_chars[g],
                     this->axes_chars[i], new_value);
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
            ROS_INFO("position k%c_%c updated to %f", this->gain_chars[g],
                     this->axes_chars[i], new_value);
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
  ddynrec6->registerVariable<bool>(
      "GYRO_LPF", GYRO_LPF,
      [this](bool new_value) {
        this->GYRO_LPF = new_value;
        this->firmware_.att_estimator_->gyro_filters(this->GYRO_LPF,
                                                     this->GYRO_NOTCH);
        //    ROS_INFO("GYRO_LPF %d", new_value);
      },
      "GYRO_LPF");
  ddynrec6->registerVariable<double>(
      "gyro_lpf_freq", gyro_lpf_freq,
      [this](double new_value) {
        this->gyro_lpf_freq = (float) new_value;
        this->firmware_.att_estimator_->set_gyro_lpf_cutoff(
            this->gyro_lpf_freq);
        //  ROS_INFO("gyro_lpf_freq updated to %f", new_value);
      },
      "gyro low-pass filter cutoff freq", 0, 300);
  ddynrec6->registerVariable<bool>(
      "GYRO_NOTCH", GYRO_NOTCH,
      [this](bool new_value) {
        this->GYRO_NOTCH = new_value;
        this->firmware_.att_estimator_->gyro_filters(this->GYRO_LPF,
                                                     this->GYRO_NOTCH);
        //  ROS_INFO("GYRO_NOTCH %d", new_value);
      },
      "GYRO_NOTCH");
  ddynrec6->registerVariable<double>(
      "gyro_notch_freq", gyro_notch_freq,
      [this](double new_value) {
        this->gyro_notch_freq = (float) new_value;
        this->firmware_.att_estimator_->set_gyro_notch_freq(
            this->gyro_notch_freq, this->gyro_notch_width);
        //  ROS_INFO("gyro_notch_freq updated to %f", new_value);
      },
      "gyro notch filter frequency", 0, 300);
  ddynrec6->registerVariable<double>(
      "gyro_notch_width", gyro_notch_width,
      [this](double new_value) {
        this->gyro_notch_width = (float) new_value;
        this->firmware_.att_estimator_->set_gyro_notch_freq(
            this->gyro_notch_freq, this->gyro_notch_width);
        //    ROS_INFO("gyro_notch_width updated to %f", new_value);
      },
      "gyro notch filter width", 0, 1);

  ddynrec6->registerVariable<bool>(
      "GYRO_RATE_LPF", GYRO_RATE_LPF,
      [this](bool new_value) {
        this->GYRO_RATE_LPF = new_value;
        this->firmware_.att_estimator_->gyro_rate_filters(
            this->GYRO_RATE_LPF, this->GYRO_RATE_NOTCH);
        //    ROS_INFO("GYRO_RATE_LPF %d", new_value);
      },
      "GYRO_RATE_LPF");
  ddynrec6->registerVariable<double>(
      "gyro_rate_lpf_freq", gyro_rate_lpf_freq,
      [this](double new_value) {
        this->gyro_rate_lpf_freq = (float) new_value;
        this->firmware_.att_estimator_->set_gyro_rate_lpf_cutoff(
            this->gyro_rate_lpf_freq);
        //    ROS_INFO("gyro_rate_lpf_freq updated to %f", new_value);
      },
      "gyro-rate low-pass filter cutoff freq", 0, 300);
  ddynrec6->registerVariable<bool>(
      "GYRO_RATE_NOTCH", GYRO_RATE_NOTCH,
      [this](bool new_value) {
        this->GYRO_RATE_NOTCH = new_value;
        this->firmware_.att_estimator_->gyro_rate_filters(
            this->GYRO_RATE_LPF, this->GYRO_RATE_NOTCH);
        //    ROS_INFO("GYRO_RATE_NOTCH %d", new_value);
      },
      "GYRO_RATE_NOTCH");
  ddynrec6->registerVariable<double>(
      "gyro_rate_notch_freq", gyro_rate_notch_freq,
      [this](double new_value) {
        this->gyro_rate_notch_freq = (float) new_value;
        this->firmware_.att_estimator_->set_gyro_rate_notch_freq(
            this->gyro_rate_notch_freq, this->gyro_rate_notch_width);
        ROS_INFO("gyro_rate_notch_freq updated to %f", new_value);
      },
      "gyro-rate notch filter frequency", 0, 300);
  ddynrec6->registerVariable<double>(
      "gyro_rate_notch_width", gyro_rate_notch_width,
      [this](double new_value) {
        this->gyro_rate_notch_width = (float) new_value;
        this->firmware_.att_estimator_->set_gyro_rate_notch_freq(
            this->gyro_rate_notch_freq, this->gyro_rate_notch_width);
        // ROS_INFO("gyro_rate_notch_width updated to %f", new_value);
      },
      "gyro-rate notch filter width", 0, 1);

  ddynrec6->registerVariable<bool>(
      "ACCEL_LPF", ACCEL_LPF,
      [this](bool new_value) {
        this->ACCEL_LPF = new_value;
        this->firmware_.att_estimator_->accel_filters(this->ACCEL_LPF,
                                                      this->ACCEL_NOTCH);
        //    ROS_INFO("ACCEL_LPF %d", new_value);
      },
      "ACCEL_LPF");
  ddynrec6->registerVariable<double>(
      "accel_lpf_freq", accel_lpf_freq,
      [this](double new_value) {
        this->accel_lpf_freq = (float) new_value;
        this->firmware_.att_estimator_->set_accel_lpf_cutoff(
            this->accel_lpf_freq);
        // ROS_INFO("accel_lpf_freq updated to %f", new_value);
      },
      "accel low-pass filter cutoff freq", 0, 300);
  ddynrec6->registerVariable<bool>(
      "ACCEL_NOTCH", ACCEL_NOTCH,
      [this](bool new_value) {
        this->ACCEL_NOTCH = new_value;
        this->firmware_.att_estimator_->accel_filters(this->ACCEL_LPF,
                                                      this->ACCEL_NOTCH);
        //    ROS_INFO("ACCEL_NOTCH %d", new_value);
      },
      "ACCEL_NOTCH");
  ddynrec6->registerVariable<double>(
      "accel_notch_freq", accel_notch_freq,
      [this](double new_value) {
        this->accel_notch_freq = (float) new_value;
        this->firmware_.att_estimator_->set_accel_notch_freq(
            this->accel_notch_freq, this->accel_notch_width);
        // ROS_INFO("accel_notch_freq updated to %f", new_value);
      },
      "accel notch filter frequency", 0, 300);
  ddynrec6->registerVariable<double>(
      "accel_notch_width", accel_notch_width,
      [this](double new_value) {
        this->accel_notch_width = (float) new_value;
        this->firmware_.att_estimator_->set_accel_notch_freq(
            this->accel_notch_freq, this->accel_notch_width);
        // ROS_INFO("accel_notch_width updated to %f", new_value);
      },
      "accel notch filter width", 0, 1);

  ddynrec6->publishServicesTopics();
  ddynrec_.push_back(ddynrec6);
}

void ParamsReconfig::set_att_clf_params() {

  ros::NodeHandle nh_clf(nh_, "attitude_clf");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec7 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_clf);
  ddynrec7->registerVariable<double>(
      "eta", eta2,
      [this](double new_value) {
        this->eta2 = (float) new_value;
        ROS_INFO("eta updated to %f", new_value);
      },
      "eta", 0, 200);
  ddynrec7->registerVariable<double>(
      "epsilon2", epsilon2,
      [this](double new_value) {
        this->epsilon2 = (float) new_value;
        ROS_INFO("epsilon2 updated to %f", new_value);
      },
      "epsilon2", 0, 50);
  ddynrec7->registerVariable<double>(
      "c2", c2,
      [this](double new_value) {
        this->c2 = (float) new_value;
        ROS_INFO("c2 updated to %f", new_value);
      },
      "c2", 0, 50);
}

void ParamsReconfig::firmware_param_cb(double new_value, Params p) {
  this->firmware_.params_->set_param(p, (float) new_value);
}

void ParamsReconfig::set_param_server() {

  ros::NodeHandle nh_firmware_params(nh_, "firmware_params");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec8 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(
          nh_firmware_params);

  for (int p = Params::START; p != Params::END; ++p) {
    if (p == Params::START)
      continue;
    ddynrec8->registerVariable<double>(
        firmware_.params_->get_param_name((Params) p),
        (double) firmware_.params_->get((Params) p),
        boost::bind(&ParamsReconfig::firmware_param_cb, this, _1, (Params) p),
        "param values", 0, 1000);
  }
  ddynrec8->publishServicesTopics();
  ddynrec_.push_back(ddynrec8);
}

void ParamsReconfig::set_setpoint() {

  ros::NodeHandle nh_sp(nh_, "setpoint");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec9 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_sp);

  for (int i = 0; i < 3; i++) {
    ddynrec9->registerVariable<double>(
        "sp_flats_" + std::to_string(i), _pos_sp(i),
        [this, i](double new_value) {
          this->_pos_sp(i) = (float) new_value;
          this->firmware_.mission_planner_->set_setpoint(_pos_sp(0), _pos_sp(1),
                                                         _pos_sp(2));
        },
        "setpoint position_" + std::to_string(i), -20.0,
        20.0); // TODO: add a group
  }

  ddynrec9->registerVariable<double>(
      "yaw_setpoint", 0.0f,
      [this](double new_value) {
        this->firmware_.att_controller_->set_yaw_sp(
            float(new_value * M_PI / 180.f));
      },
      "yaw setpoint", -180, 180);

  ddynrec9->publishServicesTopics();
  ddynrec_.push_back(ddynrec9);
}

} // namespace qrotor_firmware

#pragma clang diagnostic pop