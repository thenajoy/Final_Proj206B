#include <qrotor_gazebo/quadrotor_sil.h>

namespace qrotor_gazebo {
QuadrotorSIL::QuadrotorSIL() : gazebo::ModelPlugin(), nh_(nullptr) {
  for (int &pwm_output : pwm_outputs_) {
    pwm_output = 1000;
  }
  position_sp = matrix::Vector3f({0., 0., 1.0});
  J << 0.0049, 0.0000055, 0.0000054, 0.0000055, 0.0053, 0.000021, 0.0000054,
      0.000021, 0.0098;
}

QuadrotorSIL::~QuadrotorSIL() {
  updateConnection_.reset();
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void QuadrotorSIL::Load(gazebo::physics::ModelPtr _model,
                        sdf::ElementPtr _sdf) {
  if (!ros::isInitialized()) {
    ROS_FATAL("A ROS node for Gazebo has not been "
              "initialized, unable to load "
              "plugin");
    return;
  }
  ROS_INFO("QuadrotorSIL plugin loading... ");

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  // Connect the Plugin to the Robot and Save pointers to the various elements
  // in the simulation
  ROS_WARN("Searching for namespace... ");
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[QuadrotorSIL] Please specify a namespace.\n";
  // creating ros handle with the drone namespace
  nh_ = new ros::NodeHandle(namespace_);
  ROS_WARN("namespace found! %s", namespace_.c_str());
  gzmsg << "loading parameters from " << namespace_ << " ns\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[QuadrotorSIL] Please specify a linkName of the forces and "
             "moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow("[QuadrotorSIL] Couldn't find specified link \"" << link_name_
                                                             << "\".");

  if (!_sdf->HasElement("updateRate")) {
    gzdbg << "[QuadrotorSIL] missing <updateRate>, "
             "defaults to 0.0"
             " (as fast as possible)\n";
    this->update_rate_ = 0;
  } else {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    gzdbg << "[QuadrotorSIL] update rate " << update_rate_ << std::endl;
  }

  if (!_sdf->HasElement("publishRate")) {
    gzdbg << "[QuadrotorSIL] missing <publishRate>, "
             "defaults to 200.0\n";
    this->pub_rate_ = 200.0;
  } else {
    this->pub_rate_ = _sdf->GetElement("publishRate")->Get<double>();
    gzdbg << "[QuadrotorSIL] publish rate " << pub_rate_ << std::endl;
  }

  // Initialize the board
  board_.gazebo_setup(link_, world_, model_, nh_, mav_type_);

  // Inertial setup
  J << link_->GetInertial()->IXX(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IXY(),
      link_->GetInertial()->IYY(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IXZ(), link_->GetInertial()->IYZ(),
      link_->GetInertial()->IZZ();
  mass_ = link_->GetInertial()->Mass();
  std::cout << "mass: " << mass_ << "\ninertia \n";
  std::cout << J << std::endl;

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&QuadrotorSIL::OnUpdate, this, _1));

  // Getting the initial pose of the link
  initial_pose_ = link_->WorldCoGPose();

  plugin_loaded_time_ = world_->SimTime();
  last_plugin_update_ = plugin_loaded_time_;

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  pub_odom_est_ = nh_->advertise<nav_msgs::Odometry>("sim/odom/estimate", 1);
  pub_odom_sp_ = nh_->advertise<nav_msgs::Odometry>("sim/odom/setpoint", 1);
  pub_odom_truth_ = nh_->advertise<nav_msgs::Odometry>("odometry/mocap", 1);
  pub_log_ = nh_->advertise<qrotor_firmware::Log>("sim/log", 1);
  pub_imu_ = nh_->advertise<sensor_msgs::Imu>("sim/imu_raw", 1);

  ros_start_time_s = ros::Time::now().toSec();
  ros_last_update_s = ros::Time::now().toSec();

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
  this->last_pub_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
  this->last_pub_time_ = this->world_->GetSimTime();
#endif
  ROS_INFO("QuadrotorSIL plugin loading... DONE!");
}

void QuadrotorSIL::Init() {
  gzmsg << "QuadrotorSIL::Init()" << std::endl;
  // Firmware initialization
  firmware_ = new qrotor_firmware::FlightController(board_);
  // params_ = new qrotor_firmware::ParamsReconfig(*nh_, *firmware_);
  firmware_io_ = new qrotor_firmware::FirmwareIO(*this->nh_, *firmware_);

  if (firmware_ != nullptr) {
    firmware_->init();
    delete firmware_->ext_pose_handler_;
    firmware_->ext_pose_handler_ = new qrotor_firmware::ExternalPoseHandler();

    firmware_->state_machine_.set_event(
        qrotor_firmware::StateMachine::EVENT_REQUEST_ARM);
    firmware_->state_machine_.set_mode(
        qrotor_firmware::StateMachine::EVENT_REQUEST_OFFBOARD);

    /* Load Params from Gazebo Server */
    mav_type_ = "multirotor";
    mav_dynamics_ = new MultirotorForcesMoments(nh_, *firmware_);

    // set initial pose
    state_ = qrotor_gazebo::Pose3d(link_);
    pose_.position = {(float)state_.pos[0], (float)state_.pos[1],
                      (float)state_.pos[2]};
    pose_.velocity = {(float)state_.vel[0], (float)state_.vel[1],
                      (float)state_.vel[2]};
    pose_.ang_vel = {(float)state_.omega[0], (float)state_.omega[1],
                     (float)state_.omega[2]};
    Eigen::Quaterniond q(state_.rot);
    pose_.quat = {(float)q.w(), (float)q.x(), (float)q.y(), (float)q.z()};
    firmware_->pos_estimator_->set_pose(pose_);

    // dynamic reconfigure to tune params
    // setupDDynamicReconfigure();
    // params_->init();
    use_ground_effect_drag_force =
        nh_->param<bool>("use_ground_effect_drag_force", true);
    ROS_INFO("QuadrotorSIL firmware initialized!");
  }
}

/// This gets called by the world update event.
void QuadrotorSIL::OnUpdate(const gazebo::common::UpdateInfo &_info) {

#if GAZEBO_MAJOR_VERSION >= 8
  gazebo::common::Time cur_time = this->world_->SimTime();
#else
  gazebo::common::Time cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < last_time_) {
    gzdbg << "[QuadrotorSIL] Negative update time difference detected.\n";
    last_time_ = cur_time;
  }
  if (cur_time < last_pub_time_) {
    gzdbg << "[QuadrotorSIL] Negative update time difference detected.\n";
    last_pub_time_ = cur_time;
  }

  // rate control
  if (update_rate_ > 0 &&
      (cur_time - last_time_).Double() < (1.0 / this->update_rate_)) {

  } else {
    // compute "dt"
    double dt = cur_time.Double() - last_time_.Double();

    // get current link pose
    state_ = qrotor_gazebo::Pose3d(link_);
    state_.t = _info.simTime.Double();

    pose_.position = {(float)state_.pos[0], (float)state_.pos[1],
                      (float)state_.pos[2]};
    pose_.velocity = {(float)state_.vel[0], (float)state_.vel[1],
                      (float)state_.vel[2]};
    pose_.ang_vel = {(float)state_.omega[0], (float)state_.omega[1],
                     (float)state_.omega[2]};
    Eigen::Quaterniond q(state_.rot);
    pose_.quat = {(float)q.w(), (float)q.x(), (float)q.y(), (float)q.z()};

    if (firmware_ != nullptr) {
      firmware_->ext_pose_handler_->set_pose(pose_);
      // update time
      ros_now_s = ros::Time::now().toSec();
      ros_dt = (float)(ros_now_s - ros_last_update_s);
      // ROS_WARN("dt_s: %f", ros_dt);
      ros_last_update_s = ros_now_s;
      firmware_->update_time(float(ros_now_s - ros_start_s), ros_dt);
      firmware_->run();
    }
    // save last time stamp
    this->last_time_ = cur_time;
  }

  /// publish ros message
  if (pub_rate_ > 0 &&
      (cur_time - last_pub_time_).Double() < (1.0 / this->pub_rate_)) {
  } else {
    if (firmware_io_ != nullptr) {
      firmware_io_->run();
    }
    firmware_publish();
    publish_pose();
    this->last_pub_time_ = cur_time;
  }
  // apply wrench from the propellers
  applyInputWrench();
}

void QuadrotorSIL::Reset() {
  gzdbg << "--------------------------------------" << std::endl;
  gzdbg << "Resetting gazebo..." << std::endl;
  gzdbg << "--------------------------------------" << std::endl;
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();

  thrust_vector_.setZero();
  moment_vector_.setZero();

  delete firmware_;
  // delete params_;
  delete firmware_io_;

  gzdbg << "Reinitializing plugin..." << std::endl;
  QuadrotorSIL::Init();

  gzdbg << "Reset complete" << std::endl;
}

void QuadrotorSIL::applyInputWrench() {
  if (firmware_ != nullptr) {
    if (firmware_->state_machine_.state().armed) {
      for (int i = 0; i < 4; i++) {
        pwm_outputs_[i] = (int)firmware_->mixer_.pwm_us_array()[i];
      }
      MultirotorForcesMoments::CurrentState x;
      x.t = state_.t;
      x.pos = state_.pos;
      x.vel = state_.vel;
      x.rot = state_.rot;
      x.omega = state_.omega;

      thrust_vector_ = firmware_->att_controller_->input().thrust * e3_;
      moment_vector_ << firmware_->att_controller_->input().moment(0),
          firmware_->att_controller_->input().moment(1),
          firmware_->att_controller_->input().moment(2);
    } else {
      thrust_vector_ << 0., 0., 0;
      moment_vector_ << 0., 0., 0;
    }
  } else {
    thrust_vector_ << 0., 0., 0;
    moment_vector_ << 0., 0., 0;
  }

  GazeboVector force = vec3_to_gazebo_from_eigen(thrust_vector_);
  GazeboVector torque = vec3_to_gazebo_from_eigen(moment_vector_);
  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
}

void QuadrotorSIL::publish_imu() {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = link_name_;
  imu_msg.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  imu_msg.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;

  if (firmware_ != nullptr) {
    imu_msg.orientation.w = firmware_->att_estimator_->state().q()(0);
    imu_msg.orientation.x = firmware_->att_estimator_->state().q()(1);
    imu_msg.orientation.y = firmware_->att_estimator_->state().q()(2);
    imu_msg.orientation.z = firmware_->att_estimator_->state().q()(3);

    imu_msg.angular_velocity.x = firmware_->sensors_.imu().gyro(0);
    imu_msg.angular_velocity.y = firmware_->sensors_.imu().gyro(1);
    imu_msg.angular_velocity.z = firmware_->sensors_.imu().gyro(2);

    imu_msg.linear_acceleration.x = firmware_->sensors_.imu().accel(0);
    imu_msg.linear_acceleration.y = firmware_->sensors_.imu().accel(1);
    imu_msg.linear_acceleration.z = firmware_->sensors_.imu().accel(2);
  }
  //  pub_imu_.publish(imu_msg);
}

void QuadrotorSIL::publish_pose() {
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Publish truth
  nav_msgs::Odometry odometry_;
  odometry_.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  odometry_.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
  odometry_.header.frame_id = "world";
  odometry_.child_frame_id = namespace_ + "/base_link_truth";
  odometry_.pose.pose.orientation.w = GZ_COMPAT_GET_W(GZ_COMPAT_GET_ROT(pose));
  odometry_.pose.pose.orientation.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_ROT(pose));
  odometry_.pose.pose.orientation.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_ROT(pose));
  odometry_.pose.pose.orientation.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_ROT(pose));
  odometry_.pose.pose.position.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(pose));
  odometry_.pose.pose.position.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(pose));
  odometry_.pose.pose.position.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(pose));
  odometry_.twist.twist.linear.x = GZ_COMPAT_GET_X(vel);
  odometry_.twist.twist.linear.y = GZ_COMPAT_GET_Y(vel);
  odometry_.twist.twist.linear.z = GZ_COMPAT_GET_Z(vel);
  odometry_.twist.twist.angular.x = GZ_COMPAT_GET_X(omega);
  odometry_.twist.twist.angular.y = GZ_COMPAT_GET_Y(omega);
  odometry_.twist.twist.angular.z = GZ_COMPAT_GET_Z(omega);

  // std::cout << odometry_.pose.pose.position.x << " " <<
  // odometry_.pose.pose.position.y << " " << odometry_.pose.pose.position.z <<
  // std::endl;
  pub_odom_truth_.publish(odometry_);
}

void QuadrotorSIL::setupDDynamicReconfigure() {
  ros::NodeHandle nh2(*this->nh_, "plugin_params");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec2 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh2);
  ddynrec2->registerVariable<bool>(
      "use_rotor_dynamics", use_rotor_dynamics,
      [this](bool new_value) { this->use_rotor_dynamics = new_value; },
      "use_rotor_dynamics");
  ddynrec2->registerVariable<bool>(
      "use_ground_effect_drag_force", use_ground_effect_drag_force,
      [this](bool new_value) {
        this->use_ground_effect_drag_force = new_value;
      },
      "use_ground_effect_drag_force");
  ddynrec2->publishServicesTopics();
  ddynrec_.push_back(ddynrec2);
}

void QuadrotorSIL::windCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d wind;
  wind << msg.x, msg.y, msg.z;
  mav_dynamics_->set_wind(wind);
}

void QuadrotorSIL::firmware_publish() {
  if (firmware_ != nullptr) {
    // extract and publish t265 pose info
    nav_msgs::Odometry msg_;
    msg_.header.stamp = ros::Time::now();
    msg_.header.frame_id = "world";
    msg_.child_frame_id = namespace_ + "/base_link";
    msg_.pose.pose.position.x = firmware_->pos_estimator_->pose().position(0);
    msg_.pose.pose.position.y = firmware_->pos_estimator_->pose().position(1);
    msg_.pose.pose.position.z = firmware_->pos_estimator_->pose().position(2);

    msg_.pose.pose.orientation.w = firmware_->pos_estimator_->pose().quat(0);
    msg_.pose.pose.orientation.x = firmware_->pos_estimator_->pose().quat(1);
    msg_.pose.pose.orientation.y = firmware_->pos_estimator_->pose().quat(2);
    msg_.pose.pose.orientation.z = firmware_->pos_estimator_->pose().quat(3);

    msg_.twist.twist.linear.x = firmware_->pos_estimator_->pose().velocity(0);
    msg_.twist.twist.linear.y = firmware_->pos_estimator_->pose().velocity(1);
    msg_.twist.twist.linear.z = firmware_->pos_estimator_->pose().velocity(2);

    msg_.twist.twist.angular.x = firmware_->pos_estimator_->pose().ang_vel(0);
    msg_.twist.twist.angular.y = firmware_->pos_estimator_->pose().ang_vel(1);
    msg_.twist.twist.angular.z = firmware_->pos_estimator_->pose().ang_vel(2);
    pub_odom_est_.publish(msg_);

    // Firmware Log file
    qrotor_firmware::Log log_msg_;
    log_msg_.header.stamp = ros::Time::now();
    log_msg_.euler.x = firmware_->att_estimator_->state().roll();
    log_msg_.euler.y = firmware_->att_estimator_->state().pitch();
    log_msg_.euler.z = firmware_->att_estimator_->state().yaw();

    log_msg_.body_rates.x = firmware_->att_estimator_->state().ang_vel(0);
    log_msg_.body_rates.y = firmware_->att_estimator_->state().ang_vel(1);
    log_msg_.body_rates.z = firmware_->att_estimator_->state().ang_vel(2);

    log_msg_.angular_acceleration.x =
        firmware_->att_estimator_->state().ang_vel_rates(0);
    log_msg_.angular_acceleration.y =
        firmware_->att_estimator_->state().ang_vel_rates(1);
    log_msg_.angular_acceleration.z =
        firmware_->att_estimator_->state().ang_vel_rates(2);

    log_msg_.linear_acceleration.x =
        firmware_->att_estimator_->state().linear_accel(0);
    log_msg_.linear_acceleration.y =
        firmware_->att_estimator_->state().linear_accel(1);
    log_msg_.linear_acceleration.z =
        firmware_->att_estimator_->state().linear_accel(2);

    log_msg_.cmd_euler.x = firmware_->att_controller_->cmd_attitude().euler(0);
    log_msg_.cmd_euler.y = firmware_->att_controller_->cmd_attitude().euler(1);
    log_msg_.cmd_euler.z = firmware_->att_controller_->cmd_attitude().euler(2);

    log_msg_.thrust = firmware_->att_controller_->input().thrust;
    log_msg_.moment.x = firmware_->att_controller_->input().moment(0);
    log_msg_.moment.y = firmware_->att_controller_->input().moment(1);
    log_msg_.moment.z = firmware_->att_controller_->input().moment(2);
    log_msg_.lyapunov = firmware_->att_controller_->lyap();

    log_msg_.loop_rate = firmware_->get_current_freq();
    log_msg_.firmware_time = firmware_->get_firmware_time();
    log_msg_.attitude_mode = firmware_->state_machine_.state().mode;
    log_msg_.voltage = firmware_->sensors_.voltage();
    log_msg_.current = firmware_->sensors_.current();
    for (int i = 0; i < 4; ++i) {
      log_msg_.esc_in_us[i] = firmware_->mixer_.pwm_us_array()[i];
    }
    pub_log_.publish(log_msg_);

    nav_msgs::Odometry msg_sp;
    msg_sp.header.stamp = ros::Time::now();
    msg_sp.header.frame_id = "world";
    msg_sp.child_frame_id = namespace_ + "/base_link_setpoint";
    msg_sp.pose.pose.position.x =
        firmware_->pos_controller_->cmd_pose().position(0);
    msg_sp.pose.pose.position.y =
        firmware_->pos_controller_->cmd_pose().position(1);
    msg_sp.pose.pose.position.z =
        firmware_->pos_controller_->cmd_pose().position(2);
    pub_odom_sp_.publish(msg_sp);
  }
}

GZ_REGISTER_MODEL_PLUGIN(QuadrotorSIL);
} // namespace qrotor_gazebo
