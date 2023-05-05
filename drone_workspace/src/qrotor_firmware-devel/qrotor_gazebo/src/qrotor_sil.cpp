#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <qrotor_gazebo/qrotor_sil.h>

namespace qrotor_gazebo {
QrotorFlightSIL::QrotorFlightSIL()
    : gazebo::ModelPlugin(), nh_(nullptr), firmware_(board_) {

  this->colorCamera_ = nullptr;
  this->trackingCamera_ = nullptr;

  for (int i = 0; i < 4; i++) {
    pwm_outputs_[i] = 1000;
  }
  position_sp = matrix::Vector3f({0., 0., 1.0});
}

QrotorFlightSIL::~QrotorFlightSIL() {
  GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void QrotorFlightSIL::Load(gazebo::physics::ModelPtr _model,
                           sdf::ElementPtr _sdf) {
  if (!ros::isInitialized()) {
    ROS_FATAL("A ROS node for Gazebo has not been "
              "initialized, unable to load "
              "plugin");
    return;
  }
  ROS_INFO("QrotorFlightSIL plugin loading... ");

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  // Connect the Plugin to the Robot and Save pointers to the various elements
  // in the simulation
  ROS_WARN("Searching for namespace... ");
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[QrotorFlightSIL] Please specify a namespace.\n";
  // creating ros handle with the drone namespace
  nh_ = new ros::NodeHandle(namespace_);
  ROS_WARN("namespace found! %s", namespace_.c_str());
  gzmsg << "loading parameters from " << namespace_ << " ns\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[QrotorFlightSIL] Please specify a linkName of the forces and "
             "moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) gzthrow("[QrotorFlightSIL] Couldn't find specified link \"" << link_name_
                                                                                 << "\".");
  // Initialize the board
  board_.gazebo_setup(link_, world_, model_, nh_, mav_type_);
  // Firmware initialization
  firmware_.init();

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&QrotorFlightSIL::OnUpdate, this, _1));

  // Getting the initial pose of the link
  initial_pose_ = GZ_COMPAT_GET_WORLD_COG_POSE(link_);

  // Set the camera plugin (for arcuo pose estimation)
  if (link_->GetSensorCount()) {
    gzmsg << "Number of sensors in " << namespace_ << " :"
          << link_->GetSensorCount() << "\n";
    gazebo::sensors::SensorManager *smanager =
        gazebo::sensors::SensorManager::Instance();

    auto sensors_ = smanager->GetSensors();
    std::string sensor_scoped_name;
    for (int i = 0; i < sensors_.size(); i++) {
      if ((sensors_.at(i)->ScopedName().find(std::string("color_camera")) !=
          std::string::npos) && sensors_.at(i)->ScopedName().find(namespace_) !=
          std::string::npos) {
        sensor_scoped_name = sensors_.at(i)->ScopedName();
      }
    }
    gzmsg << sensor_scoped_name << std::endl;
    this->colorCamera_ =
        std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(
            smanager->GetSensor(sensor_scoped_name))
            ->Camera();
    if (colorCamera_ == nullptr) {
      gzerr << "colorCamera is NULLPTR\n";
    } else {
      gzmsg << "Well defined colorCameraPtr\n";
    }
    this->newColorFrameConn = this->colorCamera_->ConnectNewImageFrame(
        std::bind(&QrotorFlightSIL::OnNewFrame, this, this->colorCamera_,
                  this->colorPub));
  } else {
    gzdbg << "No sensors found" << std::endl;
  }

  QrotorFlightSIL::init();

  plugin_loaded_time_ = GZ_COMPAT_GET_SIM_TIME(world_);
  last_plugin_update_ = plugin_loaded_time_;
  ROS_INFO("QrotorFlightSIL plugin loading... DONE!");
}

void QrotorFlightSIL::get_ros_params() {

  std::vector<double> _poc;
  nh_->getParam("cable_poc", _poc);
  if (_poc.size() == 3) {
    cablePOC_PF_ << _poc.at(0), _poc.at(1), _poc.at(2);
  } else {
    cablePOC_PF_ << 0.25, -0.25, 0;
    ROS_ERROR("cable poc is not of size 3");
  }

  // read camera pose in quadrotor-body-frame
  std::vector<double> cam_rpy, cam_xyz;

  nh_->getParam("camera/rpy", cam_rpy);
  if (cam_rpy.size() == 3) {
    tf2::Quaternion q;
    q.setRPY(cam_rpy.at(0), cam_rpy.at(1), cam_rpy.at(2));
    Eigen::Quaterniond qEig(q.getW(), q.getX(), q.getY(), q.getZ());
    RCam2Quad_ = qEig.toRotationMatrix();

    ROS_INFO("camera: r: %f, p: %f, y:%f",
             cam_rpy.at(0) * M_RAD_TO_DEG,
             cam_rpy.at(1) * M_RAD_TO_DEG,
             cam_rpy.at(2) * M_RAD_TO_DEG);
  } else {
    ROS_ERROR("camera/rpy is not of size 3");
    RCam2Quad_ = Eigen::Matrix3d::Identity();
  }
  gzmsg << "RCam2Quad_ \n" << RCam2Quad_ << std::endl;

  nh_->getParam("camera/xyz", cam_xyz);
  if (cam_xyz.size() == 3) {
    tCamInQuad_ << cam_xyz.at(0), cam_xyz.at(1), cam_xyz.at(2);
  } else {
    ROS_ERROR("camera/xyz is not of size 3");
    tCamInQuad_.setZero();
  }

  tf2::Quaternion q2;
  q2.setRPY(-M_PI_2, 0, -M_PI_2);
  Eigen::Quaterniond q2Eig(q2.getW(), q2.getX(), q2.getY(), q2.getZ());
  ROptical2Cam_ = q2Eig.toRotationMatrix();
  gzmsg << "ROptical2Cam_ \n" << ROptical2Cam_ << std::endl;

  // reading constraint standard deviation
  nh_->getParam("constraint/h1_stddev", h1_sigma);
  nh_->getParam("constraint/h2_stddev", h2_sigma);
  ROS_INFO("constraint std-dev: h1: %f, h2: %f", h1_sigma, h2_sigma);

}

void QrotorFlightSIL::init() {

  // read ros params from param-server
  get_ros_params();

  /* Load Params from Gazebo Server */
  mav_type_ = "multirotor";

  // tracking camera setup
  trackingCamera_ = new FakeTrackingCamera(link_, *nh_, mav_type_);

  // arcuo pose estimator
  this->arcuo_estimator_ = new pest::PayloadArcuoPose(true, namespace_ + "_image_window");

  // ros publishers
  pub_odom_truth_ = nh_->advertise<nav_msgs::Odometry>("odom/truth", 1);
  pub_odom_est_ = nh_->advertise<nav_msgs::Odometry>("odom/estimate", 1);
  pub_log_ = nh_->advertise<qrotor_firmware::Log>("log", 1);
  pub_imu_ = nh_->advertise<sensor_msgs::Imu>("imu_raw", 1);
  pub_pointOfContact_ = nh_->advertise<geometry_msgs::PointStamped>("cable_poc/estimate", 1);
  pub_pointOfContact_truth_ = nh_->advertise<geometry_msgs::PointStamped>("cable_poc/truth", 1);
  pub_payload_odom_ = nh_->advertise<nav_msgs::Odometry>("payload/odom", 1);

  // Setting up pose estimator
  pose_iekf_ = new payload_estimation::ConstraintIEKF<double>();
  if (board_.accel_stddev() > 0 || board_.gyro_stddev() > 0) {
    pose_iekf_->noise_.set_imu_stddev(board_.accel_stddev(), board_.gyro_stddev());
  }
  if (trackingCamera_->position_stddev() > 0) {
    pose_iekf_->noise_.set_position_std(trackingCamera_->position_stddev());
  }
  if (trackingCamera_->velocity_stddev() > 0) {
    pose_iekf_->noise_.set_velocity_std(trackingCamera_->velocity_stddev());
  }
  if (trackingCamera_->rotation_stddev() > 0) {
    pose_iekf_->noise_.set_rotation_std(trackingCamera_->rotation_stddev());
  }
  pose_iekf_->updateNoiseParams();
  pose_iekf_->setConstraintCovariance(h1_sigma, h2_sigma);

  // dynamic reconfigure to tune params
  setupDDynamicReconfigure();
}

/// This gets called by the world update event.
void QrotorFlightSIL::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  // time update
  double dt_ = _info.simTime.Double() - last_plugin_update_.Double();
  last_plugin_update_ = _info.simTime.Double();

  // get current link pose
  state_ = qrotor_gazebo::Pose3d(link_);
  state_.t = _info.simTime.Double();

  // run the firmware
  firmware_.run();

  // publish imu data
  publishImu();

  timer_.pose_iekf.dtsum += dt_;
  if (timer_.pose_iekf.dtsum >= 1 / timer_.pose_iekf.freq) {
    // printf("pose_iekf dtsum: %f \n", timer_.pose_iekf.dtsum);
    // run pose invariant ekf
    runPoseEstimation();

    publishPose();
    firmware_.pos_estimator_->set_pose(pose_);

    timer_.pose_iekf.dtsum = 0;
  }

  // apply forces
  applyInputWrench();
}

void QrotorFlightSIL::Reset() {
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();

  gzdbg << "gazebo reset" << std::endl;
}

void QrotorFlightSIL::OnNewFrame(const gazebo::rendering::CameraPtr cam,
                                 const gazebo::transport::PublisherPtr pub) {
  // convert to opencv image
  char *data;
  data = new char[(cam->ImageHeight() * cam->ImageWidth() * 3) + 1]; // rgb 8 bit data
  memcpy(data, cam->ImageData(), cam->ImageHeight() * cam->ImageWidth() * 3);
  cv::Mat image(cam->ImageHeight(), cam->ImageWidth(), CV_8UC3, data);
  cv::cvtColor(image, image, CV_RGB2BGR);

  if (arcuo_estimator_->run(image, quad_pose_CF_, load_pose_CF_)) {
    if (load_pose_CF_.quat.norm() > 1e-6) {
      if (load_pose_CF_.quat.norm() != 1) {
        load_pose_CF_.quat.normalize();
      }

      Eigen::Matrix3d Rmarker_;
      Eigen::Vector3d tmarker_;

      // convert marker pose from optical frame to camera body-frame
      //      Eigen::Matrix3d Rmarker_ = ROptical2Cam_*load_pose_CF_.quat.toRotationMatrix();
      //      Eigen::Vector3d tmarker_ = ROptical2Cam_*load_pose_CF_.position;

      // camera body-frame to quadrotor body-frame
      //      Rmarker_ = RCam2Quad_ * ROptical2Cam_*load_pose_CF_.quat.toRotationMatrix();
      //      tmarker_ = RCam2Quad_ * ROptical2Cam_*load_pose_CF_.position + tCamInQuad_;

      // quadrotor frame to world-frame
      Rmarker_ = state_.rot * RCam2Quad_ * ROptical2Cam_ * load_pose_CF_.quat.toRotationMatrix();;
      tmarker_ = state_.rot * (RCam2Quad_ * ROptical2Cam_ * load_pose_CF_.position + tCamInQuad_)
          + state_.pos; // TODO use estimated pose for state

      nav_msgs::Odometry odom_;
      odom_.header.stamp = ros::Time::now();
      odom_.header.frame_id = "world";
      odom_.child_frame_id = "payload_link/marker";
      tf::quaternionEigenToMsg(Eigen::Quaterniond(Rmarker_), odom_.pose.pose.orientation);
      tf::pointEigenToMsg(tmarker_, odom_.pose.pose.position);
      pub_payload_odom_.publish(odom_);


      /**********************************************************/

//      Eigen::Vector3d cablePOC_ = load_pose_CF_.position + load_pose_CF_.quat.toRotationMatrix() * cablePOC_PF_;
//      Eigen::Vector3d cablePOC_WF_, cablePOC_BF_;
//      cablePOC_WF_ = cablePOC_PF_;
//      cablePOC_WF_(2) += 0.2; // Note value hardcoded
//
//      cablePOC_BF_ = state_.rot.transpose() * cablePOC_WF_ - state_.rot.transpose() * state_.pos;
//      pose_iekf_->pseudoMeasUpdate(cablePOC_WF_); // TODO compute the cable poc in body-frame?
//
//      geometry_msgs::PointStamped msg_;
//      msg_.header.frame_id = namespace_ + "/base_link"; // "/camera_link_optical";
//      msg_.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
//      msg_.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
//      tf::pointEigenToMsg(cablePOC_BF_, msg_.point);
//      pub_pointOfContact_.publish(msg_);
//
//      geometry_msgs::PointStamped msg2_;
//      msg2_.header.frame_id = "world";
//      msg2_.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
//      msg2_.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
//      tf::pointEigenToMsg(cablePOC_WF_, msg2_.point);
//      pub_pointOfContact_truth_.publish(msg2_);


    }
  }
  delete data;
}

void QrotorFlightSIL::runPoseEstimation() {
  // pose estimation

  if (isPoseInEKFEnabled) {
    double diff_dt =
        (GZ_COMPAT_GET_SIM_TIME(world_) - plugin_loaded_time_).Double();
    if ((diff_dt == std::floor(diff_dt)) && (!isPoseInEKFInitialized)) {
      gzmsg << diff_dt << " seconds since plugin-loaded "
            << " pose_iekf_wait_time " << pose_iekf_wait_time_ << std::endl;
    }

    if ((GZ_COMPAT_GET_SIM_TIME(world_) - plugin_loaded_time_ >
        pose_iekf_wait_time_) &&
        (!isPoseInEKFInitialized)) {

      gzdbg << "entered pose estimation initialization block" << std::endl;
      Pose3d current_pose = Pose3d(link_);
      // initialize the estimator
      pose_iekf_->init(pest::RigidbodyState<double>(
          current_pose.rot, current_pose.vel,
          current_pose.pos + Eigen::Vector3d(1., 1., 1.)));
      last_imu_update_us = firmware_.sensors_.imu().timestamp_us;

      publishEstimatedOdom();

      isPoseInEKFInitialized = true;
      ROS_DEBUG("pose iekf initialized");
      ROS_WARN("last_time_imu_update_us: %f", last_imu_update_us * 1e-6);
      return; // return after initialization, start estimation from next
      // iteration
    }
    if (isPoseInEKFInitialized) {
      // ROS_INFO("estimation loop iter %d", estimation_loop_iter_);
      estimation_loop_iter_++;
      // if (estimation_loop_iter_ > max_estimation_iters_) { isPoseInEKFEnabled
      // = false; }

      // gather imu sensor data
      dt = (firmware_.sensors_.imu().timestamp_us - last_imu_update_us) * 1e-6;
      imu_reading_.update(
          (double) firmware_.sensors_.imu().timestamp_us,
          firmware_.sensors_.imu().accel(0), firmware_.sensors_.imu().accel(1),
          firmware_.sensors_.imu().accel(2), firmware_.sensors_.imu().gyro(0),
          firmware_.sensors_.imu().gyro(1), firmware_.sensors_.imu().gyro(2));

      if (dt > 0.02) {
        gzwarn << dt << " seconds passed since last time-update" << std::endl;
        if (dt > 10) {
          gzerr << "[BAD] \"dt\" is very high! skipping this turn" << std::endl;
          last_imu_update_us = firmware_.sensors_.imu().timestamp_us;
          return;
        }
      }
      // Invariant EKF time update
      pose_iekf_->timeUpdate(dt, imu_reading_);

      // measurement update
      // TODO reduce the frequency of measurement update
      pose_iekf_->measUpdate(trackingCamera_->simulate());
      last_imu_update_us = firmware_.sensors_.imu().timestamp_us;

      // if (pose_iekf_.state().position().norm() > 0.1) { isPoseInEKFEnabled =
      // false; }
      publishEstimatedOdom();
    }
  }
}

void QrotorFlightSIL::applyInputWrench() {
  // TODO: fix this

  /*    // Convert gazebo types to Eigen and switch to NED frame
      state.pos = NWU_to_NED *
     vec3_to_eigen_from_gazebo(GZ_COMPAT_GET_POS(pose)); state.rot = NWU_to_NED
     * rotation_to_eigen_from_gazebo(GZ_COMPAT_GET_ROT(pose)); state.vel =
     NWU_to_NED * vec3_to_eigen_from_gazebo(vel); state.omega = NWU_to_NED *
     vec3_to_eigen_from_gazebo(omega); state.t = _info.simTime.Double();

      for (int i = 0; i < 4; i++) {
          pwm_outputs_[i] = firmware_.mixer_.pwm_us_array()[i];
      }
      forces_ = mav_dynamics_->updateForcesAndTorques(state, pwm_outputs_);
      //    std::cout << "pwm outputs_ " << pwm_outputs_[0] << " " <<
     pwm_outputs_[1] << " " <<  pwm_outputs_[2] << " " << pwm_outputs_[3] <<
     std::endl;

      // apply the forces and torques to the joint (apply in NWU)
      GazeboVector force = vec3_to_gazebo_from_eigen(NWU_to_NED *
     forces_.block<3, 1>(0, 0)); GazeboVector torque =
     vec3_to_gazebo_from_eigen(NWU_to_NED * forces_.block<3, 1>(3, 0));
      link_->AddRelativeForce(force);
      link_->AddRelativeTorque(torque);
  */
  // compute pwms to forces
  thrust_vector_ = firmware_.att_controller_->input().thrust * state_.rot * e3_;
  moment_vector_ << firmware_.att_controller_->input().moment(0),
      firmware_.att_controller_->input().moment(1),
      firmware_.att_controller_->input().moment(2);

  // apply the forces and torques to the joint (apply in NWU)
  GazeboVector force = vec3_to_gazebo_from_eigen(thrust_vector_);
  GazeboVector torque = vec3_to_gazebo_from_eigen(moment_vector_);
  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque);
}

void QrotorFlightSIL::publishImu() {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = link_name_;
  imu_msg.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  imu_msg.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;

  imu_msg.orientation.w = firmware_.att_estimator_.state().q()(0);
  imu_msg.orientation.x = firmware_.att_estimator_.state().q()(1);
  imu_msg.orientation.y = firmware_.att_estimator_.state().q()(2);
  imu_msg.orientation.z = firmware_.att_estimator_.state().q()(3);

  imu_msg.angular_velocity.x = firmware_.sensors_.imu().gyro(0);
  imu_msg.angular_velocity.y = firmware_.sensors_.imu().gyro(1);
  imu_msg.angular_velocity.z = firmware_.sensors_.imu().gyro(2);

  imu_msg.linear_acceleration.x = firmware_.sensors_.imu().accel(0);
  imu_msg.linear_acceleration.y = firmware_.sensors_.imu().accel(1);
  imu_msg.linear_acceleration.z = firmware_.sensors_.imu().accel(2);
  pub_imu_.publish(imu_msg);
}

void QrotorFlightSIL::publishEstimatedOdom() {
  // pack odometry message
  nav_msgs::Odometry odometry_;
  odometry_.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  odometry_.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
  odometry_.header.frame_id = "world";
  odometry_.child_frame_id = link_name_;
  // gather pose message
  Eigen::Quaterniond q(pose_iekf_->state().rotation());
  tf::quaternionEigenToMsg(q, odometry_.pose.pose.orientation);
  tf::pointEigenToMsg(pose_iekf_->state().position(),
                      odometry_.pose.pose.position);
  // extract pose covariance from the iekf covariance matrix
  Eigen::MatrixXd pose_cov_ = Eigen::Matrix<double, 6, 6>::Zero();
  pose_cov_.block(0, 0, 3, 3) = pose_iekf_->state_covar().block(6, 6, 3, 3);
  pose_cov_.block(3, 3, 3, 3) = pose_iekf_->state_covar().block(0, 0, 3, 3);
  for (int col = 0; col < 6; col++) {
    for (int row = 0; row < 6; row++) {
      odometry_.pose.covariance[row + col * 6] = pose_cov_(row, col);
    }
  }
  // gather twist message
  tf::vectorEigenToMsg(pose_iekf_->state().velocity(),
                       odometry_.twist.twist.linear);
  tf::vectorEigenToMsg(pose_iekf_->state().angular_velocity(),
                       odometry_.twist.twist.angular);
  Eigen::Matrix3d vel_cov_ = pose_iekf_->state_covar().block(3, 3, 3, 3);
  for (int col = 0; col < 3; col++) {
    for (int row = 0; row < 3; row++) {
      odometry_.twist.covariance[row + col * 6] = vel_cov_(row, col);
    }
  }
  pub_odom_est_.publish(odometry_);
}

void QrotorFlightSIL::publishPose() {
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Publish truth
  nav_msgs::Odometry odometry_;
  odometry_.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  odometry_.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
  odometry_.header.frame_id = "world";
  odometry_.child_frame_id = link_name_;
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
  pub_odom_truth_.publish(odometry_);

  pose_.position = {(float) odometry_.pose.pose.position.x,
                    (float) odometry_.pose.pose.position.y,
                    (float) odometry_.pose.pose.position.z};
  pose_.velocity = {(float) odometry_.twist.twist.linear.x,
                    (float) odometry_.twist.twist.linear.y,
                    (float) odometry_.twist.twist.linear.z};
  pose_.quat = {(float) odometry_.pose.pose.orientation.w,
                (float) odometry_.pose.pose.orientation.x,
                (float) odometry_.pose.pose.orientation.y,
                (float) odometry_.pose.pose.orientation.z};
  pose_.ang_vel = {(float) odometry_.twist.twist.angular.x,
                   (float) odometry_.twist.twist.angular.y,
                   (float) odometry_.twist.twist.angular.z};

  // sending transform
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = odometry_.header;
  transformStamped.child_frame_id = odometry_.child_frame_id;
  transformStamped.transform.translation.x = odometry_.pose.pose.position.x;
  transformStamped.transform.translation.y = odometry_.pose.pose.position.y;
  transformStamped.transform.translation.z = odometry_.pose.pose.position.z;
  transformStamped.transform.rotation = odometry_.pose.pose.orientation;
  br.sendTransform(transformStamped);

  // Firmware Log file
  qrotor_firmware::Log log_msg_;
  log_msg_.header.stamp = ros::Time::now();
  log_msg_.euler.x = firmware_.att_estimator_.state().roll();
  log_msg_.euler.y = firmware_.att_estimator_.state().pitch();
  log_msg_.euler.z = firmware_.att_estimator_.state().yaw();

  log_msg_.body_rates.x = firmware_.att_estimator_.state().ang_vel(0);
  log_msg_.body_rates.y = firmware_.att_estimator_.state().ang_vel(1);
  log_msg_.body_rates.z = firmware_.att_estimator_.state().ang_vel(2);

  log_msg_.cmd_euler.x = firmware_.att_controller_->cmd_attitude().euler(0);
  log_msg_.cmd_euler.y = firmware_.att_controller_->cmd_attitude().euler(1);
  log_msg_.cmd_euler.z = firmware_.att_controller_->cmd_attitude().euler(2);

  log_msg_.thrust = firmware_.att_controller_->input().thrust;
  log_msg_.moment.x = firmware_.att_controller_->input().moment(0);
  log_msg_.moment.y = firmware_.att_controller_->input().moment(1);
  log_msg_.moment.z = firmware_.att_controller_->input().moment(2);

  log_msg_.loop_rate = firmware_.get_current_freq();
  log_msg_.firmware_time = firmware_.get_firmware_time();
  log_msg_.attitude_mode = firmware_.command_manager_.mode();
  pub_log_.publish(log_msg_);
}

void QrotorFlightSIL::setupDDynamicReconfigure() {
  ros::NodeHandle nh3(*this->nh_, "setpoint");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec3 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh3);
  for (int i = 0; i < 3; i++) {
    ddynrec3->registerVariable<double>(
        "sp_flats_" + std::to_string(i), position_sp(i),
        [this, i](double new_value) {
          this->position_sp(i) = new_value;
          this->firmware_.pos_controller_->set_cmd_setpoint(position_sp);
        },
        "setpoint position_" + std::to_string(i), -20.0,
        20.0); // TODO: add a group
  }
  ddynrec3->publishServicesTopics();
  ddynrec_.push_back(ddynrec3);

  ros::NodeHandle nh2(*this->nh_, "plugin_params");
  std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec2 =
      std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh2);
  ddynrec2->registerVariable<double>(
      "pose_iekf_wait_time", pose_iekf_wait_time_,
      [this](double new_value) { this->pose_iekf_wait_time_ = new_value; },
      "pose_iekf_wait_time", 0.0, 600.0);
  ddynrec2->registerVariable<bool>("isPoseInEKFEnabled", isPoseInEKFEnabled,
                                   [this](bool new_value) {
                                     this->isPoseInEKFEnabled = new_value;
                                     gzmsg << "pose ekf updated " << new_value
                                           << std::endl;
                                   },
                                   "isPoseInEKFEnabled");
  ddynrec2->registerVariable<int>(
      "max_estimation_iters", max_estimation_iters_,
      [this](int new_value) { this->max_estimation_iters_ = new_value; },
      "max_estimation_iters", 0, 10000);
  ddynrec2->publishServicesTopics();
  ddynrec_.push_back(ddynrec2);
}

void QrotorFlightSIL::windCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d wind;
  wind << msg.x, msg.y, msg.z;
//  mav_dynamics_->set_wind(wind);
}

GZ_REGISTER_MODEL_PLUGIN(QrotorFlightSIL);
} // namespace qrotor_gazebo
