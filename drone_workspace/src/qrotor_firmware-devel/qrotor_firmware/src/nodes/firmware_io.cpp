//
// Created by kotaru on 5/7/21.
//
#include "nodes/firmware_io.h"
namespace qrotor_firmware {

FirmwareIO::FirmwareIO(ros::NodeHandle &_nh, FlightController &_fcu)
    : nh_(_nh), firmware_(_fcu) {

  std::cout << "FirmwareIO" << std::endl;

  // setup ros-subscriber
  sub_mocap_ =
      nh_.subscribe("odometry/mocap", 3, &FirmwareIO::callback_sub_mocap, this);
  sub_rc_ = nh_.subscribe("RC", 3, &FirmwareIO::callback_sub_rc, this);
  for (int i = 0; i < 8; ++i) {
    rc_from_ros[i] = 1000;
  }

  sub_kill_all_ = nh_.subscribe("/killall", 3, &FirmwareIO::killAllCb, this);
  sub_takeoff_all_ = nh_.subscribe("/takeoffall", 3, &FirmwareIO::takeOffAllCb, this);
  sub_land_all_ = nh_.subscribe("/landall", 3, &FirmwareIO::landAllCb, this);
  sub_mission_cmds_all_ = nh_.subscribe("/missionall", 3, &FirmwareIO::missionStartCb, this);
  sub_move_all_ = nh_.subscribe("/moveall", 3, &FirmwareIO::moveAllCb, this);

  subThrustVector = nh_.subscribe("thrust_force", 3, &FirmwareIO::thrustVectorCb, this);

  // initialize the ros-service servers
  imu_calibrate_srv_ = nh_.advertiseService(
      "calibrate_imu", &FirmwareIO::calibrateImuSrvCB, this);
  motors_kill_srv_ =
      nh_.advertiseService("kill", &FirmwareIO::killMotorsSrvCB, this);
  motors_arm_srv_ =
      nh_.advertiseService("arm", &FirmwareIO::armMotorsSrvCB, this);
  motors_disarm_srv_ =
      nh_.advertiseService("disarm", &FirmwareIO::disarmMotorsSrvCB, this);
  event_go_srv_ =
      nh_.advertiseService("activate_mission", &FirmwareIO::eventGoSrvCB, this);
  event_nogo_srv_ =
      nh_.advertiseService("kill_mission", &FirmwareIO::eventNoGoSrvCB, this);
  takeoff_srv_ =
      nh_.advertiseService("takeoff", &FirmwareIO::takeoffSrvCB, this);
  landing_srv_ = nh_.advertiseService("land", &FirmwareIO::landingSrvCB,
                                      this); // TODO add return to home
  control_mode_srv_ =
      nh_.advertiseService("mode", &FirmwareIO::controlModeSrvCB, this);
  offb_ctrl_srv_ =
      nh_.advertiseService("offboard", &FirmwareIO::offbCtrlSrvCB, this);
  move_srv_ = nh_.advertiseService("move", &FirmwareIO::moveSrvCB, this);
  setpoint_srv_ =
      nh_.advertiseService("move_to", &FirmwareIO::setpointSrvCB, this);
  flat_traj_srv_ =
      nh_.advertiseService("flat_trajectory", &FirmwareIO::flatTrajSrvCB, this);
}

// ros-service callback functions
bool FirmwareIO::calibrateImuSrvCB(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res) {
  firmware_.sensors_.imu_calibration();
  res.success = true;
  return true;
}
bool FirmwareIO::killMotorsSrvCB(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res) {
  firmware_.state_machine_.set_event(StateMachine::EVENT_KILL_MOTORS);
  res.success = true;
  return true;
}
bool FirmwareIO::armMotorsSrvCB(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res) {
  firmware_.state_machine_.set_event(StateMachine::EVENT_REQUEST_ARM);
  res.success = true;
  return true;
}
bool FirmwareIO::disarmMotorsSrvCB(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res) {
  firmware_.state_machine_.set_event(StateMachine::EVENT_REQUEST_DISARM);
  res.success = true;
  return true;
}
bool FirmwareIO::eventGoSrvCB(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res) {
  firmware_.mission_planner_->set_mission(
      MissionPlanner::EVENT_ACTIVATE_MISSION);
  res.success = true;
  return true;
}
bool FirmwareIO::eventNoGoSrvCB(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res) {
  firmware_.mission_planner_->set_mission(MissionPlanner::EVENT_KILL_MISSION);
  res.success = true;
  return true;
}
bool FirmwareIO::takeoffSrvCB(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res) {
  firmware_.mission_planner_->set_mission(
      MissionPlanner::EVENT_REQUEST_TAKEOFF);
  res.success = true;
  return true;
}
bool FirmwareIO::landingSrvCB(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res) {
  firmware_.mission_planner_->set_mission(MissionPlanner::EVENT_REQUEST_LAND);
  res.success = true;
  return true;
}
bool FirmwareIO::controlModeSrvCB(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res) {
  if (req.data) {
    firmware_.state_machine_.set_mode(
        StateMachine::EVENT_REQUEST_POSITION_HOLD);
  } else {
    firmware_.state_machine_.set_mode(StateMachine::EVENT_REQUEST_MANUAL);
  }
  res.success = true;
  return true;
}

bool FirmwareIO::offbCtrlSrvCB(std_srvs::SetBool::Request &req,
                               std_srvs::SetBool::Response &res) {
  if (req.data) {
    firmware_.state_machine_.set_mode(
        StateMachine::EVENT_REQUEST_OFFBOARD);
  } else {
    firmware_.state_machine_.set_mode(StateMachine::EVENT_REQUEST_POSITION_HOLD);
  }
  res.success = true;
  return true;
}

bool FirmwareIO::moveSrvCB(qrotor_firmware::Setpoint::Request &req,
                           qrotor_firmware::Setpoint::Response &res) {
  matrix::Vector3f sp = firmware_.pos_estimator_->pose().position;
  sp(0) += float(req.x);
  sp(1) += float(req.y);
  sp(2) += float(req.z);
  firmware_.mission_planner_->set_setpoint(sp(0), sp(1), sp(2));
  firmware_.mission_planner_->set_mission(MissionPlanner::EVENT_REQUEST_SETPOINT);
  res.success = true;
  return true;
}
void FirmwareIO::moveAllCb(const geometry_msgs::Vector3::ConstPtr &msg) {
  matrix::Vector3f sp = firmware_.pos_estimator_->pose().position;
  sp(0) += float(msg->x);
  sp(1) += float(msg->y);
  sp(2) += float(msg->z);
  firmware_.mission_planner_->set_setpoint(sp(0), sp(1), sp(2));
  firmware_.mission_planner_->set_mission(MissionPlanner::EVENT_REQUEST_SETPOINT);
}

bool FirmwareIO::setpointSrvCB(qrotor_firmware::Setpoint::Request &req,
                               qrotor_firmware::Setpoint::Response &res) {
  Logger::STATUS(
      utils::Cat("Position Setpoint: ", req.x, "\t", req.y, "\t", req.z));
  firmware_.mission_planner_->set_setpoint(float(req.x), float(req.y),
                                           float(req.z));
  firmware_.mission_planner_->set_mission(MissionPlanner::EVENT_REQUEST_SETPOINT);
  res.success = true;
  return true;
}

bool FirmwareIO::flatTrajSrvCB(qrotor_firmware::FlatTrajectory::Request &req,
                               qrotor_firmware::FlatTrajectory::Response &res) {
  if (req.type == req.TRAJECTORY_CIRCLE_2D) {
    firmware_.mission_planner_->set_flat_traj_params(float(req.center.x), float(req.center.y),
                                                     float(req.center.z), float(req.radius.x), float(req.phase.x));
  } else {
    ROS_ERROR("No other trajectory type is currently implemented!");
  }

  res.success = true;
  return true;
}

// run
void FirmwareIO::run() {
  // publish ros messages
  publish();
}

void FirmwareIO::killAllCb(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    firmware_.state_machine_.set_event(StateMachine::EVENT_KILL_MOTORS);
  }
}

void FirmwareIO::takeOffAllCb(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    firmware_.mission_planner_->set_mission(
        MissionPlanner::EVENT_REQUEST_TAKEOFF);
  }
}

void FirmwareIO::landAllCb(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    firmware_.mission_planner_->set_mission(MissionPlanner::EVENT_REQUEST_LAND);

  }
}
void FirmwareIO::missionStartCb(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    firmware_.mission_planner_->set_mission(
        MissionPlanner::EVENT_ACTIVATE_MISSION);
  } else {
    firmware_.mission_planner_->set_mission(
        MissionPlanner::EVENT_KILL_MISSION);
  }
}

void FirmwareIO::thrustVectorCb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
//  ROS_WARN("thrust vector received");
  if (firmware_.state_machine_.state().mode == StateMachine::OFFBOARD) {
    firmware_.att_controller_->set_thrust_vector(float(msg->twist.linear.x), float(msg->twist.linear.y), float(msg->twist.linear.z));
    firmware_.att_controller_->set_yaw_sp(float(msg->twist.angular.z));
  }
}

// ros topic callbacks
void FirmwareIO::callback_sub_mocap(
    const nav_msgs::Odometry::ConstPtr &msg) const {

  //  ROS_INFO("new pose received");
  PoseWithCovariance pose_;
  pose_.position = {static_cast<float>(msg->pose.pose.position.x),
                    static_cast<float>(msg->pose.pose.position.y),
                    static_cast<float>(msg->pose.pose.position.z)};
  pose_.quat = {static_cast<float>(msg->pose.pose.orientation.w),
                static_cast<float>(msg->pose.pose.orientation.x),
                static_cast<float>(msg->pose.pose.orientation.y),
                static_cast<float>(msg->pose.pose.orientation.z)};
  pose_.velocity = {static_cast<float>(msg->twist.twist.linear.x),
                    static_cast<float>(msg->twist.twist.linear.y),
                    static_cast<float>(msg->twist.twist.linear.z)};
  firmware_.ext_pose_handler_->set_mocap_pose(pose_);
}

void FirmwareIO::callback_sub_rc(const qrotor_firmware::RCRaw::ConstPtr &msg) {
  for (int i = 0; i < 8; ++i) {
    rc_from_ros[i] = msg->values[i];
  }
  firmware_.sensors_.set_rc_from_ros(rc_from_ros);
}

// ros topic publish
void FirmwareIO::publish() {
  // extract and publish t265 pose info
  pub_odom_est_.msg_.header.stamp = ros::Time::now();
  pub_odom_est_.msg_.header.frame_id = "world";
  pub_odom_est_.msg_.child_frame_id = std::string("body-frame");
  pub_odom_est_.msg_.pose.pose.position.x =
      firmware_.pos_estimator_->pose().position(0);
  pub_odom_est_.msg_.pose.pose.position.y =
      firmware_.pos_estimator_->pose().position(1);
  pub_odom_est_.msg_.pose.pose.position.z =
      firmware_.pos_estimator_->pose().position(2);

  pub_odom_est_.msg_.pose.pose.orientation.w =
      firmware_.pos_estimator_->pose().quat(0);
  pub_odom_est_.msg_.pose.pose.orientation.x =
      firmware_.pos_estimator_->pose().quat(1);
  pub_odom_est_.msg_.pose.pose.orientation.y =
      firmware_.pos_estimator_->pose().quat(2);
  pub_odom_est_.msg_.pose.pose.orientation.z =
      firmware_.pos_estimator_->pose().quat(3);

  pub_odom_est_.msg_.twist.twist.linear.x =
      firmware_.pos_estimator_->pose().velocity(0);
  pub_odom_est_.msg_.twist.twist.linear.y =
      firmware_.pos_estimator_->pose().velocity(1);
  pub_odom_est_.msg_.twist.twist.linear.z =
      firmware_.pos_estimator_->pose().velocity(2);

  pub_odom_est_.msg_.twist.twist.angular.x =
      firmware_.pos_estimator_->pose().ang_vel(0);
  pub_odom_est_.msg_.twist.twist.angular.y =
      firmware_.pos_estimator_->pose().ang_vel(1);
  pub_odom_est_.msg_.twist.twist.angular.z =
      firmware_.pos_estimator_->pose().ang_vel(2);
  pub_odom_est_.publish();

  // extract and publish t265 pose info
  pub_odom_t265_.msg_.header.stamp = ros::Time::now();
  pub_odom_t265_.msg_.header.frame_id = "world";
  pub_odom_t265_.msg_.child_frame_id = std::string("t265");
  pub_odom_t265_.msg_.pose.pose.position.x =
      firmware_.ext_pose_handler_->pose().position(0);
  pub_odom_t265_.msg_.pose.pose.position.y =
      firmware_.ext_pose_handler_->pose().position(1);
  pub_odom_t265_.msg_.pose.pose.position.z =
      firmware_.ext_pose_handler_->pose().position(2);

  pub_odom_t265_.msg_.pose.pose.orientation.w =
      firmware_.ext_pose_handler_->pose().quat(0);
  pub_odom_t265_.msg_.pose.pose.orientation.x =
      firmware_.ext_pose_handler_->pose().quat(1);
  pub_odom_t265_.msg_.pose.pose.orientation.y =
      firmware_.ext_pose_handler_->pose().quat(2);
  pub_odom_t265_.msg_.pose.pose.orientation.z =
      firmware_.ext_pose_handler_->pose().quat(3);

  pub_odom_t265_.msg_.twist.twist.linear.x =
      firmware_.ext_pose_handler_->pose().velocity(0);
  pub_odom_t265_.msg_.twist.twist.linear.y =
      firmware_.ext_pose_handler_->pose().velocity(1);
  pub_odom_t265_.msg_.twist.twist.linear.z =
      firmware_.ext_pose_handler_->pose().velocity(2);

  pub_odom_t265_.msg_.twist.twist.angular.x =
      firmware_.ext_pose_handler_->pose().ang_vel(0);
  pub_odom_t265_.msg_.twist.twist.angular.y =
      firmware_.ext_pose_handler_->pose().ang_vel(1);
  pub_odom_t265_.msg_.twist.twist.angular.z =
      firmware_.ext_pose_handler_->pose().ang_vel(2);
  pub_odom_t265_.publish();

  // sending transform
  //  geometry_msgs::TransformStamped transformStamped;
  //  transformStamped.header.stamp = ros::Time::now();
  //  transformStamped.header.frame_id = "world";
  //  transformStamped.child_frame_id = "t265_optical";
  //  transformStamped.transform.translation.x =
  //  pub_odometry_.msg_.pose.pose.position.x;
  //  transformStamped.transform.translation.y =
  //  pub_odometry_.msg_.pose.pose.position.y;
  //  transformStamped.transform.translation.z =
  //  pub_odometry_.msg_.pose.pose.position.z;
  //  transformStamped.transform.rotation =
  //  pub_odometry_.msg_.pose.pose.orientation;
  //  br.sendTransform(transformStamped);

  // extract and publish quadrotor log info
  pub_log_.msg_.header.stamp = ros::Time::now();
  pub_log_.msg_.euler.x = firmware_.att_estimator_->state().roll();
  pub_log_.msg_.euler.y = firmware_.att_estimator_->state().pitch();
  pub_log_.msg_.euler.z = firmware_.att_estimator_->state().yaw();

  pub_log_.msg_.body_rates.x = firmware_.att_estimator_->state().ang_vel(0);
  pub_log_.msg_.body_rates.y = firmware_.att_estimator_->state().ang_vel(1);
  pub_log_.msg_.body_rates.z = firmware_.att_estimator_->state().ang_vel(2);

  pub_log_.msg_.angular_acceleration.x =
      firmware_.att_estimator_->state().ang_vel_rates(0);
  pub_log_.msg_.angular_acceleration.y =
      firmware_.att_estimator_->state().ang_vel_rates(1);
  pub_log_.msg_.angular_acceleration.z =
      firmware_.att_estimator_->state().ang_vel_rates(2);

  pub_log_.msg_.linear_acceleration.x =
      firmware_.att_estimator_->state().linear_accel(0);
  pub_log_.msg_.linear_acceleration.y =
      firmware_.att_estimator_->state().linear_accel(1);
  pub_log_.msg_.linear_acceleration.z =
      firmware_.att_estimator_->state().linear_accel(2);

  pub_log_.msg_.cmd_euler.x =
      firmware_.att_controller_->cmd_attitude().euler(0);
  pub_log_.msg_.cmd_euler.y =
      firmware_.att_controller_->cmd_attitude().euler(1);
  pub_log_.msg_.cmd_euler.z =
      firmware_.att_controller_->cmd_attitude().euler(2);

  pub_log_.msg_.thrust = firmware_.att_controller_->input().thrust;
  pub_log_.msg_.moment.x = firmware_.att_controller_->input().moment(0);
  pub_log_.msg_.moment.y = firmware_.att_controller_->input().moment(1);
  pub_log_.msg_.moment.z = firmware_.att_controller_->input().moment(2);
  pub_log_.msg_.lyapunov = firmware_.att_controller_->lyap();

  pub_log_.msg_.motors_state = firmware_.state_machine_.state().armed;
  pub_log_.msg_.loop_rate = firmware_.get_current_freq();
  pub_log_.msg_.firmware_time = firmware_.get_firmware_time();
  pub_log_.msg_.attitude_mode = (int) firmware_.state_machine_.state().mode;
  pub_log_.msg_.voltage = firmware_.sensors_.voltage();
  pub_log_.msg_.current = firmware_.sensors_.current();

  for (int i = 0; i < 4; ++i) {
    pub_log_.msg_.esc_in_us[i] = firmware_.mixer_.pwm_us_array()[i];
  }
  pub_log_.publish();

  pub_imu_.msg_.header.stamp = ros::Time::now();
  pub_imu_.msg_.linear_acceleration.x = firmware_.sensors_.imu().accel(0);
  pub_imu_.msg_.linear_acceleration.y = firmware_.sensors_.imu().accel(1);
  pub_imu_.msg_.linear_acceleration.z = firmware_.sensors_.imu().accel(2);

  pub_imu_.msg_.angular_velocity.x = firmware_.sensors_.imu().gyro(0);
  pub_imu_.msg_.angular_velocity.y = firmware_.sensors_.imu().gyro(1);
  pub_imu_.msg_.angular_velocity.z = firmware_.sensors_.imu().gyro(2);
  pub_imu_.publish();

  if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD) {
    pub_odom_sp_.msg_.header.stamp = ros::Time::now();
    pub_odom_sp_.msg_.pose.pose.position.x =
        firmware_.mission_planner_->pose_des().position(0);
    pub_odom_sp_.msg_.pose.pose.position.y =
        firmware_.mission_planner_->pose_des().position(1);
    pub_odom_sp_.msg_.pose.pose.position.z =
        firmware_.mission_planner_->pose_des().position(2);
    pub_odom_sp_.msg_.twist.twist.linear.x = firmware_.mission_planner_->pose_des().velocity(0);
    pub_odom_sp_.msg_.twist.twist.linear.y = firmware_.mission_planner_->pose_des().velocity(1);
    pub_odom_sp_.msg_.twist.twist.linear.z = firmware_.mission_planner_->pose_des().velocity(2);
    pub_odom_sp_.publish();
  }

  //  pub_euler_.msg_.header.stamp = ros::Time::now();
  //  pub_euler_.msg_.quaternion.w =
  //  firmware_.test_att_estimator_->state().q()(0);
  //  pub_euler_.msg_.quaternion.x =
  //  firmware_.test_att_estimator_->state().q()(1);
  //  pub_euler_.msg_.quaternion.y =
  //  firmware_.test_att_estimator_->state().q()(2);
  //  pub_euler_.msg_.quaternion.z =
  //  firmware_.test_att_estimator_->state().q()(3); pub_euler_.publish();
  //
  //  pub_madgwick_.msg_.header.stamp = ros::Time::now();
  //  pub_madgwick_.msg_.quaternion.w =
  //  firmware_.test_att_estimator2_->state().q()(0);
  //  pub_madgwick_.msg_.quaternion.x =
  //  firmware_.test_att_estimator2_->state().q()(1);
  //  pub_madgwick_.msg_.quaternion.y =
  //  firmware_.test_att_estimator2_->state().q()(2);
  //  pub_madgwick_.msg_.quaternion.z =
  //  firmware_.test_att_estimator2_->state().q()(3); pub_madgwick_.publish();
}

} // namespace qrotor_firmware
