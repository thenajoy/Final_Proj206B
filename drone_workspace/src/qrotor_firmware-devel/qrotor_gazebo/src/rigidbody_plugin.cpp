#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <qrotor_gazebo/rigidbody_plugin.h>

#include <cstdint>
#include <cstdio>
#include <eigen3/Eigen/Core>
#include <sstream>
// #include <qrotor_gazebo/sil_board.h>

namespace qrotor_gazebo {
RigidBodyPlugin::RigidBodyPlugin() : gazebo::ModelPlugin(), nh_(nullptr) {}

RigidBodyPlugin::~RigidBodyPlugin() {
  GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void RigidBodyPlugin::Load(gazebo::physics::ModelPtr _model,
                           sdf::ElementPtr _sdf) {
  if (!ros::isInitialized()) {
    ROS_FATAL("A ROS node fsetupDDynamicReconfigureor Gazebo has not been "
              "initialized, unable to load "
              "plugin");
    return;
  }
  ROS_INFO("RigidBodyPlugin plugin loading... ");

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various
   elements
   * in the simulation
   */

  ROS_WARN("Searching for namespace... ");
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[RigidBodyPlugin] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);
  ROS_WARN("namespace found! %s", namespace_.c_str());
  gzmsg << "loading parameters from " << namespace_ << " ns\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[RigidBodyPlugin] Please specify a linkName of the forces and"
             "moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[RigidBodyPlugin] Couldn't find specified link \"" << link_name_
                                                                << "\".");

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RigidBodyPlugin::OnUpdate, this, _1));

  initial_pose_ = GZ_COMPAT_GET_WORLD_COG_POSE(link_);

  truth_NED_pub_ = nh_->advertise<nav_msgs::Odometry>("ground_truth/NED", 1);
  truth_NWU_pub_ = nh_->advertise<nav_msgs::Odometry>("ground_truth/NWU", 1);
  ROS_INFO("RigidBodyPlugin plugin loading... DONE!");
}

// This gets called by the world update event.
void RigidBodyPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  MultirotorForcesMoments::CurrentState state;
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // state in NWU frame
  state.pos = vec3_to_eigen_from_gazebo(GZ_COMPAT_GET_POS(pose));
  state.rot = rotation_to_eigen_from_gazebo(GZ_COMPAT_GET_ROT(pose));
  state.vel = vec3_to_eigen_from_gazebo(vel);
  state.omega = vec3_to_eigen_from_gazebo(omega);
  state.t = _info.simTime.Double();

  publishPose();
}

void RigidBodyPlugin::Reset() {
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
}

void RigidBodyPlugin::windCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d wind;
  //  wind << msg.x, msg.y, msg.z;
  //  mav_dynamics_->set_wind(wind);
}

void RigidBodyPlugin::publishPose() {
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
  truth_NWU_pub_.publish(odometry_);

  // sending transform
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header = odometry_.header;
  transformStamped.child_frame_id = odometry_.child_frame_id;
  transformStamped.transform.translation.x = odometry_.pose.pose.position.x;
  transformStamped.transform.translation.y = odometry_.pose.pose.position.y;
  transformStamped.transform.translation.z = odometry_.pose.pose.position.z;
  transformStamped.transform.rotation = odometry_.pose.pose.orientation;
  br.sendTransform(transformStamped);

  // Convert to NED
  odometry_.header.frame_id = link_name_ + "_NED";
  odometry_.pose.pose.orientation.y *= -1.0;
  odometry_.pose.pose.orientation.z *= -1.0;
  odometry_.pose.pose.position.y *= -1.0;
  odometry_.pose.pose.position.z *= -1.0;
  odometry_.twist.twist.linear.y *= -1.0;
  odometry_.twist.twist.linear.z *= -1.0;
  odometry_.twist.twist.angular.y *= -1.0;
  odometry_.twist.twist.angular.z *= -1.0;
  truth_NED_pub_.publish(odometry_);
}

Eigen::Vector3d RigidBodyPlugin::vec3_to_eigen_from_gazebo(GazeboVector vec) {
  Eigen::Vector3d out;
  out << GZ_COMPAT_GET_X(vec), GZ_COMPAT_GET_Y(vec), GZ_COMPAT_GET_Z(vec);
  return out;
}

GazeboVector RigidBodyPlugin::vec3_to_gazebo_from_eigen(Eigen::Vector3d vec) {
  GazeboVector out(vec(0), vec(1), vec(2));
  return out;
}

Eigen::Matrix3d
RigidBodyPlugin::rotation_to_eigen_from_gazebo(GazeboQuaternion quat) {
  Eigen::Quaterniond eig_quat(GZ_COMPAT_GET_W(quat), GZ_COMPAT_GET_X(quat),
                              GZ_COMPAT_GET_Y(quat), GZ_COMPAT_GET_Z(quat));
  return eig_quat.toRotationMatrix();
}

GZ_REGISTER_MODEL_PLUGIN(RigidBodyPlugin);
} // namespace qrotor_gazebo
