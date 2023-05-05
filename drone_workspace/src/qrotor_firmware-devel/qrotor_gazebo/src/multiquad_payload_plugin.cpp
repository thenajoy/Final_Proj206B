#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <qrotor_gazebo/multiquad_payload_plugin.h>

#include <cstdint>
#include <cstdio>
#include <eigen3/Eigen/Core>
#include <sstream>
// #include <qrotor_gazebo/sil_board.h>

namespace qrotor_gazebo {
MultiQrotorPayloadPlugin::MultiQrotorPayloadPlugin()
    : gazebo::ModelPlugin(), nh_(nullptr) {}

MultiQrotorPayloadPlugin::~MultiQrotorPayloadPlugin() {
  GZ_COMPAT_DISCONNECT_WORLD_UPDATE_BEGIN(updateConnection_);
  if (nh_) {
    nh_->shutdown();
    delete nh_;
  }
}

void MultiQrotorPayloadPlugin::Load(gazebo::physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf) {
  if (!ros::isInitialized()) {
    ROS_FATAL("A ROS node fsetupDDynamicReconfigureor Gazebo has not been "
              "initialized, unable to load "
              "plugin");
    return;
  }
  ROS_INFO("MultiQrotorPayloadPlugin plugin loading... ");

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various
   * elements in the simulation
   */

  ROS_WARN("Searching for namespace... ");
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[MultiQrotorPayloadPlugin] Please specify a namespace.\n";
  nh_ = new ros::NodeHandle(namespace_);
  ROS_WARN("namespace found! %s", namespace_.c_str());
  gzmsg << "loading parameters from " << namespace_ << " ns\n";

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[MultiQrotorPayloadPlugin] Please specify a linkName of the "
             "forces and"
             "moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) {
    gzthrow("[MultiQrotorPayloadPlugin] Couldn't find specified link \""
            << link_name_ << "\".");
  } else {
    link_names_.push_back(link_name_);
    links_.push_back(link_);
  }

  // other links
  if (_sdf->HasElement("otherLinks")) {
    auto other_links_ = _sdf->GetElement("otherLinks")->Get<std::string>();
    gzmsg << other_links_ << std::endl;
    std::istringstream iss(other_links_);
    for (std::string s; iss >> s;) {
      gazebo::physics::LinkPtr l;
      l = model_->GetLink(s);
      if (l == NULL) {
        gzwarn << "[MultiQrotorPayloadPlugin] Couldn't find "
                  "specified link "
               << s << "\n";
      } else {
        gzmsg << "\[MultiQrotorPayloadPlugin] found the pecified link " << s
              << "\n";
        link_names_.push_back(s);
        links_.push_back(l);
      }
    }
  }

  // Connect the update function to the simulation
  updateConnection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MultiQrotorPayloadPlugin::OnUpdate, this, _1));

  initial_pose_ = GZ_COMPAT_GET_WORLD_COG_POSE(link_);

  truth_NED_pub_ = nh_->advertise<geometry_msgs::PoseArray>("truth/NED", 1);
  truth_NWU_pub_ = nh_->advertise<geometry_msgs::PoseArray>("truth/NWU", 1);
  ROS_INFO("MultiQrotorPayloadPlugin plugin loading... DONE!");
}

// This gets called by the world update event.
void MultiQrotorPayloadPlugin::OnUpdate(
    const gazebo::common::UpdateInfo &_info) {
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

  //  publish_pose();
}

void MultiQrotorPayloadPlugin::Reset() {
  link_->SetWorldPose(initial_pose_);
  link_->ResetPhysicsStates();
}

void MultiQrotorPayloadPlugin::windCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d wind;
  //  wind << msg.x, msg.y, msg.z;
  //  mav_dynamics_->set_wind(wind);
}

void MultiQrotorPayloadPlugin::publishPose() {
  GazeboPose pose = GZ_COMPAT_GET_WORLD_COG_POSE(link_);
  GazeboVector vel = GZ_COMPAT_GET_RELATIVE_LINEAR_VEL(link_);
  GazeboVector omega = GZ_COMPAT_GET_RELATIVE_ANGULAR_VEL(link_);

  // Publish truth
  nav_msgs::Odometry truth;
  truth.header.stamp.sec = GZ_COMPAT_GET_SIM_TIME(world_).sec;
  truth.header.stamp.nsec = GZ_COMPAT_GET_SIM_TIME(world_).nsec;
  truth.header.frame_id = link_name_ + "_NWU";
  truth.pose.pose.orientation.w = GZ_COMPAT_GET_W(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.orientation.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.orientation.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.orientation.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_ROT(pose));
  truth.pose.pose.position.x = GZ_COMPAT_GET_X(GZ_COMPAT_GET_POS(pose));
  truth.pose.pose.position.y = GZ_COMPAT_GET_Y(GZ_COMPAT_GET_POS(pose));
  truth.pose.pose.position.z = GZ_COMPAT_GET_Z(GZ_COMPAT_GET_POS(pose));
  truth.twist.twist.linear.x = GZ_COMPAT_GET_X(vel);
  truth.twist.twist.linear.y = GZ_COMPAT_GET_Y(vel);
  truth.twist.twist.linear.z = GZ_COMPAT_GET_Z(vel);
  truth.twist.twist.angular.x = GZ_COMPAT_GET_X(omega);
  truth.twist.twist.angular.y = GZ_COMPAT_GET_Y(omega);
  truth.twist.twist.angular.z = GZ_COMPAT_GET_Z(omega);
  //  truth_NWU_pub_.publish(truth);

  // Convert to NED
  truth.header.frame_id = link_name_ + "_NED";
  truth.pose.pose.orientation.y *= -1.0;
  truth.pose.pose.orientation.z *= -1.0;
  truth.pose.pose.position.y *= -1.0;
  truth.pose.pose.position.z *= -1.0;
  truth.twist.twist.linear.y *= -1.0;
  truth.twist.twist.linear.z *= -1.0;
  truth.twist.twist.angular.y *= -1.0;
  truth.twist.twist.angular.z *= -1.0;
  //  truth_NED_pub_.publish(truth);
}

Eigen::Vector3d
MultiQrotorPayloadPlugin::vec3_to_eigen_from_gazebo(GazeboVector vec) {
  Eigen::Vector3d out;
  out << GZ_COMPAT_GET_X(vec), GZ_COMPAT_GET_Y(vec), GZ_COMPAT_GET_Z(vec);
  return out;
}

GazeboVector
MultiQrotorPayloadPlugin::vec3_to_gazebo_from_eigen(Eigen::Vector3d vec) {
  GazeboVector out(vec(0), vec(1), vec(2));
  return out;
}

Eigen::Matrix3d
MultiQrotorPayloadPlugin::rotation_to_eigen_from_gazebo(GazeboQuaternion quat) {
  Eigen::Quaterniond eig_quat(GZ_COMPAT_GET_W(quat), GZ_COMPAT_GET_X(quat),
                              GZ_COMPAT_GET_Y(quat), GZ_COMPAT_GET_Z(quat));
  return eig_quat.toRotationMatrix();
}

GZ_REGISTER_MODEL_PLUGIN(MultiQrotorPayloadPlugin);
} // namespace qrotor_gazebo
