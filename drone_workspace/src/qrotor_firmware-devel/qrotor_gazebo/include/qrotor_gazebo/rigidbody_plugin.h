#ifndef QROTOR_GAZEBO_RIGID_BODY_PLUGIN_H
#define QROTOR_GAZEBO_RIGID_BODY_PLUGIN_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <qrotor_gazebo/gz_compat.h>
#include <qrotor_gazebo/multirotor_forces_moments.h>
#include <qrotor_gazebo/sil_board.h>

namespace qrotor_gazebo {
class RigidBodyPlugin : public gazebo::ModelPlugin {
public:
  RigidBodyPlugin();
  ~RigidBodyPlugin();

protected:
  void Reset() override;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);

private:
  void windCallback(const geometry_msgs::Vector3 &msg);
  void publishPose();

  std::string namespace_;
  std::string link_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  // Pointer to the update event connection.
  gazebo::event::ConnectionPtr updateConnection_;
  // Pointer to camera (sensor)
  gazebo::sensors::SensorPtr camera_ptr_;

  ros::Subscriber wind_sub_;
  ros::Publisher truth_NED_pub_, truth_NWU_pub_;

  ros::Publisher pub_log_;

  // tf
  tf2_ros::TransformBroadcaster br;

  // Time Counters
  uint64_t start_time_us_;
  ros::NodeHandle *nh_;

  // For reset handlin
  GazeboPose initial_pose_;

  // helper functions for converting to and from eigen
  Eigen::Vector3d vec3_to_eigen_from_gazebo(GazeboVector vec);
  GazeboVector vec3_to_gazebo_from_eigen(Eigen::Vector3d vec);
  Eigen::Matrix3d rotation_to_eigen_from_gazebo(GazeboQuaternion vec);
};

} // namespace qrotor_gazebo

#endif // QROTOR_GAZEBO_RIGID_BODY_PLUGIN_H
