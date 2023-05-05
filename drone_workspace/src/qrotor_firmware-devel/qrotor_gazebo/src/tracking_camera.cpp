#include "qrotor_gazebo/tracking_camera.h"
namespace qrotor_gazebo {

FakeTrackingCamera::FakeTrackingCamera(gazebo::physics::LinkPtr _link, ros::NodeHandle &nh,
                               std::string ns)
    : link_(_link), nh_(nh), ns_(ns) {

  // get measurement parameters
  position_std_       = nh.param<double>("measurement/position_stddev", 0.001);
  position_bias_      = nh.param<double>("position_bias", 0.01);
  position_bias_std_  = nh.param<double>("position_bias_stddev", 0.001);
  velocity_std_       = nh.param<double>("measurement/velocity_stddev", 0.05);
  velocity_bias_      = nh.param<double>("velocity_bias", 0.01);
  velocity_bias_std_  = nh.param<double>("velocity_bias_stddev", 0.001);
  rotation_std_       = nh.param<double>("measurement/rotation_stddev", 0.01);
  rotation_bias_      = nh.param<double>("rotation_bias", 0.01);
  rotation_bias_std_  = nh.param<double>("rotation_bias_stddev", 0.001);

  ROS_INFO("Measurement noise params (std-dev): position %f, velocity %f, rotation %f", position_std_, velocity_std_, rotation_std_);

}

FakeTrackingCamera::~FakeTrackingCamera() {}

payload_estimation::RigidbodyState<double> FakeTrackingCamera::simulate() {

  // get the pose
  qrotor_gazebo::Pose3d pose_(link_);

  // add measurement noise
  // position
  pose_.pos += Eigen::Vector3d(position_std_ * normal_distribution_(random_generator_),
                               position_std_ * normal_distribution_(random_generator_),
                               position_std_ * normal_distribution_(random_generator_));

  // velocity
  pose_.vel += Eigen::Vector3d(velocity_std_ * normal_distribution_(random_generator_),
                               velocity_std_ * normal_distribution_(random_generator_),
                               velocity_std_ * normal_distribution_(random_generator_));

  // rotation
  Eigen::Vector3d w(rotation_std_ * normal_distribution_(random_generator_),
                    rotation_std_ * normal_distribution_(random_generator_),
                    rotation_std_ * normal_distribution_(random_generator_));
  pose_.rot = payload_estimation::utils::rodriguesRotation<double>(w)*pose_.rot;

  // TODO is it better to just output reference a state?
  payload_estimation::RigidbodyState<double> state(pose_.rot, pose_.vel, pose_.pos);
  state.set_ang_vel(pose_.omega);
  return state;
}
} // qrotor_gazebo