/*
 * @file tracking_camera.h
 * @author vkotaru
 * @date 12/17/20
 */
#ifndef QROTOR_GAZEBO_TRACKING_CAMERA_H_
#define QROTOR_GAZEBO_TRACKING_CAMERA_H_

#include <qrotor_gazebo/common.h>
//#include "rigidbody_state.hpp"

//namespace pest = payload_estimation;
namespace qrotor_gazebo {

class FakeTrackingCamera {
 private:
  /// ros
  ros::NodeHandle nh_;

  /// namespace
  std::string ns_;

  /// link pointer to the tracking camera
  gazebo::physics::LinkPtr link_;

  /// random number generators
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;
  std::uniform_real_distribution<double> uniform_distribution_;

  /// position noise standard deviation
  double position_std_;
  /// position noise bias range (random walk)
  double position_bias_;
  /// position noise standard deviation
  double position_bias_std_;
  /// velocity standard deviation
  double velocity_std_;
  /// velocity bias (random walk) range
  double velocity_bias_;
  /// velocity bias standard deviation
  double velocity_bias_std_;
  /// rotation standard deviation
  double rotation_std_;
  /// rotation bias (random walk) range
  double rotation_bias_;
  /// rotation bias standard deviation
  double rotation_bias_std_;

 public:
  FakeTrackingCamera(gazebo::physics::LinkPtr _link, ros::NodeHandle &nh, std::string ns);
  ~FakeTrackingCamera();

  /**
   * simulate a tracking camera pose estimation\name
   * gets the pose of the link and adds measurement noise
   * TODO add measurement delay
   * @return pest::RigidbodyState<double>
   */
//  payload_estimation::RigidbodyState<double> simulate();

  // get noise params
  const double position_stddev() const { return position_std_; }
  const double velocity_stddev() const { return velocity_std_; }
  const double rotation_stddev() const { return rotation_std_; }
};

} // namespace qrotor_gazebo
#endif //QROTOR_GAZEBO_TRACKING_CAMERA_H_
