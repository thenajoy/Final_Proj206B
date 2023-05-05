#ifndef QROTOR_GAZEBO_QROTOR_SIL_H
#define QROTOR_GAZEBO_QROTOR_SIL_H

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>

#include <qrotor_gazebo/common.h>
#include <qrotor_gazebo/multirotor_forces_moments.h>
#include <qrotor_gazebo/sil_board.h>
#include <qrotor_gazebo/tracking_camera.h>

#include "data_structures.h"
#include "firmware_publisher.hpp"
#include "invariant_ekf.h"
#include "constraint_iekf.h"
#include "payload_arcuo_pose.h"
#include "qrotor_flight.h"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <qrotor_firmware/Log.h>

#include <cstdio>
#include <sstream>

namespace pest = payload_estimation;

namespace qrotor_gazebo {
class QrotorFlightSIL : public gazebo::ModelPlugin {
 public:
  QrotorFlightSIL();
  ~QrotorFlightSIL() override;

 protected:
  void Reset() override;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);
  void init();
  void get_ros_params();

  // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> ddynrec_;
  void setupDDynamicReconfigure();
  matrix::Vector3f position_sp;

  /// \brief Callback that publishes a received Camera Frame as an
  /// ImageStamped message.
  virtual void OnNewFrame(const gazebo::rendering::CameraPtr cam,
                          const gazebo::transport::PublisherPtr pub);

 private:
  gazebo::common::Time plugin_loaded_time_, last_plugin_update_;
  struct CustomPluginTimer {
    CustomPluginTimer() : freq(200), dtsum(0) {}
    CustomPluginTimer(double _f, double _dtsum) : freq(_f), dtsum(_dtsum) {}
    double freq;
    double dtsum;
  };
  struct Timers {
    CustomPluginTimer firmware = CustomPluginTimer(500, 0);
    CustomPluginTimer pose_iekf = CustomPluginTimer(200, 0);
  };
  Timers timer_;

  void windCallback(const geometry_msgs::Vector3 &msg);

  SILBoard board_;
  qrotor_firmware::FlightController firmware_;
  qrotor_firmware::PoseWithCovariance pose_;

  std::string mav_type_;
  std::string namespace_;
  std::string link_name_;

  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  gazebo::physics::EntityPtr parent_link_;
  // Pointer to the update event connection.
  gazebo::event::ConnectionPtr updateConnection_;

  qrotor_gazebo::Pose3d state_;

  /// \brief to publish the truth odometry of the quadrotor
  ros::Publisher pub_odom_truth_;
  /// \brief to publish the estimated (using invariant EKF) odometry of the quadrotor
  ros::Publisher pub_odom_est_;
  /// \brief publishes the qrotor_firmware Log (TODO update the logging parameters)
  ros::Publisher pub_log_;
  /// \brief publishes the imu readings
  ros::Publisher pub_imu_;
  /// \brief publishes the measured point-of-contact of the cable
  ros::Publisher pub_pointOfContact_, pub_pointOfContact_truth_;
  /// \brief Publish payload odometry
  ros::Publisher pub_payload_odom_;

  /// \brief publish true pose
  void publishPose();

  /// \brief publish current IMU readings
  void publishImu();

  /// \brief Pointer to quadrotor dynamics class
  /// to allocate pwms to the motors and apply
  /// first order motor dynamics (currently, not in use)
//  MultirotorForcesMoments *mav_dynamics_;

  /// \brief variable to store the current pwm value to be s
  /// set to the motors (currently, not in use)
  int pwm_outputs_[14];

  /// \brief apply input wrench to the link_
  void applyInputWrench();

  /// \brief tf broadcaster to publish current true pose of the link_
  tf2_ros::TransformBroadcaster br;

  /// \brief Pointer to fake tracking camera (to simulate the pose measurement)
  FakeTrackingCamera *trackingCamera_;
  /// \brief Pointer to the Color Camera Renderer.
  gazebo::rendering::CameraPtr colorCamera_;
  /// \brief Pointer to the Color Camera callback connection.
  gazebo::event::ConnectionPtr newColorFrameConn;
  /// \brief Pointer to the Color Publisher.
  gazebo::transport::PublisherPtr colorPub;
  int newImageCounter = 0;

  /// \brief Camera pose in Quadrotor Frame
  Eigen::Matrix3d RCam2Quad_, ROptical2Cam_;
  Eigen::Vector3d tCamInQuad_;

  /// \brief Pointer to ArcuoPoseEstimator
  pest::PayloadArcuoPose *arcuo_estimator_;
  /// \brief Quadrotor & Load poses in camera-frame
  pest::utils::EigenPosed quad_pose_CF_, load_pose_CF_;
  /// \brief Point-of-contact of the cable to the payload in payload-frame
  Eigen::Vector3d cablePOC_PF_ = Eigen::Vector3d::Zero();

  /// \brief  Pointer to the pose estimator (Invariant EKF)
  pest::ConstraintIEKF<double> *pose_iekf_;
  /// \brief IMU measurement reading
  pest::ImuMeasurement<double> imu_reading_;
  /// \brief variable to store last imu update
  unsigned long last_imu_update_us;
  /// \brief time-difference between last imu update and current update
  double dt = 0;
  /// \brief Run pose estimation (invariant ekf)
  void runPoseEstimation();

  /// \brief ConstraintIEKF constraints std-deviation
  double h1_sigma= 0.01, h2_sigma=0.01;

  /// \brief Publish estimated pose
  void publishEstimatedOdom();

  bool isPoseInEKFInitialized = false;
  bool isPoseInEKFEnabled = false;
  double pose_iekf_wait_time_ = 15; // seconds
  int estimation_loop_iter_ = 0;
  int max_estimation_iters_ = 50;

  // container for forces
  Eigen::Matrix<double, 6, 1> forces_, applied_forces_;
  Eigen::Matrix<double, 3, 1> thrust_vector_, moment_vector_;
  Eigen::Matrix<double, 3, 1> e3_{0., 0., 1.};

  // Time Counters
  uint64_t start_time_us_;
  ros::NodeHandle *nh_;

  // For reset handling
  GazeboPose initial_pose_;
};

} // namespace qrotor_gazebo
#endif // QROTOR_GAZEBO_QROTOR_SIL_H
