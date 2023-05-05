#ifndef __QROTOR_GAZBOE_QUADROTOR_SIL_H__
#define __QROTOR_GAZBOE_QUADROTOR_SIL_H__
#include <boost/thread/thread.hpp>

#include <qrotor_gazebo/common.h>
#include <qrotor_gazebo/multirotor_forces_moments.h>
#include <qrotor_gazebo/sil_board.h>
#include <qrotor_gazebo/tracking_camera.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>

#include "data_structures.h"
#include "firmware_publisher.hpp"
#include "nodes/firmware_io.h"
#include "nodes/firmware_params_reconfig.h"
//#include "payload_arcuo_pose.h"
#include "qrotor_flight.h"
#include <cstdio>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <qrotor_firmware/Log.h>
#include <sstream>

namespace qrotor_gazebo {
class QuadrotorSIL : public gazebo::ModelPlugin {
public:
  QuadrotorSIL();
  ~QuadrotorSIL() override;

protected:
  void Reset() override;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &_info);
  void Init() override;
  //  void get_ros_params();

  // ddynamic reconfigure
  std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>>
      ddynrec_;
  void setupDDynamicReconfigure();
  matrix::Vector3f position_sp;

private:
  gazebo::common::Time plugin_loaded_time_, last_plugin_update_;
  double ros_start_time_s = 0.0;
  double ros_last_update_s = 0.0;
  double ros_now_s = 0.0;
  float ros_dt = 0.0;
  double dtsum = 0;

  /// run firmware_node in parallel
  // void FirmwareThread();
  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::NodeHandlePtr node_ptr_;
  /// \brief A ROS subscriber
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  /// \brief thread to run low-frequency/high-level stuff
  boost::thread hl_thread;
  void windCallback(const geometry_msgs::Vector3 &msg);

  SILBoard board_;
  qrotor_firmware::FlightController* firmware_;
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

  /// \brief save last_time
  gazebo::common::Time last_time_;
  gazebo::common::Time last_pub_time_;
  /// \brief update rate control
  double update_rate_;
  /// \brief publish rate
  double pub_rate_;

  /// \brief to publish the truth odometry of the quadrotor
  ros::Publisher pub_odom_truth_;
  /// \brief to publish the estimated odometry of the quadrotor
  ros::Publisher pub_odom_est_;
  /// \brief to publish the setpoint odometry of the quadrotor
  ros::Publisher pub_odom_sp_;
  /// \brief publishes the qrotor_firmware Log (TODO update the logging
  /// parameters)
  ros::Publisher pub_log_;
  /// \brief publishes the imu readings
  ros::Publisher pub_imu_;

  /// \brief publish true pose
  void publish_pose();

  /// \brief publish current IMU readings
  void publish_imu();

  /// \brief Pointer to quadrotor dynamics class
  /// to allocate pwms to the motors and apply
  /// first order motor dynamics (currently, not in use)
  MultirotorForcesMoments *mav_dynamics_ = nullptr;

  bool use_rotor_dynamics = false;
  bool use_ground_effect_drag_force = false;

  /// \brief variable to store the current pwm value to be s
  /// set to the motors (currently, not in use)
  int pwm_outputs_[4] = {1000, 1000, 1000, 1000};

  /// \brief apply input wrench to the link_
  void applyInputWrench();

  /// \brief tf broadcaster to publish current true pose of the link_
  tf2_ros::TransformBroadcaster br;

  // container for forces
  Eigen::Matrix<double, 6, 1> forces_, applied_forces_;
  Eigen::Vector4d output_;
  Eigen::Matrix<double, 3, 1> thrust_vector_, moment_vector_;
  Eigen::Matrix<double, 3, 1> e3_{0., 0., 1.};
  Eigen::Matrix3d J;
  double mass_ = 0.75;

  // Time Counters
  uint64_t start_time_us_{};
  ros::NodeHandle *nh_;

  double ros_start_s = ros::Time::now().toSec();
  /// For reset handling
  GazeboPose initial_pose_;

  /// Qrotor Firmware parameters
  qrotor_firmware::ParamsReconfig *params_{};
  qrotor_firmware::FirmwareIO *firmware_io_{};

  /// Firmware publish
  void firmware_publish();

  /// Vee
  static Eigen::Vector3d vee3d(Eigen::Matrix3d M) {
    return (Eigen::Vector3d() << -M(1, 2), M(0, 2), -M(0, 1)).finished();
  }
  /// Hatmap
  Eigen::Matrix3d hat3d(Eigen::Vector3d v) {
    Eigen::Matrix3d M;
    M << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
    return M;
  }
};

} // namespace qrotor_gazebo
#endif // __QROTOR_GAZBOE_QUADROTOR_SIL_H__
