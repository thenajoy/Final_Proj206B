//
// Created by kotaru on 3/30/21.
//
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

// qrotor_firmware
#include "estimation/eigen/inekf.h"
#include "estimation/eigen/invariant_ekf.h"
#include "estimation/matlab/invariant_ekf_m.h"
#include "filters/low_pass_filter.hpp"
#include "filters/notch_filter.hpp"
#include "firmware_publisher.hpp"
#include "lie_algebra.h"

// ros_msgs
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

namespace qrotor_firmware {
namespace ee = eigen_estimation;
class PoseEstimationHandler {
private:
  // ros
  ros::NodeHandle nh_;

  // time
  double start_s = 0;
  double now_s = 0., last_update_s = 0., dt_s = 0.;
  double imu_now_s = 0, imu_last_update_s = 0;
  double imu_cb_s = 0, imu_last_cb_s = 0;
  double dt_imu_update = 0;
  double dt_meas_t265 = 0;
  double dt_meas_mocap = 0;

  // estimation
  eigen_estimation::InvariantEKFd iekf_{};
  eigen_estimation::ImuMeasurementd imu_readings_{};
  eigen_estimation::RigidbodyStated rb_tcam_m{}, rb_mocap_m{};
  Eigen::Matrix3d rotation_offset = Eigen::Matrix3d::Identity();
  Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();

  ros::Subscriber _sub_mocap, _sub_t265, _sub_imu;

  bool new_mocap_meas = false, new_t265_meas = false;
  bool iekf_initialized = false;

  matrix::Vector3f a{0., 0., 9.8}, w{0., 0., 0.};

  int COUNT = 0;

  FirmwarePublisher<nav_msgs::Odometry> pub_odom_{nh_, "odometry/estimate", 1};
  FirmwarePublisher<sensor_msgs::Imu> pub_imu_{nh_, "imu_filtered", 1};
  FirmwarePublisher<std_msgs::Float32> pub_loop_rate_{nh_, "loop_rate_", 1};

  LowPassFilter accel_lpf_{500.0, 100.0};
  LowPassFilter gyro_lpf_{500.0, 100.0};

  void cb_sub_mocap(const nav_msgs::Odometry::ConstPtr &msg) {
    new_mocap_meas = true;
    Eigen::Vector3d p, v;
    Eigen::Quaterniond q;

    tf::pointMsgToEigen(msg->pose.pose.position, p);
    tf::vectorMsgToEigen(msg->twist.twist.linear, v);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, q);
    rb_mocap_m.update(q.matrix(), v, p);
  }

  void cb_sub_t265(const nav_msgs::Odometry::ConstPtr &msg) {
    new_t265_meas = true;
    Eigen::Vector3d p, v;
    Eigen::Quaterniond q;

    tf::pointMsgToEigen(msg->pose.pose.position, p);
    tf::vectorMsgToEigen(msg->twist.twist.linear, v);
    tf::quaternionMsgToEigen(msg->pose.pose.orientation, q);
    rb_tcam_m.update(q.matrix(), v, p);
  }

  void cb_sub_imu(const sensor_msgs::Imu::ConstPtr &msg) {
    if (imu_last_update_s <= 0) {
      imu_last_update_s = msg->header.stamp.toSec();
      return;
    }
    imu_now_s = msg->header.stamp.toSec();
    a = accel_lpf_.apply({static_cast<float>(msg->linear_acceleration.x),
                          static_cast<float>(msg->linear_acceleration.y),
                          static_cast<float>(msg->linear_acceleration.z)});
    w = gyro_lpf_.apply({static_cast<float>(msg->angular_velocity.x),
                         static_cast<float>(msg->angular_velocity.y),
                         static_cast<float>(msg->angular_velocity.z)});

    //    if (iekf_initialized) {
    auto dt = float(imu_now_s - imu_last_update_s);

    imu_readings_.update(imu_now_s, a(0), a(1), a(2), w(0), w(1), w(2));
    double start = ros::Time::now().toSec();
    iekf_.timeUpdate(dt, imu_readings_);
    double end = ros::Time::now().toSec();
    dt_imu_update = end - start;
    //    ROS_INFO("")
    imu_last_update_s = imu_now_s;
    publish_imu();
    //    }
  }

public:
  explicit PoseEstimationHandler(ros::NodeHandle &_nh) : nh_(_nh) {
    start_s = ros::Time::now().toSec();
    last_update_s = ros::Time::now().toSec();
    now_s = ros::Time::now().toSec();

    _sub_mocap = nh_.subscribe("odometry/mocap", 3,
                               &PoseEstimationHandler::cb_sub_mocap, this);
    _sub_t265 = nh_.subscribe("odometry/t265", 3,
                              &PoseEstimationHandler::cb_sub_t265, this);
    _sub_imu =
        nh_.subscribe("imu", 20, &PoseEstimationHandler::cb_sub_imu, this);

    init();
    run();
  }

  ~PoseEstimationHandler() = default;

  void init() { clock(); }

  void clock() {
    now_s = ros::Time::now().toSec();
    dt_s = (now_s - last_update_s);
    last_update_s = now_s;
    //    ROS_WARN(" PoseEstimatorHandler freq: %f", 1 / dt_s);
  }

  void run() {

    ros::Rate loop_handle(100);

    while (ros::ok()) {
      // spin messages
      ros::spinOnce();

      // compute loop-time
      clock();

      // ekf-initialization
      if (!iekf_initialized) {
        if (new_t265_meas && new_mocap_meas) {

          std::cout << "---------------------- mocap" << std::endl;
          rb_mocap_m.print();
          std::cout << "---------------------- tracking-cam" << std::endl;
          rb_tcam_m.print();
          std::cout << "---------------------- " << std::endl;

          iekf_.init(rb_tcam_m);

          iekf_initialized = true;
          ROS_WARN("Invariant EKF initialized!");
        }
      }

      if (iekf_initialized) {
        // tracking camera measurement update
        if (new_t265_meas) {
          double start = ros::Time::now().toSec();
          iekf_.measUpdate(rb_tcam_m);
          double end = ros::Time::now().toSec();
          dt_meas_t265 = end - start;
          new_t265_meas = false;
        }

        // mocap measurement update
//        if (new_mocap_meas) {
//          double start = ros::Time::now().toSec();
//          iekf_.measUpdate(rb_mocap_m);
//          double end = ros::Time::now().toSec();
//          dt_meas_mocap = end - start;
//          //          ROS_WARN("MeasUpdate: dt_s: %f", dt_e);
//          new_t265_meas = false;
//        }
      }

      ROS_INFO("FREQ: %.2f, \t imu: %f, \t dt_t265: %f, \tdt_mocap: %f",
               (1 / dt_s),
               dt_imu_update,
               dt_meas_t265,
               dt_meas_mocap);
      pub_loop_rate_.msg_.data = 1 / dt_s;
      pub_loop_rate_.publish();
      publish_pose();
      loop_handle.sleep();
    }
  }

  void publish_pose() {
    pub_odom_.msg_.header.stamp = ros::Time::now();
    tf::pointEigenToMsg(iekf_.state().position(),
                        pub_odom_.msg_.pose.pose.position);
    tf::vectorEigenToMsg(iekf_.state().velocity(),
                         pub_odom_.msg_.twist.twist.linear);
    Eigen::Quaterniond q(iekf_.state().rotation());
    tf::quaternionEigenToMsg(q, pub_odom_.msg_.pose.pose.orientation);
    pub_odom_.publish();
  }

  void publish_imu() {
    pub_imu_.msg_.header.stamp = ros::Time::now();
    pub_imu_.msg_.angular_velocity.x = w(0);
    pub_imu_.msg_.angular_velocity.y = w(1);
    pub_imu_.msg_.angular_velocity.z = w(2);
    pub_imu_.msg_.linear_acceleration.x = a(0);
    pub_imu_.msg_.linear_acceleration.y = a(1);
    pub_imu_.msg_.linear_acceleration.z = a(2);
    pub_imu_.publish();
  }
};
} // namespace qrotor_firmware
int main(int argc, char **argv) {
  ROS_INFO("Initializing firmware_node!");
  ros::init(argc, argv, "pose_estimation_node");
  ros::NodeHandle nh_;

  qrotor_firmware::PoseEstimationHandler pose_handler_(nh_);
}
