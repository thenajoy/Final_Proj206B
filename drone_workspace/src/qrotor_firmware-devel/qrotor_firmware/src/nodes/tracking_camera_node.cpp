#include "peripherals/rs_t265.h"
#include <ros/ros.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <firmware_publisher.hpp>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

class TrackingCameraCalib {
private:
  ros::NodeHandle nh_;
  std::thread t265_thread_;
  double px{0.}, py{0.06}, pz{0.1};
  double phi{0.}, theta{0.}, psi{0.};
  ddynamic_reconfigure::DDynamicReconfigure *ddynrec3;

public:
  explicit TrackingCameraCalib(ros::NodeHandle &_nh) : nh_(_nh) {
    init();
    run();
  }
  ~TrackingCameraCalib() = default;
  qrotor_firmware::RSt265Handler tcam_{};
  qrotor_firmware::PoseWithCovariance pose_{};

  void get_pose() {
    try {
      // Main loop
      while (true) {
        tcam_.run();
        tcam_.get_pose_info(pose_);
      }
    } catch (const rs2::error &e) {
      std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
                << e.what() << std::endl;
      printf("\033[31;1mt265 loop failed! \033[0m\n");
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      printf("\033[31;1mt265 loop failed! \033[0m\n");
    }
  }

  qrotor_firmware::FirmwarePublisher<nav_msgs::Odometry> pub_tcam_odom_{nh_, "/t265/odometry", 10};
  qrotor_firmware::FirmwarePublisher<nav_msgs::Odometry> pub_qrotor_odom_{nh_, "/qrotor/odometry/optical", 10};
  qrotor_firmware::FirmwarePublisher<nav_msgs::Odometry> pub_qrotorbf_odom_{nh_, "/qrotor/odometry/body", 10};

  void init() {

    // initialize camera and the parallel thread
    tcam_.init();
    t265_thread_ = std::thread(&TrackingCameraCalib::get_pose, this);


    // tunable parameters for tracking camera orientation
    ddynrec3 = new ddynamic_reconfigure::DDynamicReconfigure(ros::NodeHandle(nh_, "cam_pose"));
    ddynrec3->registerVariable<double>("px", px, [this](double new_value) {
      this->px = new_value;
      ROS_INFO("px updated to %f", this->px);
    }, "parameter description", -0.5, 0.5);
    ddynrec3->registerVariable<double>("py", py, [this](double new_value) {
      this->py = new_value;
      ROS_INFO("py updated to %f", this->py);
    }, "parameter description", -0.5, 0.5);
    ddynrec3->registerVariable<double>("pz", pz, [this](double new_value) {
      this->pz = new_value;
      ROS_INFO("pz updated to %f", this->pz);
    }, "parameter description", -0.5, 0.5);
    ddynrec3->registerVariable<double>("roll", phi, [this](double new_value) {
      this->phi = new_value;
      ROS_INFO("phi updated to %f", this->phi);
    }, "parameter description", -180, 180);
    ddynrec3->registerVariable<double>("pitch", theta, [this](double new_value) {
      this->theta = new_value;
      ROS_INFO("theta updated to %f", this->theta);
    }, "parameter description", -180, 180);
    ddynrec3->registerVariable<double>("yaw", psi, [this](double new_value) {
      this->psi = new_value;
      ROS_INFO("psi updated to %f", this->psi);
    }, "parameter description", -180, 180);
    ddynrec3->publishServicesTopics();
    //    ddr.RegisterVariable(&px, "px", -0.5, 0.5);
    //    ddr.RegisterVariable(&py, "py", -0.5, 0.5);
    //    ddr.RegisterVariable(&pz, "pz", -0.5, 0.5);
    //    ddr.RegisterVariable(&phi, "roll", -180, 180); // degrees
    //    ddr.RegisterVariable(&theta, "pitch", -180, 180);
    //    ddr.RegisterVariable(&psi, "yaw", -180, 180);
    //    ddr.publishServicesTopics();

  }

  void run() {

    ros::Rate loop_handle(100);
    while (ros::ok()) {
      ros::spinOnce();

      matrix::Vector3f p;
      p = pose_.position;
      matrix::Quaternionf q;
      q = pose_.quat;

      pub_tcam_odom_.msg_.header.stamp = ros::Time::now();
      pub_tcam_odom_.msg_.header.frame_id = "world_optical";
      pub_tcam_odom_.msg_.child_frame_id = "tracking_camera";
      pub_tcam_odom_.msg_.pose.pose.position.x = pose_.position(0);
      pub_tcam_odom_.msg_.pose.pose.position.y = pose_.position(1);
      pub_tcam_odom_.msg_.pose.pose.position.z = pose_.position(2);

      pub_tcam_odom_.msg_.pose.pose.orientation.w = pose_.quat(0);
      pub_tcam_odom_.msg_.pose.pose.orientation.x = pose_.quat(1);
      pub_tcam_odom_.msg_.pose.pose.orientation.y = pose_.quat(2);
      pub_tcam_odom_.msg_.pose.pose.orientation.z = pose_.quat(3);
      pub_tcam_odom_.publish();

      //////////////////////////////////////////////////////////////////////
      pub_qrotor_odom_.msg_.header.stamp = ros::Time::now();
      pub_qrotor_odom_.msg_.header.frame_id = "world_optical";
      pub_qrotor_odom_.msg_.child_frame_id = "qrotor_optical";
      //    p(0) = p(0);
      p(1) += float(py);
      p(2) += float(pz);
      pub_qrotor_odom_.msg_.pose.pose.position.x = p(0);
      pub_qrotor_odom_.msg_.pose.pose.position.y = p(1);
      pub_qrotor_odom_.msg_.pose.pose.position.z = p(2);

      matrix::Dcmf qrotor_rot;
      matrix::Eulerf euler((float) (phi * M_PI / 180.0),
                           (float) (theta * M_PI / 180.0),
                           (float) (psi * M_PI / 180.0));
      matrix::Quatf qt(euler);
      qrotor_rot = q.to_dcm() * (qt.to_dcm()).transpose();
      matrix::Quatf qfinal(qrotor_rot);

      pub_qrotor_odom_.msg_.pose.pose.orientation.w = qfinal(0);
      pub_qrotor_odom_.msg_.pose.pose.orientation.x = qfinal(1);
      pub_qrotor_odom_.msg_.pose.pose.orientation.y = qfinal(2);
      pub_qrotor_odom_.msg_.pose.pose.orientation.z = qfinal(3);
      pub_qrotor_odom_.publish();

      //////////////////////////////////////////////////////////////////////
      pub_qrotorbf_odom_.msg_.header.stamp = ros::Time::now();
      pub_qrotorbf_odom_.msg_.header.frame_id = "world_qrotor";
      pub_qrotorbf_odom_.msg_.child_frame_id = "qrotor_body";
      pub_qrotorbf_odom_.msg_.pose.pose.position.x = pose_.position(0);
      pub_qrotorbf_odom_.msg_.pose.pose.position.y = -pose_.position(2);
      pub_qrotorbf_odom_.msg_.pose.pose.position.z = pose_.position(1);

      pub_qrotorbf_odom_.msg_.pose.pose.orientation.w = qfinal(0);
      pub_qrotorbf_odom_.msg_.pose.pose.orientation.x = qfinal(1);
      pub_qrotorbf_odom_.msg_.pose.pose.orientation.y = -qfinal(3);
      pub_qrotorbf_odom_.msg_.pose.pose.orientation.z = qfinal(2);
      pub_qrotorbf_odom_.publish();

      // handle loop rate
      loop_handle.sleep();
    }
    ros::shutdown();
  }

};

int main(int argc, char **argv) {

  ROS_INFO("Initializing tracking_camera_node!");
  ros::init(argc, argv, "tracking_camera_node");
  ros::NodeHandle nh_;
  TrackingCameraCalib track_cam(nh_);

  return 0;
}

