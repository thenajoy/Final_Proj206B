#ifndef __QROTOR_FIRMWARE_EXTERNAL_POSE_H__
#define __QROTOR_FIRMWARE_EXTERNAL_POSE_H__
// Created by kotaru on 2/27/21.

#include "data_structures.h"
#include "utils.h"

namespace qrotor_firmware {
class ExternalPoseHandler {
protected:
  PoseWithCovariance pose_;
  PoseWithCovariance rst265_pose_, mocap_pose_;
  bool new_pose_obtained_ = false;
  bool new_mocap_pose_obtained_ = false;

  matrix::Vector3f cam_t;
  matrix::Eulerf cam_eul;
  matrix::Quaternionf cam_quat;
public:
  ExternalPoseHandler()  = default;
  ~ExternalPoseHandler() = default;

  bool new_pose_obtained() const {
    return new_pose_obtained_;
  }
  bool new_mocap_pose() const {
    return new_mocap_pose_obtained_;
  }
  void reset_new_pose_obtained() {
    new_pose_obtained_ = false;
  }
  void reset_new_mocap_pose_obtained() {
    new_mocap_pose_obtained_ = false;
  }

  virtual int init() { return false; }
  virtual int run() { return false; }
  void calibrate_initial_offset() {

  }

  virtual void get_pose_info(PoseWithCovariance &_pose) {
    _pose = pose_;
  }

  virtual void transform_pose() {
    // adjust for the tilt in the mount
    matrix::Dcmf rot;
    rot = pose_.quat.to_dcm() * (cam_quat.to_dcm()).transpose();
    matrix::Quatf q(rot);

    // convert to drone body-frame // TODO customize this according to the drone
    pose_.position = {pose_.position(0), -pose_.position(2), pose_.position(1)};
    pose_.quat = {q(0), q(1), -q(3), q(2)};
    pose_.velocity = {pose_.velocity(0), -pose_.velocity(2), pose_.velocity(1)};
    pose_.acceleration = {pose_.acceleration(0), -pose_.acceleration(2), pose_.acceleration(1)};
    pose_.ang_vel = {pose_.ang_vel(0), -pose_.ang_vel(2), pose_.ang_vel(1)};
  }

  /// getters
  inline const PoseWithCovariance &pose() const {
    return pose_;
  }
  inline const PoseWithCovariance &mocap_pose() const {
    return mocap_pose_;
  }

  /// setters
  virtual void set_pose(const PoseWithCovariance &_pose) {
    pose_ = _pose;
    new_pose_obtained_ = true;
  }
  virtual void set_mocap_pose(const PoseWithCovariance &_pose) {
    mocap_pose_ = _pose;
    new_mocap_pose_obtained_ = true;
  }
  void set_cam_offset(const matrix::Vector3f t) {
    cam_t = t;
  }
  void set_cam_eul(const matrix::Vector3f e) {
    cam_eul = e;
    cam_quat = matrix::Quaternionf(cam_eul);
  }
  void set_cam_offset(float tx, float ty, float tz) {
    cam_t = {tx, ty, tz};
    std::cout << "camera_offset [m]: " << tx << " " << ty << " " << tz << std::endl;
  }
  void set_cam_eul(float r, float p, float y) {
    std::cout << "camera_euler [rad]: " << r << " " << p << " " << y << std::endl;
    cam_eul = {r, p, y};
    cam_quat = matrix::Quaternionf(cam_eul);
    cam_quat.to_dcm().print();
  }
};

}

#endif //__QROTOR_FIRMWARE_EXTERNAL_POSE_H__
