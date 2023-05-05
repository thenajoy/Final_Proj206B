#ifndef __QROTOR_GROUND_STATES_H__
#define __QROTOR_GROUND_STATES_H__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <tf/transform_datatypes.h>

namespace qrotor_ground {

class RigidbodyState {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* constructors */
  RigidbodyState() {
    position = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    angular_velocity = Eigen::Vector3d::Zero();
    quaternion = Eigen::Quaterniond();
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }
  RigidbodyState(const Eigen::Vector3d &_x, const Eigen::Vector3d &_v, const Eigen::Quaterniond &_q,
                 const Eigen::Vector3d &_Om) {
    position = _x;
    velocity = _v;
    quaternion = _q;
    angular_velocity = _Om;
  }
  RigidbodyState(const RigidbodyState &other) {
    position = other.position;
    velocity = other.velocity;
    quaternion = other.quaternion;
    angular_velocity = other.angular_velocity;
  }

  /* destructor */
  ~RigidbodyState() = default;

  Eigen::Vector3d position, velocity;
  Eigen::Matrix3d rotation() {
    return quaternion.toRotationMatrix();
  }
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d euler;
  Eigen::Quaterniond quaternion;
  double roll{}, pitch{}, yaw{};

};

class LoadState {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LoadState() {
    attitude << 0.0, 0.0, -1.0;
    angular_velocity << 0.0, 0.0, 0.0;
    position << 0, 0, 0;
    velocity << 0, 0, 0;
  }

  LoadState(Eigen::Vector3d _q, Eigen::Vector3d _om, Eigen::Vector3d _x,
            Eigen::Vector3d _v) {
    attitude = _q;
    angular_velocity = _om;
    position = _x;
    velocity = _v;
  }
  ~LoadState() = default;

  Eigen::Vector3d attitude;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

} // namespace qrotor_ground

#endif /* __QROTOR_GROUND_STATES_H__ */