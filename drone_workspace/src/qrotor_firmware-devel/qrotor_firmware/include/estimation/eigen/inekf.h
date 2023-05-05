//
// Created by kotaru on 4/1/21.
//
#ifndef QROTOR_FIRMWARE_EIGEN_INEKF_H_
#define QROTOR_FIRMWARE_EIGEN_INEKF_H_
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <memory>
#include <iostream>

namespace eigen {
class InEKF {

private:
  /// universal constants
  const float g;
  const Eigen::Vector3f e1, e2, e3;

  /// gravity vector: -g*e3
  Eigen::Vector3f gVec;
  /// skew-symmetric matrix of gravity vector: hat(gVec)
  Eigen::Matrix3f gSkew;

  Eigen::Matrix3f R;
  Eigen::Vector3f v, p;

  Eigen::Matrix<float, 9, 9> A, H;
  Eigen::Matrix<float, 9, 9> P;
  Eigen::Matrix<float, 9, 9> Q, N;

  Eigen::Matrix3f hat(const Eigen::Vector3f vector_) {
    return (Eigen::Matrix3f() << 0.0, -vector_(2), vector_(1), vector_(2),
        0.0, -vector_(0), -vector_(1), vector_(0), 0.0)
        .finished();
  }
  Eigen::Vector3f vee(const Eigen::Matrix3f mat_) {
    return (Eigen::Vector3f() << mat_(2, 1), mat_(0, 2), mat_(1, 0))
        .finished();
  }
  Eigen::Matrix3f rodriguesRotation(const Eigen::Vector3f n) {
    Eigen::Matrix3f R_ = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f K = hat(n);
    float th = n.norm();
    if (th < 1e-6) {
      return R;
    }
    R += (sin(th) / th) * K + ((1 - cos(th)) / (th * th)) * K * K;
    return R;
  }

public:
  InEKF();
  ~InEKF();

  void init();
  void timeUpdate(const float dt, const Eigen::Vector3f& _accel, const Eigen::Vector3f& _gyro);
  void measUpdate(Eigen::Matrix3f _Rm, Eigen::Vector3f _vm, Eigen::Vector3f _pm);

};
}

#endif //QROTOR_FIRMWARE_EIGEN_INEKF_H_
