/** @file utils.hpp
 * File containing utils functions related to geometry, lie-groups, timers
 */
#ifndef __QROTOR_FIRMWARE_EIGEN_UTILS_HPP__
#define __QROTOR_FIRMWARE_EIGEN_UTILS_HPP__

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <memory>
#include <iostream>

namespace eigen_estimation {
namespace eigen_utils {
template<typename T>
class EigenPose {
public:
  EigenPose() = default;
  EigenPose(Eigen::Quaternion<T> q, Eigen::Matrix<T, 3, 1> w,
            Eigen::Matrix<T, 3, 1> p, Eigen::Matrix<T, 3, 1> v)
      : quat(q), ang_vel(w), position(p), velocity(v) {}

  explicit EigenPose(Eigen::Matrix<T, 3, 1> n) {
    T norm_of_n = n.norm();
    n.normalize();
    quat = Eigen::AngleAxis<T>(norm_of_n, n);
  }
  explicit EigenPose(Eigen::AngleAxis<T> n) { quat = n; }

  // pose
  Eigen::Quaternion<T> quat;
  Eigen::Matrix<T, 3, 1> ang_vel, position, velocity;

};

template<typename T>
inline Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> vector_) {
  return (Eigen::Matrix<T, 3, 3>() << 0.0, -vector_(2), vector_(1), vector_(2),
      0.0, -vector_(0), -vector_(1), vector_(0), 0.0)
      .finished();
}

template<typename T>
inline Eigen::Matrix<T, 3, 1> vee(const Eigen::Matrix<T, 3, 3> mat_) {
  return (Eigen::Matrix<T, 3, 1>() << mat_(2, 1), mat_(0, 2), mat_(1, 0))
      .finished();
}

/**
 * exponential map of SO(3) using Rodrigue's rotation formula\n
 * @tparam T
 * @param n [in] a vector of size 3
 * @return Eigen::Matrix<T, 3, 3>
 */
template<typename T>
inline Eigen::Matrix<T, 3, 3>
rodriguesRotation(const Eigen::Matrix<T, 3, 1> n) {
  Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 3> K = hat<T>(n);
  T th = n.norm();
  if (th < 1e-6) {
    return R;
  }
  R += (sin(th) / th) * K + ((1 - cos(th)) / (th * th)) * K * K;
  return R;
}

/**
 * left Jacobian of  SO(3) \n
 * Ref: https://github.com/RossHartley/invariant-ekf/blob/ef16e8a1df72f9272111a488880e3fe9d161f59f/src/LieGroup.cpp#L62
 * @tparam T
 * @param n [in] a vector of size 3
 * @return Eigen::Matrix<T, 3, 3>
 */
template<typename T>
inline Eigen::Matrix<T, 3, 3> leftJacobianSO3(const Eigen::Matrix<T, 3, 1> n) {

  Eigen::Matrix<T, 3, 3> Jl = Eigen::Matrix<T, 3, 3>::Identity();
  Eigen::Matrix<T, 3, 3> K = hat<T>(n);
  T th = n.norm();
  if (th < 1e-6) {
    return Jl;
  }
  T theta2 = th * th;
  T stheta = sin(th);
  T ctheta = cos(th);
  T oneMinusCosTheta2 = (1 - ctheta) / (theta2);
  Jl += oneMinusCosTheta2 * K + ((th - stheta) / (theta2 * th)) * K * K;
  return Jl;
}

/**
 * exponential map of matrix Lie group SE_K(3)\n
 * Ref: Hartley, Ross, et al. "Contact-aided invariant extended Kalman\n
 * filtering for robot state estimation." The International Journal of Robotics\n
 * Research 39.4 (2020): 402-430.
 * @tparam T
 * @param v [in] a vector of size 3(K+1)
 * @return Eigen::Matrix<T, 3+K, 3+K>
 */
template<typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> expSEK3(const Eigen::Matrix<T, Eigen::Dynamic, 1> vec) {

  int K = (vec.size() - 3) / 3;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat, mat_exp;
  mat = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(3 + K, 3 + K);

//  std::cout << " before exponentiating "  << "mat \n" << mat << std::endl;
  // lie-algebra
  mat.block(0, 0, 3, 3) = hat<T>(vec.head(3));
  for (int i = 0; i < K; ++i) {
    mat.block(0, 3 + i, 3, 1) = vec.block(3 + 3 * i, 0, 3, 1);
  }
  mat_exp = mat.exp();

//  std::cout << " after exponentiating "  << "mat_exp \n" << mat_exp << std::endl;
  //  Eigen::Matrix<T, 3, 1> w = v.head(3);
  //  Eigen::Matrix<T, 3, 3> R = rodriguesRotation(w);
  //  Eigen::Matrix<T, 3, 3> Jl = leftJacobianSO3(w);
  //
  //  X.block(0,0, 3, 3) = R;
  //  for (int i=0; i<K; ++i) {
  //    X.block(0,3+i, 3,1) = Jl * v.block(3+3*i, 0, 3, 1);
  //  }
  return mat_exp; // TODO find the computation time
}

/**
 * logarithmic map of matrix Lie group SE_K(3)\n to vector in R^(K+1)
 * @tparam T
 * @param v [in] Eigen::Matrix<T, 3+K, 3+K>
 * @return vector of size 3(K+1)
 */
template<typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, 1> logSEK3(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat) {

  std::cout << "inside: logSEK3" << std::endl;

  int K = (mat.rows() - 3);
  std::cout << "K: " << K << std::endl;
  Eigen::Matrix<T, Eigen::Dynamic, 1> vec; // this is throwing an error in ros-node
  vec.resize(3 + 3 * K, 1);
  vec.setZero();

  std::cout << "vec: " << vec.transpose() << std::endl;

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mat_log;
  mat_log.resize(K + 3, K + 3);
  std::cout << "mat\n"  << mat << std::endl;

  mat_log = mat.log(); // TODO find the computation time
  std::cout << "mat_log\n"  << mat_log << std::endl;

  std::cout << "K " << K << " done logarthimicking" << std::endl;

  // lie-algebra-inverse
  vec.block(0, 0, 3, 1) = vee<T>(mat_log.block(0, 0, 3, 3));
  for (int i = 0; i < K; ++i) {
    vec.block(3 + 3 * i, 0, 3, 1) = mat_log.block(0, 3 + i, 3, 1);
  }
  return vec;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> Rx(const T roll) {
  Eigen::Matrix<T, 3, 3> R;
  R.row(0) << 1, 0, 0;
  R.row(1) << 0, cos(roll), -sin(roll);
  R.row(2) << 0, sin(roll), cos(roll);
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> Ry(const T pitch) {
  Eigen::Matrix<T, 3, 3> R;
  R.row(0) << cos(pitch), 0, sin(pitch);
  R.row(1) << 0, 1, 0;
  R.row(2) << -sin(pitch), 0, cos(pitch);
  return R;
}

template<typename T>
inline Eigen::Matrix<T, 3, 3> Rz(const T yaw) {
  Eigen::Matrix<T, 3, 3> R;
  R.row(0) << cos(yaw), -sin(yaw), 0;
  R.row(1) << sin(yaw), cos(yaw), 0;
  R.row(2) << 0, 0, 1;
  return R;
}


///////////////////////////////////////////
template<typename T> using Vec3 = Eigen::Matrix<T, 3, 1>;
template<typename T> using Mat3 = Eigen::Matrix<T, 3, 3>;
template<typename T, int N> using VecN = Eigen::Matrix<T, N, 1>;
template<typename T, int N> using MatN = Eigen::Matrix<T, N, N>;

//////////////////////////////////////////
#define cvVec3d2AngleAxisf cvVec3d2AngleAxis<float>
#define cvVec3d2AngleAxisd cvVec3d2AngleAxis<double>

#define EigenPosef EigenPose<float>
#define EigenPosed EigenPose<double>

#define hatf hat<float>
#define hatd hat<double>

#define veef vee<float>
#define veed vee<double>

#define Rxf Rx<float>
#define Rxd Rx<double>

#define Ryf Ry<float>
#define Ryd Ry<double>

#define Rzf Rz<float>
#define Rzd Rz<double>

//////////////////////////////////////////
} // namespace utils
} // namespace pose_estimation
#endif // __QROTOR_FIRMWARE_EIGEN_UTILS_HPP__
