#ifndef __QROTOR_FIRMWARE_EIGEN_IEKF_RIGIDBODY_STATE_H__
#define __QROTOR_FIRMWARE_EIGEN_IEKF_RIGIDBODY_STATE_H__

#include "estimation/eigen/eigen_utils.hpp"

namespace eigen_estimation {

/**
 * \brief RigidbodyState
 * \attention rot_, ang_vel_, pos_, vel_
 * \tparam T (float or double).
 * \author vkotaru
 */
template<typename T>
class RigidbodyState {
 private:
  Eigen::Matrix<T, 3, 3> rot_;
  Eigen::Matrix<T, 3, 1> pos_, vel_, ang_vel_;

 public:
  RigidbodyState() {
    rot_ = Eigen::Matrix<T, 3, 3>::Identity();
    pos_ = Eigen::Matrix<T, 3, 1>::Zero();
    vel_ = Eigen::Matrix<T, 3, 1>::Zero();
    ang_vel_ = Eigen::Matrix<T, 3, 1>::Zero();
  }
  RigidbodyState(Eigen::Matrix<T, 3, 3> _R, Eigen::Matrix<T, 3, 1> _vel, Eigen::Matrix<T, 3, 1> _pos) {
    rot_ = _R;
    vel_ = _vel;
    pos_ = _pos;
    ang_vel_ = Eigen::Matrix<T, 3, 1>::Zero();
  }

  /**
   * returns the orientation as a rotation matrix
   * @return rotation matrix
   */
  Eigen::Matrix<T, 3, 3> rotation() const { return rot_; }
  /// @return position
  Eigen::Matrix<T, 3, 1> position() const { return pos_; }
  /// @return velocity
  Eigen::Matrix<T, 3, 1> velocity() const { return vel_; }
  /// @return angular_velocity
  Eigen::Matrix<T, 3, 1> angular_velocity() const { return ang_vel_; }
  ///
  void set_velocity(const Eigen::Matrix<T, 3, 1> v) {
    vel_ = v;
  }
  void set_ang_vel(const Eigen::Matrix<T, 3, 1> w) {
    ang_vel_ = w;
  }

  /**
   * updates the current pose (odometry)
   * @param _R [in] rotation matrix (Eigen::Matrix<T, 3, 3>)
   * @param _v [in] velocity (Eigen::Matrix<T, 3, 1>)
   * @param _p [in] position (Eigen::Matrix<T, 3, 1>)
   * @return None
   */
  void update(const Eigen::Matrix<T, 3, 3> _R, const Eigen::Matrix<T, 3, 1> _v,
              const Eigen::Matrix<T, 3, 1> _p) {
    rot_ << _R;
    vel_ << _v;
    pos_ << _p;
  }
  void update(const Eigen::Matrix<T, 5, 5> X) {
    rot_ = X.block(0, 0, 3, 3);
    vel_ = X.block(0, 3, 3, 1);
    pos_ = X.block(0, 4, 3, 1);
  }

  Eigen::Matrix<T, 5, 5> X() const {
    Eigen::Matrix<T, 5, 5> X_;
    X_ << rot_, vel_, pos_, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    return X_;
  }

  Eigen::Matrix<T, 5, 5> invX() {
    Eigen::Matrix<T, 5, 5> X_;
    X_ << rot_.transpose(), -rot_.transpose() * vel_, -rot_.transpose() * pos_, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    return X_;
  }

  Eigen::Matrix<T, 5, 5> operator()() {
    return this->X();
  }

  // Overload += operator
  void operator+=(const Eigen::Matrix<T, Eigen::Dynamic, 1> &delta) {
    Eigen::Matrix<T, 5, 5> X = eigen_utils::expSEK3<T>(delta) * this->X();
    this->update(X);
  }

  void print() const {
    std::cout << "rotation: \n"
              << rot_ << "\nvelocity: \n"
              << vel_ << "\nposition: \n"
              << pos_ << std::endl;
  }
};

template<typename T>
class RigidbodyStatewBias : public RigidbodyState<T> {
 private:
  Eigen::Matrix<T, 3, 1> gyro_bias_;
  Eigen::Matrix<T, 3, 1> accel_bias_;

 public:
  RigidbodyStatewBias() : RigidbodyState<T>() {
    gyro_bias_.setZero();
    accel_bias_.setZero();
  }
  RigidbodyStatewBias(Eigen::Matrix<T, 3, 3> _R, Eigen::Matrix<T, 3, 1> _vel, Eigen::Matrix<T, 3, 1> _pos) :
      RigidbodyState<T>(_R, _vel, _pos) {
    gyro_bias_.setZero();
    accel_bias_.setZero();
  }

  Eigen::Matrix<T, 3, 1> gyro_bias() { return gyro_bias_; }
  Eigen::Matrix<T, 3, 1> accel_bias() { return accel_bias_; }

};

typedef RigidbodyState<double> RigidbodyStated;
typedef RigidbodyState<float> RigidbodyStatef;
typedef RigidbodyStatewBias<double> RigidbodyStatewBiasd;
typedef RigidbodyStatewBias<float> RigidbodyStatewBiasf;

} // namespace pose_estimation

#endif // __QROTOR_FIRMWARE_EIGEN_IEKF_RIGIDBODY_STATE_H__
