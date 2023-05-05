/**
 * @file invariant_ekf.h
 * @author vkotaru
 */
#ifndef __QROTOR_FIRMWARE_EIGEN_INVARIANT_EKF_H__
#define __QROTOR_FIRMWARE_EIGEN_INVARIANT_EKF_H__

#include "estimation/eigen/imu_measurement.hpp"
#include "estimation/eigen/noise_params.hpp"
#include "estimation/eigen/rigidbody_state.hpp"
#include "estimation/eigen/eigen_utils.hpp"
#include <iostream>
// TODO create a mex/C-array version of this code

namespace eigen_estimation {
using namespace eigen_utils; // Note

/**
 * \brief Invariant Extended Kalman Filter\n
 *        (rigidbody pose estimation using IMU motion model)
 * \tparam T (float or double).
 */
template<typename T>
class InvariantEKF {
protected:
  /// estimated state (pose) of a rigidbody
  RigidbodyStatewBias <T> state_, previous_state_;

  /// universal constants
  const T g;
  const Eigen::Matrix<T, 3, 1> e1, e2, e3;

  /// gravity vector: -g*e3
  Eigen::Matrix<T, 3, 1> gVec;
  /// skew-symmetric matrix of gravity vector: hat(gVec)
  Eigen::Matrix<T, 3, 3> gSkew;

  /// linear dynamics
  Eigen::Matrix<T, 9, 9> A;
  /// measurement matrix
  Eigen::Matrix<T, 9, 9> H;
  /// second measurement matrix for pose from motion-capture
  Eigen::Matrix<T, 6, 9> H2;

  /// input covariance matrix
  Eigen::Matrix<T, 9, 9> Q;
  /// state covariance matrix
  Eigen::Matrix<T, 9, 9> state_covar_;
  /// measurement covariance matrix
  Eigen::Matrix<T, 9, 9> N;
  /// measurement covariance matrix
  Eigen::Matrix<T, 6, 6> N2;

  /// unbiased gyroscope reading
  Vec3 <T> w;
  /// unbiased accelerometer reading
  Vec3 <T> a;

  /// verbose
  bool verbose_;

  /// Identity Matrix
  Eigen::Matrix<T, 9, 9> I;

public:
  InvariantEKF();
  explicit InvariantEKF(bool _verbose);
  InvariantEKF(NoiseParams <T> _noise, bool _verbose);
  ~InvariantEKF();

  /// sensor & measurement noise parameters
  NoiseParams <T> noise_;

  /**
   * Update noise params and calls updateNoiseParams()
   * @param [in] _noise
   */
  virtual void updateNoiseParams(NoiseParams <T> _noise);

  /**
   * Reconstructs the covariance matrices
   */
  virtual void updateNoiseParams();

  /**
   * initialize Invariant Extended Kalman Filter
   * @param [in] _state (initial state)
   */
  void init(RigidbodyState <T> _state) {
    state_.update(_state.X());

    state_.print();
    std::cout << "state_covar: " << state_covar_.diagonal().transpose() << std::endl;
    std::cout << "Q: " << Q.diagonal().transpose() << std::endl;
    std::cout << "N: " << N.diagonal().transpose() << std::endl;
  }

  /**
   * initialize Invariant EKF
   * @param [in] _state (state with bias)
   */
  void init(const RigidbodyStatewBias <T> _state) {
    state_ = _state;
  }

  /**
   * Kalman filter time-update (state propagate step)\n
   * performs Euler integration of state and covariance update
   * @tparam T (double/float)
   * @param [in] dt: loop time
   * @param [in] _imu: ImuMeasurement<T>
   */
  void timeUpdate(T dt, ImuMeasurement <T> _imu);

  /**
   * Kalman filter measurement update (correction step)
   * @tparam T (double/float)
   * @param [n] _state_meas (RigibodyState<T>)
   */
  void measUpdate(RigidbodyState <T> _state_meas);

  /**
 * Kalman filter measurement update (correction step)
 * @tparam T (double/float)
 * @param [n] _state_meas (RigibodyState<T>)
 */
  void measUpdate2(RigidbodyState <T> _state_meas);

  /**
   * computes right invariant error of SEK3 state
   * @param [in] state1
   * @param [in] state2
   * @return error in R^9
   */
  Eigen::Matrix<T, 9, 1> invariantErrorRight(RigidbodyState <T> state1, RigidbodyState <T> state2);

  /**
   * returns the current estimated state
   * @return state_
   */
  inline const RigidbodyStatewBias <T> &state() const {
    return state_;
  }

  /**
   * returns the current state covariance
   * @return state_covar_
   */
  const Eigen::Matrix<T, 9, 9> state_covar() const {
    return state_covar_;
  }
};

typedef InvariantEKF<float> InvariantEKFf;
typedef InvariantEKF<double> InvariantEKFd;

} // namespace pose_estimation
#endif // __QROTOR_FIRMWARE_EIGEN_INVARIANT_EKF_H__
