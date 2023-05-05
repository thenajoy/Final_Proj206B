#include "estimation/eigen/invariant_ekf.h"

namespace eigen_estimation {
#define G_SI 9.81

template <typename T>
InvariantEKF<T>::InvariantEKF() : InvariantEKF<T>(NoiseParams<T>(), false) {}

//  TODO fix this constructor
template <typename T>
InvariantEKF<T>::InvariantEKF(bool _verbose)
    : InvariantEKF<T>(NoiseParams<T>(), _verbose) {}

template <typename T>
InvariantEKF<T>::InvariantEKF(NoiseParams<T> _noise, const bool _verbose)
    : g(9.80), e1(1.0, 0.0, 0.0), e2(0.0, 1.0, 0.0), e3(0.0, 0.0, 1.0),
      verbose_(_verbose), noise_(_noise) {

  // gravity parameters
  gVec = -g * e3;
  gSkew = hat(gVec);

  a << 0., 0., g;
  w << 0., 0., 0;

  // linear dynamics
  A = Eigen::Matrix<T, 9, 9>::Zero();
  A.block(3, 0, 3, 3) = gSkew;
  A.block(6, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();

  // measurement matrix
  H = Eigen::Matrix<T, 9, 9>::Identity();

  // setup noise
  updateNoiseParams(noise_);

  // identity matrix
  I = Eigen::Matrix<T, 9, 9>::Identity();
}

template <typename T> InvariantEKF<T>::~InvariantEKF() = default;

template <typename T> void InvariantEKF<T>::updateNoiseParams() {
  // input covariance
  Q.setZero();
  Q.block(0, 0, 3, 3) = noise_.gyro_covar();
  Q.block(3, 3, 3, 3) = noise_.accel_covar();
  Q.block(6, 6, 3, 3) =
      0.0001 * Eigen::Matrix<T, 3, 3>::Identity(); // To ensure a well-defined
                                                   // covariance matrix

  // measurement covariance
  N.setZero();
  N.block(0, 0, 3, 3) = noise_.meas_rotation_covar();
  N.block(3, 3, 3, 3) = noise_.meas_velocity_covar();
  N.block(6, 6, 3, 3) = noise_.meas_position_covar();

  state_covar_.setZero();
  state_covar_ = Eigen::Matrix<T, 9, 9>::Identity();
}

template <typename T>
void InvariantEKF<T>::updateNoiseParams(const NoiseParams<T> _noise) {
  this->noise_ = _noise;
  InvariantEKF<T>::updateNoiseParams();
}

template <typename T>
void InvariantEKF<T>::timeUpdate(const T dt, ImuMeasurement<T> _imu) {

  // remove bias from the measurements
  // TODO convert the hardcoded values to parameters
  if (_imu.gyro().norm() < 10) {
    w = _imu.gyro() - state_.gyro_bias();
  }
  if (_imu.accel().norm() < 20) {
    a = _imu.accel() - state_.accel_bias();
  }

  if (verbose_) {
    std::cout << "-----------------------------------------------\n"
              << "dt_s " << dt << "\n"
              << "accel: " << _imu.accel().transpose() << "\n"
              << "gyro: " << _imu.gyro().transpose() << std::endl;
    std::cout << "previous state: \n" << state_.X() << std::endl;
    std::cout << "net accel:" << (state_.rotation() * a + gVec).transpose()
              << std::endl;
  }

  // time-propagate IMU motion model
  Mat3<T> R_ =
      state_.rotation() * rodriguesRotation((Vec3<T>() << w * dt).finished());
  Vec3<T> v_ = state_.velocity() + (state_.rotation() * a + gVec) * dt;
  Vec3<T> p_ = state_.position() + state_.velocity() * dt +
               0.5 * (state_.rotation() * a + gVec) * dt * dt;

  // discretization
  Eigen::Matrix<T, 9, 9> Ak = Eigen::Matrix<T, 9, 9>::Identity() + this->A * dt;

  // TODO fix this
  //  Eigen::Matrix<T, 3, 3> O = Eigen::Matrix<T, 3, 3>::Zero();
  //  Eigen::Matrix<T, 9, 9> AdjX;
  //  AdjX << state_.rotation(), O, O,
  //      hat<T>(state_.velocity()) * state_.rotation(), state_.rotation(), O,
  //      hat<T>(state_.position()) * state_.rotation(), O, state_.rotation();
  //
  //  Eigen::Matrix<T, 9, 9> Qk =
  //      Ak * AdjX * Q * AdjX.transpose() * Ak.transpose() * dt_s;

  // store the updated states
  state_.update(R_, v_, p_);
  state_.set_ang_vel(w); // TODO apply a low-pass filter

  //  covariance update
  state_covar_ = Ak * state_covar_ * Ak.transpose() + Q;

  if (verbose_) {
    std::cout << "Ak: \n"
              << Ak << "\n"
              << "propagated state: \n"
              << state_.X() << "\n"
              << "state_covar: \n"
              << state_covar_ << std::endl;
  }
}

template <typename T>
void InvariantEKF<T>::measUpdate(RigidbodyState<T> _state_meas) {
  // Kalman gain
  Eigen::Matrix<T, 9, 9> K = Eigen::Matrix<T, 9, 9>::Zero();
  Eigen::Matrix<T, 9, 9> S = H * state_covar_ * H.transpose() + N;

  //  std::cout << "computing K" << std::endl;
  K = (state_covar_ * H.transpose()) * S.inverse();
  //  std::cout << "K:\n" << K << std::endl;

  // Kalman update
  //  std::cout << "initializing delta " << std::endl;
  Eigen::Matrix<T, 9, 1> delta;
  //  std::cout << "computing delta " << std::endl;
  delta = K * invariantErrorRight(_state_meas, state_);
  //  std::cout << "delta: \n" << delta << std::endl;

  Eigen::Matrix<T, 5, 5> delta_exp, delta_hat;
  delta_hat << hat<T>(delta.block(0, 0, 3, 1)), delta.block(3, 0, 3, 1),
      delta.block(6, 0, 3, 1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  delta_exp = Eigen::Matrix<T, 5, 5>::Identity() + delta_hat +
              0.5 * delta_hat * delta_hat;
  //      + (1 / 6) * delta_hat * delta_hat * delta_hat;

  state_.update(delta_exp * state_.X());
  //  std::cout << "state_ += delta;" << std::endl;
  previous_state_ = state_;
  //  std::cout << "previous_state_ = state_;" << std::endl;

  // covariance update
  //  I_KH = Eigen::Matrix<T, 9, 9>::Identity() - K * H;
  //  state_covar_ = I_KH * state_covar_ * I_KH.transpose() + K * N *
  //  K.transpose();
  state_covar_ = (I - K * H) * state_covar_ * (I - K * H).transpose() +
                 K * N * K.transpose(); // Joseph update form
  //  std::cout << "state_covar_:\n" << state_covar_ << std::endl;

  if (verbose_) {
    std::cout << "K: \n" << K << std::endl;
    std::cout << "measurement :\n" << _state_meas.X() << std::endl;
    std::cout << "delta: \n" << delta.transpose() << std::endl;
    std::cout << "corrected state\n" << state_.X() << std::endl;
    std::cout << "state covar\n" << state_covar_ << std::endl;
  }
}

template <typename T>
void InvariantEKF<T>::measUpdate2(RigidbodyState<T> _state_meas) {
  // Kalman gain
  Eigen::Matrix<T, 9, 6> K = Eigen::Matrix<T, 9, 6>::Zero();
  Eigen::Matrix<T, 6, 6> S = H2 * state_covar_ * H2.transpose() + N2;

  //  std::cout << "computing K" << std::endl;
  K = (state_covar_ * H2.transpose()) * S.inverse();
  //  std::cout << "K:\n" << K << std::endl;

  // Kalman update
  //  std::cout << "initializing delta " << std::endl;
  Eigen::Matrix<T, 9, 1> delta= invariantErrorRight(_state_meas, state_);
  //  std::cout << "computing delta " << std::endl;
//  delta = K *
  //  std::cout << "delta: \n" << delta << std::endl;

  //  Eigen::Matrix<T, 5, 5> delta_exp, delta_hat;
  //  delta_hat << hat<T>(delta.block(0, 0, 3, 1)), delta.block(3, 0, 3, 1),
  //  delta.block(6, 0, 3, 1),
  //      0, 0, 0, 0, 0,
  //      0, 0, 0, 0, 0;
  //  delta_exp = Eigen::Matrix<T, 5, 5>::Identity() + delta_hat; // + 0.5 *
  //  delta_hat * delta_hat
  ////      + (1 / 6) * delta_hat * delta_hat * delta_hat;
  //
  //  state_.update(delta_exp * state_.X());
  ////  std::cout << "state_ += delta;" << std::endl;
  //  previous_state_ = state_;
  ////  std::cout << "previous_state_ = state_;" << std::endl;
  //
  //  // covariance update
  ////  I_KH = Eigen::Matrix<T, 9, 9>::Identity() - K * H;
  ////  state_covar_ = I_KH * state_covar_ * I_KH.transpose() + K * N *
  ///K.transpose();
  //  state_covar_ = (I - K * H) * state_covar_ * (I - K * H).transpose() + K *
  //  N * K.transpose(); // Joseph update form
  ////  std::cout << "state_covar_:\n" << state_covar_ << std::endl;
  //
  //  if (verbose_) {
  //    std::cout << "K: \n" << K << std::endl;
  //    std::cout << "measurement :\n" << _state_meas.X() << std::endl;
  //    std::cout << "delta: \n" << delta.transpose() << std::endl;
  //    std::cout << "corrected state\n" << state_.X() << std::endl;
  //    std::cout << "state covar\n" << state_covar_ << std::endl;
  //  }
}

template <typename T>
Eigen::Matrix<T, 9, 1>
InvariantEKF<T>::invariantErrorRight(RigidbodyState<T> state1,
                                     RigidbodyState<T> state2) {
  Eigen::Matrix<T, 5, 5> eta, eta_log, BI;
  //  std::cout << "state.X\n" << state1.X() << std::endl;
  //  std::cout << "state2.invX\n" << state2.invX() << std::endl;
  eta = state1.X() * state2.invX();
  //  std::cout << "eta\n" << eta << std::endl;

  BI = eta - Eigen::Matrix<T, 5, 5>::Identity();
  eta_log = BI - (0.5) * BI * BI; //+ (1 / 3) * BI * BI * BI;
  //  std::cout << "eta_log  " << eta_log << std::endl;
  Eigen::Matrix<T, 9, 1> error;
  error.block(0, 0, 3, 1) = vee<T>(eta_log.block(0, 0, 3, 3));
  error.block(3, 0, 3, 1) = eta_log.block(0, 3, 3, 1);
  error.block(6, 0, 3, 1) = eta_log.block(0, 4, 3, 1);
  return error;
}

/// templates predefined
template class InvariantEKF<float>;
template class InvariantEKF<double>;

} // namespace eigen_estimation
