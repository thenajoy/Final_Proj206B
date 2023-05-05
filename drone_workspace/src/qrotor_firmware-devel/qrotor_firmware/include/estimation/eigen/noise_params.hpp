#ifndef IEKF_NOISE_PARAMS_HPP
#define IEKF_NOISE_PARAMS_HPP

#include "estimation/eigen/eigen_utils.hpp"

namespace eigen_estimation {

template <typename T> class NoiseParams {
private:
  Eigen::Matrix<T, 3, 1> accel_mean_;
  Eigen::Matrix<T, 3, 3> accel_covar_;

  Eigen::Matrix<T, 3, 1> gyro_mean_;
  Eigen::Matrix<T, 3, 3> gyro_covar_;

  Eigen::Matrix<T, 3, 1> mag_mean_;
  Eigen::Matrix<T, 3, 3> mag_covar_;

  Eigen::Matrix<T, 3, 3> meas_rotation_covar_;
  Eigen::Matrix<T, 3, 3> meas_velocity_covar_;
  Eigen::Matrix<T, 3, 3> meas_position_covar_;

public:
  NoiseParams() {
    // TODO update the default values

    // accelerometer
    accel_mean_ = Eigen::Matrix<T, 3, 1>::Zero();
    accel_covar_ = 0.0001*Eigen::Matrix<T, 3, 3>::Identity();

    // gyroscope
    gyro_mean_ = Eigen::Matrix<T, 3, 1>::Zero();
    gyro_covar_ = 0.0001*Eigen::Matrix<T, 3, 3>::Identity();

    // magnetometer
    mag_mean_ = Eigen::Matrix<T, 3, 1>::Zero();
    mag_covar_ = 0.0001*Eigen::Matrix<T, 3, 3>::Identity();

    // measurement covariances
    meas_rotation_covar_ = 0.001*Eigen::Matrix<T, 3, 3>::Identity(); // 0.001
    meas_velocity_covar_ = 0.001*Eigen::Matrix<T, 3, 3>::Identity();  // 0.04
    meas_position_covar_ = 0.001*Eigen::Matrix<T, 3, 3>::Identity(); // 0.001
  }

  // setters
  void set_accel_mean() {}
  void set_accel_std(const T sigma) {
    accel_covar_ = sigma*sigma*Eigen::Matrix<T, 3, 3>::Identity();
  }
  void set_accel_std(const T sx, const T sy, const T sz) {
    accel_covar_ << sx*sx, 0., 0.,
                    0., sy*sy, 0.,
                    0., 0., sz*sz;
  }
  void set_gyro_std(const T  sigma) {
    gyro_covar_ = sigma*sigma*Eigen::Matrix<T, 3, 3>::Identity();
  }
  void set_gyro_std(const T sx, const T sy, const T sz) {
    gyro_covar_ << sx*sx, 0., 0.,
        0., sy*sy, 0.,
        0., 0., sz*sz;
  }
  void set_rotation_std(const T  sigma) {
    meas_rotation_covar_ = sigma*sigma*Eigen::Matrix<T, 3, 3>::Identity();
  }
  void set_velocity_std(const T  sigma) {
    meas_velocity_covar_ = sigma*sigma*Eigen::Matrix<T, 3, 3>::Identity();
  }
  void set_position_std(const T  sigma) {
    meas_position_covar_ = sigma*sigma*Eigen::Matrix<T, 3, 3>::Identity();
  }
  void set_imu_stddev(const T _accel_sigma, const T _gyro_sigma) {
    accel_covar_ = _accel_sigma*_accel_sigma*Eigen::Matrix<T, 3, 3>::Identity();
    gyro_covar_ = _gyro_sigma*_gyro_sigma*Eigen::Matrix<T, 3, 3>::Identity();

  }
  void set_meas_stddev(const T _rot_sigma, const T _vel_sigma, const T _pos_sigma) {
    meas_rotation_covar_ = _rot_sigma*_rot_sigma*Eigen::Matrix<T, 3, 3>::Identity();
    meas_velocity_covar_ = _vel_sigma*_vel_sigma*Eigen::Matrix<T, 3, 3>::Identity();
    meas_position_covar_ = _pos_sigma*_pos_sigma*Eigen::Matrix<T, 3, 3>::Identity();
  }

  // getters
  const Eigen::Matrix<T, 3, 1> accel_mean() { return accel_mean_; }
  const Eigen::Matrix<T, 3, 3> accel_covar() { return accel_covar_; }
  const Eigen::Matrix<T, 3, 1> gyro_mean() { return gyro_mean_; }
  const Eigen::Matrix<T, 3, 3> gyro_covar() { return gyro_covar_; }

  const Eigen::Matrix<T, 3, 1> mag_mean() { return mag_mean_; }
  const Eigen::Matrix<T, 3, 3> mag_covar() { return mag_covar_; }

  const Eigen::Matrix<T, 3, 3> meas_rotation_covar() { return meas_rotation_covar_; }
  const Eigen::Matrix<T, 3, 3> meas_velocity_covar() { return meas_velocity_covar_; }
  const Eigen::Matrix<T, 3, 3> meas_position_covar() { return meas_position_covar_; }

};

} // namespace pose_estimation

#endif // IEKF_NOISE_PARAMS_HPP
