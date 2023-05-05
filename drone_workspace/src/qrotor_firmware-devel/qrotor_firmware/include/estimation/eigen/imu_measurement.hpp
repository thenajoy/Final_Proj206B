#ifndef IEKF_IMU_HPP
#define IEKF_IMU_HPP

#include "estimation/eigen/eigen_utils.hpp"

namespace eigen_estimation {

template <typename T> class ImuMeasurement {
private:
  T time_stamp_; // TODO: (find the correct data-type for time (int64?)
  Eigen::Matrix<T, 3, 1> accel_;
  Eigen::Matrix<T, 3, 1> gyro_;
  Eigen::Matrix<T, 3, 1> mag_;

public:
  ImuMeasurement() {
    accel_.setZero();
    gyro_.setZero();
    mag_.setZero();
    time_stamp_ = 0;
  }
  ImuMeasurement(const T t, Eigen::Matrix<T, 9, 1> readings) {
    time_stamp_ = t;
    accel_ = readings.block<3, 1>(0, 0);
    gyro_ = readings.block<3, 1>(3, 0);
    mag_ = readings.block<3, 1>(6, 0);
  }
  ImuMeasurement(const T t, Eigen::Matrix<T, 6, 1> readings) {
    time_stamp_ = t;
    accel_ = readings.head(3);
    gyro_ = readings.tail(3);
  }

  void update(const T t, const T ax, const T ay, const T az, const T gx,
              const T gy, const T gz) {
    time_stamp_ = t;
    accel_ << ax, ay, az;
    gyro_ << gx, gy, gz;
  }

  void update(const T t, const T ax, const T ay, const T az, const T gx,
              const T gy, const T gz, const T mx, const T my, const T mz) {
    time_stamp_ = t;
    accel_ << ax, ay, az;
    gyro_ << gx, gy, gz;
    mag_ << mx, my, mz;
  }

  T timeStamp() const { return time_stamp_; }
  Eigen::Matrix<T, 3, 1> gyro() const { return gyro_; }
  Eigen::Matrix<T, 3, 1> accel() const { return accel_; }
  Eigen::Matrix<T, 6, 1> accel_gyro() const {
    return (Eigen::Matrix<T, 6, 1>() << accel_, gyro_).finished();
  }
  Eigen::Matrix<T, 3, 1> mag() const { return mag_; }
};

typedef ImuMeasurement<double> ImuMeasurementd;
typedef ImuMeasurement<float> ImuMeasurementf;

} // namespace eigen_estimation

#endif // IEKF_IMU_HPP