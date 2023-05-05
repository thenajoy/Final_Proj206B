#ifndef QROTOR_FIRMWARE_ATTITUDE_ESTIMATOR_H
#define QROTOR_FIRMWARE_ATTITUDE_ESTIMATOR_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <memory>
#include <unistd.h>

#include "data_structures.h"
#include "estimation/attitude_estimator.h"
#include "filters/low_pass_filter.hpp"
#include "filters/notch_filter.hpp"
#include "parameters.h"
#include <Matrix/matrix/math.hpp>

namespace qrotor_firmware {
class FlightController;
class AttitudeEstimator {
protected:
  FlightController &firmware_;

  matrix::Vector3f gyro_bias_;
  matrix::Vector3f accel_{0., 0., 9.8};
  matrix::Vector3f gyro_{0., 0., 0.};
  matrix::Vector3f gyro_prev_{0., 0., 0.};
  matrix::Vector3f gyro_rate_{0., 0., 0.};

  matrix::Quatf quat{1.0, 0.0, 0.0, 0.0};
  Attitude state_;

  bool GYRO_LPF{true}, GYRO_NOTCH{false};
  LowPassFilter gyro_low_pass_{500.0, 60.0};
  NotchFilter gyro_notch_{500.0f, 120.0f, 0.55};

  bool GYRO_RATE_LPF{true}, GYRO_RATE_NOTCH{false};
  LowPassFilter gyro_rate_low_pass_{500.0, 50.0};
  NotchFilter gyro_rate_notch_{500.0f, 80.0f, 0.55};

  bool ACCEL_LPF{true}, ACCEL_NOTCH{false};
  LowPassFilter accel_low_pass_{500.0, 100.0};
  NotchFilter accel_notch_{500.0f, 120.0f, 0.55};

  void filtering(float dt);

  matrix::Quatf omega2qdot(const matrix::Quaternionf &quat,
                           const matrix::Vector3f &omega) {

    matrix::Quatf qdot(
        0.5f * (-omega(0) * quat(1) - omega(1) * quat(2) - omega(2) * quat(3)),
        0.5f * (omega(0) * quat(0) + omega(2) * quat(2) - omega(1) * quat(3)),
        0.5f * (omega(1) * quat(0) - omega(2) * quat(1) + omega(0) * quat(3)),
        0.5f * (omega(2) * quat(0) + omega(1) * quat(1) - omega(0) * quat(2)));
    return qdot;
  }

  void getEuler(float &roll, float &pitch, float &yaw);

public:
  explicit AttitudeEstimator(FlightController &_flightcontroller);
  ~AttitudeEstimator() = default;

  virtual void init() {}
  virtual void run(float dt) {}
  void external_quat_fusion(matrix::Quatf &_q, matrix::Quatf &_qm) const;
  void external_yaw_fusion (matrix::Quatf& _q) const;

  void reset_state() {
    state_.reset();
    gyro_bias_.setZero();
  }

  /* getter */
  inline const Attitude &state() const { return state_; }

  /* setters */
  void gyro_filters(bool lpf, bool notch);
  void gyro_rate_filters(bool lpf, bool notch);
  void accel_filters(bool lpf, bool notch);
  void set_gyro_lpf_cutoff(float fc);
  void set_gyro_rate_lpf_cutoff(float fc);
  void set_gyro_notch_freq(float fc, float w);
  void set_gyro_rate_notch_freq(float fc, float w);
  void set_accel_lpf_cutoff(float fc);
  void set_accel_notch_freq(float fc, float w);
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_ATTITUDE_ESTIMATOR_H
