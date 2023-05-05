#ifndef __QROTOR_FIRMWARE_ESTIMATOR_H__
#define __QROTOR_FIRMWARE_ESTIMATOR_H__

#include <cstdint>
#include <cstdio>
#include <memory>
#include <unistd.h>
#include <ctime>
#include <cmath>

#include "parameters.h"
#include "data_structures.h"
#include <Matrix/matrix/math.hpp>
#include "filters/low_pass_filter.hpp"
#include "filters/notch_filter.hpp"

namespace qrotor_firmware {

class FlightController;

class AHRSDefault { // TODO: Customize this to separate folder (containing the estimators)

public:
  explicit AHRSDefault(FlightController &_flightcontroller);

  // state
  inline const Attitude &state() const {
    return state_;
  }

  // void updateIMU(float dt_s);
  void getEuler(float *roll, float *pitch, float *yaw);
  void init();

  void update(float dt, float ax, float ay, float az, float gx, float gy, float gz);
  void run(float dt);

  void gyro_filters(bool lpf, bool notch);
  void gyro_rate_filters(bool lpf, bool notch);
  void accel_filters(bool lpf, bool notch);
  void set_gyro_lpf_cutoff(float fc);
  void set_gyro_rate_lpf_cutoff(float fc);
  void set_gyro_notch_freq(float fc, float w);
  void set_gyro_rate_notch_freq(float fc, float w);
  void set_accel_lpf_cutoff(float fc);
  void set_accel_notch_freq(float fc, float w);

protected:
  FlightController &firmware_;

  uint64_t last_time_{};
  uint64_t last_acc_update_us_{};
  uint64_t last_extatt_update_us_{};

  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float twoKi = 0;
  float twoKp = 2;
  float integralFBx{}, integralFBy{}, integralFBz{};

  float invSqrt(float x);
  float getW();
  float getX();
  float getY();
  float getZ();

  Attitude state_;
  void reset_state();
  void filtering(float dt);
  matrix::Vector3f gyro_offset_{-0.0240, 0.0142, -0.0065};
  matrix::Vector3f accel_offset_{-0.0016, -0.0070, -0.0170};
  matrix::Vector3f gyro_{0., 0., 0.};
  matrix::Vector3f gyro_prev_{0., 0., 0.};
  matrix::Vector3f gyro_rate_{0., 0., 0.};
  matrix::Vector3f accel_{0., 0., 0.};

  bool GYRO_LPF{true}, GYRO_NOTCH{false};
  LowPassFilter gyro_low_pass_{500.0, 60.0};
  NotchFilter gyro_notch_{500.0f, 120.0f, 0.55};

  bool GYRO_RATE_LPF{true}, GYRO_RATE_NOTCH{false};
  LowPassFilter gyro_rate_low_pass_{500.0, 50.0};
  NotchFilter gyro_rate_notch_{500.0f, 80.0f, 0.55};

  bool ACCEL_LPF{true}, ACCEL_NOTCH{false};
  LowPassFilter accel_low_pass_{500.0, 100.0};
  NotchFilter accel_notch_{500.0f, 120.0f, 0.55};

};
}

#endif /* __QROTOR_FIRMWARE_ESTIMATOR_H__ */