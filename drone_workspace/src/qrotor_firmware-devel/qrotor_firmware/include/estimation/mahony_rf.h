#ifndef __QROTOR_FIRMWARE_ESTIMATION_MAHONY_RF_H__
#define __QROTOR_FIRMWARE_ESTIMATION_MAHONY_RF_H__
//
// Created by kotaru on 3/10/21.
// Customized from rosflight/firmware/estimator.cpp
//
#include "estimation/attitude_estimator.h"
namespace qrotor_firmware {

class MahonyRF : public AttitudeEstimator {
protected:
  const matrix::Vector3f g_ = {0.0f, 0.0f, 1.0f};

  void run_LPF();
  void set_external_attitude_update();
  matrix::Vector3f accel_correction() const;
  matrix::Vector3f extatt_correction() const;
  matrix::Vector3f smoothed_gyro_measurement();
  static void quaternion_to_dcm(const matrix::Quatf &q, matrix::Vector3f &X,
                         matrix::Vector3f &Y, matrix::Vector3f &Z) ;
  void integrate_angular_rate(matrix::Quaternionf &quat,
                              const matrix::Vector3f &omega,
                              const float dt) const;
  matrix::Vector3f accel_LPF_, gyro_LPF_;
  matrix::Vector3f w1_, w2_;

  bool extatt_update_next_run_{};
  matrix::Quatf q_extatt_;
  matrix::Quatf q;

  unsigned long start_us{};
  unsigned long last_time_{};
  unsigned long last_acc_update_us_{};
  unsigned long last_extatt_update_us_{};

  float alpha_acc{0.5f}, alpha_gyro_xy{0.3f}, alpha_gyro_z{0.3f};
  float PARAM_FILTER_KI{0.01f}, PARAM_FILTER_KP_ACC{0.5f}, PARAM_FILTER_KP_EXT{1.5f};
  int PARAM_FILTER_USE_QUAD_INT = 1;
  int PARAM_INIT_TIME{3000};
  int PARAM_FILTER_USE_MAT_EXP{1};

  bool can_use_extatt() const;

public:
  explicit MahonyRF(FlightController &_flightcontroller);
  ~MahonyRF();

  virtual void init();
  virtual void run(float dt);
};

} // namespace qrotor_firmware

#endif // __QROTOR_FIRMWARE_ESTIMATION_MAHONY_RF_H__