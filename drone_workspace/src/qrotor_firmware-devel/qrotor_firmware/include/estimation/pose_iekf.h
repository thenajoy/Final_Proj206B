#ifndef QROTOR_FIRMWARE_POSE_IEKF_H
#define QROTOR_FIRMWARE_POSE_IEKF_H
#include "estimation/eigen/invariant_ekf.h"
#include "estimation/matlab/invariant_ekf_m.h"
#include "estimation/pose_estimator.h"
#include "lie_algebra.h"
#include "filters/low_pass_filter.hpp"
#include "filters/notch_filter.hpp"

namespace qrotor_firmware {
namespace eest = eigen_estimation;

class PoseInvariantEKF : public PoseEstimator {
private:
  matrix::Vector3f g_;
  matrix::Vector3f w{0., 0., 0.};
  matrix::Vector3f a{0., 0., G_SI}; // gyro, accel

  matrix::Matrix<float, 9, 9> A; // linear dynamics

  bool verbose_ = false;
  bool iekf_initialized = false;

  eest::InvariantEKFf invariant_ekf_{false};
  matlab::InvariantEKFm iekf_{} ;
  eest::ImuMeasurementf imu_meas_;
  eest::RigidbodyStatef pose_meas_;

  void externalPoseWithCovariance_to_RigidbodyState();
  void estimatedRigidbodyState_to_PoseWithCovariance();


  LowPassFilter accel_low_pass_{500.0, 60.0};
  LowPassFilter gyro_low_pass_{500.0, 50.0};
public:
  explicit PoseInvariantEKF(FlightController &_flightcontroller);
  ~PoseInvariantEKF();

  virtual void init() override;
  virtual void run(float dt) override;
  virtual void time_update(const float &_dt) override;
  virtual void meas_update() override;
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_POSE_IEKF_H
