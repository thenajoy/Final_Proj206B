#ifndef QROTOR_FIRMWARE_POSE_EST_NAIVE_H_
#define QROTOR_FIRMWARE_POSE_EST_NAIVE_H_
#include "estimation/pose_estimator.h"
#include "mathlib/math/filter/LowPassFilter2pVector3f.hpp"

namespace qrotor_firmware {

class PoseEstNaive : public PoseEstimator {
private:
  /// velocity related terms
  math::LowPassFilter2pVector3f _lpf_velocity{500, 50};

  /**
   * function to compute the initial tracking
   * camera offset in the mocap setup
   */
  void compute_tracking_camera_init_offset();
  PoseWithCovariance tcam_init_offset_{};
  matrix::Quatf mocap_tcam_quat_offset{};

  bool is_initialized = false;

public:
  explicit PoseEstNaive(FlightController &_flightcontroller);
  ~PoseEstNaive();

  void init() override;
  void run(float dt) override;
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_POSE_EST_NAIVE_H_
