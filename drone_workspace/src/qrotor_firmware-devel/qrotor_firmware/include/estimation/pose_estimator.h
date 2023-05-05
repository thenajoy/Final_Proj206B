#ifndef __QROTOR_FIRMWARE_POSE_ESTIMATOR_H__
#define __QROTOR_FIRMWARE_POSE_ESTIMATOR_H__
#include "parameters.h"
#include "data_structures.h"

namespace qrotor_firmware {

class FlightController;

class PoseEstimator {
protected:
  FlightController &firmware_;
  PoseWithCovariance pose_{};

public:
  explicit PoseEstimator(FlightController &_flightcontroller);
  ~PoseEstimator();

  // estimator
  virtual void init();
  virtual void run(float dt);
  virtual void time_update(const float &_dt) {}
  virtual void meas_update() {}

  // getters
  inline const PoseWithCovariance &pose() const {
    return pose_;
  }

  // setters
  void set_pose(const PoseWithCovariance& _pose) {
    pose_ = _pose;
  }

};

}

#endif // __QROTOR_FIRMWARE_POSE_ESTIMATOR_H__
