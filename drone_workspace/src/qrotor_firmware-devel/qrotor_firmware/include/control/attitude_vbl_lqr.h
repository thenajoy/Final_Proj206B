#ifndef QROTOR_FIRMWARE_ATTITUDE_VBL_LQR_H
#define QROTOR_FIRMWARE_ATTITUDE_VBL_LQR_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <qpOASES.hpp>

#include "control/attitude_controller.h"
namespace qrotor_firmware {

class AttitudeVariationLQR : public AttitudeController {
private:
  matrix::Matrix3f inertia_scaled;
  matrix::Matrix3f kR;
  matrix::Matrix3f kOm;

  /// storage variables for errors and such
  matrix::Vector3f eR, eOmega;

public:
  AttitudeVariationLQR(FlightController &_flightController);
  ~AttitudeVariationLQR();

  virtual bool init() override;
  virtual void compute(float _dt) override;
  void thrust_vector_to_cmd_rotation();
  void compute_error_vectors();
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_ATTITUDE_VBL_LQR_H
