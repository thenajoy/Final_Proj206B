//
// Created by kotaru on 4/9/21.
//

#ifndef __QROTOR_FIRMWARE_ATTITUDE_GEOMETRIC_CONTROLLER_H_
#define __QROTOR_FIRMWARE_ATTITUDE_GEOMETRIC_CONTROLLER_H_

#include "control/attitude_controller.h"

namespace qrotor_firmware {
class FlightController;
class AttitudeGeometricController : public AttitudeController {
protected:
public:
  explicit AttitudeGeometricController(FlightController &_flightController);
  ~AttitudeGeometricController();

  bool init() override;
  void compute(float _dt) override;

  matrix::Vector3f cmd_attitude_to_rate_sp();
  matrix::Vector3f rate_control(const matrix::Vector3f &rate,
                                const matrix::Vector3f &rate_sp,
                                const matrix::Vector3f &angular_accel);
  void updateIntegral(const matrix::Vector3f &rate_error);
};

} // namespace qrotor_firmware

#endif //__QROTOR_FIRMWARE_ATTITUDE_GEOMETRIC_CONTROLLER_H_
