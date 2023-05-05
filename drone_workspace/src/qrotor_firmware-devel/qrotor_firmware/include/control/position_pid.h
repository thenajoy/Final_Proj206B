#ifndef QROTOR_FIRMWARE_POSITION_PID_H
#define QROTOR_FIRMWARE_POSITION_PID_H

#include "Matrix/matrix/math.hpp"
#include "control/position_controller.h"

namespace qrotor_firmware {

class PositionPID : public PositionController {
protected:

    void _positionControl();
    void _velocityControl();
    void _accelerationControl();
public:
  explicit PositionPID(FlightController &_flightcontroller);
  ~PositionPID();

  virtual void run(float dt) override;

  /**
   * Implements PID on position to compute the command thrust vector
   * @param dt
   */
  void run_position_pid();

  /**
   * Implement PID on velocity control
   * @param dt
   */
  void run_velocity_control();
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_POSITION_PID_H
