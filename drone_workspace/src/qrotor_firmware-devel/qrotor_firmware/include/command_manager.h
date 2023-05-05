#ifndef __QROTOR_FIRMWARE_COMMAND_MANAGER_H__
#define __QROTOR_FIRMWARE_COMMAND_MANAGER_H__

#include "board.h"
#include "parameters.h"
#include "sensors.h"
#include "trajectories/trajectories.h"

namespace qrotor_firmware {
class FlightController;

class CommandManager {
public:
  explicit CommandManager(FlightController &flightcontroller);
  ~CommandManager();

  void init();
  void run();
  void set_current_position_to_setpoint();

private:
  FlightController &firmware_;
  bool new_ctrl_mode_{};

  const float MAX_ROLL_PITCH_ANGLE = M_PI_3_F; // rad
  const float MAX_YAW_RATE = M_PI_10_F;        // rad/s
  const float MAX_THRUST_FORCE = 20;           // N
  const float MAX_XYZ_POS = 1.0;

  matrix::Vector3f position_sp{0.f, 0.f, 0.f};
  matrix::Vector3f pos_stick_old_{0.f, 0.f, 0.f};

  void map_throttle_to_thrust() const;
  void map_sticks_to_cmd_attitude();
  void map_sticks_to_cmd_position();
  static float linear_mapping(float x, const float &max_x);
  static float quadratic_linear_mapping(float x, const float &max_x);
};

} // namespace qrotor_firmware

#endif // __QROTOR_FIRMWARE_COMMAND_MANAGER_H__
