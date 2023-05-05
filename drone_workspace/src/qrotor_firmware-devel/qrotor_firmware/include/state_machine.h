#ifndef QROTOR_FIRMWARE_STATE_MACHINE_H
#define QROTOR_FIRMWARE_STATE_MACHINE_H
/* Original Source:
 * https://github.com/rosflight/firmware/blob/master/include/state_manager.h */

#include "parameters.h"
#include <cstddef>
#include <cstdint>

namespace qrotor_firmware {

class FlightController;

class StateMachine {
public:
  enum Event {
    /* arm-disarming */
    EVENT_INITIALIZED,
    EVENT_REQUEST_ARM,
    EVENT_REQUEST_DISARM,
    EVENT_RC_LOST,
    EVENT_RC_FOUND,
    EVENT_ERROR,
    EVENT_NO_ERROR,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_CALIBRATION_FAILED,
    EVENT_KILL_MOTORS,

    /* control modes */
    EVENT_REQUEST_MANUAL,
    EVENT_REQUEST_ATT_STABILIZED,
    EVENT_REQUEST_POSITION_HOLD,
    EVENT_REQUEST_OFFBOARD,
    EVENT_MISSION_GO,
    EVENT_MISSION_NO_GO,
  };
  enum : uint16_t {
    ERROR_NONE = 0x0000,
    ERROR_INVALID_MIXER = 0x0001,
    ERROR_IMU_NOT_RESPONDING = 0x0002,
    ERROR_RC_LOST = 0x0004,
    ERROR_UNHEALTHY_ESTIMATOR = 0x0008,
    ERROR_TIME_GOING_BACKWARDS = 0x0010,
    ERROR_UNCALIBRATED_IMU = 0x0020,
    ERROR_INVALID_FAILSAFE = 0x0040,
  };

  typedef enum {
    POSITION_HOLD,       // Channel is directly controlling throttle max/1000
    MANUAL_CONTROL,      // manual control using rc transmitter
    ATTITUDE_STABILIZED, // Position Hold
    PASSTHROUGH,         // Channel directly passes PWM input to the mixer
    OFFBOARD /// thrust commands coming from offboard control
  } Mode;

  struct MachineState {
    bool armed;
    bool failsafe;
    bool error;
    uint16_t error_codes;
    Mode mode;
  };

private:
  FlightController &firmware_;
  MachineState state_{};

  enum FsmState {
    FSM_STATE_INIT,
    FSM_STATE_PREFLIGHT,
    FSM_STATE_ARMED,
    FSM_STATE_ERROR,
    FSM_STATE_FAILSAFE,
    FSM_STATE_CALIBRATING
  };
  FsmState fsm_state_;

  void process_errors();
  void update_led();

  unsigned long next_led_blink_ms_ = 0;
  unsigned long next_arming_error_msg_ms_ = 0;

  uint32_t hardfault_count_ = 0;

public:
  explicit StateMachine(FlightController &_flightcontroller);
  ~StateMachine();
  void init();
  void run();

  inline const MachineState &state() const { return state_; }

  void change_mode2position_hold() {
    state_.mode = POSITION_HOLD;
  }

  void set_event(Event event);
  void set_mode(Event event);
  void set_error(uint16_t error);
  void clear_error(uint16_t error);
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_STATE_MACHINE_H
