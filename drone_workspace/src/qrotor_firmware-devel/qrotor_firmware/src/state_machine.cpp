#include "state_machine.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

StateMachine::StateMachine(FlightController &_flightcontroller)
    : firmware_(_flightcontroller) {
  fsm_state_ = FSM_STATE_INIT;
  state_.armed = false;
  state_.error = false;
  state_.failsafe = false;
  state_.error_codes = 0x00;
  state_.mode = Mode::MANUAL_CONTROL;
}

StateMachine::~StateMachine() = default;

void StateMachine::init() {
  set_event(EVENT_INITIALIZED);
  process_errors();

  // initialize led
  firmware_.board_.led1_off();
  qrotor_firmware::Logger::STATUS(std::string("StateMachine initialized!"));
}

void StateMachine::run() {
  process_errors();
  update_led();
}

void StateMachine::set_error(uint16_t error) {
  // Set the error code
  state_.error_codes |= error;

  // Tell the FSM that we have had an error change
  process_errors();
}

void StateMachine::clear_error(uint16_t error) {
  // If this error code was set,
  if (state_.error_codes & error) {
    // Clear the error code
    state_.error_codes &= ~(error);

    // If there are no errors, tell the FSM
    process_errors();

    // TODO: fix the missing comm manager
  }
}

void StateMachine::set_event(StateMachine::Event event) {
  FsmState start_state = fsm_state_;
  uint16_t start_errors = state_.error_codes;
  switch (fsm_state_) {
    case FSM_STATE_INIT:
      if (event == EVENT_INITIALIZED) {
        fsm_state_ = FSM_STATE_PREFLIGHT;
      }
      break;

    case FSM_STATE_PREFLIGHT:
      switch (event) {
        case EVENT_REQUEST_ARM:
          // require low RC throttle to arm
          if (firmware_.sensors_.radio().stick_f() < PARAM_ARM_F_THRESHOLD) {
            if (PARAM_CALIBRATE_GYRO_ON_ARM) {
              fsm_state_ = FSM_STATE_CALIBRATING;
              qrotor_firmware::Sensors::start_gyro_calibration();
            } else {
              state_.armed = true;
              Logger::SUCCESS(utils::Cat("ARMED!"));
              fsm_state_ = FSM_STATE_ARMED;
            }
          } else {
            Logger::ERROR(utils::Cat("Cannot arm with RC throttle high!"));
          }
          break;
        case EVENT_REQUEST_DISARM:state_.armed = false;
          Logger::SUCCESS(utils::Cat("DISARMED!"));
          fsm_state_ = FSM_STATE_PREFLIGHT;
          break;
        case EVENT_KILL_MOTORS:state_.armed = false;
          Logger::ERROR(std::string("KILL MOTORS"));
          if (state_.error)
            fsm_state_ = FSM_STATE_ERROR;
          else
            fsm_state_ = FSM_STATE_PREFLIGHT;
          break;
        default:break;
      }
      break;

    case FSM_STATE_ARMED:
      switch (event) {
        case EVENT_KILL_MOTORS:state_.armed = false;
          qrotor_firmware::Logger::ERROR(std::string("KILL MOTORS"));
          if (state_.error)
            fsm_state_ = FSM_STATE_ERROR;
          else
            fsm_state_ = FSM_STATE_PREFLIGHT;
          break;

        case EVENT_RC_LOST:state_.armed = false;
          Logger::ERROR(std::string("RC LOST"));
          fsm_state_ = FSM_STATE_PREFLIGHT;
          break;

        case EVENT_REQUEST_DISARM:state_.armed = false;
          Logger::SUCCESS(utils::Cat("DISARMED!"));
          fsm_state_ = FSM_STATE_PREFLIGHT;
          break;

        case EVENT_ERROR:state_.error = true;
          break;

        case EVENT_NO_ERROR:state_.error = false;
          break;

        default:break;
      }
      break;

    default:break;
  }

  //    // If there has been a change, then report it to the user
  //    if (start_state != fsm_state_ || state_.error_codes != start_errors)
  //        firmware_.logger_.STATUS("update");
}

void StateMachine::set_mode(Event event) {
  switch (state_.mode) {
    case ATTITUDE_STABILIZED:
      if (event == StateMachine::EVENT_REQUEST_POSITION_HOLD) {
        Logger::STATUS(utils::Cat("POSITION HOLD"));
        firmware_.mission_planner_->set_current_position_to_setpoint();
        state_.mode = POSITION_HOLD;

      } else if (event == StateMachine::EVENT_REQUEST_MANUAL) {
        Logger::STATUS(utils::Cat("MANUAL CONTROL"));
        state_.mode = MANUAL_CONTROL;
      } else if (event == StateMachine::EVENT_REQUEST_OFFBOARD) {
        Logger::STATUS(utils::Cat("OFFBOARD CONTROL"));
        state_.mode = OFFBOARD;
      }
      break;

    case MANUAL_CONTROL:
      if (event == StateMachine::EVENT_REQUEST_ATT_STABILIZED) {
        Logger::STATUS(utils::Cat("ATTITUDE STABILIZED"));
        state_.mode = ATTITUDE_STABILIZED;

      } else if (event == StateMachine::EVENT_REQUEST_POSITION_HOLD) {
        Logger::STATUS(utils::Cat("POSITION HOLD"));
        firmware_.mission_planner_->set_current_position_to_setpoint();
        state_.mode = POSITION_HOLD;
      } else if (event == StateMachine::EVENT_REQUEST_OFFBOARD) {
        Logger::STATUS(utils::Cat("OFFBOARD CONTROL"));
        state_.mode = OFFBOARD;
      }
      break;

    case POSITION_HOLD:
      if (event == StateMachine::EVENT_REQUEST_MANUAL) {
        Logger::STATUS(utils::Cat("MANUAL CONTROL"));
        state_.mode = MANUAL_CONTROL;
      } else if (event == StateMachine::EVENT_REQUEST_ATT_STABILIZED) {
        Logger::STATUS(utils::Cat("ATTITUDE STABILIZED"));
        state_.mode = ATTITUDE_STABILIZED;
      } else if (event == StateMachine::EVENT_REQUEST_OFFBOARD) {
        Logger::STATUS(utils::Cat("OFFBOARD CONTROL"));
        state_.mode = OFFBOARD;
      }
      break;

    case OFFBOARD:
      if (event == StateMachine::EVENT_REQUEST_MANUAL) {
        Logger::STATUS(utils::Cat("MANUAL CONTROL"));
        state_.mode = MANUAL_CONTROL;
      } else if (event == StateMachine::EVENT_REQUEST_ATT_STABILIZED) {
        Logger::STATUS(utils::Cat("ATTITUDE STABILIZED"));
        state_.mode = ATTITUDE_STABILIZED;
      } else if (event == StateMachine::EVENT_REQUEST_POSITION_HOLD) {
        Logger::STATUS(utils::Cat("POSITION HOLD"));
        firmware_.mission_planner_->set_current_position_to_setpoint();
        state_.mode = POSITION_HOLD;
      }
      break;

    case PASSTHROUGH:break;
  }
}

void StateMachine::process_errors() {
  // if (state_.error_codes)
  //     set_event(EVENT_ERROR);
  // else
  //     set_event(EVENT_NO_ERROR);
}

void StateMachine::update_led() {
  // blink fast if in failsafe
  if (state_.failsafe) {
    if (next_led_blink_ms_ < firmware_.clock_millis()) {
      firmware_.board_.led1_toggle();
      next_led_blink_ms_ = firmware_.clock_millis() + 100;
    }
  }
    // blink slowly if in error
  else if (state_.error) {
    if (next_led_blink_ms_ < firmware_.clock_millis()) {
      firmware_.board_.led1_toggle();
      next_led_blink_ms_ = firmware_.clock_millis() + 500;
    }
  }
    // off if disarmed, on if armed
  else if (!state_.armed)
    firmware_.board_.led1_off();
  else
    firmware_.board_.led1_on();
}

} // namespace qrotor_firmware
