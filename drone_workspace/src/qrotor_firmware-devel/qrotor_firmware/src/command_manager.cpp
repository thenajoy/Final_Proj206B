#include "command_manager.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

CommandManager::CommandManager(FlightController &flightcontroller)
    : firmware_(flightcontroller) {
  new_ctrl_mode_ = false;
  pos_stick_old_.zero();
}

CommandManager::~CommandManager() = default;

void CommandManager::init() {

  firmware_.state_machine_.set_event(StateMachine::EVENT_INITIALIZED);
  Logger::STATUS(std::string("CommandManager initialized!"));
}

void CommandManager::run() {
  if (firmware_.state_machine_.state().mode==StateMachine::POSITION_HOLD) {
    // check mission planner for this
  } else if (firmware_.state_machine_.state().mode == StateMachine::ATTITUDE_STABILIZED) {
    //
    // ATTITUDE_STABILIZED
    // if switch is in the center then just thrust mode

    CommandManager::map_throttle_to_thrust();
    firmware_.att_controller_->set_cmd_euler(0, 0, 0);
    firmware_.att_controller_->set_cmd_yaw_rate(0);

  } else if (firmware_.state_machine_.state().mode == StateMachine::MANUAL_CONTROL) {
    //
    // MANUAL_CONTROL
    // manual mode if switch is close (max) 2064

    // read throttle value from the radio
    CommandManager::map_throttle_to_thrust();
    CommandManager::map_sticks_to_cmd_attitude();
  }
}

void CommandManager::set_current_position_to_setpoint() {
  // setting current position to cmd_position
  firmware_.att_controller_->reset_rate_integral();
  position_sp = firmware_.pos_estimator_->pose().position;
  Logger::STATUS(utils::Cat("Position Setpoint: ", position_sp(0), "\t",
                            position_sp(1), "\t", position_sp(2)));

  firmware_.pos_controller_->reset_pos_integral_err();
  firmware_.pos_controller_->set_cmd_setpoint(position_sp);
}

void CommandManager::map_throttle_to_thrust() const {
  auto f_in_ms = float(firmware_.sensors_.radio().stick_f());
  float f_in_newtons = 0.12f + ((f_in_ms - RC_MIN_READING) / (RC_MAX_READING - RC_MIN_READING)) *
      float(MAX_THRUST_FORCE - 0.12);
  firmware_.att_controller_->set_thrust(f_in_newtons);
  firmware_.att_controller_->set_thrust_vector(0, 0, f_in_newtons);
}

float CommandManager::linear_mapping(float x, const float &max_x) {
  x = x > 1 ? 1 : x < -1 ? -1 : x;
  return x * max_x;
}

float CommandManager::quadratic_linear_mapping(float x, const float &max_x) {
  x = x > 1 ? 1 : x < -1 ? -1 : x;
  float ang = 0;
  if (-1 <= x && x < -0.5) {
    ang = ((4 * max_x / 3) * x + (max_x / 3));
  } else if (-0.5 <= x && x < 0) {
    ang = -(4 * max_x / 3) * x * x;
  } else if (0 <= x && x < 0.5) {
    ang = (4 * max_x / 3) * x * x;
  } else if (0.5 <= x && x <= 1) {
    ang = (4 * max_x / 3) * x + (-max_x / 3);
  } else {
    ang = x * max_x;
  }
  return ang;
}

void CommandManager::map_sticks_to_cmd_attitude() {
  // mapping sticks to command attitude

  /* cmd_roll */
  float cmd_roll = 0;
  auto roll_in_us = (float) firmware_.sensors_.radio().stick_x();
  // linear-mapping
  //    cmd_roll =  linear_mapping(float(roll_in_us -
  //    firmware_.sensors_.radio().roll.center) /
  //    (firmware_.sensors_.radio().roll.hrange), MAX_ROLL_PITCH_ANGLE);
  // quadratic-linear mapping
  cmd_roll = quadratic_linear_mapping(
      float(roll_in_us - RC_MID_READING) / RC_HALF_RANGE, MAX_ROLL_PITCH_ANGLE);

  /* cmd_pitch */
  float cmd_pitch = 0;
  auto pitch_in_us = (float) firmware_.sensors_.radio().stick_y();
  // linear-mapping
  //    cmd_pitch =  linear_mapping(float(pitch_in_us -
  //    firmware_.sensors_.radio().pitch.center) /
  //    (firmware_.sensors_.radio().pitch.hrange), MAX_ROLL_PITCH_ANGLE);
  // quadratic-linear mapping
  cmd_pitch = quadratic_linear_mapping(float(pitch_in_us - RC_MID_READING) /
                                           RC_HALF_RANGE,
                                       MAX_ROLL_PITCH_ANGLE);

  /* cmd_yaw_rate */
  float cmd_yaw_rate = 0;
  auto yaw_rate_in_us = (float) firmware_.sensors_.radio().stick_z();
  // linear-mapping
  //    cmd_yaw_rate = linear_mapping(float(yaw_rate_in_us -
  //    firmware_.sensors_.radio().yaw.center) /
  //    (firmware_.sensors_.radio().yaw.hrange), MAX_YAW_RATE);
  // quadratic-linear mapping
  cmd_yaw_rate = quadratic_linear_mapping(
      float(yaw_rate_in_us - RC_MID_READING) / RC_HALF_RANGE, MAX_YAW_RATE);

  //    printf("commands: roll: %f, pitch: %f, yaw_rate: %f\n", cmd_roll *
  //    M_RAD_TO_DEG_F, cmd_pitch * M_RAD_TO_DEG_F, cmd_yaw_rate *
  //    M_RAD_TO_DEG_F);
  firmware_.att_controller_->set_cmd_euler(cmd_roll, cmd_pitch, 0);
  firmware_.att_controller_->set_cmd_yaw_rate(cmd_yaw_rate);
}

void CommandManager::map_sticks_to_cmd_position() {
  // mapping sticks to command position

  /* position x */
  float dx = 0;
  auto stick_x = (float) firmware_.sensors_.radio().stick_x();
  dx = linear_mapping(float(stick_x - RC_MID_READING) / (RC_HALF_RANGE),
                      MAX_XYZ_POS);
  if (stick_x > RC_MID_READING) {
    if (stick_x > pos_stick_old_(0)) {
      position_sp(0) += dx;
    }
  } else if (stick_x < RC_MID_READING) {
    if (stick_x < pos_stick_old_(0)) {
      position_sp(0) += dx;
    }
  }
  pos_stick_old_(0) = stick_x;

  /* position y */
  float dy = 0;
  auto stick_y = (float) firmware_.sensors_.radio().stick_y();
  dy = linear_mapping(float(stick_y - RC_MID_READING) / (RC_HALF_RANGE),
                      MAX_XYZ_POS);
  if (stick_y > RC_MID_READING) {
    if (stick_y > pos_stick_old_(1)) {
      position_sp(1) += dy;
    }
  } else if (stick_y < RC_MID_READING) {
    if (stick_y < pos_stick_old_(1)) {
      position_sp(1) += dy;
    }
  }
  pos_stick_old_(1) = stick_y;

  /* cmd_pitch */
  float dz = 0;
  auto stick_z = (float) firmware_.sensors_.radio().stick_z();
  dz = linear_mapping(float(stick_z - pos_stick_old_(2)) / RC_HALF_RANGE,
                      MAX_XYZ_POS);
  position_sp(2) += dz;
  pos_stick_old_(2) = stick_z;

  /* cmd_yaw_rate */
  float cmd_yaw_rate = 0;
  auto yaw_rate_in_us = (float) firmware_.sensors_.radio().stick_z();
  // quadratic-linear mapping
  cmd_yaw_rate = linear_mapping(
      float(yaw_rate_in_us - RC_MID_READING) / RC_HALF_RANGE, MAX_YAW_RATE);

  firmware_.att_controller_->set_cmd_yaw_rate(cmd_yaw_rate);
  Logger::STATUS(utils::Cat("Position Setpoint: ", position_sp(0), "\t",
                            position_sp(1), "\t", position_sp(2)));
  firmware_.pos_controller_->set_cmd_setpoint(position_sp);
}

} // namespace qrotor_firmware
