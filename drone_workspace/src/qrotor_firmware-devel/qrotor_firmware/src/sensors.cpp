#include "sensors.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

/***********************************************************/
/********************* RadioHandler ************************/
/***********************************************************/

RadioHandler::RadioHandler(FlightController &_fcu) : firmware_(_fcu) {
  for (int i = 0; i < 8; ++i) {
    rc_from_ros[i] = 1000;
  }
}

void RadioHandler::init() {

  init_sticks();
  init_switches();
  new_command_ = false;
}

int RadioHandler::read_channel(int _ch) {
  if (firmware_.params_->get(Params::RC_TYPE) == 1)
    return rc_from_ros[_ch];
  else
    return firmware_.board_.read_channel(_ch);
}

void RadioHandler::set_rc_from_ros(const int *_rc) {
  for (int i = 0; i < 8; ++i) {
    rc_from_ros[i] = _rc[i];
  }
}

int RadioHandler::stick(Stick channel) { return stick_values[channel]; }

int RadioHandler::switch_on(Switch channel) { return switch_values[channel]; }

void RadioHandler::init_sticks() {
  sticks[STICK_X].channel = PARAM_RC_X_CHANNEL;
  sticks[STICK_X].one_sided = false;

  sticks[STICK_Y].channel = PARAM_RC_Y_CHANNEL;
  sticks[STICK_Y].one_sided = false;

  sticks[STICK_Z].channel = PARAM_RC_Z_CHANNEL;
  sticks[STICK_Z].one_sided = false;

  sticks[STICK_F].channel = PARAM_RC_F_CHANNEL;
  sticks[STICK_F].one_sided = true;

  //    // radio channels TODO: optimize this assignment
  //    radio_.roll.init(ROLL_CHANNEL);
  //    radio_.pitch.init(PITCH_CHANNEL);
  //    radio_.yaw.init(YAW_CHANNEL);
  //    radio_.thrust.init(THRUST_CHANNEL, 1104, 1924, 1483);
}

void RadioHandler::init_switches() {
  switches[KILL_SWITCH].channel = PARAM_RC_SAFETY_CHANNEL;
  switches[KILL_SWITCH].three_way = false;

  switches[MODE_SWITCH].channel = PARAM_RC_MODE_CHANNEL;
  switches[MODE_SWITCH].three_way = true;

  switches[SWITCH_SW1].channel = PARAM_RC_SW1_CHANNEL;
  switches[SWITCH_SW1].three_way = false;

  switches[SWITCH_SW2].channel = PARAM_RC_SW2_CHANNEL;
  switches[SWITCH_SW2].three_way = false;

  //    radio_.mode.init(MODE_CHANNEL, 964, 2064, 1514);
  //    radio_.safety_switch.init(SAFETY_SWITCH_CHANNEL, 964, 2064, 1514);
  //    radio_.switch1.init(SWITCH1_CHANNEL, 964, 2064, 1514);
  //    radio_.switch2.init(SWITCH2_CHANNEL, 964, 2064, 1514);
}

bool RadioHandler::check_rc_lost() {
  bool failsafe = false;

  // If the board reports that we have lost RC, tell the state manager
  /*Navio cannot distinguish between rc lost */

  // go into failsafe if we get an invalid RC command for any channel
  for (int8_t i = 0; i < PARAM_RC_NUM_CHANNELS; i++) {
    int pwm = read_channel(i);
    if (pwm < 900 || pwm > 2200) {
      failsafe = true;
    }
  }

  if (failsafe)
    // set the RC Lost error flag
    firmware_.state_machine_.set_event(StateMachine::EVENT_RC_LOST);
  else
    // Clear the RC Lost Error
    firmware_.state_machine_.set_event(StateMachine::EVENT_RC_FOUND);

  return failsafe;
}

void RadioHandler::look_for_arm_disarm_signal() {
  uint32_t now_ms = firmware_.clock_millis();
  uint32_t dt = now_ms - prev_time_ms;
  prev_time_ms = now_ms;

  // verify kill switch
  if (firmware_.sensors_.radio().kill_switch() == 1) {
    if (!firmware_.state_machine_.state().armed) { // we are DISARMED
      // if left stick is down and to the right
      if ((firmware_.sensors_.radio().stick_f() < PARAM_ARM_F_THRESHOLD) &&
          (firmware_.sensors_.radio().stick_z() > PARAM_ARM_Z_THRESHOLD)) {
        time_sticks_have_been_in_arming_position_ms += dt;
      } else {
        time_sticks_have_been_in_arming_position_ms = 0;
      }
      if (time_sticks_have_been_in_arming_position_ms > 1000) {
        firmware_.state_machine_.set_event(StateMachine::EVENT_REQUEST_ARM);
        time_sticks_have_been_in_arming_position_ms = 0;
      }
    } else { // we are ARMED
      // if left stick is down and to the left
      if ((firmware_.sensors_.radio().stick_f() < PARAM_ARM_F_THRESHOLD) &&
          (firmware_.sensors_.radio().stick_z() < PARAM_DISARM_Z_THRESHOLD)) {
        time_sticks_have_been_in_arming_position_ms += dt;
      } else {
        time_sticks_have_been_in_arming_position_ms = 0;
      }
      if (time_sticks_have_been_in_arming_position_ms > 1000) {
        firmware_.state_machine_.set_event(StateMachine::EVENT_REQUEST_DISARM);
        time_sticks_have_been_in_arming_position_ms = 0;
      }
    }
  } else { // kill switch turned off:
    if (firmware_.state_machine_.state().armed) {
      // KILL MOTORS
      firmware_.state_machine_.set_event(StateMachine::EVENT_KILL_MOTORS);
    }
  }
}

void RadioHandler::process_switch_positions() {
  if (this->mode_switch() == 2) {
    // Note: this option disabled for now to account for OFFBOARD CONTROL
    //    firmware_.state_machine_.set_mode(
    //        StateMachine::EVENT_REQUEST_POSITION_HOLD);
  } else if (this->mode_switch() == 1) {
    firmware_.state_machine_.set_mode(
        StateMachine::EVENT_REQUEST_ATT_STABILIZED);
  } else {
    firmware_.state_machine_.set_mode(StateMachine::EVENT_REQUEST_MANUAL);
  }

  //  if (this->mission_switch()==1) {
  //    firmware_.state_machine_.set_mode(StateMachine::EVENT_MISSION_GO);
  //  } else {
  //    firmware_.state_machine_.set_mode(StateMachine::EVENT_MISSION_NO_GO);
  //  }

}

///
/// run
///
bool RadioHandler::run() {
  uint32_t now = firmware_.clock_millis();
  // if it has been more than 20ms then look for new RC values and parse them
  if (now - last_rc_receive_time < 20) {
    return false;
  }
  last_rc_receive_time = now;
  // Check for rc lost
  if (check_rc_lost())
    return false;

  // read and normalize stick values
  for (uint8_t channel = 0; channel < static_cast<uint8_t>(STICKS_COUNT);
       channel++) {
    stick_values[channel] = read_channel(sticks[channel].channel); // raw readings
  }

  // read and interpret switch values
  for (uint8_t channel = 0; channel < static_cast<uint8_t>(SWITCHES_COUNT);
       channel++) {
    int readings = read_channel(switches[channel].channel);
    if (switches[channel].three_way) {
      if (readings > 1790)
        switch_values[channel] = 2;
      else if (readings > 1240)
        switch_values[channel] = 1;
      else
        switch_values[channel] = 0;
    } else {
      if (readings > 1514)
        switch_values[channel] = 1;
      else
        switch_values[channel] = 0;
    }
  }
  // Look for arming and disarming signals
  look_for_arm_disarm_signal();
  // Process the different switch positions
  process_switch_positions();

  // Signal to the mux that we need to compute a new combined command
  new_command_ = true;
  return true;
}

/***********************************************************/
/*********************** SENSORS ***************************/
/***********************************************************/

Sensors::Sensors(FlightController &_flightcontroller)
    : firmware_(_flightcontroller), radio_(_flightcontroller) {}

void Sensors::init() {
  // initialize sensors on the board
  firmware_.board_.sensors_init();
  // initialize radio receiver
  radio_.init();
  // imu update
  update_imu();
  //  imu_calibration();
  //  [STATUS] [27901041]: Calibrating imu...
  //  Gyro offsets are: -0.040453 0.016198 -0.007171
  //  Accel offsets are: 0.202502 -0.154761 -0.266379
  //[SUCCESS] [28953652]: .. imu calibration complete!

  Logger::STATUS(std::string("Sensors initialized!"));
}

void Sensors::update_imu() {
  float ax{}, ay{}, az{};
  float gx{}, gy{}, gz{};
  float mx{}, my{}, mz{};
  // reading imu readings
  firmware_.board_.read_accel_gyro_mag(ax, ay, az, gx, gy, gz, mx, my, mz);
  imu_.timestamp_us = utils::get_current_time();
  imu_.accel = {ax - accel_offset_(0), ay - accel_offset_(1),
                az - accel_offset_(2)};
  imu_.gyro = {gx - gyro_offset_(0), gy - gyro_offset_(1),
               gz - gyro_offset_(2)};
  imu_.mag = {mx, my, mz};
}

void Sensors::update_radio() { radio_.run(); }

void Sensors::imu_calibration() {
  Logger::STATUS("Calibrating imu...");
  matrix::Vector3f goffset = {0.0, 0.0, 0.0};
  matrix::Vector3f aoffset = {0.0, 0.0, 0.0};
  float _ax, _ay, _az, _gx, _gy, _gz;
  for (int i = 0; i < 100; i++) {
    firmware_.board_.read_accel_gyro(_ax, _ay, _az, _gx, _gy, _gz);
    _gx *= 180 / M_PI;
    _gy *= 180 / M_PI;
    _gz *= 180 / M_PI;

    goffset(0) += _gx * 0.0175f;
    goffset(1) += _gy * 0.0175f;
    goffset(2) += _gz * 0.0175f;

    aoffset(0) += _ax;
    aoffset(1) += _ay;
    aoffset(2) += (_az - float(G_SI));
    usleep(10000);
  }
  goffset(0) /= 100.0;
  goffset(1) /= 100.0;
  goffset(2) /= 100.0;
  gyro_offset_ = {goffset(0), goffset(1), goffset(2)};

  aoffset(0) /= 100;
  aoffset(1) /= 100;
  aoffset(2) /= 100;
  accel_offset_ = {aoffset(0), aoffset(1), aoffset(2)};
  printf("Gyro offsets are: %f %f %f\n", goffset(0), goffset(1), goffset(2));
  printf("Accel offsets are: %f %f %f\n", aoffset(0), aoffset(1), aoffset(2));
  Logger::SUCCESS(".. imu calibration complete!");
}

void Sensors::read_battery() {
  int v, c;
  firmware_.board_.read_adc(v, c);
  //  std::cout << "v: " << v << " c: " << c << " " <<
  //  firmware_.params_->get(Params::VOLTAGE_MULTIPLIER) << " "
  //            << firmware_.params_->get(Params::CURRENT_COEFFICIENT) <<
  //            std::endl;
  voltage_ =
      (float(v) / 1000.f) * firmware_.params_->get(Params::VOLTAGE_MULTIPLIER);
  current_ =
      (float(c) / 1000.f) * firmware_.params_->get(Params::CURRENT_COEFFICIENT);
}

void Sensors::run() {
  // radio readings
  update_radio();
  // imu readings
  update_imu();
  // barometer readings

  // battery readings
  read_battery();
}

} // namespace qrotor_firmware
