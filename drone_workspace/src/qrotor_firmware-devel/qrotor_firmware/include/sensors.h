#ifndef __QROTOR_FIRMWARE_SENSORS_H__
#define __QROTOR_FIRMWARE_SENSORS_H__

#include "parameters.h"
#include "data_structures.h"
#include "state_machine.h"
#include "log.hpp"

#include <stdint.h>

namespace qrotor_firmware {

class FlightController;

class RadioHandler {

public:
  enum Stick {
    STICK_X,
    STICK_Y,
    STICK_Z,
    STICK_F,
    STICKS_COUNT
  };

  enum Switch {
    KILL_SWITCH,
    MODE_SWITCH,
    SWITCH_SW1,
    SWITCH_SW2,
    SWITCHES_COUNT
  };
  explicit RadioHandler(FlightController &_fcu);
  void init();
  int stick(Stick channel);
  int switch_on(Switch channel);
  bool run();

  inline int stick_f() const {
    return stick_values[STICK_F];
  }
  inline int stick_x() const {
    return stick_values[STICK_X];
  }
  inline int stick_y() const {
    return stick_values[STICK_Y];
  }
  inline int stick_z() const {
    return stick_values[STICK_Z];
  }
  inline int kill_switch() const {
    return switch_values[KILL_SWITCH];
  }
  inline int mode_switch() const {
    return switch_values[MODE_SWITCH];
  }
  inline int mission_switch() const {
    return switch_values[SWITCH_SW2];
  }

  int read_channel(int _ch);
  void set_rc_from_ros(const int *_rc);

private:
  FlightController &firmware_;

  typedef struct {
    uint8_t channel;
    bool three_way;
  } rc_switch_config_t;

  typedef struct {
    uint8_t channel;
    bool one_sided;

  } rc_stick_config_t;

  bool new_command_{};

  uint32_t time_of_last_stick_deviation = 0;
  uint32_t time_sticks_have_been_in_arming_position_ms = 0;
  uint32_t prev_time_ms = 0;
  uint32_t last_rc_receive_time = 0;

  rc_stick_config_t sticks[STICKS_COUNT]{};
  rc_switch_config_t switches[SWITCHES_COUNT]{};

  volatile int switch_values[SWITCHES_COUNT]{};
  volatile int stick_values[STICKS_COUNT]{};
  void init_switches();
  void init_sticks();
  bool check_rc_lost();
  void look_for_arm_disarm_signal();
  void process_switch_positions();

  int rc_from_ros[8];
};

class Sensors {

private:
  FlightController &firmware_;
  RadioHandler radio_;
  IMURaw imu_;
  // imu variables
  matrix::Vector3f gyro_offset_, accel_offset_;
  float accel_scaling_factor_{};

  float voltage_ = 0.f;
  float current_ = 0.f;

public:
  struct Data {
    // turbomath::Vector accel = {0, 0, 0};
    // turbomath::Vector gyro = {0, 0, 0};
    // turbomath::Quaternion fcu_orientation = {1, 0, 0, 0};
    float imu_temperature = 0;
    uint64_t imu_time = 0;
  };
  explicit Sensors(FlightController &_flightcontroller);
  inline const RadioHandler &radio() const {
    return radio_;
  }
  inline const IMURaw &imu() const {
    return imu_;
  }

  void init();
  void run();
  void update_imu();
  void update_radio();
  static void start_gyro_calibration() {
    std::cout << "Not Implemented!" << std::endl;
  }
  void imu_calibration();
  void read_battery();

  // setters
  void set_accel_offset(const float &_ax, const float &_ay, const float &_az) {
    Logger::WARN(utils::Cat("Updating accel offsets", _ax, " ", _ay, " ", _az));
    accel_offset_(0) = _ax;
    accel_offset_(1) = _ay;
    accel_offset_(2) = _az;
  }
  void set_gyro_offset(const float &_gx, const float &_gy, const float &_gz) {
    Logger::WARN(utils::Cat("Updating gyro offsets", _gx, " ", _gy, " ", _gz));
    gyro_offset_(0) = _gx;
    gyro_offset_(1) = _gy;
    gyro_offset_(2) = _gz;
  }
  void set_rc_from_ros(const int *_rc) {
    radio_.set_rc_from_ros(_rc);
  }

  // getters
  matrix::Vector3f accel_offset() const {
    return accel_offset_;
  }
  matrix::Vector3f gyro_offset() const {
    return gyro_offset_;
  }
  float voltage() const {
    return voltage_;
  }
  float current() const {
    return current_;
  }

};

} // namespace qrotor_firmware

#endif /* __QROTOR_FIRMWARE_SENSORS_H__ */
