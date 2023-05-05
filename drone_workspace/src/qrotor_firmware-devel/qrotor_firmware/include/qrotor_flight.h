#ifndef __QROTOR_FIRMWARE_FLIGHT_H__
#define __QROTOR_FIRMWARE_FLIGHT_H__

#include <arpa/inet.h>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <memory>

// qrotor_firmware

#include "board.h"
#include "command_manager.h"
#include "control/attitude_control_fsm.h"
#include "control/controllers.h"
#include "estimation/estimators.h"
#include "log.hpp"
#include "mixer.h"
#include "mixer2.h"
#include "param_server.h"
#include "peripherals/peripherals.h"
#include "planning/mission_planner.h"
#include "sensors.h"
#include "state_machine.h"

namespace qrotor_firmware {

const double LOOPRATE = 500.0;
const double POS_LOOPRATE = 200.0;

class FlightController {
private:
  // time variables
  float t = 0.f;
  float dt = 0.002;
  unsigned long startime, previoustime{}, currenttime;
  float loop_rate_;
  float pos_loop_rate_;
  int pos_iter_, POS_CTRL_COUNT;

  void update_time();

public:
  explicit FlightController(Board &_board);

  void init();
  void run();
  [[noreturn]] void main();
  void update_time(const float &_t, const float &_dt);
  void inner_loop(const float &_dt);
  void outer_loop(const float &_dt) const;

  inline float loop_rate() const { return loop_rate_; }

  inline float get_current_freq() const { return (1 / dt); }
  inline float get_time() const { return t; }
  inline unsigned long get_firmware_time() const { return currenttime; }

  unsigned long clock_micros() const {
    return utils::get_current_time() - startime;
  }
  unsigned long clock_millis() const { return clock_micros() / 1000; }

  /// vehicle parameters
  VehicleParameters vehicle_params_{};

  /// State Machine
  StateMachine state_machine_;

  /// Parameter server
  ParamServer *params_{};

  /// logger
  Logger logger_;

  /// board
  Board &board_;

  /// Sensors
  Sensors sensors_;

  /// Mixer
  Mixer mixer_;

  /// Command Manager
  CommandManager command_manager_;

  /// Mission Planner
  MissionPlanner *mission_planner_{};

  /* position */
  /// pose-estimation
  PoseEstimator *pos_estimator_{};

  /// Position controller
  PositionController *pos_controller_{};

  /// External pose handler (mocap, realsense t265, etc)
  ExternalPoseHandler *ext_pose_handler_{};

  /* attitude */
  /// Attitude Orientation Estimator
  AttitudeEstimator *att_estimator_{};

  /// Attitude Control
  //  AttitudeControlFSM att_ctrl_fsm_; // TODO
  AttitudeController *att_controller_{};
};

} // namespace qrotor_firmware

#endif /* __QROTOR_FIRMWARE_FLIGHT_H__ */
