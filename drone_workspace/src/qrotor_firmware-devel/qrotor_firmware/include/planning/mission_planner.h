//
// Created by kotaru on 4/27/21.
//

#ifndef QROTOR_FIRMWARE_PLANNER_H
#define QROTOR_FIRMWARE_PLANNER_H

#include <queue>

#include "Matrix/matrix/math.hpp"
#include "data_structures.h"
#include "log.hpp"
#include "parameters.h"
#include "planning/flatness.hpp"
#include "trajectories/base_trajectory.hpp"

namespace qrotor_firmware {
class FlightController;
class MissionPlanner {
public:
  enum mission_type_t {
    IDLE,
    SETPOINT,
    WAYPOINTS,
    TRAJECTORY_TRACKING,
    LANDING,
    TAKEOFF
  };

  enum Event {
    EVENT_REQUEST_LAND,
    EVENT_REQUEST_TAKEOFF,
    EVENT_ACTIVATE_MISSION,
    EVENT_KILL_MISSION,
    EVENT_REQUEST_SETPOINT
  };

  class MissionConfig {
  public:
    /// mission type
    mission_type_t type{mission_type_t::IDLE};
    /// trajectory start time
    float start_time{0.f};
    /// trajectory current time
    float end_time{0.f};
    /// mission time
    float time(const float &t) const { return t - start_time; }
  };

  struct FlatTrajectoryParams {
    matrix::Vector3f center{0., 0., 0.};
    matrix::Vector3f radius{1., 1., 0.};
    matrix::Vector3f phase{0., 0., 0.};
  };

protected:
  /// Firmware
  FlightController &firmware_;

  /// System Parameters
  float m{0.85f};
  float J[9]{};

  /// Mission
  MissionConfig mission_config_;

  /// Setpoint
  QrotorFlats *sp_flats_ = nullptr;
  /// FlatVariables
  Trajectory *traj_ = nullptr;
  FlatTrajectoryParams flat_params_;

  /// Home position
  matrix::Vector3f home_location_, home_takeoff_location_, landing_hover_location_, landing_site_;

  /// state
  QuadrotorState *state_ = nullptr;
  /// Command pose (setpoint/trajectories)
  PoseWithCovariance pose_des_;

  /// mission tasks
  void run_trajectory_tracking(const float &t);
  void run_takeoff(const float &t);
  void run_landing(const float &t);

  /// in flight flag
  bool is_in_flight_ = false;

public:
  explicit MissionPlanner(FlightController &_flightcontroller);
  ~MissionPlanner() = default;

  virtual void init();
  virtual void run(const float &t);

  ///
  PoseWithCovariance getCmdTrajectory(const float &t);

  /// updaters
  void reset_state();
  void reset_flats();
  void get_vehicle_parameters();

  /// mission controls
  void set_mission(Event _event);
  void take_off();
  void land();
  void return_home() { Logger::WARN("Not implemented"); }
  void kill_mission();
  void activate_mission();
  void set_current_position_to_setpoint();
  void set_setpoint(const float &px, const float &py, const float &pz);

  /* setters */
  void set_flat_traj_params(const float &cx, const float &cy, const float &cz, const float &radius, const float &phase);

  /* getters */
  /**
   * Variable to securely access command pose
   * @return
   */
  inline const PoseWithCovariance &pose_des() const { return pose_des_; }

  std::queue<PoseWithCovariance> xd;

  /**
   * Returns the state of flight of the mission planner mode
   * @return
   */
  bool is_in_flight() const {
    return !(mission_config_.type == mission_type_t::IDLE);
  }

};
} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_PLANNER_H
