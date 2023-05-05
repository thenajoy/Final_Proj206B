//
// Created by kotaru on 4/27/21.
//
#include "planning/mission_planner.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

MissionPlanner::MissionPlanner(FlightController &_flightcontroller)
    : firmware_(_flightcontroller) {
  state_ = new QuadrotorState();
  sp_flats_ = new QrotorFlats();
}

void MissionPlanner::init() {
  get_vehicle_parameters();
  Logger::STATUS("Mission Planner Initialized!");
}

void MissionPlanner::set_mission(Event _event) {
  switch (mission_config_.type) {
  case mission_type_t::IDLE:
    if (_event == EVENT_REQUEST_TAKEOFF) {
      this->firmware_.state_machine_.change_mode2position_hold();
      this->take_off();
    } else {
      Logger::WARN("Mission Planner: Currently in IDLE, can only TAKEOFF! Use "
                   "transmitter!");
    }
    break;

  case mission_type_t::SETPOINT:
    switch (_event) {
    case EVENT_REQUEST_LAND:this->land();
      break;
    case EVENT_REQUEST_TAKEOFF:Logger::ERROR("Cannot request TAKEOFF in SETPOINT MODE");
      break;
    case EVENT_ACTIVATE_MISSION:this->activate_mission();
      break;
    case EVENT_KILL_MISSION:this->kill_mission();
      break;
    case EVENT_REQUEST_SETPOINT:break;
    }
    break;
  case mission_type_t::WAYPOINTS:
    switch (_event) {
    case EVENT_REQUEST_LAND:this->land();
      break;
    case EVENT_REQUEST_TAKEOFF:Logger::ERROR("Cannot request TAKEOFF in WAYPOINTS MODE");
      break;
    case EVENT_ACTIVATE_MISSION:
      Logger::ERROR(
          "Mission Planner: In WAYPOINTS mode, this has not been implemented!");
      break;
    case EVENT_KILL_MISSION:this->kill_mission();
      break;
    case EVENT_REQUEST_SETPOINT:mission_config_.type = SETPOINT;
      break;
    }
    break;
  case mission_type_t::TRAJECTORY_TRACKING:
    switch (_event) {
    case EVENT_REQUEST_LAND:this->land();
      break;
    case EVENT_REQUEST_TAKEOFF:Logger::ERROR("Cannot request TAKEOFF in TRAJECTORY_TRACKING MODE");
      break;
    case EVENT_ACTIVATE_MISSION:
      Logger::ERROR("Mission Planner: In TRAJECTORY_TRACKING mode, cannot "
                    "activate until complete!");
      break;
    case EVENT_KILL_MISSION:this->kill_mission();
      break;
    case EVENT_REQUEST_SETPOINT:mission_config_.type = SETPOINT;
      break;
    }
    break;

  case mission_type_t::TAKEOFF:
    Logger::WARN(
        "Mission Planner: In TAKEOFF mode, cannot change mode until TAKEOFF COMPLETE!");
    break;

  case mission_type_t::LANDING:
    Logger::WARN(
        "Mission Planner: In LANDING mode, cannot change mode until landed!");
    break;

  default:break;
  }
}

void MissionPlanner::run(const float &t) {
  switch (mission_config_.type) {
  case mission_type_t::IDLE:break;
  case mission_type_t::TAKEOFF:run_takeoff(t);
    break;
  case mission_type_t::SETPOINT:break;
  case mission_type_t::WAYPOINTS:break;
  case mission_type_t::TRAJECTORY_TRACKING:run_trajectory_tracking(t);
    break;
  case mission_type_t::LANDING:run_landing(t);
    break;
  default:break;
  }
}

PoseWithCovariance MissionPlanner::getCmdTrajectory(const float &t) {
  if (mission_config_.type == TRAJECTORY_TRACKING) {

  }
  switch (mission_config_.type) {
  case mission_type_t::IDLE:break;
  case mission_type_t::SETPOINT:break;
  case mission_type_t::WAYPOINTS:break;
  case mission_type_t::TRAJECTORY_TRACKING:run_trajectory_tracking(t);
    break;
  case mission_type_t::LANDING:
    if (mission_config_.time(t) > mission_config_.end_time) {
      mission_config_.type = mission_type_t::IDLE;
    }
    break;
  default:break;
  }
}

void MissionPlanner::set_current_position_to_setpoint() {
  // setting current position to cmd_position
  firmware_.att_controller_->reset_rate_integral();
  matrix::Vector3f sp = firmware_.pos_estimator_->pose().position;
  firmware_.pos_controller_->reset_pos_integral_err();

  Logger::STATUS(
      utils::Cat("Position Setpoint: ", sp(0), "\t", sp(1), "\t", sp(2)));
  set_setpoint(sp(0), sp(1), sp(2));
}

void MissionPlanner::take_off() {
  mission_config_.type = mission_type_t::TAKEOFF;
  mission_config_.start_time = firmware_.get_time();
  mission_config_.end_time = 3;

  firmware_.att_controller_->reset_rate_integral();
  firmware_.pos_controller_->reset_pos_integral_err();

  home_location_ = firmware_.pos_estimator_->pose().position;
  home_takeoff_location_ = home_location_;
  home_takeoff_location_(2) += 1.0;
  Logger::STATUS(utils::Cat("Taking off to ",
                            home_takeoff_location_(0),
                            "\t",
                            home_takeoff_location_(1),
                            "\t",
                            home_takeoff_location_(2)));
}

void MissionPlanner::land() {
  mission_config_.type = mission_type_t::LANDING;
  mission_config_.start_time = firmware_.get_time();
  mission_config_.end_time = 5;

  firmware_.att_controller_->reset_rate_integral();
  firmware_.pos_controller_->reset_pos_integral_err();

  landing_hover_location_ = firmware_.pos_estimator_->pose().position;
  landing_site_ = firmware_.pos_estimator_->pose().position;
  landing_site_(2) = home_location_(2);
  Logger::STATUS(utils::Cat("Landing at: ", landing_site_(0), "\t", landing_site_(1), "\t", landing_site_(2)));
}

void MissionPlanner::activate_mission() {
  Logger::STATUS("Activate Mission!");
  traj_ = new CircularTrajectory();
  matrix::Vector3f sp = firmware_.pos_estimator_->pose().position;
  traj_->p0[0] = flat_params_.center(0);
  traj_->p0[1] = flat_params_.center(1);
  traj_->p0[2] = sp(2);

  traj_->r[0] = flat_params_.radius(0);
  traj_->r[1] = flat_params_.radius(1);

  traj_->phi[0] = flat_params_.phase(0);
  traj_->phi[1] = flat_params_.phase(1);

  mission_config_.start_time = firmware_.get_time();
  mission_config_.end_time = 30;
  mission_config_.type = TRAJECTORY_TRACKING;
}

void MissionPlanner::kill_mission() {
  Logger::WARN("Mission Killed!");
  mission_config_.type = SETPOINT;
  set_current_position_to_setpoint();
}

void MissionPlanner::set_setpoint(const float &px, const float &py,
                                  const float &pz) {

  if (mission_config_.type == IDLE) {
    Logger::WARN("Cannot set a setpoint in IDLE mode! Request TAKEOFF");
  } else {
    reset_flats();
    sp_flats_->p[0] = px;
    sp_flats_->p[1] = py;
    sp_flats_->p[2] = pz;
    flat2state(sp_flats_, state_, pose_des_, m, J);
  }

  //  pose_des_.position.print();
}

void MissionPlanner::run_takeoff(const float &t) {
  if (mission_config_.time(t) <= mission_config_.end_time) {
    matrix::Vector3f sp = home_location_;
    sp += (mission_config_.time(t) / mission_config_.end_time) * (home_takeoff_location_ - home_location_);
    set_setpoint(sp(0), sp(1), sp(2));
  }
  if (mission_config_.time(t) > mission_config_.end_time) {
    Logger::SUCCESS("Take off complete!");
    // changing the trajectory to home-takeoff-location point setpoint
    set_setpoint(home_takeoff_location_(0), home_takeoff_location_(1), home_takeoff_location_(2));
    // mission complete
    mission_config_.type = SETPOINT;
  }
}

void MissionPlanner::run_landing(const float &t) {
  if (mission_config_.time(t) <= mission_config_.end_time) {
    matrix::Vector3f sp = landing_hover_location_;
    sp += (mission_config_.time(t) / mission_config_.end_time) * (landing_site_ - landing_hover_location_);
    set_setpoint(sp(0), sp(1), sp(2));
  } else { // (mission_config_.time(t) > mission_config_.end_time)
    Logger::SUCCESS("Landed!");
    // once reached the goal location
    mission_config_.type = mission_type_t::IDLE;
    firmware_.state_machine_.set_event(StateMachine::EVENT_REQUEST_DISARM);
  }
}

void MissionPlanner::run_trajectory_tracking(const float &t) {

  if (mission_config_.time(t) <= mission_config_.end_time) {
    traj_->run(mission_config_.time(t));
    flat2state(traj_->flats_ptr(), state_, pose_des_, m, J);
  }
  if (mission_config_.time(t) > mission_config_.end_time) {
    // changing the trajectory to end point setpoint
    reset_flats();
    sp_flats_->p[0] = state_->p[0];
    sp_flats_->p[1] = state_->p[1];
    sp_flats_->p[2] = state_->p[2];
    flat2state(sp_flats_, state_, pose_des_, m, J);
    // mission complete
    mission_config_.type = SETPOINT;
  }
}

void MissionPlanner::reset_state() {
  for (int i = 0; i < 3; ++i) {
    state_->p[i] = 0;
    state_->v[i] = 0;
    state_->a[i] = 0;
    state_->da[i] = 0;
    state_->d2a[i] = 0;
    state_->Omega[i] = 0;
    state_->dOmega[i] = 0;
    state_->F[i] = 0;
    state_->M[i] = 0;
  }
  matrix::Matrix3f I;
  I.setIdentity();
  I.copyToColumnMajor(state_->R);
  state_->f = m * 9.8065f;
}

void MissionPlanner::reset_flats() {
  for (int i = 0; i < 3; ++i) {
    sp_flats_->p[i] = 0.f;
    sp_flats_->v[i] = 0.f;
    sp_flats_->a[i] = 0.f;
    sp_flats_->da[i] = 0.f;
    sp_flats_->d2a[i] = 0.f;
    sp_flats_->b1[i] = 0.f;
    sp_flats_->db1[i] = 0.f;
    sp_flats_->d2b1[i] = 0.f;
  }
  sp_flats_->b1[0] = 1.f;
}
void MissionPlanner::set_flat_traj_params(const float &cx,
                                          const float &cy,
                                          const float &cz,
                                          const float &radius,
                                          const float &phase) {
  flat_params_.center(0) = cx;
  flat_params_.center(1) = cy;
  flat_params_.center(2) = cz;

  flat_params_.radius(0) = radius;
  flat_params_.radius(1) = radius;
  flat_params_.radius(2) = 0;

  flat_params_.phase(0) = phase * M_PI_F / 180.0f;
  flat_params_.phase(1) = phase * M_PI_F / 180.0f;
  flat_params_.phase(2) = 0;
}

void MissionPlanner::get_vehicle_parameters() {
  m = firmware_.vehicle_params_.mass_;
  firmware_.att_controller_->inertia_matrix().copyToColumnMajor(J);
}
} // namespace qrotor_firmware
