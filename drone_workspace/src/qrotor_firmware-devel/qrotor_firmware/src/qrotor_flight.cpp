#include "qrotor_flight.h"

namespace qrotor_firmware {

FlightController::FlightController(Board &_board)
    : board_(_board), sensors_(*this), command_manager_(*this),
      state_machine_(*this), mixer_(*this) {

  startime = utils::get_current_time();
  Logger::STATUS(utils::Cat("....................................."));
  currenttime = utils::get_current_time();

  // control loop rates
  loop_rate_ = (float) LOOPRATE;
  dt = 1 / loop_rate_;
  pos_loop_rate_ = (float) POS_LOOPRATE;
  pos_iter_ = 0;
  if (std::remainder(float(loop_rate_), float(pos_loop_rate_)) == 0.0) {
    POS_CTRL_COUNT = int(loop_rate_ / pos_loop_rate_);
  } else {
    POS_CTRL_COUNT = int(std::ceil(float(loop_rate_) / pos_loop_rate_));
    Logger::WARN(
        utils::Cat("Setting position loop delay count to ", POS_CTRL_COUNT));
  }
  // control loop rates

  Logger::STATUS(std::string("Firmware constructed!"));
} // constructor

void FlightController::update_time() {

  previoustime = currenttime;
  currenttime = utils::get_current_time();
  dt = float(currenttime - previoustime) * 1e-6f;

  // sleep-off the extra time
  if (dt < 1 / LOOPRATE)
    usleep(((1.0 / float(LOOPRATE)) - dt) * 1000000);

  currenttime = utils::get_current_time();
  dt = float(currenttime - previoustime) / 1000000.0f;
  t = float(currenttime - startime) / 1000000.0f;
  //    logger_.INFO(utils::Cat("freq: ", 1 / dt_s));

} // FlightController::update_time()

void FlightController::update_time(const float &_t, const float &_dt) {
  this->t = _t;
  this->dt = _dt;
} // FlightController::clock(float &dt_s)

void FlightController::init() {
  Logger::STATUS(std::string("Starting firmware initialization..."));

  // initializing params server TODO find a better way for this
  params_ = new ParamServer();
  params_->init();

  // initializing state machine
  state_machine_.init();

  // logger
  logger_.init();

  // board
  board_.init();
  Logger::STATUS(std::string("Board initialized!"));

  // sensors
  sensors_.init();

  // mixer
  mixer_.init();

  // external pose handler
  ext_pose_handler_ = new RSt265Handler();

  /* position */
  // position estimation
  pos_estimator_ = new PoseEstNaive(*this);
  pos_estimator_->init();

  // position controller
  pos_controller_ = new PositionPID(*this);
//  pos_controller_ = new PositionMPC(*this);
  pos_controller_->init();

  // attitude estimation
  att_estimator_ = new MadgwickAHRS(*this);
  att_estimator_->init();

  // control
  att_controller_ = new EulerAnglePPID(*this);
  //  att_controller_ = new Attitude1GeometricController(*this);
  //  att_controller_ = new AttitudeGeometricClfQP(*this);
  att_controller_->init();

  // command manager
  command_manager_.init();

  // mission planner
  mission_planner_ = new MissionPlanner(*this);
  mission_planner_->init();

  // reset time
  update_time();
  Logger::STATUS(std::string("Firmware initialized!"));
} // FlightController::init()

void FlightController::run() {

  // process sensors
  sensors_.run();
  // state machine
  state_machine_.run();
  // command manager
  command_manager_.run();
  // mission planner
  mission_planner_->run(t);
  // position estimation
  pos_estimator_->run(dt);
  // attitude estimation
  att_estimator_->run(dt);

  // control
  if (state_machine_.state().mode == StateMachine::POSITION_HOLD) {
    // run position controller at 1/3 rate of attitude control
    if (pos_iter_ == POS_CTRL_COUNT) {
      pos_controller_->run(dt);
      pos_iter_ = 0;
    } else {
      pos_iter_++;
    }
  }
  if (state_machine_.state().armed) {
    // attitude control
    att_controller_->run(dt);
  }
  // hardware
  mixer_.run();

} // FlightController::run

[[noreturn]] void FlightController::main() {
  while (true) {
    update_time();
    run();
  }
} // FlightController::main

} // namespace qrotor_firmware
