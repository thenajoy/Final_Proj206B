#include "control/position_controller.h"

#include "qrotor_flight.h"

namespace qrotor_firmware {

PositionController::PositionController(FlightController &_flightcontroller)
    : firmware_(_flightcontroller) {
  // default gains
  _gains_pos.set_gains(matrix::Vector3f(4.0, 4.0, 8.0),
                       matrix::Vector3f(3.0, 3.0, 6.0),
                       matrix::Vector3f(0.5, 0.5, 0.5));
  _gains_vel.set_gains(matrix::Vector3f(3.0, 3.0, 6.0),
                       matrix::Vector3f(0.0001, 0.0001, 0.0),
                       matrix::Vector3f(0.0, 0.0, 0.0));
  INTEGRAL_UPDATE = false;

  THRUST_VECTOR_LB = {-10.0, -10.0, 0.0};
  THRUST_VECTOR_UB = {10.0, 10.0, 20.0};
  POS_INTEGRAL_ERR_LB = {-5.0, -5.0, -5.0};
  POS_INTEGRAL_ERR_UB = {5.0, 5.0, 5.0};
  VEL_INTEGRAL_ERR_LB = {-0.10, -0.10, -0.10};
  VEL_INTEGRAL_ERR_UB = {0.10, 0.10, 0.10};
}

PositionController::~PositionController() = default;

void PositionController::init() {
  Logger::STATUS(std::string("PositionController initialized!"));
}

void PositionController::run(float dt) { /* virtual function */
}

void PositionController::set_pos_gains(const matrix::Vector3f &_kp,
                                       const matrix::Vector3f &_kd,
                                       const matrix::Vector3f &_ki) {
  _gains_pos.set_gains(_kp, _kd, _ki);
}
void PositionController::set_vel_gains(const matrix::Vector3f &_kp,
                                       const matrix::Vector3f &_kd,
                                       const matrix::Vector3f &_ki) {
  _gains_vel.set_gains(_kp, _kd, _ki);
}

void PositionController::set_cmd_setpoint(const matrix::Vector3f &_pos) {
  cmd_pose_.reset();
  cmd_pose_.position = _pos;
  Logger::STATUS(
      utils::Cat("setpoint changed to: ", _pos(0), " ", _pos(1), " ", _pos(2)));
}
void PositionController::set_cmd_setpoint(const float px, const float py,
                                          const float pz) {
  cmd_pose_.reset();
  cmd_pose_.position = {px, py, pz};
  Logger::STATUS(utils::Cat("setpoint changed to: ", px, " ", py, " ", pz));
}

void PositionController::set_pid_flag(const bool flag) {
  INTEGRAL_UPDATE = flag;
  reset_pos_integral_err();
  if (flag)
    Logger::STATUS("position PID flag set to TRUE");
  else
    Logger::STATUS("position PID flag set to FALSE");
}

void PositionController::debug() {
  std::cout << "position " << firmware_.pos_estimator_->pose().position(0)
            << " " << firmware_.pos_estimator_->pose().position(1) << " "
            << firmware_.pos_estimator_->pose().position(2) << "\nvelocity "
            << firmware_.pos_estimator_->pose().velocity(0) << " "
            << firmware_.pos_estimator_->pose().velocity(1) << " "
            << firmware_.pos_estimator_->pose().velocity(2) << std::endl;

  std::cout << "p_des " <<
            firmware_.mission_planner_->pose_des().position(0) << " "
            << firmware_.mission_planner_->pose_des().position(1) << " "
            << firmware_.mission_planner_->pose_des().position(2) <<
            std::endl;
  std::cout << "v_des " <<
            firmware_.mission_planner_->pose_des().velocity(0) << " "
            << firmware_.mission_planner_->pose_des().velocity(1) << " "
            << firmware_.mission_planner_->pose_des().velocity(2) <<
            std::endl;
  std::cout << "a_des "
            << firmware_.mission_planner_->pose_des().acceleration(0) << " "
            << firmware_.mission_planner_->pose_des().acceleration(1) << " "
            << firmware_.mission_planner_->pose_des().acceleration(2)
            << std::endl;
}

} // namespace qrotor_firmware
