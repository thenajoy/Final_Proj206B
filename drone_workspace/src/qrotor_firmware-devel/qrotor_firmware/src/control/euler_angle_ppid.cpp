
#include "control/euler_angle_ppid.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

EulerAnglePPID::EulerAnglePPID(FlightController &_flightcontroller)
    : AttitudeController(_flightcontroller) {}

EulerAnglePPID::~EulerAnglePPID() = default;

bool EulerAnglePPID::init() {
  Logger::STATUS("EulerAnglePPID Initialized!");
  return true;
}

void EulerAnglePPID::compute(float _dt) {

  this->dt_ = (float)_dt;

  /* kp */
  matrix::Vector3f cmd_ang_vel = -_gains_att.kp().emult(
      firmware_.att_estimator_->state().euler - this->cmd_attitude_.euler);

  if (firmware_.state_machine_.state().mode == StateMachine::MANUAL_CONTROL) {
    // update command yaw-rate to the receiver yaw rate
    cmd_ang_vel(2) = cmd_attitude_.ang_vel(2);
  }
  if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD) {
    cmd_ang_vel += firmware_.mission_planner_->pose_des().ang_vel;
  }
  //  std::cout << "Om_des " <<
  //  firmware_.mission_planner_->pose_des().ang_vel(0)
  //            << " " << firmware_.mission_planner_->pose_des().ang_vel(1) << "
  //            "
  //            << firmware_.mission_planner_->pose_des().ang_vel(2) <<
  //            std::endl;

  /* rate controller implementation */
  rate_pid_rough(cmd_ang_vel);
  //  std::cout << "dt_: " << dt_ << std::endl;
}

void EulerAnglePPID::rate_pid_rough(const matrix::Vector3f &cmd_ang_vel) {

  /* angular rates::control */
  matrix::Vector3f rates_err =
      firmware_.att_estimator_->state().ang_vel - cmd_ang_vel;

  /* apply low-pass filtering to the rates for D-term */
  //    matrix::Vector3f _rates_derv = _lp_rate_derv.apply(rates_err / dt_);
  matrix::Vector3f _rates_derv =
      firmware_.att_estimator_->state().ang_vel_rates;

  /* integrating rates error */
  _rates_integral += rates_err * dt_;
  //    _rates_integral.print();
  _rates_integral = constrain(_rates_integral, RATES_INTEGRAL_LB,
                              RATES_INTEGRAL_UB); // basic anti-windup
  //    _rates_integral.print();

  input_.moment = -_gains_ang_vel.kp().emult(rates_err) +      /* p-term */
                  -_gains_ang_vel.kd().emult(_rates_derv) +    /* d-term */
                  -_gains_ang_vel.kp().emult(_rates_integral); /* i-term */

  if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD) {
    input_.moment += firmware_.mission_planner_->pose_des().moment;
  }

  //  std::cout << "M_des " << firmware_.mission_planner_->pose_des().moment(0)
  //            << " " << firmware_.mission_planner_->pose_des().moment(1) << "
  //            "
  //            << firmware_.mission_planner_->pose_des().moment(2) <<
  //            std::endl;

  //    input_.moment.print();
  //  std::cout << "rdx: " << _rates_derv(0) <<  "rdy: " << _rates_derv(1) <<
  //  "rdz: " << _rates_derv(2)  << std::endl; std::cout << "rix: " <<
  //  _rates_integral(0) <<  "riy: " << _rates_integral(1) <<  "riz: " <<
  //  _rates_integral(2)  << std::endl;
  //  std::cout << "PPID: mx: " <<
  //            input_.moment(0) << "\t my: " << input_.moment(1) << "\t mz: "
  //            << input_.moment(2) << std::endl;
}

void EulerAnglePPID::rate_pid_rosflight() {
  // TODO: update this rate controller using PID class
}

} // namespace qrotor_firmware
