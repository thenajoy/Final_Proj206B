
#include "control/euler_pid.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

EulerPID::EulerPID(FlightController &_flightcontroller)
    : AttitudeController(_flightcontroller) {

  printf("EulerPID\n");
}

EulerPID::~EulerPID() = default;

void EulerPID::compute(float _dt) {

  this->dt_ = _dt;
  /* kp */
  matrix::Vector3f cmd_ang_vel; //  = -_gains_att.kp().emult(firmware_.att_estimator_.state().euler - this->cmd_attitude_.euler);
  cmd_ang_vel.setZero();
  if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD) {
    // update command yaw-rate to the receiver yaw rate
    cmd_ang_vel(2) = cmd_attitude_.ang_vel(2);
  }


  /* angular rates::control */
  matrix::Vector3f rates_err = firmware_.att_estimator_->state().ang_vel - cmd_ang_vel;

  /* apply low-pass filtering to the rates for D-term */
  //    matrix::Vector3f _rates_derv = _lp_rate_derv.apply(rates_err / dt_);
  matrix::Vector3f _rates_derv = firmware_.att_estimator_->state().ang_vel_rates;

//  /* integrating rates error */
//  _rates_integral += rates_err * dt_;
//  //    _rates_integral.print();
//  _rates_integral =
//      constrain(_rates_integral, RATES_INTEGRAL_LB, RATES_INTEGRAL_UB); // basic anti-windup
//  //    _rates_integral.print();

  input_.moment = -_gains_att.kp().emult(firmware_.att_estimator_->state().euler - this->cmd_attitude_.euler) + /* p-term */
      -_gains_att.kd().emult(rates_err); //+       /* d-term */
//      -_gains_att.kp().emult(_rates_integral);    /* i-term */

  /* rate controller implementation */
//  rate_pid_rough(cmd_ang_vel);
//  std::cout << "dt_: " << dt_ << std::endl;
}

void EulerPID::rate_pid_rough(matrix::Vector3f cmd_ang_vel) {

//  /* angular rates::control */
//  matrix::Vector3f rates_err = firmware_.att_estimator_.state().ang_vel - cmd_ang_vel;
//
//  /* apply low-pass filtering to the rates for D-term */
//  //    matrix::Vector3f _rates_derv = _lp_rate_derv.apply(rates_err / dt_);
//  matrix::Vector3f _rates_derv = firmware_.att_estimator_.state().ang_vel_rates;
//
//  /* integrating rates error */
//  _rates_integral += rates_err * dt_;
//  //    _rates_integral.print();
//  _rates_integral =
//      constrain(_rates_integral, RATES_INTEGRAL_LB, RATES_INTEGRAL_UB); // basic anti-windup
//  //    _rates_integral.print();
//
//  input_.moment = -_gains_ang_vel.kp().emult(rates_err) +         /* p-term */
//      -_gains_ang_vel.kd().emult(_rates_derv) +       /* d-term */
//      -_gains_ang_vel.kp().emult(_rates_integral);    /* i-term */

  //    input_.moment.print();
//  std::cout << "rdx: " << _rates_derv(0) <<  "rdy: " << _rates_derv(1) <<  "rdz: " << _rates_derv(2)  << std::endl;
//
//  std::cout << "rix: " << _rates_integral(0) <<  "riy: " << _rates_integral(1) <<  "riz: " << _rates_integral(2)  << std::endl;
//
//  std::cout << "mx: " << input_.moment(0) <<  "my: " << input_.moment(1) <<  "mz: " << input_.moment(2)  << std::endl;
}

void EulerPID::rate_pid_rosflight() {
  // TODO: update this rate controller using PID class

}

} // namespace qrotor_firmware
