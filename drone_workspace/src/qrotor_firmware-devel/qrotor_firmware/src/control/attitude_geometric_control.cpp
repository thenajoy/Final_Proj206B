//
// Created by kotaru on 4/9/21.
//
#include "control/attitude_geometric_control.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeGeometricController::AttitudeGeometricController(
    FlightController &_flightController)
    : AttitudeController(_flightController) {
  RATES_INTEGRAL_UB = {0.3, 0.3, 0.1};
  RATES_INTEGRAL_LB = -RATES_INTEGRAL_UB;
}

AttitudeGeometricController::~AttitudeGeometricController() = default;

bool AttitudeGeometricController::init() {
  Logger::STATUS(std::string("AttitudeGeometricController initialized!"));
  return true;
}

void AttitudeGeometricController::compute(float _dt) {
  this->dt_ = _dt;

  // compute rate_setpoint from attitude
  matrix::Vector3f rate_sp = cmd_attitude_to_rate_sp();

  // computing torque by using PID on rates
  matrix::Vector3f torque =
      rate_control(firmware_.att_estimator_->state().ang_vel, rate_sp,
                   firmware_.att_estimator_->state().ang_vel_rates);

  input_.moment = torque; // TODO does the torque need to be bounded!

//  std::cout << "TILT: mx: " << input_.moment(0) << "\t my: " << input_.moment(1)
//            << "\t mz: " << input_.moment(2) << std::endl;
}

matrix::Vector3f AttitudeGeometricController::cmd_attitude_to_rate_sp() {

//  std::cout << "command rotation" << std::endl;
//  cmd_attitude_.R().print();

  // rotation error
  matrix::Dcmf Re;
  Re = cmd_attitude_.R().transpose() * firmware_.att_estimator_->state().R();

//  std::cout << "rotation error" << std::endl;
//  Re.print();

  // computing the tilt-angle between command and actual
  float rho_r =
      acos(math::constrain((_e3.transpose() * Re * _e3)(0, 0), -1.f, 1.f));
  if (std::isnan(rho_r)) {
    Logger::ERROR("Tilt angle is NaN! making it zero!");
    rho_r = 0.f;
  }

  // compute primary (tilt) angle-axis error
  matrix::Vector3f n_r;
  if (abs(rho_r) > 1e-6) {
    n_r = lie_algebra::hat(Re.transpose() * _e3) * _e3 * (1 / sin(rho_r));
  } else {
    // if magnitude is negligible chose arbitrary (vertical) axis
    n_r = _e3;
  }

  // tilt-prioritized rotation-error
  matrix::Dcmf Rr = matrix::Dcmf(matrix::AxisAnglef(n_r, rho_r));

  // remaining yaw-rotation
  matrix::Dcmf Ry = matrix::Dcmf(Re * Rr.transpose());

  // axis-angle rep of Ry
  matrix::AxisAnglef eta_y(Ry);

  float rho_y = eta_y.norm();
  matrix::Vector3f n_y;
  if (abs(rho_y) > 1e-6) {
    n_y = eta_y.axis();
  } else {
    n_y = _e3;
  }

  // computing the rate-setpoint with tilt-prioritization
  // kr > k_yaw
  matrix::Vector3f kr{_gains_att.kp()(0), _gains_att.kp()(1), 0.f};
  float k_yaw = _gains_att.kp()(2);
  matrix::Vector3f rate_sp = kr.emult(n_r) * sin(rho_r / 2) * (-2.f) -
                             2.f * k_yaw * n_y * sin(rho_y / 2);
//  std::cout << "eta_r: angle: " << rho_r << " axis: " << n_r(0) << " " << n_r(1)
//            << " " << n_r(2) << std::endl;
//  std::cout << "eta_y: angle: " << rho_y << " axis: " << n_y(0) << " " << n_y(1)
//            << " " << n_y(2) << std::endl;
  // limiting the setpoint
  rate_sp = matrix::constrain(rate_sp, -_rate_lim, _rate_lim);

  return rate_sp;
}

matrix::Vector3f AttitudeGeometricController::rate_control(
    const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
    const matrix::Vector3f &angular_accel) {
  //   angular rates error
  matrix::Vector3f rate_error = rate_sp - rate;

  // PID control with feed forward
  matrix::Vector3f torque = _gains_ang_vel.kp().emult(rate_error) +
                            this->_rates_integral -
                            _gains_ang_vel.kd().emult(angular_accel);

  // update integral only if we are not landed
  updateIntegral(rate_error);

  return torque;
}

void AttitudeGeometricController::updateIntegral(
    const matrix::Vector3f &rate_error) {
  for (int i = 0; i < 3; i++) {
    // I term factor: reduce the I gain with increasing rate error.
    // This counteracts a non-linear effect where the integral builds up quickly
    // upon a large setpoint change (noticeable in a bounce-back effect after a
    // flip). The formula leads to a gradual decrease w/o steps, while only
    // affecting the cases where it should: with the parameter set to 400
    // degrees, up to 100 deg rate error, i_factor is almost 1 (having no
    // effect), and up to 200 deg error leads to <25% reduction of I.
    float i_factor = rate_error(i) / math::radians(400.f);
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

    // Perform the integration using a first order method
    float rate_i = _rates_integral(i) +
                   i_factor * _gains_ang_vel.ki()(i) * rate_error(i) * dt_;

    // do not propagate the result if out of range or invalid
    if (!std::isnan(rate_i)) {
      _rates_integral(i) =
          math::constrain(rate_i, RATES_INTEGRAL_LB(i), RATES_INTEGRAL_UB(i));
    }
  }
}

} // namespace qrotor_firmware
