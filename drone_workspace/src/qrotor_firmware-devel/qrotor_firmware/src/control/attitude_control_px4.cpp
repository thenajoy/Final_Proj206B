//
// Created by kotaru on 3/11/21.
#include "control/attitude_control_px4.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeControlPX4::AttitudeControlPX4(FlightController &_flightcontroller) :
    AttitudeController(_flightcontroller) {

  setProportionalGain(_gains_att.kp(), 0.5);
  _gain_ff.setZero();

  matrix::Vector3f MC_ATT_P{6.5, 6.5, 2.8};
  matrix::Vector3f MC_ATT_D{0.0, 0.0, 0.0};
  matrix::Vector3f MC_ATT_I{0.0, 0.0, 0.0};

  matrix::Vector3f MC_RATE_P{0.15, 0.15, 0.2};
  matrix::Vector3f MC_RATE_D{0.003, 0.003, 0.0};
  matrix::Vector3f MC_RATE_I{0.05, 0.05, 0.1};

  matrix::Vector3f MC_RATE_MAX;
  matrix::Vector3f MC_ATTITUDE;

  // default gains
  _gains_att.set_gains(MC_ATT_P, MC_ATT_D, MC_ATT_I);
  _gains_ang_vel.set_gains(MC_RATE_P, MC_RATE_D, MC_RATE_I);

  _lim_int = {0.3, 0.3, 0.3};

}

bool AttitudeControlPX4::init() {
  Logger::STATUS("AttitudeControlPX4 initialized");
  return true;
}

void AttitudeControlPX4::compute(float _dt) {

  _attitude_setpoint_q = matrix::Quatf(firmware_.att_controller_->cmd_attitude().euler);

  if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD) {
    // update command yaw-rate to the receiver yaw rate
    _yawspeed_setpoint = cmd_attitude_.ang_vel(2);
  } else {
    _yawspeed_setpoint = 0.0;
  }
  matrix::Vector3f rate_sp = this->update(this->firmware_.att_estimator_->state().q());

//  float thrust_orientation_scaling = 1/acos();
//  input_.thrust = input_.thrust_vector(2);
  matrix::Vector3f moment = rate_control_update(firmware_.att_estimator_->state().ang_vel,
                                      rate_sp,
                                      firmware_.att_estimator_->state().ang_vel_rates,
                                      _dt,
                                      (!firmware_.state_machine_.state().armed));
//  input_.moment = moment.emult(MAX_BODY_MOMENT);
  input_.moment = moment;
  std::cout << "PX4 : mx: " << input_.moment(0) << "\t my: " << input_.moment(1) << "\t mz: " << input_.moment(2) << std::endl;

}

matrix::Vector3f AttitudeControlPX4::update(const matrix::Quatf &q) const {
  matrix::Quatf qd = _attitude_setpoint_q;

  // calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
  const matrix::Vector3f e_z = q.dcm_z();
  const matrix::Vector3f e_z_d = qd.dcm_z();
  matrix::Quatf qd_red(e_z, e_z_d);

  if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
    // In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
    // full attitude control anyways generates no yaw input and directly takes the combination of
    // roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
    qd_red = qd;

  } else {
    // transform rotation from current to desired thrust vector into a world frame reduced desired attitude
    qd_red *= q;
  }

  // mix full and reduced desired attitude
  matrix::Quatf q_mix = qd_red.inversed() * qd;
  q_mix.canonicalize();
  // catch numerical problems with the domain of acosf and asinf
  q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
  q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
  qd = qd_red * matrix::Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

  // quaternion attitude control law, qe is rotation from q to qd
  const matrix::Quatf qe = q.inversed() * qd;

  // using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
  // also taking care of the antipodal unit quaternion ambiguity
  const matrix::Vector3f eq = 2.f * qe.canonical().imag();

  // calculate angular rates setpoint
  matrix::Vector3f rate_setpoint = eq.emult(_gains_att.kp());

  // Feed forward the yaw setpoint rate.
  // yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
  // but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
  // Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
  // and multiply it by the yaw setpoint rate (yawspeed_setpoint).
  // This yields a vector representing the commanded rotation around the world z-axis expressed in the body frame
  // such that it can be added to the rates setpoint.
  rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;

  // limit rates
  for (int i = 0; i < 3; i++) {
    rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
  }
  std::cout << "rate_setpoint: " << rate_setpoint(0) << " "<< rate_setpoint(1) << " "<< rate_setpoint(2) << std::endl;

  return rate_setpoint;
}

matrix::Vector3f AttitudeControlPX4::rate_control_update(const matrix::Vector3f &rate,
                                                         const matrix::Vector3f &rate_sp,
                                                         const matrix::Vector3f &angular_accel,
                                                         const float dt,
                                                         const bool landed) {
  // angular rates error
  matrix::Vector3f rate_error = rate_sp - rate;

  // PID control with feed forward
  const matrix::Vector3f torque = _gains_ang_vel.kp().emult(rate_error)
      + _rate_int - _gains_ang_vel.kd().emult(angular_accel)
      + _gain_ff.emult(rate_sp);

  // update integral only if we are not landed
  if (!landed) {
    rate_control_updateIntegral(rate_error, dt);
  }

  return torque;
}

void AttitudeControlPX4::rate_control_updateIntegral(matrix::Vector3f &rate_error, const float dt) {
  for (int i = 0; i < 3; i++) {
    // prevent further positive control saturation
    if (_mixer_saturation_positive[i]) {
      rate_error(i) = math::min(rate_error(i), 0.f);
    }

    // prevent further negative control saturation
    if (_mixer_saturation_negative[i]) {
      rate_error(i) = math::max(rate_error(i), 0.f);
    }

    // I term factor: reduce the I gain with increasing rate error.
    // This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
    // change (noticeable in a bounce-back effect after a flip).
    // The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
    // with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
    // and up to 200 deg error leads to <25% reduction of I.
    float i_factor = rate_error(i) / math::radians(400.f);
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

    // Perform the integration using a first order method
    float rate_i = _rate_int(i) + i_factor * _gains_ang_vel.ki()(i) * rate_error(i) * dt;

    // do not propagate the result if out of range or invalid
    if (!std::isnan(rate_i)) {
      _rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
    }
  }
}

}


