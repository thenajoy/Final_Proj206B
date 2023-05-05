#include "control/position_pid.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

PositionPID::PositionPID(FlightController &_flightcontroller)
    : PositionController(_flightcontroller) {

  pos_integral_err.zero();
  vel_integral_err.zero();
  cmd_pose_.reset();
  INTEGRAL_UPDATE = true;
}

PositionPID::~PositionPID() = default;

void PositionPID::run(float dt) {
  this->dt_ = dt;

  _pos_sp = firmware_.mission_planner_->pose_des().position;
  _vel_sp = firmware_.mission_planner_->pose_des().velocity;
  _acc_sp = firmware_.mission_planner_->pose_des().acceleration;
  matrix::Quatf q = firmware_.pos_estimator_->pose().quat;
  _vel_dot = (q.to_dcm() * firmware_.att_estimator_->state().linear_accel -
      matrix::Vector3f(0, 0, 1)) *
      G_SI_F;

  run_position_pid();
//  run_velocity_control();
}

void PositionPID::run_position_pid() {
  matrix::Vector3f pos_err = firmware_.pos_estimator_->pose().position -
      firmware_.mission_planner_->pose_des().position;
  matrix::Vector3f vel_err = firmware_.pos_estimator_->pose().velocity -
      firmware_.mission_planner_->pose_des().velocity;

  matrix::Vector3f thrust_v;
  thrust_v.zero();

  // feedback: PD input
  thrust_v = -_gains_pos.kp().emult(pos_err) - _gains_pos.kd().emult(vel_err);

  // integral update
  if (INTEGRAL_UPDATE) {
    pos_integral_err += pos_err * dt_;

    // basic anti-windup (bounding the integral error);
    pos_integral_err =
        constrain(pos_integral_err, POS_INTEGRAL_ERR_LB, POS_INTEGRAL_ERR_UB);

    // adding integral force
    thrust_v -= _gains_pos.ki().emult(pos_integral_err);
  }

  // feed-forward input
  thrust_v +=
      (firmware_.mission_planner_->pose_des().acceleration + E3 * G_SI) *
          firmware_.vehicle_params_.mass_;

  // input-bounds
  thrust_v = constrain(thrust_v, THRUST_VECTOR_LB, THRUST_VECTOR_UB);

  //  std::cout << "dt_s " << dt_ << " pos_err " << pos_err(0) << " " <<
  //  pos_err(1)
  //            << " " << pos_err(2) << "\npos_integral_err " <<
  //            pos_integral_err(0)
  //            << " " << pos_integral_err(1) << " " << pos_integral_err(2)
  //            << " thrust_v " << thrust_v(0) << " " << thrust_v(1) << " "
  //            << thrust_v(2) << "\n"
  //            << std::endl;

  // send thrust vector to attitude controller
  firmware_.att_controller_->set_thrust_vector(thrust_v);
}

void PositionPID::run_velocity_control() {

  _positionControl();
  _velocityControl();

  // scaling back the thrust to SI units
  // TODO this is a temporary fix, move this to Mixer2 later
  firmware_.att_controller_->set_thrust_vector((_thr_sp * G_SI_F));
}

void PositionPID::_positionControl() {
  //////////////////////////
  /// P-position control ///
  //////////////////////////

  // computing the position error
  matrix::Vector3f pos_err =
      firmware_.pos_estimator_->pose().position - _pos_sp;
  // Computing velocity setpoint from position error
  matrix::Vector3f vel_sp_position = -_gains_pos.kp().emult(pos_err);
  // Adding feedforward velocity setpoint, ignoring NaNs
  control_math::addIfNotNanVector3f(_vel_sp, vel_sp_position);
  // make sure there are no NAN elements for further reference while
  // constraining
  control_math::setZeroIfNanVector3f(vel_sp_position);

  // Constrain horizontal velocity by prioritizing the velocity component along
  // the the desired position setpoint over the feed-forward term.
  _vel_sp.xy() = control_math::constrainXY(vel_sp_position.xy(),
                                           (_vel_sp - vel_sp_position).xy(),
                                           _lim_vel_horizontal);
  // Constrain velocity in z-direction.
  _vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_down, _lim_vel_up);
}

void PositionPID::_velocityControl() {
  ////////////////////////////
  /// PID-velocity control ///
  ////////////////////////////

  // computing velocity error
  matrix::Vector3f vel_error =
      _vel_sp - firmware_.pos_estimator_->pose().velocity;
  // computing accel setpoint using PID on accel_sp
  matrix::Vector3f acc_sp_velocity = vel_error.emult(_gains_vel.kp()) +
      _vel_int - _vel_dot.emult(_gains_vel.kd());

  // No control input from setpoints or corresponding states which are NAN
  control_math::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

  _accelerationControl();
  // Integrator anti-windup in vertical direction TODO verify this
  if ((_thr_sp(2) >= _lim_thr_max && vel_error(2) >= 0.0f) ||
      (_thr_sp(2) <= _lim_thr_min && vel_error(2) <= 0.0f)) {
    vel_error(2) = 0.f;
  }

  // Saturate maximal vertical thrust
  _thr_sp(2) = math::min(_thr_sp(2), _lim_thr_max);
  _thr_sp(2) = math::max(_thr_sp(2), _lim_thr_min);

  // Get allowed horizontal thrust after prioritizing vertical control
  const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
  const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
  const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
  float thrust_max_xy = 0;

  if (thrust_max_xy_squared > 0) {
    thrust_max_xy = sqrtf(thrust_max_xy_squared);
  }

  // Saturate thrust in horizontal direction
  const matrix::Vector2f thrust_sp_xy(_thr_sp);
  const float thrust_sp_xy_norm = thrust_sp_xy.norm();

  if (thrust_sp_xy_norm > thrust_max_xy) {
    _thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
  }

  // Use tracking Anti-Windup for horizontal direction: during saturation,the
  // integrator is used to unsaturate the output see Anti-Reset Windup for PID
  // controllers, L.Rundqwist, 1990
  const matrix::Vector2f acc_sp_xy_limited =
      matrix::Vector2f(_thr_sp) * (G_SI_F / _hover_thrust);
  const float arw_gain = 2.f / _gains_vel.kp()(0);
  vel_error.xy() = matrix::Vector2f(vel_error) -
      (arw_gain * (matrix::Vector2f(_acc_sp) - acc_sp_xy_limited));

  // Make sure integral doesn't get NAN
  control_math::setZeroIfNanVector3f(vel_error);
  // Update integral part of velocity control
  _vel_int += vel_error.emult(_gains_vel.ki()) * dt_;

  // limit thrust integral
  _vel_int(2) = std::min(fabsf(_vel_int(2)), G_SI_F) * matrix::sign(_vel_int(2));
}

void PositionPID::_accelerationControl() {
  // Assume standard acceleration due to gravity in vertical direction for
  // attitude generation
  matrix::Vector3f body_z =
      matrix::Vector3f(_acc_sp(0), _acc_sp(1), G_SI_F).normalized();
  control_math::limitTilt(body_z, matrix::Vector3f(0, 0, 1), _lim_tilt);

  // Scale thrust assuming hover thrust produces standard gravity
  float collective_thrust =
      _acc_sp(2) * (_hover_thrust / G_SI_F) + _hover_thrust;
  // Project thrust to planned body attitude
  collective_thrust /= (matrix::Vector3f(0, 0, 1).dot(body_z));
  collective_thrust = math::min(collective_thrust, _lim_thr_max);
  _thr_sp = body_z * collective_thrust;
}

} // namespace qrotor_firmware
