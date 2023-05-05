#include "control/attitude_controller.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeController::AttitudeController(FlightController &_flightcontroller)
    : firmware_(_flightcontroller), _e1({1.0, 0.0, 0.0}), _e2({0.0, 1.0, 0.0}),
      _e3({0.0, 0.0, 1.0}) {
  // inertia
  float J[9] = {0.0049, 0.0000055, 0.0000054, 0.0000055, 0.0053,
                0.000021, 0.0000054, 0.000021, 0.0098};
  inertia_matrix_ = matrix::Matrix3f(J);
  inertia_matrix_inv = geninv(inertia_matrix_ * 10e6) * 10e6;
  min_eigval_inertia =
      std::min(std::min(inertia_matrix_(0, 0), inertia_matrix_(1, 1)),
               inertia_matrix_(2, 2));
  inertia_scaled = inertia_matrix_ / min_eigval_inertia;

  // reset command attitude
  this->cmd_attitude_.reset();
  this->input_.zero();
  this->ff_input_.zero();

  // default gains
  _gains_att.set_gains(matrix::Vector3f(8.0, 8.0, 3.0),
                       matrix::Vector3f(0.3, 0.3, 0.225),
                       matrix::Vector3f(0.0, 0.0, 0.0));
  _gains_ang_vel.set_gains(matrix::Vector3f(0.15, 0.15, 0.05),
                           matrix::Vector3f(0.0001, 0.0001, 0.0),
                           matrix::Vector3f(0.0, 0.0, 0.0));

  // rate errors
  _lp_rate_derv.set_cutoff_frequency(firmware_.loop_rate(),
                                     50.0f); // cut_off freq
  _rates_integral.zero();
  RATES_INTEGRAL_LB = {-0.15, -0.15, -0.15}; // rad
  RATES_INTEGRAL_UB = {0.15, 0.15, 0.15};    // rad
  _rate_lim = {3.839, 3.839, 3.49};          // rad/s i.e. {220,220,200} deg/s

  MOMENT_UPPER_BOUND = {2, 2, 1};
  MOMENT_LOWER_BOUND = -MOMENT_UPPER_BOUND;
}

AttitudeController::~AttitudeController() = default;

void AttitudeController::set_pos_gains(const matrix::Vector3f &_kp,
                                       const matrix::Vector3f &_kd,
                                       const matrix::Vector3f &_ki) {
  _gains_att.set_gains(_kp, _kd, _ki);
}
void AttitudeController::set_vel_gains(const matrix::Vector3f &_kp,
                                       const matrix::Vector3f &_kd,
                                       const matrix::Vector3f &_ki) {
  _gains_ang_vel.set_gains(_kp, _kd, _ki);
}

bool AttitudeController::init() {
  // reset rate error integral
  reset_rate_integral();
  Logger::STATUS(std::string("AttitudeController initialized!"));
  return true;
}

void AttitudeController::run(float _dt) {
  if (firmware_.state_machine_.state().mode == StateMachine::POSITION_HOLD ||
      firmware_.state_machine_.state().mode == StateMachine::OFFBOARD) {
    thrust_vector_to_cmd_rotation();
  }

  // printf("commands: roll: %f, pitch: %f, yaw_rate: %f\n",
  // cmd_attitude_.roll()* M_RAD_TO_DEG_F, cmd_attitude_.pitch() *
  // M_RAD_TO_DEG_F, cmd_attitude_.yaw_rate() * M_RAD_TO_DEG_F);
  //  moment
  this->compute(_dt);
}

void AttitudeController::thrust_vector_to_cmd_attitude() {
  /* computing desired attitude from thrust vector */
  float fx = input_.thrust_vector(0), fy = input_.thrust_vector(1),
      fz = input_.thrust_vector(2);
  float yaw = firmware_.att_estimator_->state().yaw();

  input_.thrust = fz; // TODO: update this to f=F*R*e3

  // compute command-attitude from thrust-vector
  this->cmd_attitude_.euler = {
      ((fx / firmware_.vehicle_params_.mass_) * sin(yaw) -
          (fy / firmware_.vehicle_params_.mass_) * cos(yaw)) /
          firmware_.vehicle_params_.g_,
      ((fx / firmware_.vehicle_params_.mass_) * cos(yaw) +
          (fy / firmware_.vehicle_params_.mass_) * sin(yaw)) /
          firmware_.vehicle_params_.g_,
      yaw_sp};
}

void AttitudeController::thrust_vector_to_cmd_rotation() {
  matrix::Vector3f e3_cmd, e1_des, e1_cmd;
  e3_cmd = input_.thrust_vector;

  bool ATT_MODE = false;
  float tilt_angle = acos(e3_cmd.dot(_e3));
  // if tilt angle is less than 2 degrees
  //  ATT_MODE = (tilt_angle < M_PI * (2. / 180.));
//    printf("F: %f, %f, %f\n", e3_cmd(0),e3_cmd(1),e3_cmd(2));
  if (std::isnan(e3_cmd.norm()) || (e3_cmd.norm() < 1e-4)) {
    e3_cmd = _e3;
  }
  e3_cmd.normalize();
//    printf("e3_cmd: %f, %f, %f\n", e3_cmd(0),e3_cmd(1),e3_cmd(2));

  e1_des = matrix::Vector3f(cos(yaw_sp), sin(yaw_sp), 0.0); // fix this
//  printf("e1_des %f, %f, %f\n", e1_des(0), e1_des(1), e1_des(2));
  e1_cmd = -e3_cmd.hat() * e3_cmd.hat() * e1_des;
  e1_cmd.normalize();

  this->cmd_attitude_.set_axis2Dcm(e1_cmd, e3_cmd.cross(e1_cmd), e3_cmd);

  input_.thrust = input_.thrust_vector(2); // TODO: update this to f=F*R*e3
}

void AttitudeController::compute(float _dt) { /* virtual function */
}

} // namespace qrotor_firmware
