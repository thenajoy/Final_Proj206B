#include "control/attitude_vbl_lqr.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeVariationLQR::AttitudeVariationLQR(FlightController &_flightController) :
    AttitudeController(_flightController) {

  float kR_[9] = {10.890492, 0.006636, 0.001013,
                  0.006637, 11.374058, 0.003857,
                  0.000632, 0.001840, 2.553196};
  kR = matrix::Matrix3f(kR_);

  float kOm_[9] = {2.202462, 0.001355, 0.000724,
                   0.001355, 2.301159, 0.002757,
                   0.000153, 0.000470, 1.819189};
  kOm = matrix::Matrix3f(kOm_);

}

AttitudeVariationLQR::~AttitudeVariationLQR() {}

bool AttitudeVariationLQR::init() {
  firmware_.logger_.STATUS(std::string("AttitudeVariationLQR initialized!"));
  return true;
}
void AttitudeVariationLQR::compute(float _dt) {
  this->dt_ = _dt;

  /* compute desired attitude rotation matrix */
  thrust_vector_to_cmd_rotation();

  /* compute error vectors */
  compute_error_vectors();

  /* feed-back control */
  input_.moment = -kR * eR - kOm * eOmega;
  input_.thrust = input_.thrust_vector.dot(firmware_.att_estimator_->state().Re3());

}

void AttitudeVariationLQR::thrust_vector_to_cmd_rotation() {
  matrix::Vector3f e3_cmd, e1_des, e1_cmd;
  e3_cmd = input_.thrust_vector;
  e3_cmd.normalize();

  e1_des = matrix::Vector3f(1.0, 0.0, 0.0); // fix this
  e1_cmd = -e3_cmd.hat() * e3_cmd.hat() * e1_des;
  e1_cmd.normalize();

  this->cmd_attitude_.set_axis2Dcm(e1_cmd, e3_cmd.cross(e1_cmd), e3_cmd);
}

void AttitudeVariationLQR::compute_error_vectors() {
  matrix::Dcmf Rt, diffRt;
  Rt = cmd_attitude_.R().T() * firmware_.att_estimator_->state().R();
  diffRt = Rt - Rt.T();
  eR = diffRt.vee() * 0.5;

  eOmega = firmware_.att_estimator_->state().ang_vel - firmware_.att_estimator_->state().R().T() * cmd_attitude_.R() * cmd_attitude_.ang_vel;
}

} // namespace qrotor_firmware
