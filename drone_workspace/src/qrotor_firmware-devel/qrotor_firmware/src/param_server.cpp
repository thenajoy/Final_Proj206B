//
// Created by kotaru on 3/14/21.
//
#include "param_server.h"

namespace qrotor_firmware {

ParamServer::ParamServer() {
  init();
}

ParamServer::~ParamServer() = default;

void ParamServer::init() {
  /* initialize default parameters */
  init_default_params();
}

void ParamServer::init_default_params() {

  /* ESTIMATION: MAHONY FILTER */
  set_param(Params::MAHONY_GYRO_XY_LPF, "MAHONY_GYRO_XY_LPF", 0.3f);
  set_param(Params::MAHONY_GYRO_Z_LPF, "MAHONY_GYRO_Z_LPF", 0.3f);
  set_param(Params::MAHONY_ACCEL_LPF, "MAHONY_ACCEL_LPF", 0.5f);
  set_param(Params::MAHONY_FILTER_KI, "MAHONY_FILTER_KI", 0.01f);
  set_param(Params::MAHONY_FILTER_KP_ACC, "MAHONY_FILTER_KP_ACC", 2.0f);
  set_param(Params::MAHONY_FILTER_KP_EXT, "MAHONY_FILTER_KP_EXT", 1.5f);
  set_param(Params::MAHONY_FILTER_USE_QUAD_INT, "MAHONY_FILTER_USE_QUAD_INT", 1);

  /* ATTITUDE ESTIMATION: EXTERNAL ATT FUSION */
  set_param(Params::ATT_EST_USE_EXTERNAL_ATT, "ATT_EST_USE_EXTERNAL_ATT", 1);
  set_param(Params::ATT_EST_EXTERNAL_ATT_COVAR, "ATT_EST_EXTERNAL_ATT_COVAR", 0.01);
  set_param(Params::ATT_EST_INTERNAL_ATT_COVAR, "ATT_EST_INTERNAL_ATT_COVAR", 1);

  /* BATTERY CONSTANTS */
  set_param(Params::VOLTAGE_MULTIPLIER, "VOLTAGE_MULTIPLIER", 11.3);
  set_param(Params::CURRENT_COEFFICIENT, "CURRENT_COEFFICIENT", 17.0);

  /* ATTITUDE GEOMETRIC CONTROL*/
  set_param(Params::ATT_CLF_QP_ETA, "ATT_CLF_QP_ETA", 150.0);
  set_param(Params::ATT_CLF_QP_EPSILON, "ATT_CLF_QP_EPSILON", 4.0);
  set_param(Params::ATT_CLF_QP_RATE_OF_CONVERGENCE, "ATT_CLF_QP_RATE_OF_CONVERGENCE", 20.0);

  /* EXTERNAL POSE INFORMATION */
  set_param(Params::EXTERNAL_POSE, "EXTERNAL_POSE", 2); // 0 -tracking, 1-mocap, 2- fuse
  set_param(Params::EXTERNAL_POSE_MOCAP_COVAR, "EXTERNAL_POSE_MOCAP_COVAR", 0.01);
  set_param(Params::EXTERNAL_POSE_TCAM_COVAR, "EXTERNAL_POSE_TCAM_COVAR", 0.1);

  /* MIXER PARAMETERS */
  set_param(Params::MIXER_UNITS, "MIXER_UNITS", 0);

  /* RC RECEIVER TYPE */
  set_param(Params::RC_TYPE, "RC_TYPE", 0); // 0 - RC transmitter , 1- ROS message /RC
}

void ParamServer::set_param(Params _key, const float &_value) {
  if (contains(_key)) {
    this->_params.at(_key) = _value;
    std::cout << "setting " + _names[_key] << " " << _key << " value " << _params[_key] << std::endl;
  } else {
    this->_params.insert(std::make_pair(_key, _value));
  }
  debug_print();
}

void ParamServer::set_param(Params _key, const std::string &_name, const float &_value) {
  if (contains(_key)) {
    _params[_key] = _value;
    _names[_key] = _name;
  } else {
    _params.insert(std::make_pair(_key, _value));
    _names.insert(std::make_pair(_key, _name));
  }
}

float ParamServer::get(Params _key) { return (_params[_key]); }
float ParamServer::get_float(Params _key) { return _params[_key]; }
int ParamServer::get_int(Params _key) { return (int) _params[_key]; }
int ParamServer::get_bool(Params _key) { return (bool) _params[_key]; }
float ParamServer::operator[](Params _key) { return (_params[_key]); }

std::string ParamServer::get_param_name(Params _key) { return _names[_key]; }

void ParamServer::debug_print() {
  for (auto const &p : _params) {
    std::cout << _names[p.first] // parameter name (key)
              << ':' << p.second // string's value
              << std::endl;
  }
}

void ParamServer::run() {
  debug_print();
}

} // namespace qrotor_firmware
