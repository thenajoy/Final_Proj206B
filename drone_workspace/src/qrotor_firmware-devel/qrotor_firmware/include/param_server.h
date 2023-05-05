#ifndef __QROTOR_FIRMWARE_PARAMS_HPP__
#define __QROTOR_FIRMWARE_PARAMS_HPP__
//
// Created by kotaru on 1/30/21.
//
#include <iostream>
#include <map>
#include <string>

namespace qrotor_firmware {
enum Params {
  START,

  /* ESTIMATION: MAHONY FILTER */
  MAHONY_GYRO_XY_LPF,
  MAHONY_GYRO_Z_LPF,
  MAHONY_ACCEL_LPF,
  MAHONY_FILTER_KI,
  MAHONY_FILTER_KP_ACC,
  MAHONY_FILTER_KP_EXT,
  MAHONY_FILTER_USE_QUAD_INT,

  /* ATTITUDE ESTIMATION: EXTERNAL ATT FUSION */
  ATT_EST_USE_EXTERNAL_ATT,
  ATT_EST_EXTERNAL_ATT_COVAR,
  ATT_EST_INTERNAL_ATT_COVAR,

  /* BATTERY PARAMETERS */
  VOLTAGE_MULTIPLIER,
  CURRENT_COEFFICIENT,

  /* ATTITUDE GEOMETRIC CLF QP */
  ATT_CLF_QP_ETA,
  ATT_CLF_QP_EPSILON,
  ATT_CLF_QP_RATE_OF_CONVERGENCE,

  /* EXTERNAL POSE */
  EXTERNAL_POSE, // 0 -tracking, 1-mocap, 2- fuse
  EXTERNAL_POSE_MOCAP_COVAR,
  EXTERNAL_POSE_TCAM_COVAR,

  /* MIXER PARAMETERS */
  MIXER_UNITS, // 0: SI units, 1: scaled

  /* RC TYPE */
  RC_TYPE,

  END
};

class ParamServer {
protected:
  std::map<Params, float> _params;
  std::map<Params, std::string> _names;

  bool contains(Params _key) { return !(_params.find(_key) == _params.end()); }

public:
  ParamServer();
  ~ParamServer();

  void init();
  void run();
  void init_default_params();

  /**
   * @brief parameter setter, sets float values for given parameter key
   * @param _key
   * @param _value
   */
  void set_param(Params _key, const float &_value);

  /**
   * @brief sets params value and name given the key
   * @param _key
   * @param _name
   * @param _value
   */
  void set_param(Params _key, const std::string &_name, const float &_value);

  /**
   * functions to the parameters
   * @param _key
   * @return the float/bool/int value of the parameter
   */
  float get(Params _key);
  float get_float(Params _key);
  int get_int(Params _key);
  int get_bool(Params _key);
  /**
   * Overloaded operator [] to return the key value
   * @param _key
   * @return float value of the operator
   */
  float operator[](Params _key);

  std::string get_param_name(Params _key);
  void debug_print();

  float mahony_gyro_xy_lpf = 0.3;
  float mahony_gyro_z_lpf = 0.3;
  float mahony_accel_lpf = 0.5;
  float mahony_filter_ki = 0.01;
  float mahony_filter_kp_acc = 2.0;
  float mahony_filter_kp_ext = 1.5;
  int mahony_filter_use_quad_int = 1;
};

} // namespace qrotor_firmware

#endif //__QROTOR_FIRMWARE_PARAMS_HPP__
