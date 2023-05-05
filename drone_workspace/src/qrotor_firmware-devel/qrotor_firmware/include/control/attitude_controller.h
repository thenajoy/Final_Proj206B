#ifndef __QROTOR_FIRMWARE_ATTITUDE_CONTROLLER_H__
#define __QROTOR_FIRMWARE_ATTITUDE_CONTROLLER_H__

#include <cmath>
#include <cstdint>
#include <iostream>

#include "Matrix/matrix/math.hpp"
#include "data_structures.h"
#include "mathlib/math/filter/LowPassFilter2pVector3f.hpp"
#include "parameters.h"

namespace qrotor_firmware {

class PID {
public:
  PID() = default;
  void init(float kp, float ki, float kd, float max, float min, float tau) {}
  float run(float dt, float x, float x_c, bool update_integrator) { return 0; }
  float run(float dt, float x, float x_c, bool update_integrator, float xdot) {
    return 0;
  }

private:
  Gains _gains;

  float max_{};
  float min_{};

  float integrator_{};
  float differentiator_{};
  float prev_x_{};
  float tau_{};
};

class FlightController;

class AttitudeController {

protected:
  /// FlightController firmware
  FlightController &firmware_;
  /// Loop time in seconds
  float dt_{0.02f};

  /// Lyapunov function value
  float lyap_{0.f};

  /// Vehicle inertia matrix
  matrix::Matrix3f inertia_matrix_;
  /// Vehicle inertia matrix inverse
  matrix::Matrix3f inertia_matrix_inv;
  /// Minimum eigen value of inertia
  float min_eigval_inertia;
  /// Vehicle inertia scaled TODO move these to vehicle parameters
  matrix::Matrix3f inertia_scaled;
  /// Vehicle mass in [kg]
  double _mass{0.85};

  /// Acceleration due to gravity
  const float _g = 9.80665;
  /// Basis vectors
  const matrix::Vector3f _e1, _e2, _e3;

  /// Command attitude computed to track desired thrust-vector
  Attitude cmd_attitude_;
  /// Command Yaw-Direction (temporary)
  float yaw_sp{0.0f};
  /// Input struct (thrust, moment) in SI units
  InputWrench input_;
  /// Feed-forward input for trajectories tracking
  InputWrench ff_input_;

  /// attitude (p,i,d) gains
  Gains _gains_att;
  /// angular-velocity (p,i,d) gains
  Gains _gains_ang_vel;
  /// yaw weight [0,1] to de-prioritize compared to roll and pitch
  float _yaw_w{0.f};

  /// Low-pass-filter for computing angular acceleration
  math::LowPassFilter2pVector3f _lp_rate_derv{500, 50};
  /// Rate-integral error
  matrix::Vector3f _rates_integral;
  /// Rate setpoint limits
  matrix::Vector3f _rate_lim;
  /// Rate integral bounds
  matrix::Vector3f RATES_INTEGRAL_LB, RATES_INTEGRAL_UB;
  /// Moment bounds in SI
  matrix::Vector3f MOMENT_UPPER_BOUND, MOMENT_LOWER_BOUND;

public:
  explicit AttitudeController(FlightController &_flightcontroller);
  ~AttitudeController();

  /**
   * Initialization for the controller
   * @return successful initialization
   */
  virtual bool init();
  /**
   * Run function that implements control algo every loop iteration
   * @param _dt
   */
  void run(float _dt);

  /**
   * Computes command Euler-angles from thrust-vector
   * using small-angle approximation
   */
  void thrust_vector_to_cmd_attitude();
  /**
   * Computes the command rotation matrix from thrust-vector
   * with fixed yaw direction TODO modify yaw-setpoint
   */
  void thrust_vector_to_cmd_rotation();
  /**
   * Function to implement the control-algorithm(s)
   * @param _dt
   */
  virtual void compute(float _dt);

  /* setters and getters */
  inline const InputWrench &input() const { return input_; }
  inline const Attitude &cmd_attitude() const { return cmd_attitude_; }

  void set_cmd_euler(float roll, float pitch, float yaw) {
    cmd_attitude_.set_euler(roll, pitch, yaw);
    cmd_attitude_.refreshEuler();
  }
  void set_cmd_yaw_rate(float gz) { cmd_attitude_.ang_vel(2) = gz; }
  void reset_rate_integral() { _rates_integral.zero(); }

  /**
   * Set proportional attitude control gain
   * @param proportional_gain 3D vector containing gains for roll, pitch, yaw
   * @param yaw_weight A fraction [0,1] deprioritizing yaw compared to roll and
   * pitch
   */
  void setProportionalGain(const matrix::Vector3f &proportional_gain,
                           const float yaw_weight) {
    _yaw_w = math::constrain(yaw_weight, 0.f, 1.f);
    matrix::Vector3f _kp = proportional_gain;
    // compensate for the effect of the yaw weight rescaling the output
    if (_yaw_w > 1e-4f) {
      _kp(2) /= _yaw_w;
    }
    _gains_att.set_kp(_kp);
  }
  void set_pos_gains(const matrix::Vector3f &_kp, const matrix::Vector3f &_kd,
                     const matrix::Vector3f &_ki);
  void set_vel_gains(const matrix::Vector3f &_kp, const matrix::Vector3f &_kd,
                     const matrix::Vector3f &_ki);
  void set_yaw_sp(const float &_yaw) {
    yaw_sp = _yaw;
//    std::cout << "yaw setpoint changed to " << _yaw * 180.f / M_PI << std::endl;
  }
  float get_yaw_sp() const { return yaw_sp; }
  float lyap() const { return lyap_; }
  matrix::Matrix3f inertia_matrix() const { return inertia_matrix_; }

  /**
   * Function to set the scaled_thrust value
   * @param _th_scaled (range between 0 & 1)
   */
  void set_thrust_scaled(const float &_th_scaled) {
    this->input_.thrust_scaled = _th_scaled;
  }
  /**
   * Set the thrust value (magnitude)
   * @param _th
   */
  void set_thrust(float _th) { input_.thrust = _th; }
  /**
   * Set the desired Thrust vector
   * @param fx
   * @param fy
   * @param fz
   */
  void set_thrust_vector(float fx, float fy, float fz) {
    input_.thrust_vector = {fx, fy, fz};
  }
  /**
   * Set the desired thrust vector
   * @param thrust_vector
   */
  void set_thrust_vector(const matrix::Vector3f &thrust_vector) {
    input_.thrust_vector = thrust_vector;
  }
};

} // namespace qrotor_firmware

#endif /* __QROTOR_FIRMWARE_ATTITUDE_CONTROLLER_H__ */
