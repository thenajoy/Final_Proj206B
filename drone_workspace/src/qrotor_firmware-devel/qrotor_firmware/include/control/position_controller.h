#ifndef QROTOR_FIRMWARE_POSITION_CONTROLLER_H
#define QROTOR_FIRMWARE_POSITION_CONTROLLER_H

#include "Matrix/matrix/math.hpp"
#include "data_structures.h"
#include "mathlib/math/filter/LowPassFilter2pVector3f.hpp"
#include "parameters.h"
#include "control/control_math.h"
#include "mathlib/mathlib.h"

namespace qrotor_firmware {
class FlightController;

class PositionController {
protected:
  ///  Flight controller firmware
  FlightController &firmware_;
  /// Command pose (setpoint/trajectories)
  PoseWithCovariance cmd_pose_;
  /// Integral update flag
  bool INTEGRAL_UPDATE;
  /// Position gains
  Gains _gains_pos{};
  /// Velocity gains
  Gains _gains_vel{};

  /// \brief position setpoint
  matrix::Vector3f _pos_sp{};
  /// \brief velocity setpoint
  matrix::Vector3f _vel_sp{};
  /// \brief acceleration setpoint
  matrix::Vector3f _acc_sp{};
  /// \brief desired thrust
  matrix::Vector3f _thr_sp{};


  // Limits (from PX4)
  /// \brief Horizontal velocity limit with feed forward and position control "MPC_XY_CRUISE"
  float _lim_vel_horizontal{5.0};
  /// \brief Upwards velocity limit with feed forward and position control "MPC_Z_VEL_MAX_UP"
  float _lim_vel_up{3.0};
  /// \brief Downwards velocity limit with feed forward and position control  "MPC_Z_VEL_MAX_DN"
  float _lim_vel_down{1.0};
  /// \brief TODO Minimum collective thrust allowed as output [-1,0] e.g. -0.9
  float _lim_thr_min{0.12};
  /// \brief TODO Maximum collective thrust allowed as output [-1,0] e.g. -0.1
  float _lim_thr_max{1.0};
  /// \brief Maximum tilt from level the output attitude is allowed to have "MPC_TILTMAX_AIR"
  float _lim_tilt{M_PI_4_F}; // 45-degrees

  /// \brief Thrust [0.1, 0.9] with which the vehicle hovers not accelerating down or up with level orientation
  float _hover_thrust{0.45};

  /// \brief velocity derivative (replacement for acceleration estimate)
  matrix::Vector3f _vel_dot{0.f, 0.f, 0.f};
  /// \brief  integral term of the velocity controller
  matrix::Vector3f _vel_int{0.f, 0.f, 0.f};

  /// Position Integral Error
  matrix::Vector3f pos_integral_err;
  /// Velocity Integral Error
  matrix::Vector3f vel_integral_err;
  /// Position Integral Error Bounds
  matrix::Vector3f POS_INTEGRAL_ERR_LB, POS_INTEGRAL_ERR_UB;
  /// Position Integral Error Bounds
  matrix::Vector3f VEL_INTEGRAL_ERR_LB, VEL_INTEGRAL_ERR_UB;
  /// Thrust Vector Bounds
  matrix::Vector3f THRUST_VECTOR_LB, THRUST_VECTOR_UB;

  /// Loop time
  float dt_{0.002f};

public:
  explicit PositionController(FlightController &_flightcontroller);
  ~PositionController();

  /**
   * Position Controller Initialization
   */
  virtual void init();
  /**
   * Position Control main function to run the control algorithm
   * @param dt
   */
  virtual void run(float dt);

  /**
   * Function to debug and print data
   */
  void debug();

  /* setters */
  /**
   * Reset position integral error to zero
   */
  void reset_pos_integral_err() {
    pos_integral_err.setZero();
  }
  /**
   * Function to update p,i,d gains for position
   * @param _kp
   * @param _kd
   * @param _ki
   */
  void set_pos_gains(const matrix::Vector3f &_kp,
                     const matrix::Vector3f &_kd,
                     const matrix::Vector3f &_ki);
  /**
   * Function to update p,i,d gains for velocity
   * @param _kp
   * @param _kd
   * @param _ki
   */
  void set_vel_gains(const matrix::Vector3f &_kp,
                     const matrix::Vector3f &_kd,
                     const matrix::Vector3f &_ki);
  /**
   * Function to update the position setpoint
   * @param _pos
   */
  void set_cmd_setpoint(const matrix::Vector3f &_pos);
  /**
   * Function to update the position setpoint
   * @param px
   * @param py
   * @param pz
   */
  void set_cmd_setpoint(float px, float py, float pz);
  /**
   * Function to update the integral flag
   * @param flag
   */
  void set_pid_flag(bool flag);

  /* getters */
  /**
   * Variable to securely access command pose
   * @return
   */
  inline const PoseWithCovariance &cmd_pose() const { return cmd_pose_; }
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_POSITION_CONTROLLER_H
