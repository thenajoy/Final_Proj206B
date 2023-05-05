//
// Created by kotaru on 3/11/21.
//
#ifndef __QROTOR_FIRMWARE_CONTROL_ATTITUDE_PX4_H__
#define __QROTOR_FIRMWARE_CONTROL_ATTITUDE_PX4_H__

#include "control/attitude_controller.h"
namespace qrotor_firmware {

class AttitudeControlPX4 : public AttitudeController {
/* source:  https://github.com/PX4/PX4-Autopilot/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp
 */
protected:
  matrix::Vector3f _rate_limit{0.4, 0.4, 0.2};

  matrix::Quatf _attitude_setpoint_q; ///< latest known attitude setpoint e.g. from position control
  float _yawspeed_setpoint{0.f}; ///< latest known yawspeed feed-forward setpoint

  void rate_control_updateIntegral(matrix::Vector3f &rate_error, const float dt);

  // Gains
  matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
  matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters

  // States
  matrix::Vector3f _rate_int; ///< integral term of the rate controller

  bool _mixer_saturation_positive[3]{};
  bool _mixer_saturation_negative[3]{};

  matrix::Vector3f MAX_BODY_MOMENT{1.65, 1.65, 0.0155  }; // Nm

public:
  explicit AttitudeControlPX4(FlightController &_flightcontroller);
  ~AttitudeControlPX4() = default;

  bool init() override;
  void compute(float _dt) override;

  /**
   * Set hard limit for output rate setpoints
   * @param rate_limit [rad/s] 3D vector containing limits for roll, pitch, yaw
   */
  void setRateLimit(const matrix::Vector3f &rate_limit) { _rate_limit = rate_limit; }

  /**
   * Set a new attitude setpoint replacing the one tracked before
   * @param qd desired vehicle attitude setpoint
   * @param yawspeed_setpoint [rad/s] yaw feed forward angular rate in world frame
   */
  void setAttitudeSetpoint(const matrix::Quatf &qd, const float yawspeed_setpoint) {
    _attitude_setpoint_q = qd;
    _attitude_setpoint_q.normalize();
    _yawspeed_setpoint = yawspeed_setpoint;
  }

  /**
   * Adjust last known attitude setpoint by a delta rotation
   * Optional use to avoid glitches when attitude estimate reference e.g. heading changes.
   * @param q_delta delta rotation to apply
   */
  void adaptAttitudeSetpoint(const matrix::Quatf &q_delta) { _attitude_setpoint_q = q_delta * _attitude_setpoint_q; }

  /**
   * Run one control loop cycle calculation
   * @param q estimation of the current vehicle attitude unit quaternion
   * @return [rad/s] body frame 3D angular rate setpoint vector to be executed by the rate controller
   */
  matrix::Vector3f update(const matrix::Quatf &q) const;

  /**
 * Run one control loop cycle calculation
 * @param rate estimation of the current vehicle angular rate
 * @param rate_sp desired vehicle angular rate setpoint
 * @param dt desired vehicle angular rate setpoint
 * @return [-1,1] normalized torque vector to apply to the vehicle
 */
  matrix::Vector3f rate_control_update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
                                       const matrix::Vector3f &angular_accel, const float dt, const bool landed);

};

};

#endif //__QROTOR_FIRMWARE_CONTROL_ATTITUDE_PX4_H_
