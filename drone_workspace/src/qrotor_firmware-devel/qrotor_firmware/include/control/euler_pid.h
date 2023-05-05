#ifndef __QROTOR_CONTROL_EULER_PID_H__
#define __QROTOR_CONTROL_EULER_PID_H__


#include "control/attitude_controller.h"


namespace qrotor_firmware {

class EulerPID : public AttitudeController {

  private:
    void rate_pid_rough(matrix::Vector3f cmd_ang_vel); /* rough implementation of the pid controller for angular rates */
    void rate_pid_rosflight();

  public:

  explicit EulerPID(FlightController& _flightcontroller);
    ~EulerPID();

    virtual void compute(float _dt) override;
};

}

#endif /* __QROTOR_CONTROL_EULER_ANGLE_CONTROLLER_H__ */
