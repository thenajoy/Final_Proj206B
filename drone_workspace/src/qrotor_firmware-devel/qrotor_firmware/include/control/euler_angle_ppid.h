#ifndef __QROTOR_CONTROL_EULER_ANGLE_PPID_H__
#define __QROTOR_CONTROL_EULER_ANGLE_PPID_H__


#include "control/attitude_controller.h"


namespace qrotor_firmware {

class EulerAnglePPID : public AttitudeController {

  private:
    /**
     * Rough implementation of the pid controller for angular rates
     * @param cmd_ang_vel
     */
    void rate_pid_rough(const matrix::Vector3f& cmd_ang_vel);
    void rate_pid_rosflight();

  public:

    explicit EulerAnglePPID(FlightController& _flightcontroller);
    ~EulerAnglePPID();

    virtual void compute(float _dt) override;
    virtual bool init() override;
};

}

#endif /* __QROTOR_CONTROL_EULER_ANGLE_CONTROLLER_H__ */
