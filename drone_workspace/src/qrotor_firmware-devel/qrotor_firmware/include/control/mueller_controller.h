#ifndef QROTOR_FIRMWARE_MUELLER_CONTROLLER_H
#define QROTOR_FIRMWARE_MUELLER_CONTROLLER_H

#include "attitude_controller.h"

namespace qrotor_firmware {

class MuellerController : public AttitudeController  {
  private:

  public:
    MuellerController (FlightController& _flightcontroller);
    ~MuellerController();

    virtual void compute(float _dt) override;

};

}

#endif // QROTOR_FIRMWARE_MUELLER_CONTROLLER_H
