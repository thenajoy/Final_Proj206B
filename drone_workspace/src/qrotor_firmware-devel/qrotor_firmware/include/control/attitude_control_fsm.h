#ifndef __QROTOR_FIRMWARE_ATTITUDE_CONTROL_FSM_H__
#define __QROTOR_FIRMWARE_ATTITUDE_CONTROL_FSM_H__

#include "control/controllers.h"

namespace qrotor_firmware {
class FlightController;

class AttitudeControlFSM {
public:
  enum Name {
    INVALID,
    EULER_PPID,
    GEOMETRIC_CONTROL,
    ATTITUDE_CLF_QP,
    ATTITUDE_CBF_QP,
    ATTITUDE_CLFCBF_QP,
  };
  struct AttitudeControlList {
    AttitudeController *invalid = nullptr;
    AttitudeController *euler_ppid = nullptr;
    AttitudeController *att_geometric = nullptr;
    AttitudeController *att_geom_clf_qp = nullptr;
  };

private:
  AttitudeControlList controllers_{};
  AttitudeController *currentController{};
  Name currentControllerName;

  FlightController &firmware_;

  void refreshCurrentController(AttitudeControlFSM::Name ctrlName);

public:
  explicit AttitudeControlFSM(FlightController &_flightcontroller);
  ~AttitudeControlFSM();

  void init();
  void run(float dt);
  void setCurrentController(AttitudeControlFSM::Name ctrlName);
};

} // namespace qrotor_firmware

#endif // __QROTOR_FIRMWARE_ATTITUDE_CONTROL_FSM_H__
