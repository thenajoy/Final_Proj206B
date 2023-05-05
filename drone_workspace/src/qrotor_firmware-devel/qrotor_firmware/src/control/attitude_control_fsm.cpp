#include "control/attitude_control_fsm.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeControlFSM::AttitudeControlFSM(FlightController &_flightcontroller)
    : firmware_(_flightcontroller) {

  // Initializing all the controllers
  controllers_.invalid = nullptr;
  controllers_.euler_ppid = new EulerAnglePPID(firmware_);
  controllers_.att_geometric = new AttitudeGeometricController(firmware_);
  controllers_.att_geom_clf_qp = new AttitudeGeometricClfQP(firmware_);

  init();
}

AttitudeControlFSM::~AttitudeControlFSM() = default;

void AttitudeControlFSM::init() {
  setCurrentController(AttitudeControlFSM::EULER_PPID);
}

void AttitudeControlFSM::setCurrentController(
    AttitudeControlFSM::Name ctrlName) {
  refreshCurrentController(ctrlName);
}

void AttitudeControlFSM::refreshCurrentController(
    AttitudeControlFSM::Name ctrlName) {
  switch (ctrlName) {
  case AttitudeControlFSM::EULER_PPID:
    Logger::SUCCESS("changing attitude controller to: EULER_PPID ");
    currentController = controllers_.euler_ppid;
    break;

  case GEOMETRIC_CONTROL:
    Logger::SUCCESS("changing attitude controller to: GEOMETRIC_CONTROL ");
    currentController = controllers_.att_geometric;
    break;

  case ATTITUDE_CLF_QP:
    Logger::SUCCESS("changing attitude controller to: ATTITUDE_CLF_QP ");
    currentController = controllers_.att_geom_clf_qp;
    break;

  case ATTITUDE_CBF_QP:
    Logger::ERROR("AttitudeControlFSM invalid option");
    break;

  case ATTITUDE_CLFCBF_QP:
    Logger::ERROR("AttitudeControlFSM invalid option");

    break;

  case INVALID:
    Logger::SUCCESS("Invalid option!"
                    "changing attitude controller to: ATTITUDE_CLF_QP ");
    currentController = controllers_.euler_ppid;
    break;

  default:
    currentController = controllers_.euler_ppid;
    break;
  }
  currentController->init();
  currentControllerName = ctrlName;
}

void AttitudeControlFSM::run(float dt) { currentController->run(dt); }

} // namespace qrotor_firmware
