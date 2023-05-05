#include "control/mueller_controller.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

MuellerController::MuellerController(FlightController& _flightcontroller)
    : AttitudeController(_flightcontroller) {

    printf("MuellerController\n");
}

MuellerController::~MuellerController() {

}

void MuellerController::compute(float _dt) {

}

}

