//
// Created by kotaru on 4/11/21.
//
#include "control/position_clf_qp.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

PositionClfQP::PositionClfQP(FlightController &_flightcontroller)
    : PositionController(_flightcontroller) {}

PositionClfQP::~PositionClfQP() = default;

void PositionClfQP::run(float dt) {
  this->dt_ = dt;

  run_clf_qp();
}

bool PositionClfQP::run_clf_qp() {
  return false;
}

} // namespace qrotor_firmware
