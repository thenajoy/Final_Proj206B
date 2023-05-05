//
// Created by kotaru on 6/5/21.
//
#include "mixer2.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

Mixer2::Mixer2(FlightController &_flightController)
    : firmware_(_flightController) {

}

Mixer2::~Mixer2() = default;

void Mixer2::init() {}

void Mixer2::run() {}

void Mixer2::allocate_pwms() {}

} // namespace qrotor_firmware
