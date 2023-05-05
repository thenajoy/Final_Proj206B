//
// Created by kotaru on 4/27/21.
//

#ifndef QROTOR_FIRMWARE_TRAJECTORY_H_
#define QROTOR_FIRMWARE_TRAJECTORY_H_

#include <cmath>
#include <cstdint>
#include <iostream>

#include "Matrix/matrix/math.hpp"
#include "data_structures.h"
#include "mathlib/math/filter/LowPassFilter2pVector3f.hpp"
#include "parameters.h"

namespace qrotor_firmware {

class Trajectory {
protected:
  /// Flat variable
  QrotorFlats *flats_ = nullptr;

public:
  Trajectory() { flats_ = new QrotorFlats(); }
  ~Trajectory() = default;

  /// Compute flat variables
  virtual void run(const float &t) {}

  /// Flat variables
  QrotorFlats flats() const { return *flats_; }

  /// Flat Variables pointer
  QrotorFlats *flats_ptr() { return flats_; }

  /// setpoint
  void set_setpoint(const matrix::Vector3f &_setpoint) {
    _setpoint.copyToColumnMajor(flats_->p);
  }


  /// parameters
  float p0[3] = {0.f, 0.f, 0.f};
  float r[3] = {1.f, 1.f, 0.f};
  float phi[3] = {0.f, 0.f, 0.f};
  float w[3] = {M_PI_4_F, M_PI_4_F, 0.f};
  float yaw = 0.f;
  float yaw_rate = 0.f;
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_TRAJECTORY_H_
