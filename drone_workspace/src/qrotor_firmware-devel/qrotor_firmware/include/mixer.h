#ifndef QROTOR_FIRMWARE_MIXER_H
#define QROTOR_FIRMWARE_MIXER_H

#include "parameters.h"
#include "data_structures.h"

namespace qrotor_firmware {

class FlightController;

class Mixer {
private:
  FlightController &firmware_;

  // allocation matrix (forces to pwms)
  matrix::Matrix<float, 4, 4> allocation_matrix_;
  matrix::Matrix<float, 4, 1> allocation_offset_;
  matrix::Matrix<float, 4, 4> forces_to_rotor_speed_;

  void allocate_pwms();
  float pwm_us_array_[4] = {1000.0, 1000.0, 1000.0, 1000.0};
  float pwm_us_array_old[4] = {1000.0, 1000.0, 1000.0, 1000.0};
  float alpha = 0.3;

  void write_to_board();

public:
  explicit Mixer(FlightController &_flightController);

  void init();
  void run();
  inline const float *pwm_us_array() const {
    return pwm_us_array_;
  }

};

}

#endif // QROTOR_FIRMWARE_MIXER_H
