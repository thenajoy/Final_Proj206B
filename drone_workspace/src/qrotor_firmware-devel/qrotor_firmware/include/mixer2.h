//
// Created by kotaru on 6/4/21.
//

#ifndef QROTOR_FIRMWARE_MIXER2_H_
#define QROTOR_FIRMWARE_MIXER2_H_

#include "parameters.h"
#include "data_structures.h"

namespace qrotor_firmware {

class FlightController;

class Mixer2 {
protected:
  FlightController &firmware_;

  /// \brief rotor positions
  matrix::Matrix<float, 4, 4> rotor_positions_;
  /// brief rotor axes
  matrix::Matrix<float, 4, 4> rotor_axes_;
  /// \brief rotor directions
  matrix::Matrix<float, 4, 1> rotor_direction_;

  /// \brief allocation matrix (forces to pwms)
  matrix::Matrix<float, 4, 4> allocation_matrix_;
  /// \brief allocation offset
  matrix::Matrix<float, 4, 1> allocation_offset_;
  /// \brief forces to rotor speed
  matrix::Matrix<float, 4, 4> forces_to_rotor_speed_;
  /// \brief allocate pwms
  void allocate_pwms();

  /// \brief esc pwm values
  float pwm_us_array_[4] = {1000.0, 1000.0, 1000.0, 1000.0};
  /// \brief esc pwm previous values
  float pwm_us_array_prev[4] = {1000.0, 1000.0, 1000.0, 1000.0};
  /// \brief alpha
  float alpha = 0.3;

  /**
   * \brief forces parameters to convert forces to ESCs
   */
  matrix::Vector3f p{375.5990256f, 0.7880031f, 948.8590558f};

  /**
   * Function to convert forces to ESCs
   * esc = p0*(sqrt(sqrt(f))^2 + p1*(sqrt(f))+ p2
   * @param _f
   * @return
   */
  float force2esc(const float &_f) {
    float sq_sq_f = sqrt(sqrt(_f));
    return p(0) * sq_sq_f * sq_sq_f + p(1) * sq_sq_f + p(2);
  }

public:
  explicit Mixer2(FlightController &_flightController);
  ~Mixer2();

  void init();
  void run();
  inline const float *pwm_us_array() const { return pwm_us_array_; }
};
} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_MIXER2_H_
