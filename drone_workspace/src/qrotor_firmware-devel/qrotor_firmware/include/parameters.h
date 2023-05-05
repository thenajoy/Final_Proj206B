#ifndef __QROTOR_CONTROL_PARAMETERS_H__
#define __QROTOR_CONTROL_PARAMETERS_H__

/*
Borrowed from
https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_control/include/rotors_control/parameters.h
*/

#define _USE_MATH_DEFINES

#include <cmath>
#include <Matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <vector>
#include <iostream>

namespace qrotor_firmware {

#define G_SI 9.80665
#define G_SI_F 9.80665f
static const matrix::Vector3f E1(1.0, 0.0, 0.0);
static const matrix::Vector3f E2(0.0, 1.0, 0.0);
static const matrix::Vector3f E3(0.0, 0.0, 1.0);

/********************************************
// #   (2)CW    CCW(0)           y^
// #        \___/                 |
// #         |_|                  |
// #        /   \                 |
// #  (1)CCW     CW(3)           z.------> x
********************************************/

// Default values for the Mark-I rotor configuration.
static constexpr int kDefaultNoOfRotors = 4;
static constexpr float kDefaultRotorAngles[4] = {M_PI_4, (5 * M_PI_4), (3 * M_PI_4), (7 * M_PI_4)}; // rads

// Default vehicle parameters for Mark-I
static constexpr float kDefaultRotorForceConstants[4] =
    {0.0000063983308147, 0.0000067233514341, 0.0000071046901669, 0.0000063988289979}; // N/(rad/s)^2
static constexpr float kDefaultRotorForceOffsets[4] =
    {-0.0386953486892228, -0.1196745867898153, -0.0276828237811946, -0.0446194039873629}; // N
static constexpr float kDefaultRotorMomentConstants[4] =
    {0.0000001047389761, 0.0000001099407241, 0.0000001024806446, 0.0000000932989003}; // (Nm)/(rad/s)^2
static constexpr float kDefaultRotorMomentOffsets[4] =
    {0.0006480653804795, 0.0008455143290322, -0.0006829572570590, 0.0022646832884231}; // (Nm)
static constexpr float kDefaultRotorMomentDirections[4] = {-1, -1, 1, 1};

static constexpr float kDefaultMass = 0.9;
static constexpr float kDefaultArmLength = 0.1524;
static constexpr float kDefaultInertiaXx = 0.0104;
static constexpr float kDefaultInertiaYy = 0.0104;
static constexpr float kDefaultInertiaZz = 0.0196;

static constexpr float kDefaultIxx = 0.005315307431627; // kg*m^2
static constexpr float kDefaultIxy = 0.000005567447099; // kg*m^2
static constexpr float kDefaultIxz = 0.000005445855427; // kg*m^2

static constexpr float kDefaultIyx = 0.000005567447099; // kg*m^2
static constexpr float kDefaultIyy = 0.004949258422243; // kg*m^2
static constexpr float kDefaultIyz = 0.000020951458431; // kg*m^2

static constexpr float kDefaultIzx = 0.000005445855427; // kg*m^2
static constexpr float kDefaultIzy = 0.000020951458431; // kg*m^2
static constexpr float kDefaultIzz = 0.009806225007686; // kg*m^2

static constexpr float kDefaultRotorForceConstant = 4.104890333e-6; // 10.64e-06; // 6.656300353404825e-06;
static constexpr float kDefaultRotorForceOffset = 0; //-0.057668040811899;
static constexpr float kDefaultRotorMomentConstant = 1.026e-07; // 1.026148112683309e-07;
static constexpr float kDefaultRotorMomentOffset = 7.68e-04; // 0; //7.688264352189356e-04;

static constexpr float kDefaultPWM2RotorSpeed_scale = 1166.3755960945295556;
static constexpr float kDefaultPWM2RotorSpeed_offset = -1156.5120044647383111;
//static constexpr float kDefaultRotorspeed2PWM_scale = 0.0008514439377261; // rad/s to milli-seconds
//static constexpr float kDefaultRotorspeed2PWM_offset = 0.9946035424300900; // rad/s to milli-seconds
static constexpr float kDefaultRotorspeed2PWM_scale = 0.7113841082512; // 0.8514439377261; // rad/s to micro-seconds
static constexpr float kDefaultRotorspeed2PWM_offset = 969.4061952591920; //994.6035424300900; // rad/s to micro-seconds

// Default physics parameters.
static constexpr float kDefaultGravity = 9.80665;

struct Rotor {
  Rotor()
      : angle(0.0), arm_length(kDefaultArmLength),
        rotor_force_constant(kDefaultRotorForceConstant),
        rotor_force_offset(kDefaultRotorForceOffset),
        rotor_moment_constant(kDefaultRotorMomentConstant),
        rotor_moment_offset(kDefaultRotorMomentOffset), direction(1) {}
  Rotor(float _angle, float _arm_length, float _rotor_force_constant,
        float _rotor_force_offset, float _rotor_moment_constant,
        float _rotor_moment_offset, int _direction)
      : angle(_angle), arm_length(_arm_length),
        rotor_force_constant(_rotor_force_constant),
        rotor_force_offset(_rotor_force_offset),
        rotor_moment_constant(_rotor_moment_constant),
        rotor_moment_offset(_rotor_moment_offset), direction(_direction) {}
  float angle;
  float arm_length;
  float rotor_force_constant;
  float rotor_force_offset;
  float rotor_moment_constant;
  float rotor_moment_offset;
  matrix::Vector3f position;
  matrix::Vector3f normal;
  int direction;
};

struct RotorConfiguration {
  RotorConfiguration() {
    // Rotor configuration.
    rotors.push_back(Rotor(kDefaultRotorAngles[0], kDefaultArmLength, kDefaultRotorForceConstant,
                           kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                           kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[0]));
    rotors.push_back(Rotor(kDefaultRotorAngles[1], kDefaultArmLength, kDefaultRotorForceConstant,
                           kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                           kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[1]));
    rotors.push_back(Rotor(kDefaultRotorAngles[2], kDefaultArmLength, kDefaultRotorForceConstant,
                           kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                           kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[2]));
    rotors.push_back(Rotor(kDefaultRotorAngles[3], kDefaultArmLength, kDefaultRotorForceConstant,
                           kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                           kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[3]));
  }
  std::vector<Rotor> rotors;
};

static constexpr float _J[12] = {kDefaultIxx, kDefaultIxy, kDefaultIxz,
                                 kDefaultIyx, kDefaultIyy, kDefaultIyz,
                                 kDefaultIzx, kDefaultIzy, kDefaultIzz
};

class VehicleParameters {

private:
public:
  VehicleParameters()
      : mass_(kDefaultMass), g_(kDefaultGravity), inertia_(_J),
        pwm2rotorspeed_scale_(kDefaultPWM2RotorSpeed_scale),
        pwm2rotorspeed_offset_(kDefaultPWM2RotorSpeed_offset),
        rotorspeed2pwm_scale_(kDefaultRotorspeed2PWM_scale),
        rotorspeed2pwm_offset_(kDefaultRotorspeed2PWM_offset),
        num_rotors_(kDefaultNoOfRotors) {

    rotors_.clear();
    // Rotor configuration.
    rotors_.push_back(Rotor(kDefaultRotorAngles[0], kDefaultArmLength, kDefaultRotorForceConstant,
                            kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                            kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[0]));
    rotors_.push_back(Rotor(kDefaultRotorAngles[1], kDefaultArmLength, kDefaultRotorForceConstant,
                            kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                            kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[1]));
    rotors_.push_back(Rotor(kDefaultRotorAngles[2], kDefaultArmLength, kDefaultRotorForceConstant,
                            kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                            kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[2]));
    rotors_.push_back(Rotor(kDefaultRotorAngles[3], kDefaultArmLength, kDefaultRotorForceConstant,
                            kDefaultRotorForceOffset, kDefaultRotorMomentConstant,
                            kDefaultRotorMomentOffset, kDefaultRotorMomentDirections[3]));
  }
  float mass_;
  matrix::Matrix3f inertia_;
  const float g_;
  std::vector<Rotor> rotors_;
  matrix::Vector3f euler_offset;
  matrix::Vector3f imu_accel_offset;
  matrix::Vector3f imu_gyro_offset;

  float rotor_force_constant_{};
  float rotor_force_offset_{};
  float rotor_moment_constant_{};
  float rotor_moment_offset_{};

  float pwm2rotorspeed_scale_;
  float pwm2rotorspeed_offset_;
  float rotorspeed2pwm_scale_;
  float rotorspeed2pwm_offset_;
  int num_rotors_;
  float max_thrust_{};
  std::vector<double> cameraEul;

  // setters
  /// \brief update rotor configuration
  void update_rotors(const std::vector<Rotor> &_rotors) {
    rotors_ = _rotors;
  }
  /// \brief set mass value
  void update_mass(const float &_mass) {
    mass_ = _mass;
  }
  /// \brief update inertia matrix
  void update_inertia(const matrix::Matrix3f &_inertia) {
    inertia_ = _inertia;
  }

  // getters
  /// \brief vehicle mass
  float mass() const { return mass_; }
  /// \brief vehicle inertia matrix
  matrix::Matrix3f inertia() const {
    return inertia_;
  }

  /**
   * debug print
   */
  void print() const {
    printf("force constant: %fe-06, force offset: %fe-04, torque constant: %fe-06, torque offset: %fe-04",
           rotor_force_constant_ * 1e6,
           rotor_force_offset_ * 1e4,
           rotor_moment_constant_ * 1e6,
           rotor_moment_offset_ * 1e4);
    std::cout << std::fixed << std::endl;
    std::cout << "angles: ";
    for (int i = 0; i < num_rotors_; ++i) {
      std::cout << i << ": " << rotors_.at(i).angle;
    }
    std::cout << std::endl;
  }
};

#define ROLL_CHANNEL            0
#define PITCH_CHANNEL           1
#define THRUST_CHANNEL          2
#define YAW_CHANNEL             3

#define SAFETY_SWITCH_CHANNEL   4
#define MODE_CHANNEL            6
#define SWITCH1_CHANNEL         7
#define SWITCH2_CHANNEL         5

#define PARAM_RC_X_CHANNEL      0
#define PARAM_RC_Y_CHANNEL      1
#define PARAM_RC_F_CHANNEL      2
#define PARAM_RC_Z_CHANNEL      3

#define PARAM_RC_SAFETY_CHANNEL 4
#define PARAM_RC_MODE_CHANNEL   6
#define PARAM_RC_SW1_CHANNEL    7
#define PARAM_RC_SW2_CHANNEL   5
#define PARAM_RC_NUM_CHANNELS 8

#define PWM_NUM_OUTPUTS 4
static const unsigned int PWM_CHANNELS[PWM_NUM_OUTPUTS] = {0, 1, 2, 3};

class NoiseParameters {
public:
  NoiseParameters() {
    gyro.bias.zero();
    gyro.covar.zero();

    accel.bias.zero();
    accel.covar.zero();

    mag.bias.zero();
    mag.covar.zero();
  }
  ~NoiseParameters() = default;

  struct RandomVariable {
    RandomVariable() = default;
    matrix::Vector3f bias;
    matrix::Vector3f covar;
  };

  RandomVariable gyro;
  RandomVariable accel;
  RandomVariable mag;

  void setGyroNoise(const float *b, const float *c) {
    gyro.bias = {b[0], b[1], b[2]};
    gyro.covar = {c[0], c[1], c[2]};
  }

  void setAccelNoise(const float *b, const float *c) {
    accel.bias = {b[0], b[1], b[2]};
    accel.covar = {c[0], c[1], c[2]};
  }

  void setMagNoise(const float *b, const float *c) {
    mag.bias = {b[0], b[1], b[2]};
    mag.covar = {c[0], c[1], c[2]};
  }

};

/* tuneable parameters */
static const int32_t PARAM_ARM_F_THRESHOLD = 1200;
static const int32_t PARAM_ARM_Z_THRESHOLD = 1800;
static const int32_t PARAM_DISARM_Z_THRESHOLD = 1250;
static const bool PARAM_CALIBRATE_GYRO_ON_ARM = false;

static const float RC_MIN_READING = 964;
static const float RC_MID_READING = 1514;
static const float RC_MAX_READING = 2064;
static const float RC_HALF_RANGE = 400;

} // namespace qrotor_firmware

#endif /* __QROTOR_CONTROL_PARAMETERS_H__ */
