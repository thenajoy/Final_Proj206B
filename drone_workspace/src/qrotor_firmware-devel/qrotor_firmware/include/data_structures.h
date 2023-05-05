#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#ifndef __QROTOR_FIRMWARE_DATA_STRUCTURES_H__
#define __QROTOR_FIRMWARE_DATA_STRUCTURES_H__

#include <Matrix/matrix/math.hpp>
#include <iostream>
#include <string>

namespace qrotor_firmware {

class IMURaw {
public:
  IMURaw() {
    timestamp_us = 0;
    reset();
  }
  ~IMURaw() = default;

  matrix::Vector3f gyro;
  matrix::Vector3f accel;
  matrix::Vector3f mag;
  unsigned long timestamp_us;

  void reset() {
    gyro.zero();
    accel.zero();
    mag.zero();
    timestamp_us = 0;
  }
};

class PoseWithCovariance {
public:
  PoseWithCovariance() {
    timestamp_us = 0;
    reset();
  }
  ~PoseWithCovariance() = default;

  unsigned long timestamp_us;
  matrix::Vector3f position;
  matrix::Vector3f velocity;
  matrix::Vector3f acceleration;
  matrix::Quatf quat;
  matrix::Matrix3f rotation;
  matrix::Vector3f ang_vel;
  matrix::Vector3f ang_accel;
  matrix::Vector3f accel_bias;
  matrix::Vector3f gyro_bias;
  float f{0.f};
  matrix::Vector3f moment;

  // TODO: add covariances

  void reset() {
    f = 0;
    moment.zero();
    position.zero();
    velocity.zero();
    acceleration.zero();
    accel_bias.zero();
    gyro_bias.zero();
    rotation.zero();
    ang_vel.zero();
    ang_accel.zero();
    quat = matrix::Quatf();
    timestamp_us = 0;
  }

  void print() const {
    std::cout << "--------" << std::endl;
    std::cout << "pos: x: " << position(0) << " y: " << position(1)
              << " z: " << position(2) << std::endl;
    std::cout << "vel: x: " << velocity(0) << " y: " << velocity(1)
              << " z: " << velocity(2) << std::endl;
    std::cout << "quat: w: " << quat(0) << " x: " << quat(1)
              << " y: " << quat(2) << " z: " << quat(3) << std::endl;
    std::cout << "ang_vel: x: " << ang_vel(0) << " y: " << ang_vel(1)
              << " z: " << ang_vel(2) << std::endl;
    std::cout << "--------" << std::endl;
  }
};

class Attitude {
private:
  matrix::Quatf q_;
  matrix::Dcmf rot_;

public:
  Attitude() {
    timestamp_us = 0;
    reset();
  }
  ~Attitude() = default;

  matrix::Vector3f ang_vel;
  matrix::Vector3f ang_vel_rates;
  matrix::Vector3f linear_accel;
  matrix::Eulerf euler;
  uint64_t timestamp_us;

  void reset() {
    euler.zero();
    ang_vel.zero();
    ang_vel_rates.zero();
    q_ = matrix::Quatf();
    timestamp_us = 0;
  }

  /// setters
  inline void set_euler(const matrix::Vector3f &_euler) { euler = _euler; }
  inline void set_euler(float r, float p, float y) {
    euler(0) = r;
    euler(1) = p;
    euler(2) = y;
  }
  inline void set_quat(float q0, float q1, float q2, float q3) {
    q_(0) = q0;
    q_(1) = q1;
    q_(2) = q2;
    q_(3) = q3;
  }
  inline void set_quat(const matrix::Quatf& _q) {
    q_ = {_q(0), _q(1), _q(2), _q(3)};
  }
  inline void set_axis2Dcm(matrix::Vector3f b1, matrix::Vector3f b2,
                           matrix::Vector3f b3) {
    for (int i = 0; i < 3; i++) {
      rot_(i, 0) = b1(i);
      rot_(i, 1) = b2(i);
      rot_(i, 2) = b3(i);
    }
    refreshDcm();
  }

  inline void refreshEuler() {
    q_ = matrix::Quatf(euler);
    rot_ = matrix::Dcmf(euler);
  }
  inline void refreshQuat() {
    rot_ = matrix::Dcmf(q_);
    euler = matrix::Eulerf(q_);
  }
  inline void refreshDcm() {
    euler = matrix::Eulerf(rot_);
    q_ = matrix::Quatf(rot_);
  }

  /// getters
  inline float roll() const { return euler.phi(); }
  inline float pitch() const { return euler.theta(); }
  inline float yaw() const { return euler.psi(); }
  inline float yaw_rate() const { return ang_vel(2); }
  inline matrix::Quatf q() const { return q_; }
  inline matrix::Dcmf R() const { return rot_; }
  inline matrix::Vector3f Re3() const {
    return {rot_(0, 2), rot_(1, 2), rot_(2, 2)};
  }
};

class InputWrench {
public:
  InputWrench() { zero(); }
  ~InputWrench() = default;

  inline void zero() {
    thrust = 0.0;
    thrust_vector.zero();
    moment.zero();
    pwm_us.zero();
    for (int i = 0; i < 4; ++i) {
      this->pwm_us_array[i] = 0;
    }
  }
  float thrust{0};
  float thrust_scaled{0}; // [0,1)
  matrix::Vector3f thrust_vector;
  matrix::Vector3f moment;
  matrix::Matrix<float, 4, 1> pwm_us;
  float pwm_us_array[4]{};
};

class Gains {
private:
  matrix::Vector3f kp_;
  matrix::Vector3f kd_;
  matrix::Vector3f ki_;

public:
  Gains() {
    kp_.zero();
    kd_.zero();
    ki_.zero();
  }

  Gains(matrix::Vector3f _kp, matrix::Vector3f _kd, matrix::Vector3f _ki) {
    kp_ = _kp;
    kd_ = _kd;
    ki_ = _ki;
  }

  inline matrix::Vector3f kp() const { return kp_; }
  inline matrix::Vector3f kd() const { return kd_; }
  inline matrix::Vector3f ki() const { return ki_; }

  void set_gains(const matrix::Vector3f &_kp, const matrix::Vector3f &_kd,
                 const matrix::Vector3f &_ki) {
    kp_ = _kp;
    kd_ = _kd;
    ki_ = _ki;

    // std::cout << "gains updated: kp" << std::endl;
    // kp_.print();
    // std::cout << "kd: " << std::endl;
    // kd_.print();
    // std::cout << "ki: " << std::endl;
    // ki_.print();
  }
  void set_kp(matrix::Vector3f &_kp) { kp_ = _kp; }
  void set_kd(matrix::Vector3f &_kd) { kd_ = _kd; }
  void set_ki(matrix::Vector3f &_ki) { ki_ = _ki; }
};

class RadioChannel {
public:
  int CHANNEL_ID = 0;
  int reading = 1514;
  float max = 1924;
  float min = 1104;
  float center = 1514;
  float trim = 0;
  float hrange = 400; // half range

  void init(int _ch) { CHANNEL_ID = _ch; }
  void init(int _ch, float _min, float _max, float _center) {
    CHANNEL_ID = _ch;
    max = _max;
    min = _min;
    center = _center;
    hrange = ((_max - _min) / 2);
  }
};

class RadioInput {
public:
  RadioChannel thrust;
  RadioChannel roll;
  RadioChannel pitch;
  RadioChannel yaw;
  RadioChannel mode;
  RadioChannel safety_switch;
  RadioChannel switch1;
  RadioChannel switch2;
};

struct DataContainer {
  Attitude attitude;
  Attitude cmd_attitude;
  PoseWithCovariance pose;
  PoseWithCovariance cmd_pose;
  InputWrench input;
};

// class FlightController;
typedef struct TerminalColors {
  std::string GREEN = "\033[01;32m";
  std::string NC = "\033[0m"; // No Color
  std::string BLACK = "\033[01;30m";
  std::string RED = "\033[01;31m";
  std::string YELLOW = "\033[01;33m";
  std::string BLUE = "\033[01;34m";
  std::string MAGENTA = "\033[01;35m";
  std::string CYAN = "\033[01;36m";
  std::string WHITE = "\033[0;37m";
  std::string Reset = "\033[0m";
} tColors;

struct QrotorFlats {
  float p[3] = {0.f, 0.f, 0.f};
  float v[3] = {0.f, 0.f, 0.f};
  float a[3] = {0.f, 0.f, 0.f};
  float da[3] = {0.f, 0.f, 0.f};
  float d2a[3] = {0.f, 0.f, 0.f};
  float b1[3] = {1.f, 0.f, 0.f};
  float db1[3] = {0.f, 0.f, 0.f};
  float d2b1[3] = {0.f, 0.f, 0.f};
};

struct QuadrotorState {
  float p[3] = {0.f, 0.f, 0.f};
  float v[3] = {0.f, 0.f, 0.f};
  float a[3] = {0.f, 0.f, 0.f};
  float da[3] = {0.f, 0.f, 0.f};
  float d2a[3] = {0.f, 0.f, 0.f};
  float R[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
  float Omega[3] = {0.f, 0.f, 0.f};
  float dOmega[3] = {0.f, 0.f, 0.f};
  float F[3] = {0.f, 0.f, 0.f};
  float f{0.f};
  float M[3] = {0.f, 0.f, 0.f};
};

} // namespace qrotor_firmware

#endif /* __QROTOR_FIRMWARE_DATA_STRUCTURES_H__ */

#pragma clang diagnostic pop