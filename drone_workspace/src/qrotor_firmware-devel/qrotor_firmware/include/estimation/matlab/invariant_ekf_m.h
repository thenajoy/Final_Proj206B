//
// Created by kotaru on 3/31/21.
//

#ifndef QROTOR_FIRMWARE_MATLAB_INVARIANT_EKF_H_
#define QROTOR_FIRMWARE_MATLAB_INVARIANT_EKF_H_

#include <cstddef>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace matlab {

class InvariantEKFm {
private:
  float covar[81] = {
      0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F};
public:
  InvariantEKFm();
  ~InvariantEKFm();

  float R[9] = {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f};
  float v[3] = {0.f, 0.f, 0.f};
  float p[3] = {0.f, 0.f, 0.f};

  void init();
  void timeUpdate(float dt, const float accel[3], const float gyro[3]);
  void measUpdate_tcam(const float Rm[9], const float vm[3], const float pm[3]);
  void measUpdate_mocap();

};

}

#endif //QROTOR_FIRMWARE_MATLAB_INVARIANT_EKF_H_
