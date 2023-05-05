//
// Created by kotaru on 3/31/21.
//
#include "estimation/matlab/invariant_ekf_m.h"

namespace matlab {

InvariantEKFm::InvariantEKFm() = default;

InvariantEKFm::~InvariantEKFm() = default;

void InvariantEKFm::init() {}

void InvariantEKFm::timeUpdate(float dt, const float accel[3], const float gyro[3]) {
  static const float a[81]{
      0.0F, 0.0F, 0.0F, 0.0F, -9.81F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 9.81F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F};
  static const float fv1[81]{
      0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.01F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0001F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0001F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0001F};
  static const float fv[3]{0.0F, 0.0F, -9.81F};
  static const signed char iv1[81]{
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  float Ak[81];
  float b_Ak[81];
  float K[9];
  float rot[9];
  float b_accel[3];
  float absxk;
  float eta_idx_0;
  float eta_idx_1;
  float f;
  float fcnOutput;
  float scale;
  float t;
  int i;
  int i1;
  int rot_tmp;
  //  predefined variables
  //  computing the rotation
  scale = 1.29246971E-26F;
  f = gyro[0] * dt;
  eta_idx_0 = f;
  absxk = std::abs(f);
  if (absxk > 1.29246971E-26F) {
    fcnOutput = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    fcnOutput = t * t;
  }
  f = gyro[1] * dt;
  eta_idx_1 = f;
  absxk = std::abs(f);
  if (absxk > scale) {
    t = scale / absxk;
    fcnOutput = fcnOutput * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    fcnOutput += t * t;
  }
  f = gyro[2] * dt;
  absxk = std::abs(f);
  if (absxk > scale) {
    t = scale / absxk;
    fcnOutput = fcnOutput * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    fcnOutput += t * t;
  }
  t = scale * std::sqrt(fcnOutput);
  eta_idx_0 /= t;
  eta_idx_1 /= t;
  scale = f / t;
  K[0] = 0.0F;
  K[3] = -scale;
  K[6] = eta_idx_1;
  K[1] = scale;
  K[4] = 0.0F;
  K[7] = -eta_idx_0;
  K[2] = -eta_idx_1;
  K[5] = eta_idx_0;
  K[8] = 0.0F;
  if (t < 1.0E-6F) {
    for (i = 0; i < 9; i++) {
      rot[i] = iv[i];
    }
  } else {
    absxk = std::sin(t);
    scale = std::cos(t);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 3; i1++) {
        rot_tmp = i + 3 * i1;
        rot[rot_tmp] = (static_cast<float>(iv[rot_tmp]) + absxk * K[rot_tmp]) +
            (((1.0F - scale) * K[i] * K[3 * i1] +
                (1.0F - scale) * K[i + 3] * K[3 * i1 + 1]) +
                (1.0F - scale) * K[i + 6] * K[3 * i1 + 2]);
      }
    }
  }
  //  net acceleration in inertial-frame
  b_accel[0] = accel[0];
  b_accel[1] = accel[1];
  b_accel[2] = accel[2];
  //  time-propagating the IMU motion model
  absxk = 0.5F * dt * dt;
  for (i = 0; i < 3; i++) {
    f = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      rot_tmp = i + 3 * i1;
      f += R[rot_tmp] * b_accel[i1];
      K[rot_tmp] = (R[i] * rot[3 * i1] + R[i + 3] * rot[3 * i1 + 1]) +
          R[i + 6] * rot[3 * i1 + 2];
    }
    f += fv[i];
    p[i] = (p[i] + v[i] * dt) + absxk * f;
    v[i] += f * dt;
  }
  for (i = 0; i < 9; i++) {
    R[i] = K[i];
  }
  //  discretization of the linear dynamics
  for (i = 0; i < 81; i++) {
    Ak[i] = static_cast<float>(iv1[i]) + a[i] * dt;
  }
  //  covariance update
  for (i = 0; i < 9; i++) {
    for (i1 = 0; i1 < 9; i1++) {
      f = 0.0F;
      for (rot_tmp = 0; rot_tmp < 9; rot_tmp++) {
        f += Ak[i + 9 * rot_tmp] * covar[rot_tmp + 9 * i1];
      }
      b_Ak[i + 9 * i1] = f;
    }
  }
  for (i = 0; i < 9; i++) {
    for (i1 = 0; i1 < 9; i1++) {
      f = 0.0F;
      for (rot_tmp = 0; rot_tmp < 9; rot_tmp++) {
        f += b_Ak[i + 9 * rot_tmp] * Ak[i1 + 9 * rot_tmp];
      }
      rot_tmp = i + 9 * i1;
      covar[rot_tmp] = f + fv1[rot_tmp];
    }
  }
}

void InvariantEKFm::measUpdate_tcam(const float Rm[9], const float vm[3], const float pm[3]) {
  static const float b[81]{
      0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.001F};
  static const signed char b_a[81]{
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv1[25]{1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
                                   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1};
  static const signed char iv[10]{0, 0, 0, 0, 0, 0, 1, 0, 0, 1};
  float K[81];
  float a[81];
  float b_K[81];
  float fcnOutput[81];
  float x[81];
  float b_Rm[25];
  float eta_log[25];
  float fv[25];
  float hat_delta[25];
  float b_R[9];
  float delta[9];
  float b_eta_tmp[3];
  float eta_tmp[3];
  float s;
  float smax;
  int b_i;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  int kAcol;
  int x_tmp;
  signed char b_p[9];
  signed char ipiv[9];
  std::memset(&fcnOutput[0], 0, 81U * sizeof(float));
  for (i = 0; i < 9; i++) {
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax +=
            static_cast<float>(b_a[i + 9 * x_tmp]) * covar[x_tmp + 9 * jp1j];
      }
      a[i + 9 * jp1j] = smax;
    }
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax += a[i + 9 * x_tmp] * static_cast<float>(b_a[x_tmp + 9 * jp1j]);
      }
      x_tmp = i + 9 * jp1j;
      x[x_tmp] = smax + b[x_tmp];
    }
    ipiv[i] = static_cast<signed char>(i + 1);
  }
  for (j = 0; j < 8; j++) {
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 7 - j;
    b_tmp = j * 10;
    jp1j = b_tmp + 2;
    jA = 9 - j;
    kAcol = 0;
    smax = std::abs(x[b_tmp]);
    for (k = 2; k <= jA; k++) {
      s = std::abs(x[(b_tmp + k) - 1]);
      if (s > smax) {
        kAcol = k - 1;
        smax = s;
      }
    }
    if (x[b_tmp + kAcol] != 0.0F) {
      if (kAcol != 0) {
        jA = j + kAcol;
        ipiv[j] = static_cast<signed char>(jA + 1);
        for (k = 0; k < 9; k++) {
          kAcol = j + k * 9;
          smax = x[kAcol];
          x_tmp = jA + k * 9;
          x[kAcol] = x[x_tmp];
          x[x_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 9;
      for (b_i = jp1j; b_i <= i; b_i++) {
        x[b_i - 1] /= x[b_tmp];
      }
    }
    jA = b_tmp;
    for (kAcol = 0; kAcol <= mmj_tmp; kAcol++) {
      smax = x[(b_tmp + kAcol * 9) + 9];
      if (smax != 0.0F) {
        i = jA + 11;
        jp1j = (jA - j) + 18;
        for (x_tmp = i; x_tmp <= jp1j; x_tmp++) {
          x[x_tmp - 1] += x[((b_tmp + x_tmp) - jA) - 10] * -smax;
        }
      }
      jA += 9;
    }
  }
  for (i = 0; i < 9; i++) {
    b_p[i] = static_cast<signed char>(i + 1);
  }
  for (k = 0; k < 8; k++) {
    signed char i1;
    i1 = ipiv[k];
    if (i1 > k + 1) {
      jA = b_p[i1 - 1];
      b_p[i1 - 1] = b_p[k];
      b_p[k] = static_cast<signed char>(jA);
    }
  }
  for (k = 0; k < 9; k++) {
    x_tmp = 9 * (b_p[k] - 1);
    fcnOutput[k + x_tmp] = 1.0F;
    for (j = k + 1; j < 10; j++) {
      i = (j + x_tmp) - 1;
      if (fcnOutput[i] != 0.0F) {
        jp1j = j + 1;
        for (b_i = jp1j; b_i < 10; b_i++) {
          jA = (b_i + x_tmp) - 1;
          fcnOutput[jA] -= fcnOutput[i] * x[(b_i + 9 * (j - 1)) - 1];
        }
      }
    }
  }
  for (j = 0; j < 9; j++) {
    jA = 9 * j;
    for (k = 8; k >= 0; k--) {
      kAcol = 9 * k;
      i = k + jA;
      smax = fcnOutput[i];
      if (smax != 0.0F) {
        fcnOutput[i] = smax / x[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          x_tmp = b_i + jA;
          fcnOutput[x_tmp] -= fcnOutput[i] * x[b_i + kAcol];
        }
      }
    }
  }
  for (i = 0; i < 9; i++) {
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax +=
            covar[i + 9 * x_tmp] * static_cast<float>(b_a[x_tmp + 9 * jp1j]);
      }
      x[i + 9 * jp1j] = smax;
    }
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax += x[i + 9 * x_tmp] * fcnOutput[x_tmp + 9 * jp1j];
      }
      K[i + 9 * jp1j] = smax;
    }
  }
  //  inv_state = [[R', -R'*v, -R'*p];[zeros(2,3), eye(2)]];
  //  meas= [[Rm, vm, pm];[zeros(2,3), eye(2)]];
  //  eta = meas*inv_state;
  for (i = 0; i < 3; i++) {
    b_R[3 * i] = R[i];
    b_R[3 * i + 1] = R[i + 3];
    b_R[3 * i + 2] = R[i + 6];
  }
  for (i = 0; i < 9; i++) {
    smax = b_R[i];
    delta[i] = smax;
    smax = -smax;
    b_R[i] = smax;
  }
  //  performing logm
  //  matrix logarithm
  //  upto second order
  for (i = 0; i < 3; i++) {
    float f;
    smax = b_R[i];
    s = smax * v[0];
    f = smax * p[0];
    b_Rm[5 * i] = Rm[3 * i];
    smax = b_R[i + 3];
    s += smax * v[1];
    f += smax * p[1];
    b_Rm[5 * i + 1] = Rm[3 * i + 1];
    smax = b_R[i + 6];
    s += smax * v[2];
    f += smax * p[2];
    b_Rm[5 * i + 2] = Rm[3 * i + 2];
    b_eta_tmp[i] = f;
    eta_tmp[i] = s;
    b_Rm[i + 15] = vm[i];
    b_Rm[i + 20] = pm[i];
  }
  for (i = 0; i < 5; i++) {
    jA = i << 1;
    b_Rm[5 * i + 3] = iv[jA];
    b_Rm[5 * i + 4] = iv[jA + 1];
  }
  for (i = 0; i < 3; i++) {
    eta_log[5 * i] = delta[3 * i];
    eta_log[5 * i + 1] = delta[3 * i + 1];
    eta_log[5 * i + 2] = delta[3 * i + 2];
    eta_log[i + 15] = eta_tmp[i];
    eta_log[i + 20] = b_eta_tmp[i];
  }
  for (i = 0; i < 5; i++) {
    jA = i << 1;
    eta_log[5 * i + 3] = iv[jA];
    eta_log[5 * i + 4] = iv[jA + 1];
  }
  for (i = 0; i < 5; i++) {
    for (jp1j = 0; jp1j < 5; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 5; x_tmp++) {
        smax += b_Rm[i + 5 * x_tmp] * eta_log[x_tmp + 5 * jp1j];
      }
      jA = i + 5 * jp1j;
      hat_delta[jA] = smax - static_cast<float>(iv1[jA]);
    }
  }
  for (i = 0; i < 5; i++) {
    for (jp1j = 0; jp1j < 5; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 5; x_tmp++) {
        smax += 0.5F * hat_delta[i + 5 * x_tmp] * hat_delta[x_tmp + 5 * jp1j];
      }
      jA = i + 5 * jp1j;
      eta_log[jA] = hat_delta[jA] - smax;
    }
  }
  b_R[0] = eta_log[7];
  b_R[1] = eta_log[10];
  b_R[2] = eta_log[1];
  b_R[3] = eta_log[15];
  b_R[6] = eta_log[20];
  b_R[4] = eta_log[16];
  b_R[7] = eta_log[21];
  b_R[5] = eta_log[17];
  b_R[8] = eta_log[22];
  for (i = 0; i < 9; i++) {
    smax = 0.0F;
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax += K[i + 9 * jp1j] * b_R[jp1j];
    }
    delta[i] = smax;
  }
  hat_delta[0] = 0.0F;
  hat_delta[5] = -delta[2];
  hat_delta[10] = delta[1];
  hat_delta[1] = delta[2];
  hat_delta[6] = 0.0F;
  hat_delta[11] = -delta[0];
  hat_delta[2] = -delta[1];
  hat_delta[7] = delta[0];
  hat_delta[12] = 0.0F;
  hat_delta[15] = delta[3];
  hat_delta[20] = delta[6];
  hat_delta[16] = delta[4];
  hat_delta[21] = delta[7];
  hat_delta[17] = delta[5];
  hat_delta[22] = delta[8];
  for (i = 0; i < 5; i++) {
    hat_delta[5 * i + 3] = 0.0F;
    hat_delta[5 * i + 4] = 0.0F;
  }
  //  performing matrix exponential
  //  matrix exponential
  //  upto second order
  for (i = 0; i < 5; i++) {
    for (jp1j = 0; jp1j < 5; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 5; x_tmp++) {
        smax += 0.5F * hat_delta[i + 5 * x_tmp] * hat_delta[x_tmp + 5 * jp1j];
      }
      x_tmp = i + 5 * jp1j;
      fv[x_tmp] = (static_cast<float>(iv1[x_tmp]) + hat_delta[x_tmp]) + smax;
    }
  }
  for (i = 0; i < 3; i++) {
    b_Rm[5 * i] = R[3 * i];
    b_Rm[5 * i + 1] = R[3 * i + 1];
    b_Rm[5 * i + 2] = R[3 * i + 2];
    b_Rm[i + 15] = v[i];
    b_Rm[i + 20] = p[i];
  }
  for (i = 0; i < 5; i++) {
    jA = i << 1;
    b_Rm[5 * i + 3] = iv[jA];
    b_Rm[5 * i + 4] = iv[jA + 1];
  }
  for (i = 0; i < 5; i++) {
    for (jp1j = 0; jp1j < 5; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 5; x_tmp++) {
        smax += fv[i + 5 * x_tmp] * b_Rm[x_tmp + 5 * jp1j];
      }
      eta_log[i + 5 * jp1j] = smax;
    }
  }
  for (i = 0; i < 3; i++) {
    R[3 * i] = eta_log[5 * i];
    R[3 * i + 1] = eta_log[5 * i + 1];
    R[3 * i + 2] = eta_log[5 * i + 2];
    v[i] = eta_log[i + 15];
    p[i] = eta_log[i + 20];
  }
  // Joseph update form
  for (i = 0; i < 9; i++) {
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax += K[i + 9 * x_tmp] * static_cast<float>(b_a[x_tmp + 9 * jp1j]);
      }
      x[i + 9 * jp1j] = smax;
    }
  }
  for (i = 0; i < 81; i++) {
    a[i] = static_cast<float>(b_a[i]) - x[i];
  }
  for (i = 0; i < 9; i++) {
    for (jp1j = 0; jp1j < 9; jp1j++) {
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax += a[i + 9 * x_tmp] * covar[x_tmp + 9 * jp1j];
      }
      fcnOutput[i + 9 * jp1j] = smax;
    }
  }
  for (i = 0; i < 9; i++) {
    for (jp1j = 0; jp1j < 9; jp1j++) {
      jA = i + 9 * jp1j;
      a[jp1j + 9 * i] = static_cast<float>(b_a[jA]) - x[jA];
      smax = 0.0F;
      for (x_tmp = 0; x_tmp < 9; x_tmp++) {
        smax += K[i + 9 * x_tmp] * b[x_tmp + 9 * jp1j];
      }
      b_K[jA] = smax;
    }
  }
  for (i = 0; i < 9; i++) {
    for (jp1j = 0; jp1j < 9; jp1j++) {
      x_tmp = i + 9 * jp1j;
      covar[x_tmp] = 0.0F;
      smax = 0.0F;
      for (kAcol = 0; kAcol < 9; kAcol++) {
        jA = i + 9 * kAcol;
        covar[x_tmp] += fcnOutput[jA] * a[kAcol + 9 * jp1j];
        smax += b_K[jA] * K[jp1j + 9 * kAcol];
      }
      x[x_tmp] = smax;
    }
  }
  for (i = 0; i < 81; i++) {
    covar[i] += x[i];
  }
}

void InvariantEKFm::measUpdate_mocap() {}

}


