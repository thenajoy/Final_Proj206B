//
// Created by kotaru on 5/23/21.
//
#ifndef QROTOR_FIRMWARE_PLANNING_FLATNESS_HPP_
#define QROTOR_FIRMWARE_PLANNING_FLATNESS_HPP_
#include "data_structures.h"
namespace qrotor_firmware {

inline static void flat2state(QrotorFlats *flats_,
                                            QuadrotorState *state_,
                                            PoseWithCovariance &pose_,
                                            const float &m,
                                            const float J[9]) {
  float Omega_hat[9];
  float Omega_hat_tmp[9];
  float b_state[9];
  float dR[9];
  float b3[3];
  float b3_b1d[3];
  float b_db3[3];
  float d2b3_b1d[3];
  float d2fb3[3];
  float db1[3];
  float db3[3];
  float db3_b1d[3];
  float absxk;
  float b1_idx_0;
  float b1_idx_1;
  float b1_idx_2;
  float b3_idx_0;
  float b3_idx_1;
  float b3_idx_2;
  float dnorm_b3_b1d;
  float f;
  float f1;
  float f2;
  float f3;
  float fcnOutput;
  float norm_b3_b1d;
  float norm_fb3;
  float scale;
  float t;
  float y_tmp;
  int i;
  int k;
  //
  scale = 1.29246971E-26F;
  state_->p[0] = flats_->p[0];
  state_->v[0] = flats_->v[0];
  state_->a[0] = flats_->a[0];
  state_->da[0] = flats_->da[0];
  state_->d2a[0] = flats_->d2a[0];
  f = m * flats_->a[0];
  state_->F[0] = f;
  absxk = std::abs(f);
  if (absxk > 1.29246971E-26F) {
    norm_fb3 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    norm_fb3 = t * t;
  }
  state_->p[1] = flats_->p[1];
  state_->v[1] = flats_->v[1];
  state_->a[1] = flats_->a[1];
  state_->da[1] = flats_->da[1];
  state_->d2a[1] = flats_->d2a[1];
  f = m * flats_->a[1];
  state_->F[1] = f;
  absxk = std::abs(f);
  if (absxk > scale) {
    t = scale / absxk;
    norm_fb3 = norm_fb3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_fb3 += t * t;
  }
  state_->p[2] = flats_->p[2];
  state_->v[2] = flats_->v[2];
  state_->a[2] = flats_->a[2];
  state_->da[2] = flats_->da[2];
  state_->d2a[2] = flats_->d2a[2];
  f = m * (flats_->a[2] + 9.80665F);
  state_->F[2] = f;
  absxk = std::abs(f);
  if (absxk > scale) {
    t = scale / absxk;
    norm_fb3 = norm_fb3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_fb3 += t * t;
  }
  norm_fb3 = scale * std::sqrt(norm_fb3);
  b3_idx_0 = state_->F[0] / norm_fb3;
  b3_idx_1 = state_->F[1] / norm_fb3;
  b3_idx_2 = f / norm_fb3;
  b3_b1d[0] = b3_idx_1 * flats_->b1[2] - flats_->b1[1] * b3_idx_2;
  b3_b1d[1] = flats_->b1[0] * b3_idx_2 - b3_idx_0 * flats_->b1[2];
  b3_b1d[2] = b3_idx_0 * flats_->b1[1] - flats_->b1[0] * b3_idx_1;
  scale = 1.29246971E-26F;
  absxk = std::abs(b3_b1d[0]);
  if (absxk > 1.29246971E-26F) {
    norm_b3_b1d = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    norm_b3_b1d = t * t;
  }
  absxk = std::abs(b3_b1d[1]);
  if (absxk > scale) {
    t = scale / absxk;
    norm_b3_b1d = norm_b3_b1d * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_b3_b1d += t * t;
  }
  absxk = std::abs(b3_b1d[2]);
  if (absxk > scale) {
    t = scale / absxk;
    norm_b3_b1d = norm_b3_b1d * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_b3_b1d += t * t;
  }
  norm_b3_b1d = scale * std::sqrt(norm_b3_b1d);
  b1_idx_0 = -(b3_idx_1 * b3_b1d[2] - b3_b1d[1] * b3_idx_2) / norm_b3_b1d;
  b1_idx_1 = -(b3_b1d[0] * b3_idx_2 - b3_idx_0 * b3_b1d[2]) / norm_b3_b1d;
  b1_idx_2 = -(b3_idx_0 * b3_b1d[1] - b3_b1d[0] * b3_idx_1) / norm_b3_b1d;
  state_->R[3] = b3_idx_1 * b1_idx_2 - b1_idx_1 * b3_idx_2;
  state_->R[4] = b1_idx_0 * b3_idx_2 - b3_idx_0 * b1_idx_2;
  state_->R[5] = b3_idx_0 * b1_idx_1 - b1_idx_0 * b3_idx_1;
  state_->R[0] = b1_idx_0;
  state_->R[6] = b3_idx_0;
  state_->dOmega[0] = m * flats_->da[0];
  state_->R[1] = b1_idx_1;
  state_->R[7] = b3_idx_1;
  state_->dOmega[1] = m * flats_->da[1];
  state_->R[2] = b1_idx_2;
  state_->R[8] = b3_idx_2;
  state_->dOmega[2] = m * flats_->da[2];
  t = ((state_->F[0] * state_->dOmega[0] + state_->F[1] * state_->dOmega[1]) +
      f * state_->dOmega[2]) /
      norm_fb3;
  y_tmp = norm_fb3 * norm_fb3;
  db3[0] = (state_->dOmega[0] * norm_fb3 - state_->F[0] * t) / y_tmp;
  db3[1] = (state_->dOmega[1] * norm_fb3 - state_->F[1] * t) / y_tmp;
  db3[2] = (state_->dOmega[2] * norm_fb3 - f * t) / y_tmp;
  db3_b1d[0] = (db3[1] * flats_->b1[2] - flats_->b1[1] * db3[2]) +
      (b3_idx_1 * flats_->db1[2] - flats_->db1[1] * b3_idx_2);
  db3_b1d[1] = (flats_->b1[0] * db3[2] - db3[0] * flats_->b1[2]) +
      (flats_->db1[0] * b3_idx_2 - b3_idx_0 * flats_->db1[2]);
  db3_b1d[2] = (db3[0] * flats_->b1[1] - flats_->b1[0] * db3[1]) +
      (b3_idx_0 * flats_->db1[1] - flats_->db1[0] * b3_idx_1);
  fcnOutput = (b3_b1d[0] * db3_b1d[0] + b3_b1d[1] * db3_b1d[1]) +
      b3_b1d[2] * db3_b1d[2];
  dnorm_b3_b1d = fcnOutput / norm_b3_b1d;
  db1[0] = ((-(db3[1] * b3_b1d[2] - b3_b1d[1] * db3[2]) -
      (b3_idx_1 * db3_b1d[2] - db3_b1d[1] * b3_idx_2)) -
      b1_idx_0 * dnorm_b3_b1d) /
      norm_b3_b1d;
  db1[1] = ((-(b3_b1d[0] * db3[2] - db3[0] * b3_b1d[2]) -
      (db3_b1d[0] * b3_idx_2 - b3_idx_0 * db3_b1d[2])) -
      b1_idx_1 * dnorm_b3_b1d) /
      norm_b3_b1d;
  db1[2] = ((-(db3[0] * b3_b1d[1] - b3_b1d[0] * db3[1]) -
      (b3_idx_0 * db3_b1d[1] - db3_b1d[0] * b3_idx_1)) -
      b1_idx_2 * dnorm_b3_b1d) /
      norm_b3_b1d;
  b_db3[0] = db3[1] * b1_idx_2 - b1_idx_1 * db3[2];
  b_db3[1] = b1_idx_0 * db3[2] - db3[0] * b1_idx_2;
  b_db3[2] = db3[0] * b1_idx_1 - b1_idx_0 * db3[1];
  b3[0] = b3_idx_1 * db1[2] - db1[1] * b3_idx_2;
  b3[1] = db1[0] * b3_idx_2 - b3_idx_0 * db1[2];
  b3[2] = b3_idx_0 * db1[1] - db1[0] * b3_idx_1;
  for (i = 0; i < 3; i++) {
    dR[i] = db1[i];
    dR[i + 3] = b_db3[i] + b3[i];
    dR[i + 6] = db3[i];
    Omega_hat_tmp[3 * i] = state_->R[i];
    Omega_hat_tmp[3 * i + 1] = state_->R[i + 3];
    Omega_hat_tmp[3 * i + 2] = state_->R[i + 6];
  }
  scale = 0.0F;
  absxk = 0.0F;
  for (k = 0; k < 3; k++) {
    f1 = Omega_hat_tmp[k];
    f2 = Omega_hat_tmp[k + 3];
    f3 = Omega_hat_tmp[k + 6];
    for (i = 0; i < 3; i++) {
      Omega_hat[k + 3 * i] =
          (f1 * dR[3 * i] + f2 * dR[3 * i + 1]) + f3 * dR[3 * i + 2];
    }
    f1 = m * flats_->d2a[k];
    d2fb3[k] = f1;
    f2 = state_->dOmega[k];
    scale += f2 * f2;
    absxk += state_->F[k] * f1;
    f2 *= t;
    state_->dOmega[k] = f2;
  }
  state_->Omega[0] = Omega_hat[5];
  state_->Omega[1] = Omega_hat[6];
  state_->Omega[2] = Omega_hat[1];
  absxk = ((scale + absxk) - t * t) / norm_fb3;
  scale = std::pow(norm_fb3, 4.0F);
  d2fb3[0] = ((((d2fb3[0] * norm_fb3 + state_->dOmega[0]) - state_->dOmega[0]) -
      state_->F[0] * absxk) *
      y_tmp -
      db3[0] * y_tmp * 2.0F * norm_fb3 * t) /
      scale;
  d2fb3[1] = ((((d2fb3[1] * norm_fb3 + state_->dOmega[1]) - state_->dOmega[1]) -
      state_->F[1] * absxk) *
      y_tmp -
      db3[1] * y_tmp * 2.0F * norm_fb3 * t) /
      scale;
  d2fb3[2] = ((((d2fb3[2] * norm_fb3 + state_->dOmega[2]) - state_->dOmega[2]) -
      f * absxk) *
      y_tmp -
      db3[2] * y_tmp * 2.0F * norm_fb3 * t) /
      scale;
  scale = db3[1] * flats_->db1[2] - flats_->db1[1] * db3[2];
  d2b3_b1d[0] =
      (((d2fb3[1] * flats_->b1[2] - flats_->b1[1] * d2fb3[2]) + scale) +
          scale) +
          (b3_idx_1 * flats_->d2b1[2] - flats_->d2b1[1] * b3_idx_2);
  scale = flats_->db1[0] * db3[2] - db3[0] * flats_->db1[2];
  d2b3_b1d[1] =
      (((flats_->b1[0] * d2fb3[2] - d2fb3[0] * flats_->b1[2]) + scale) +
          scale) +
          (flats_->d2b1[0] * b3_idx_2 - b3_idx_0 * flats_->d2b1[2]);
  scale = db3[0] * flats_->db1[1] - flats_->db1[0] * db3[1];
  d2b3_b1d[2] =
      (((d2fb3[0] * flats_->b1[1] - flats_->b1[0] * d2fb3[1]) + scale) +
          scale) +
          (b3_idx_0 * flats_->d2b1[1] - flats_->d2b1[0] * b3_idx_1);
  scale = norm_b3_b1d * norm_b3_b1d;
  absxk = ((((db3_b1d[0] * db3_b1d[0] + db3_b1d[1] * db3_b1d[1]) +
      db3_b1d[2] * db3_b1d[2]) +
      ((b3_b1d[0] * d2b3_b1d[0] + b3_b1d[1] * d2b3_b1d[1]) +
          b3_b1d[2] * d2b3_b1d[2])) *
      norm_b3_b1d -
      fcnOutput * dnorm_b3_b1d) /
      scale;
  f = db3[1] * db3_b1d[2] - db3_b1d[1] * db3[2];
  state_->dOmega[0] =
      ((((((-(d2fb3[1] * b3_b1d[2] - b3_b1d[1] * d2fb3[2]) - f) - f) -
          (b3_idx_1 * d2b3_b1d[2] - d2b3_b1d[1] * b3_idx_2)) -
          db1[0] * dnorm_b3_b1d) -
          b1_idx_0 * absxk) *
          norm_b3_b1d -
          db1[0] * norm_b3_b1d * dnorm_b3_b1d) /
          scale;
  f = db3_b1d[0] * db3[2] - db3[0] * db3_b1d[2];
  state_->dOmega[1] =
      ((((((-(b3_b1d[0] * d2fb3[2] - d2fb3[0] * b3_b1d[2]) - f) - f) -
          (d2b3_b1d[0] * b3_idx_2 - b3_idx_0 * d2b3_b1d[2])) -
          db1[1] * dnorm_b3_b1d) -
          b1_idx_1 * absxk) *
          norm_b3_b1d -
          db1[1] * norm_b3_b1d * dnorm_b3_b1d) /
          scale;
  f = db3[0] * db3_b1d[1] - db3_b1d[0] * db3[1];
  state_->dOmega[2] =
      ((((((-(d2fb3[0] * b3_b1d[1] - b3_b1d[0] * d2fb3[1]) - f) - f) -
          (b3_idx_0 * d2b3_b1d[1] - d2b3_b1d[0] * b3_idx_1)) -
          db1[2] * dnorm_b3_b1d) -
          b1_idx_2 * absxk) *
          norm_b3_b1d -
          db1[2] * norm_b3_b1d * dnorm_b3_b1d) /
          scale;
  db3_b1d[0] = d2fb3[1] * b1_idx_2 - b1_idx_1 * d2fb3[2];
  db3_b1d[1] = b1_idx_0 * d2fb3[2] - d2fb3[0] * b1_idx_2;
  db3_b1d[2] = d2fb3[0] * b1_idx_1 - b1_idx_0 * d2fb3[1];
  scale = db3[1] * db1[2] - db1[1] * db3[2];
  b_db3[0] = scale;
  absxk = db1[0] * db3[2] - db3[0] * db1[2];
  b_db3[1] = absxk;
  t = db3[0] * db1[1] - db1[0] * db3[1];
  b_db3[2] = t;
  b3_b1d[0] = scale;
  b3_b1d[1] = absxk;
  b3_b1d[2] = t;
  b3[0] = b3_idx_1 * state_->dOmega[2] - state_->dOmega[1] * b3_idx_2;
  b3[1] = state_->dOmega[0] * b3_idx_2 - b3_idx_0 * state_->dOmega[2];
  b3[2] = b3_idx_0 * state_->dOmega[1] - state_->dOmega[0] * b3_idx_1;
  for (i = 0; i < 3; i++) {
    b_state[i] = state_->dOmega[i];
    b_state[i + 3] = ((db3_b1d[i] + b_db3[i]) + b3_b1d[i]) + b3[i];
    b_state[i + 6] = d2fb3[i];
    for (k = 0; k < 3; k++) {
      Omega_hat[i + 3 * k] =
          (dR[3 * i] * dR[3 * k] + dR[3 * i + 1] * dR[3 * k + 1]) +
              dR[3 * i + 2] * dR[3 * k + 2];
    }
  }
  for (i = 0; i < 3; i++) {
    f = Omega_hat_tmp[i];
    f1 = Omega_hat_tmp[i + 3];
    f2 = Omega_hat_tmp[i + 6];
    for (k = 0; k < 3; k++) {
      dR[i + 3 * k] = (f * b_state[3 * k] + f1 * b_state[3 * k + 1]) +
          f2 * b_state[3 * k + 2];
    }
  }
  for (i = 0; i < 9; i++) {
    Omega_hat[i] += dR[i];
  }
  state_->dOmega[0] = Omega_hat[5];
  state_->dOmega[1] = Omega_hat[6];
  state_->dOmega[2] = Omega_hat[1];
  // vee( dR'*dR + R'*d2R, true ) ;
  f = state_->Omega[0];
  f1 = Omega_hat[5];
  f2 = state_->Omega[1];
  f3 = Omega_hat[6];
  scale = state_->Omega[2];
  absxk = Omega_hat[1];
  for (i = 0; i < 3; i++) {
    t = J[i];
    y_tmp = t * f;
    b1_idx_0 = t * f1;
    t = J[i + 3];
    y_tmp += t * f2;
    b1_idx_0 += t * f3;
    t = J[i + 6];
    y_tmp += t * scale;
    b1_idx_0 += t * absxk;
    b3_b1d[i] = b1_idx_0;
    d2b3_b1d[i] = y_tmp;
  }
  state_->M[0] = b3_b1d[0] + (state_->Omega[1] * d2b3_b1d[2] -
      d2b3_b1d[1] * state_->Omega[2]);
  state_->M[1] = b3_b1d[1] + (d2b3_b1d[0] * state_->Omega[2] -
      state_->Omega[0] * d2b3_b1d[2]);
  state_->M[2] = b3_b1d[2] + (state_->Omega[0] * d2b3_b1d[1] -
      d2b3_b1d[0] * state_->Omega[1]);
  state_->f = norm_fb3;

  pose_.position = matrix::Vector3f(state_->p);
  pose_.velocity = matrix::Vector3f(state_->v);
  pose_.acceleration = matrix::Vector3f(state_->a);
  pose_.rotation = matrix::Matrix3f(state_->R);
  pose_.ang_vel = matrix::Vector3f(state_->Omega);
  pose_.ang_accel = matrix::Vector3f(state_->dOmega);
  pose_.f = state_->f;
  pose_.moment = matrix::Vector3f(state_->M);
}
}

#endif //QROTOR_FIRMWARE_PLANNING_FLATNESS_HPP_
