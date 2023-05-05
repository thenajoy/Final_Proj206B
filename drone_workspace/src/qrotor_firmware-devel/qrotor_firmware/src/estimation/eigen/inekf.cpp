//
// Created by kotaru on 4/1/21.
//
#include "estimation/eigen/inekf.h"

namespace eigen {

InEKF::InEKF() : g(9.80), e1(1.0, 0.0, 0.0), e2(0.0, 1.0, 0.0), e3(0.0, 0.0, 1.0) {
  R = Eigen::Matrix3f::Identity();
  v = Eigen::Vector3f::Zero();
  p = Eigen::Vector3f::Zero();

  // gravity parameters
  gVec = -g * e3;
  gSkew = this->hat(gVec);

  // linear dynamics
  A.setZero();
  A.block(3, 0, 3, 3) = gSkew;
  A.block(6, 3, 3, 3) = Eigen::Matrix3f::Identity();

  H = Eigen::Matrix<float, 9, 9>::Identity();

  // covariances
  P = 0.01 * Eigen::Matrix<float, 9, 9>::Identity();
  Q.setZero();
  Q.block(0, 0, 3, 3) = 0.01 * Eigen::Matrix3f::Identity();
  Q.block(3, 3, 3, 3) = 0.01 * Eigen::Matrix3f::Identity();
  Q.block(6, 6, 3, 3) = 0.0001 * Eigen::Matrix3f::Identity();

  N = 0.001 * Eigen::Matrix<float, 9, 9>::Identity();

}

InEKF::~InEKF() = default;

void InEKF::init() {}

void InEKF::timeUpdate(const float dt,
                       const Eigen::Vector3f &_accel,
                       const Eigen::Vector3f &_gyro) {
  Eigen::Vector3f net_accel = R * _accel + gVec;

  // Imu-motion model
  p = p + v * dt + 0.5 * dt * dt * net_accel;
  v = v + net_accel * dt;
  R = R * this->rodriguesRotation((Eigen::Vector3f() << _gyro * dt).finished());

  // Discretization
  Eigen::Matrix<float, 9, 9> Ak = Eigen::Matrix<float, 9, 9>::Identity() + A * dt;

  //  covariance update
  P = Ak * P * Ak.transpose() + Q;
}

void InEKF::measUpdate(Eigen::Matrix3f _Rm, Eigen::Vector3f _vm, Eigen::Vector3f _pm) {
  // Kalman gain
  Eigen::Matrix<float, 9, 9> K = Eigen::Matrix<float, 9, 9>::Zero();
  Eigen::Matrix<float, 9, 9> S = H * P * H.transpose() + N;

  K = (P * H.transpose()) * S.inverse();

  Eigen::Matrix<float, 5, 5> eta, eta_log, BI;
  Eigen::Matrix<float, 5, 5> X, Xm, iX;

  X << R, v, p, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  Xm << _Rm, _vm, _pm, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  iX << R.transpose(), -R.transpose() * v, -R.transpose() * p, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

  eta = Xm * iX;

  BI = eta - Eigen::Matrix<float, 5, 5>::Identity();
  eta_log = BI - (0.5) * BI * BI; //+ (1 / 3) * BI * BI * BI;

  Eigen::Matrix<float, 9, 1> error;
  error.block(0, 0, 3, 1) = this->vee(eta_log.block(0, 0, 3, 3));
  error.block(3, 0, 3, 1) = eta_log.block(0, 3, 3, 1);
  error.block(6, 0, 3, 1) = eta_log.block(0, 4, 3, 1);

  Eigen::Matrix<float, 9, 1> delta = K * error;

  Eigen::Matrix<float, 5, 5> delta_exp, delta_hat;
  delta_hat << this->hat(delta.block(0, 0, 3, 1)), delta.block(3, 0, 3, 1),
      delta.block(6, 0, 3, 1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  delta_exp = Eigen::Matrix<float, 5, 5>::Identity() + delta_hat + 0.5 * delta_hat * delta_hat;
  //      + (1 / 6) * delta_hat * delta_hat * delta_hat;

  Eigen::Matrix<float, 5, 5> new_state = delta_exp * X;
  this->R = new_state.block(0, 0, 3, 3);
  this->v = new_state.block(0, 3, 3, 1);
  this->p = new_state.block(0, 4, 3, 1);

  // covariance update
  Eigen::Matrix<float, 9, 9> I_KH = Eigen::Matrix<float, 9, 9>::Identity() - K * H;
  P = I_KH * P * I_KH.transpose() + K * N * K.transpose();
}

}


