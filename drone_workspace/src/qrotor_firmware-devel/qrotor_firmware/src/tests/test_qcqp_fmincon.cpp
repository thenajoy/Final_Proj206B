//
// Created by kotaru on 4/17/21.
//

#include "control/matlab/qcqp_fmincon.hpp"
#include <iostream>

int main() {

  std::cout << "Testing qcqp_fmincon" << std::endl;
  float J[9] = {0.0049,   0.0000055, 0.0000054, 0.0000055, 0.0053,
                0.000021, 0.0000054, 0.000021,  0.0098};
  matrix::Matrix3f inertia_matrix = matrix::Matrix3f(J);
  matrix::Matrix3f inertia_matrix_inv = geninv(inertia_matrix);
  float min_eigval_inertia =
      std::min(std::min(inertia_matrix(0, 0), inertia_matrix(1, 1)),
               inertia_matrix(2, 2));
  matrix::Matrix3f inertia_scaled = inertia_matrix/min_eigval_inertia;

  fmincon::QCQPfmincon4 solver_{inertia_matrix, inertia_matrix_inv, inertia_scaled};

  // variables
  matrix::Matrix<float, 4, 4> Q, H;
  matrix::Vector<float, 4> f, k;
  matrix::Vector<float, 4> x_lb, x_ub, x0;
  float c{0.f}, d{0.1f};

  // cost function
  Q.setIdentity();
  Q(3, 3) = 4e2;
  f.setZero();
  c = 0;

  // constraint function
  H.setIdentity();
  H(3, 3) = 0;
  float k_[4] = {1, 1, 1, -1};
  k = matrix::Vector<float, 4>(k_);
  d = 0.1f;

  // bounds and ic
  for (int i = 0; i < 4; ++i) {
    x_lb(i) = -1;
    x_ub(i) = 1;
    x0(i) = 0;
  }

  solver_.assign(Q, f, c, H, k, d, x_lb, x_ub, x0);
  solver_.run();

}
