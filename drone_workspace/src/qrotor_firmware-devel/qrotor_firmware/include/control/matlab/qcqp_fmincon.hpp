//
// Created by kotaru on 4/17/21.
//

#ifndef QCQP_FMINCON_HPP
#define QCQP_FMINCON_HPP

#include "matlab_codegen/qcqp/qcqp.h"
#include "matlab_codegen/qcqp/qcqp_types.h"
#include "utils.h"
#include <Matrix/matrix/math.hpp>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>

namespace fmincon {

class QCQPfmincon4 {
public:
  // arrays
  double Q_array[16]{}, H_array[16]{};
  double f_array[4]{}, k_array[4]{};
  double xlb_array[4]{}, xub_array[4]{}, x0_array[4]{};
  double c_{0.f}, d_{0.f};

  // outputs
  double eflag{};
  double fval{};
  struct0_T output{};
  struct1_T lambda{};
  double xOpt[4]{};

  /// state variables
  double dt{0.002};
  double R[9] = {1., 0., 0, 0., 1.0, 0., 0., 0., 1.0};
  double Rc[9] = {1., 0., 0, 0., 1.0, 0., 0., 0., 1.0};
  double Om[3] = {0., 0., 0};
  double Omc[3] = {0., 0., 0};

  // clf
  double gamma_{0.9};
  double epsilon2{8};
  double c2{30};
  double lyap_{0};

  /// system parameters
  double _inertia_scaled[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 3.3045};
  double _inertia_inv[9] = {434.7826, -0.0189, -0.0057, -0.0189, 434.7826,
                            -0.0057,  -0.0057, -0.0057, 131.5789};
  double _inertia[9] = {0.0023, 1.0E-7, 1.0E-7, 1.0E-7, 0.0023,
                        1.0E-7, 1.0E-7, 1.0E-7, 0.0076};

public:
  QCQPfmincon4(const matrix::Matrix3f &J, const matrix::Matrix3f &Jinv,
               const matrix::Matrix3f &Jscaled) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        _inertia[3 * i + j] = J(j, i);
        _inertia_inv[3 * i + j] = Jinv(j, i);
        _inertia_scaled[3 * i + j] = Jscaled(j, i);
      }
    }
  }
  ~QCQPfmincon4() = default;

  static void matrix2array(double *_array,
                           const matrix::Matrix<float, 4, 4> &_matrix) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        _array[4 * i + j] = (double)_matrix(j, i);
      }
    }
  }
  static void vector2array(double *_array,
                           const matrix::Vector<float, 4> &_vector) {
    for (int i = 0; i < 4; ++i) {
      _array[i] = (double)_vector(i);
    }
  }

  void assign(const matrix::Matrix<float, 4, 4> &Q,
              const matrix::Vector<float, 4> &f, const float &c,
              const matrix::Matrix<float, 4, 4> &H,
              const matrix::Vector<float, 4> &k, const float &d,
              const matrix::Vector<float, 4> &xlb,
              const matrix::Vector<float, 4> &xub,
              const matrix::Vector<float, 4> &x0) {

    matrix2array(Q_array, Q);
    vector2array(f_array, f);
    c_ = c;

    matrix2array(H_array, H);
    vector2array(k_array, k);
    d_ = d;

    vector2array(xlb_array, xlb);
    vector2array(xub_array, xub);
    vector2array(x0_array, x0);
  }

  void cost(const matrix::Matrix<float, 4, 4> &Q,
            const matrix::Vector<float, 4> &f, const float &c) {
    matrix2array(Q_array, Q);
    vector2array(f_array, f);
    c_ = c;
  }

  void bounds(const matrix::Vector<float, 4> &xlb,
              const matrix::Vector<float, 4> &xub) {
    vector2array(xlb_array, xlb);
    vector2array(xub_array, xub);
  }

  void warm_start(const matrix::Vector<float, 4> &x0) {
    vector2array(x0_array, x0);
  }

  void constraint(const float &_dt, const matrix::Matrix3f &_R,
                  const matrix::Vector3f &_Om, const matrix::Matrix3f &_Rc,
                  const matrix::Vector3f &_Omc) {
    dt = (double)_dt;

    //    std::cout << "----------------------------------------" << std::endl;
    //    std::cout << "dt_s: " << dt_s << std::endl;
    //    std::cout << "R\n" << std::endl;
    //    _R.print();
    //    std::cout << "Om\n" << std::endl;
    //    _Om.print();
    //    std::cout << "Rc\n" << std::endl;
    //    _Rc.print();
    //    std::cout << "Omc\n" << std::endl;
    //    _Omc.print();
    //    std::cout << "----------------------------------------" << std::endl;

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        R[3 * i + j] = _R(j, i);
        Rc[3 * i + j] = _Rc(j, i);
      }
      Om[i] = _Om(i);
      Omc[i] = 0 * _Omc(i);
    }
  }

  void quadratic_dclf_const(double dt, const double R[9], const double Om[3],
                            const double Rc[9], const double Omc[3],
                            double b_gamma, double c, double epsilon,
                            const double inertia[9],
                            const double inertia_inv[9],
                            const double inertia_scaled[9], double H[16],
                            double k[4], double *d, double *V) {
    int i;
    double b_d;
    double tmp;
    double eR[3];
    double A[9];
    double y_tmp[9];
    double th;
    int i1;
    int K_tmp;
    double eOmega;
    double b_eOmega[3];
    double K[9];
    double b_eR;
    double a;
    double b_A[9];
    double b_tmp[3];
    double OmJOm[3];
    double c_eOmega[3];
    double k2_tmp[3];

    //  compute SO(3) error
    for (i = 0; i < 3; i++) {
      b_d = R[3 * i];
      tmp = R[3 * i + 1];
      th = R[3 * i + 2];
      for (i1 = 0; i1 < 3; i1++) {
        K_tmp = i + 3 * i1;
        K[K_tmp] =
            (b_d * Rc[3 * i1] + tmp * Rc[3 * i1 + 1]) + th * Rc[3 * i1 + 2];
        y_tmp[i1 + 3 * i] = Rc[K_tmp];
      }
    }

    for (i = 0; i < 3; i++) {
      b_d = y_tmp[i + 3];
      tmp = y_tmp[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        K_tmp = i + 3 * i1;
        A[K_tmp] = ((y_tmp[i] * R[3 * i1] + b_d * R[3 * i1 + 1]) +
                    tmp * R[3 * i1 + 2]) -
                   K[K_tmp];
      }

      b_eOmega[i] =
          Om[i] - ((K[i] * Omc[0] + K[i + 3] * Omc[1]) + K[i + 6] * Omc[2]);
    }

    eR[0] = 0.5 * A[5];
    eR[1] = 0.5 * A[6];
    eR[2] = 0.5 * A[1];

    //  computing lyapunov function at current time-step
    eOmega = 0.0;
    b_eR = 0.0;
    th = 0.0;

    //  integrated dynamics by 1 step
    for (K_tmp = 0; K_tmp < 3; K_tmp++) {
      eOmega += ((b_eOmega[0] * inertia_scaled[3 * K_tmp] +
                  b_eOmega[1] * inertia_scaled[3 * K_tmp + 1]) +
                 b_eOmega[2] * inertia_scaled[3 * K_tmp + 2]) *
                b_eOmega[K_tmp];
      b_eR += eR[K_tmp] * b_eOmega[K_tmp];
      th += eR[K_tmp] * eR[K_tmp];
      eR[K_tmp] = Om[K_tmp] * dt;
    }

    *V = (eOmega / 2.0 + epsilon * b_eR) + c * th / 2.0;
    th = std::sqrt((eR[0] * eR[0] + eR[1] * eR[1]) + eR[2] * eR[2]);
    if (th < 1.0E-6) {
      eR[0] = 0.0;
      eR[1] = 0.0;
      eR[2] = 1.0;
    } else {
      eR[0] /= th;
      eR[1] /= th;
      eR[2] /= th;
    }

    K[0] = 0.0;
    K[3] = -eR[2];
    K[6] = eR[1];
    K[1] = eR[2];
    K[4] = 0.0;
    K[7] = -eR[0];
    K[2] = -eR[1];
    K[5] = eR[0];
    K[8] = 0.0;
    a = std::sin(th);
    th = 1.0 - std::cos(th);
    std::memset(&A[0], 0, 9U * sizeof(double));
    A[0] = 1.0;
    A[4] = 1.0;
    A[8] = 1.0;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 3; i1++) {
        K_tmp = i + 3 * i1;
        b_A[K_tmp] = (A[K_tmp] + a * K[K_tmp]) +
                     ((th * K[i] * K[3 * i1] + th * K[i + 3] * K[3 * i1 + 1]) +
                      th * K[i + 6] * K[3 * i1 + 2]);
      }
    }

    for (i = 0; i < 3; i++) {
      b_d = R[i + 3];
      tmp = R[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        K[i + 3 * i1] = (R[i] * b_A[3 * i1] + b_d * b_A[3 * i1 + 1]) +
                        tmp * b_A[3 * i1 + 2];
      }
    }

    //  computing the quadratic constraint
    for (i = 0; i < 3; i++) {
      b_d = 0.0;
      tmp = y_tmp[i + 3];
      th = y_tmp[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        int i2;
        K_tmp = 3 * i1 + 1;
        i2 = 3 * i1 + 2;
        a = (K[3 * i] * Rc[3 * i1] + K[3 * i + 1] * Rc[K_tmp]) +
            K[3 * i + 2] * Rc[i2];
        A[i + 3 * i1] =
            ((y_tmp[i] * K[3 * i1] + tmp * K[K_tmp]) + th * K[i2]) - a;
        b_d += a * Omc[i1];
      }

      b_eOmega[i] = Om[i] - b_d;
    }

    eR[0] = 0.5 * A[5];
    eR[1] = 0.5 * A[6];
    eR[2] = 0.5 * A[1];
    for (i = 0; i < 3; i++) {
      b_tmp[i] = epsilon * eR[i];
      b_d = inertia_inv[3 * i];
      tmp = inertia_inv[3 * i + 1];
      th = inertia_inv[3 * i + 2];
      for (i1 = 0; i1 < 3; i1++) {
        y_tmp[i + 3 * i1] = (b_d * (inertia_scaled[3 * i1] * dt * dt) +
                             tmp * (inertia_scaled[3 * i1 + 1] * dt * dt)) +
                            th * (inertia_scaled[3 * i1 + 2] * dt * dt);
      }

      b_d = y_tmp[i + 3];
      tmp = y_tmp[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        K[i + 3 * i1] =
            (y_tmp[i] * inertia_inv[3 * i1] + b_d * inertia_inv[3 * i1 + 1]) +
            tmp * inertia_inv[3 * i1 + 2];
      }
    }

    y_tmp[0] = 0.0;
    y_tmp[3] = -Om[2];
    y_tmp[6] = Om[1];
    y_tmp[1] = Om[2];
    y_tmp[4] = 0.0;
    y_tmp[7] = -Om[0];
    y_tmp[2] = -Om[1];
    y_tmp[5] = Om[0];
    y_tmp[8] = 0.0;
    for (i = 0; i < 3; i++) {
      b_d = 0.0;
      tmp = 0.0;
      th = y_tmp[i + 3];
      a = y_tmp[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        tmp += ((y_tmp[i] * inertia[3 * i1] + th * inertia[3 * i1 + 1]) +
                a * inertia[3 * i1 + 2]) *
               Om[i1];
        b_d += dt * b_eOmega[i1] * inertia_scaled[i1 + 3 * i];
      }

      OmJOm[i] = tmp;
      c_eOmega[i] = b_d + dt * b_tmp[i];
    }

    eOmega = 0.0;
    th = 0.0;
    b_eR = 0.0;
    a = 0.0;
    tmp = 0.0;
    for (i = 0; i < 3; i++) {
      i1 = 3 * i + 1;
      K_tmp = 3 * i + 2;
      b_d = (c_eOmega[0] * inertia_inv[3 * i] + c_eOmega[1] * inertia_inv[i1]) +
            c_eOmega[2] * inertia_inv[K_tmp];
      k2_tmp[i] = b_d;
      eOmega += ((0.5 * OmJOm[0] * K[3 * i] + 0.5 * OmJOm[1] * K[i1]) +
                 0.5 * OmJOm[2] * K[K_tmp]) *
                OmJOm[i];
      th += b_d * OmJOm[i];
      b_eR += eR[i] * eR[i];
      a += ((0.5 * b_eOmega[0] * inertia_scaled[3 * i] +
             0.5 * b_eOmega[1] * inertia_scaled[i1]) +
            0.5 * b_eOmega[2] * inertia_scaled[K_tmp]) *
           b_eOmega[i];
      tmp += b_tmp[i] * b_eOmega[i];
    }

    *d = (eOmega - th) + (((0.5 * c * b_eR + a) + tmp) + (b_gamma - 1.0) * *V);
    std::memset(&H[0], 0, 16U * sizeof(double));
    H[15] = 0.0;
    for (i = 0; i < 3; i++) {
      b_d = K[3 * i];
      K_tmp = i << 2;
      H[K_tmp] = b_d;
      tmp = OmJOm[0] * b_d;
      b_d = K[3 * i + 1];
      H[K_tmp + 1] = b_d;
      tmp += OmJOm[1] * b_d;
      b_d = K[3 * i + 2];
      H[K_tmp + 2] = b_d;
      tmp += OmJOm[2] * b_d;
      k[i] = k2_tmp[i] - tmp;
    }

    k[3] = -1.0;
  }

  void run() {
    auto start = utils::get_current_time();
    quadratic_dclf_const(dt, R, Om, Rc, Omc, gamma_, c2, epsilon2, _inertia,
                         _inertia_inv, _inertia_scaled, H_array, k_array, &d_,
                         &lyap_);

    std::cout << "*************** :QCQP: ***************" << std::endl;
    printf("R[9] = \n");
    for (int i = 0; i < 3; ++i) {
      printf("[ %f, %f, %f]\n", R[i], R[i + 3], R[i + 6]);
    }
    printf("Rc[9] = \n");
    for (int i = 0; i < 3; ++i) {
      printf("[ %f, %f, %f]\n", Rc[i], Rc[i + 3], Rc[i + 6]);
    }
    printf("Om.T = [ %f, %f, %f]\n", Om[0], Om[1], Om[2]);
    printf("Omc.T = [%f, %f, %f]\n", Omc[0], Omc[1], Omc[2]);

    //    printf("---\n");
    //    printf("_inertia=\n");
    //    for (int i = 0; i < 3; ++i) {
    //      printf("[%f, %f, %f]\n", _inertia[i], _inertia[i + 3], _inertia[i +
    //      6]);
    //    }
    //    printf("_inertia_inv=\n");
    //    for (int i = 0; i < 3; ++i) {
    //      printf("[%f, %f, %f]\n", _inertia_inv[i], _inertia_inv[i + 3],
    //             _inertia_inv[i + 6]);
    //    }
    //    printf("_inertia_scaled=\n");
    //    for (int i = 0; i < 3; ++i) {
    //      printf("[%f, %f, %f]\n", _inertia_scaled[i], _inertia_scaled[i + 3],
    //             _inertia_scaled[i + 6]);
    //    }
    //    printf("---\n");
    printf("H_array[16] = \n");
    for (int i = 0; i < 4; ++i) {
      printf("[ %f, %f, %f, %f]\n", H_array[i], H_array[i + 4], H_array[i + 8],
             H_array[i + 12]);
    }
    printf("k = [%f, %f, %f, %f]\n", k_array[0], k_array[1], k_array[2],
           k_array[3]);
    printf("d =%f\n", d_);
    std::cout << "***************************************" << std::endl;

    qcqp(Q_array, f_array, c_, H_array, k_array, d_, xlb_array, xub_array,
         x0_array, xOpt, &fval, &eflag, &output, &lambda);
    auto end = utils::get_current_time();

    printf("eflag: %f, comp_time: %f\n", eflag, float(end - start) * 1e-6f);
    for (int i = 0; i < 4; ++i) {
      printf("x[%d] = %f, ", i, xOpt[i]);
      //      x0_array[i] = xOpt[i];
    }
    printf("\n");
    std::cout << "***************************************" << std::endl;
  }
};

} // namespace fmincon

#endif // QCQP_FMINCON_HPP
