#ifndef QROTOR_FIRMWARE_ATTITUDE_GEOMETRIC_CLF_QP_H
#define QROTOR_FIRMWARE_ATTITUDE_GEOMETRIC_CLF_QP_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <qpOASES.hpp>
#include "control/matlab/qcqp_fmincon.hpp"

#include "control/attitude_controller.h"
namespace qrotor_firmware {

class AttitudeGeometricClfQP : public AttitudeController {
private:
  matrix::Matrix3f inertia_scaled;

  /// \brief Rotation errors on SO3 manifold
  matrix::Vector3f eR, deR, eOmega;
  /// \brief Derivatives of rotation matrix and command R
  matrix::Dcmf dR, dRc;

  /// CLF
  float eta = 100.0;
  float epsilon = 4.0;
  float c = 20.0;
  matrix::Matrix<float, 1, 1> V, LfV;
  matrix::Matrix<float, 1, 3> LgV;
  matrix::Vector3f _moment{0.f, 0.f, 0.f};
  matrix::Vector3f _thrust_vec{0.f, 0.f, 0.7f};
  float _thrust{0.7f};

  /// Quadratic Programming
  qpOASES::real_t H[16]{};
  qpOASES::real_t g[4]{}, A[4]{};
  qpOASES::real_t lb[4]{}, ub[4]{};
  qpOASES::real_t lbA[1]{}, ubA[1]{};
  qpOASES::real_t xOpt[4]{};

  qpOASES::SQProblem *solver;
  qpOASES::int_t nWSR = 10;
  qpOASES::Options options;
  qpOASES::real_t cpu_time_limit{};

  // Quadratic Constraints Quadratic programming
  matrix::Matrix<float, 4, 4> Q_qp, H_qc{};
  matrix::Vector<float, 4> f_qp, k_qc{};
  matrix::Vector<float, 4> xlb_qcqp, xub_qcqp, x0_qcqp;
  float c_qp{0.f}, d_qc{0.f};
  fmincon::QCQPfmincon4* qcqp_solver_= nullptr;

public:
  explicit AttitudeGeometricClfQP(FlightController &_flightController);
  ~AttitudeGeometricClfQP();

  virtual bool init() override;
  virtual void compute(float _dt) override;

  /**
   * Computes the attitude errors on the SO3 manifol
   */
  void compute_error_vectors();
  /**
   * Implements geometric pd control on SO3
   * @param moment
   */
  void run_geom_pd(matrix::Vector3f &moment);
  /**
   * Runs a geometric attitude continuous time clf-qp
   * @param moment
   * @return
   */
  bool run_clf_qp(matrix::Vector3f &moment);
  /**
   * Implements discrete time geometric-attitude clf-qp
   * @param moment
   * @return
   */
  bool run_dclf_qp(matrix::Vector3f &moment);

};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_ATTITUDE_GEOMETRIC_CLF_QP_H
