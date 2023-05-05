#ifndef QROTOR_FIRMWARE_POSITION_CLFQP_H_
#define QROTOR_FIRMWARE_POSITION_CLFQP_H_

#include "control/position_controller.h"
#include <iostream>
#include <qpOASES.hpp>

namespace qrotor_firmware {

class PositionClfQP : public PositionController {
protected:
  /// CLF
  float eta2 = 100.0;
  float epsilon2 = 4.0;
  float c2 = 20.0;
  matrix::Matrix<float, 1, 1> V2, LfV2;
  matrix::Matrix<float, 1, 3> LgV2;
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

public:
  explicit PositionClfQP(FlightController &_flightcontroller);
  ~PositionClfQP();

  virtual void run(float dt) override;

  /**
   * Implements Control-Lyapunov Quadratic programming
   * @return
   */
  bool run_clf_qp();
};

} // namespace qrotor_firmware

#endif // QROTOR_FIRMWARE_POSITION_CLFQP_H_
