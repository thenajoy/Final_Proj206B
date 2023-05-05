#include "control/attitude_geometric_clf_qp.h"
#include "qrotor_flight.h"

namespace qrotor_firmware {

AttitudeGeometricClfQP::AttitudeGeometricClfQP(
    FlightController &_flightController)
    : AttitudeController(_flightController) {

  inertia_scaled =
      inertia_matrix_ *
          (1 / 0.00489992); // 0.00489992 is min eigenvalue of interia matrix

  /// QP setup
  solver = new qpOASES::SQProblem(4, 1);
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  solver->setOptions(options);

  for (int i = 0; i < 16; i++) {
    H[i] = 0.0;
  }
  for (int j = 0; j < 4; j++) {
    H[4 * j + j] = 1.0;
    A[j] = 0.0;
    g[j] = 0.0;
    lb[j] = -1000;
    ub[j] = 1000;
  }
  H[15] = 4e2;
  A[3] = -1;
  lbA[0] = -INFINITY;
  ubA[0] = INFINITY;

  /**
   * fmincon
   */
  qcqp_solver_ = new fmincon::QCQPfmincon4{inertia_matrix_, inertia_matrix_inv,
                                           inertia_scaled};
  Q_qp.setIdentity();
  Q_qp(3, 3) = 4e2;
  f_qp.setZero();

  H_qc.setIdentity();
  H_qc(3, 3) = 0.f;
  k_qc.setZero();
  k_qc(3) = -1;
  d_qc = 0.f;

  for (int i = 0; i < 3; ++i) {
    xlb_qcqp(i) = MOMENT_LOWER_BOUND(i);
    xub_qcqp(i) = MOMENT_UPPER_BOUND(i);
  }
  xlb_qcqp(3) = -1e9;
  xub_qcqp(3) = 1e9;
  x0_qcqp.setZero();
}

AttitudeGeometricClfQP::~AttitudeGeometricClfQP() = default;

bool AttitudeGeometricClfQP::init() {

  // Initializing qpOASES
  solver->init(H, g, A, lb, ub, lbA, ubA, nWSR, 0);

  // Initializing fmincon
  printf("Q_qp\n");
  Q_qp.print();
  printf("f_qp\n");
  f_qp.print();
  printf("c_qp: %f\n", c_qp);
  printf("xlb: %f, %f, %f, %f\n", xlb_qcqp(0), xlb_qcqp(1), xlb_qcqp(2),
         xlb_qcqp(3));
  printf("xub: %f, %f, %f, %f\n", xub_qcqp(0), xub_qcqp(1), xub_qcqp(2),
         xub_qcqp(3));
  printf("x0: %f, %f, %f, %f\n", x0_qcqp(0), x0_qcqp(1), x0_qcqp(2),
         x0_qcqp(3));
  qcqp_solver_->cost(Q_qp, f_qp, c_qp);
  qcqp_solver_->bounds(xlb_qcqp, xub_qcqp);

  Logger::STATUS(std::string("AttitudeGeometricClfQP initialized!"));
  return true;
}

void AttitudeGeometricClfQP::compute(float _dt) {
  this->dt_ = _dt;

  /* compute error vectors */
  compute_error_vectors();

  // temporary moment storage
  matrix::Vector3f moment{0., 0., 0.};

  /* run clf quadratic programming */
      bool successful_clf_run = run_clf_qp(moment);
//  bool successful_clf_run = run_dclf_qp(moment);
  if (!successful_clf_run) {
    // if QP is unsuccessful run normal geometric control
    Logger::WARN("Unsuccessful CLF QP, running geometric pd control");
    run_geom_pd(moment);
  }
  input_.moment = moment;
  input_.thrust =
      input_.thrust_vector.dot(firmware_.att_estimator_->state().Re3());

  //  std::cout << "CLFQP:mx: " << input_.moment(0) << "\t my: " <<
  //  input_.moment(1)
  //            << "\t mz: " << input_.moment(2) << " f " << input_.thrust
  //            << std::endl;

  // storage
  _moment = moment;
}

void AttitudeGeometricClfQP::compute_error_vectors() {

  matrix::Dcmf Rt, diffRt;
  Rt = cmd_attitude_.R().T() * firmware_.att_estimator_->state().R();
  diffRt = Rt - Rt.T();
  eR = diffRt.vee() * 0.5;

  eOmega = firmware_.att_estimator_->state().ang_vel -
      firmware_.att_estimator_->state().R().T() * cmd_attitude_.R() *
          cmd_attitude_.ang_vel;

  dR = firmware_.att_estimator_->state().R() *
      firmware_.att_estimator_->state().ang_vel.hat();
  dRc = cmd_attitude_.R() * cmd_attitude_.ang_vel.hat();

  matrix::Dcmf tmp1, tmp2;
  tmp1 = dRc.T() * firmware_.att_estimator_->state().R() -
      firmware_.att_estimator_->state().R().T() * dRc;
  tmp2 = cmd_attitude_.R().T() * dR - dR.T() * cmd_attitude_.R();
  deR = (tmp1.vee() + tmp2.vee()) * 0.5;
}

void AttitudeGeometricClfQP::run_geom_pd(matrix::Vector3f &moment) {

  moment = -_gains_att.kp().emult(eR) - _gains_att.kd().emult(eOmega);
  moment += firmware_.att_estimator_->state().ang_vel.hat() * inertia_matrix_ *
      firmware_.att_estimator_->state().ang_vel;
}

bool AttitudeGeometricClfQP::run_clf_qp(matrix::Vector3f &moment) {
  // read paramters
  c = firmware_.params_->get(Params::ATT_CLF_QP_RATE_OF_CONVERGENCE);
  eta = firmware_.params_->get(Params::ATT_CLF_QP_ETA);
  epsilon = firmware_.params_->get(Params::ATT_CLF_QP_EPSILON);

  V = eOmega.T() * inertia_scaled * eOmega * 0.5;
  V += eR.T() * eOmega * epsilon;
  V += eR.T() * eR * (0.5f * c);

  // TODO write different variable to log everything
  lyap_ = V(0, 0); // logging the value

  LgV = eOmega.T() * inertia_scaled + eR.T() * epsilon;
  LfV = (eOmega.T() * epsilon + eR.T() * c) * eR;
  LfV += -LgV * (dR.T() * cmd_attitude_.R() *
      cmd_attitude_.ang_vel); // TODO: add the trajd.dOmega term

  for (int i = 0; i < 3; i++) {
    A[i] = LgV(0, i);
  }
  ubA[0] = -LfV(0, 0) - eta * V(0, 0);

  nWSR = 1e7;

  qpOASES::returnValue sol =
      solver->hotstart(H, g, A, lb, ub, lbA, ubA, nWSR, 0);

  solver->getPrimalSolution(xOpt);
  matrix::Vector3f dOmega =
      matrix::Vector3f((float) xOpt[0], (float) xOpt[1], (float) xOpt[2]);

  moment = inertia_matrix_ * dOmega +
      firmware_.att_estimator_->state().ang_vel.hat() * inertia_matrix_ *
          firmware_.att_estimator_->state().ang_vel;

  if (sol == qpOASES::SUCCESSFUL_RETURN) {
    return true;
  }
  if (sol == qpOASES::RET_MAX_NWSR_REACHED) {
    Logger::ERROR("qpOASES::RET_MAX_NWSR_REACHED");
  }
  return false;
}

bool AttitudeGeometricClfQP::run_dclf_qp(matrix::Vector3f &moment) {
  // read parameters
  c = firmware_.params_->get(Params::ATT_CLF_QP_RATE_OF_CONVERGENCE);
  eta = firmware_.params_->get(Params::ATT_CLF_QP_ETA);
  epsilon = firmware_.params_->get(Params::ATT_CLF_QP_EPSILON);

  //  // lyapunov functions
  //  V = eOmega.T() * _inertia_scaled * eOmega * 0.5;
  //  V += eR.T() * eOmega * epsilon;
  //  V += eR.T() * eR * (0.5f * c);
  //
  //  LgV = eOmega.T() * _inertia_scaled + eR.T() * epsilon;
  //  LfV = (eOmega.T() * epsilon + eR.T() * c) * eR;
  //  LfV += -LgV * (dR.T() * cmd_attitude_.R() * cmd_attitude_.ang_vel);
  //
  //  // step integration
  //  matrix::Vector3f Om = firmware_.att_estimator_->state().ang_vel;
  //  matrix::Matrix3f R2 =
  //      firmware_.att_estimator_->state().R() * lie_algebra::expSO3((Om *
  //      dt_));
  //
  //  // new errors
  //  matrix::Dcmf Rt, diffRt;
  //  Rt = cmd_attitude_.R().T() * R2;
  //  diffRt = Rt - Rt.T();
  //  matrix::Vector3f eR2 = diffRt.vee() * 0.5;
  //
  //  float b = (float)(eR2.T() * eR2 * c * 0.5f)(0, 0);
  //  matrix::Vector3f rho =
  //      Om - R2.T() * cmd_attitude_.R() * cmd_attitude_.ang_vel;
  //  matrix::Matrix<float, 1, 3> tmp = eR2.T() * epsilon;
  //
  //  matrix::Matrix<float, 1, 3> k_123;
  //  k_123 = dt_ * rho.T() * _inertia_scaled + tmp * dt_;
  //
  //  std::cout<<"V: " << V(0,0) << " eta*dt_s " << eta*dt_ << std::endl;
  //  d_qc = b + (rho.T() * _inertia_scaled * rho * 0.5)(0, 0) + (tmp * rho)(0,
  //  0) +
  //         (V * (eta * dt_ - 1))(0, 0);
  //
  //  // Quadratic constraint
  //  H_qc.setZero();
  //  for (int i = 0; i < 3; i++) {
  //    for (int j = 0; j < 3; j++) {
  //      H_qc(i, j) = (_inertia_scaled(i, j) * dt_ * dt_);
  //    }
  //    k_qc(i) = k_123(0, i);
  //  }
  //
  //  printf("H\n");
  //  H_qc.print();
  //  printf("k_qc");
  //  k_qc.print();
  //  printf("d_qc: %f", d_qc);
  //
  qcqp_solver_->constraint(dt_, firmware_.att_estimator_->state().R(),
                           firmware_.att_estimator_->state().ang_vel,
                           cmd_attitude_.R(), cmd_attitude_.ang_vel);
  //  qcqp_solver_->warm_start(x0_qcqp);
  qcqp_solver_->run();
  //
  moment(0) = (float) qcqp_solver_->xOpt[0];
  moment(1) = (float) qcqp_solver_->xOpt[1];
  moment(2) = (float) qcqp_solver_->xOpt[2];

  // TODO write different variable to log everything
  lyap_ = (float) qcqp_solver_->lyap_; // logging the value

  if (qcqp_solver_->eflag > 0) {
    switch ((int) qcqp_solver_->eflag) {
    case 2:
      Logger::STATUS(
          "Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 3:
      Logger::STATUS(
          "Change in the objective function value was less than options.FunctionTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 4:
      Logger::STATUS(
          "Magnitude of the search direction was less than 2*options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 5:
      Logger::STATUS(
          "Magnitude of directional derivative in search direction was less than 2*options.OptimalityTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    }
    return true;
  } else {
    switch ((int) qcqp_solver_->eflag) {
    case 2:
      Logger::STATUS(
          "Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 3:
      Logger::STATUS(
          "Change in the objective function value was less than options.FunctionTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 4:
      Logger::STATUS(
          "Magnitude of the search direction was less than 2*options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 5:
      Logger::STATUS(
          "Magnitude of directional derivative in search direction was less than 2*options.OptimalityTolerance and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    case 0:
      Logger::WARN(
          "Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations!");
      break;
    case -1:Logger::WARN("Stopped by an output function or plot function!");
      break;
    case -2:Logger::WARN("No feasible point was found!");
      break;
    case -3:
      Logger::WARN(
          "Objective function at current iteration went below options.ObjectiveLimit and maximum constraint violation was less than options.ConstraintTolerance!");
      break;
    }
    return false;
  }
  //  moment.setZero();
  return true;
}

} // namespace qrotor_firmware
