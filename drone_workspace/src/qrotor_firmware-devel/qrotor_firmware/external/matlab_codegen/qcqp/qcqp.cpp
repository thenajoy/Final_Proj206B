//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qcqp.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "qcqp.h"
#include "anonymous_function.h"
#include "driver.h"
#include "evalObjAndConstrAndDerivatives.h"
#include "qcqp_internal_types.h"
#include "qcqp_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Quadtractically constrained quadratic programming
//  cost: structure with fields Q, f, c
//  const: structure with fields H, k, d
//
//
//
// Arguments    : const double Q[16]
//                const double f[4]
//                double c
//                const double H[16]
//                const double k[4]
//                double d
//                const double lb[4]
//                const double ub[4]
//                const double x0[4]
//                double xOpt[4]
//                double *fval
//                double *eflag
//                struct0_T *output
//                struct1_T *lambda
// Return Type  : void
//
void qcqp(const double Q[16], const double f[4], double c, const double H[16],
          const double k[4], double d, const double lb[4], const double ub[4],
          const double x0[4], double xOpt[4], double *fval, double *eflag,
          struct0_T *output, struct1_T *lambda)
{
  b_struct_T MeritFunction;
  f_struct_T memspace;
  g_struct_T QRManager;
  h_struct_T CholManager;
  j_struct_T TrialState;
  k_struct_T WorkingSet;
  l_struct_T FcnEvaluator;
  struct_T QPObjective;
  double Hessian[16];
  double b_TrialState[4];
  double absxk;
  double scale;
  double t;
  double y;
  int colOffsetATw;
  int i;
  int i1;
  int i2;
  int idx;
  int mFixed;
  int mUB;
  signed char obj_tmp[5];
  signed char b_i;
  bool guard1{false};
  FcnEvaluator.objfun.workspace.c = c;
  FcnEvaluator.objfun.workspace.f[0] = f[0];
  FcnEvaluator.objfun.workspace.f[1] = f[1];
  FcnEvaluator.objfun.workspace.f[2] = f[2];
  FcnEvaluator.objfun.workspace.f[3] = f[3];
  FcnEvaluator.nonlcon.workspace.d = d;
  std::copy(&Q[0], &Q[16], &FcnEvaluator.objfun.workspace.Q[0]);
  std::copy(&H[0], &H[16], &FcnEvaluator.nonlcon.workspace.H[0]);
  FcnEvaluator.nonlcon.workspace.k[0] = k[0];
  FcnEvaluator.nonlcon.workspace.k[1] = k[1];
  FcnEvaluator.nonlcon.workspace.k[2] = k[2];
  FcnEvaluator.nonlcon.workspace.k[3] = k[3];
  TrialState.nVarMax = 6;
  TrialState.mNonlinIneq = 1;
  TrialState.mNonlinEq = 0;
  TrialState.mIneq = 1;
  TrialState.mEq = 0;
  TrialState.iNonIneq0 = 1;
  TrialState.iNonEq0 = 1;
  TrialState.sqpFval_old = 0.0;
  TrialState.sqpIterations = 0;
  TrialState.sqpExitFlag = 0;
  std::memset(&TrialState.lambdasqp[0], 0, 11U * sizeof(double));
  TrialState.steplength = 1.0;
  for (i = 0; i < 6; i++) {
    TrialState.delta_x[i] = 0.0;
  }
  TrialState.fstar = 0.0;
  TrialState.firstorderopt = 0.0;
  std::memset(&TrialState.lambda[0], 0, 11U * sizeof(double));
  TrialState.state = 0;
  TrialState.maxConstr = 0.0;
  TrialState.iterations = 0;
  TrialState.xstarsqp[0] = x0[0];
  TrialState.xstarsqp[1] = x0[1];
  TrialState.xstarsqp[2] = x0[2];
  TrialState.xstarsqp[3] = x0[3];
  FcnEvaluator.nVar = 4;
  FcnEvaluator.mCineq = 1;
  FcnEvaluator.mCeq = 0;
  FcnEvaluator.NonFiniteSupport = true;
  FcnEvaluator.SpecifyObjectiveGradient = true;
  FcnEvaluator.SpecifyConstraintGradient = true;
  FcnEvaluator.ScaleProblem = false;
  WorkingSet.nVar = 4;
  WorkingSet.nVarOrig = 4;
  WorkingSet.nVarMax = 6;
  WorkingSet.ldA = 6;
  WorkingSet.bineq = 0.0;
  for (i = 0; i < 6; i++) {
    WorkingSet.Aineq[i] = 0.0;
    WorkingSet.lb[i] = 0.0;
    WorkingSet.ub[i] = 0.0;
  }
  WorkingSet.mEqRemoved = 0;
  std::memset(&WorkingSet.ATwset[0], 0, 66U * sizeof(double));
  WorkingSet.nActiveConstr = 0;
  std::memset(&WorkingSet.bwset[0], 0, 11U * sizeof(double));
  std::memset(&WorkingSet.maxConstrWorkspace[0], 0, 11U * sizeof(double));
  for (i = 0; i < 11; i++) {
    WorkingSet.isActiveConstr[i] = false;
    WorkingSet.Wid[i] = 0;
    WorkingSet.Wlocalidx[i] = 0;
  }
  for (i = 0; i < 5; i++) {
    WorkingSet.nWConstr[i] = 0;
  }
  WorkingSet.probType = 3;
  WorkingSet.SLACK0 = 1.0E-5;
  for (i = 0; i < 6; i++) {
    WorkingSet.indexLB[i] = 0;
    WorkingSet.indexUB[i] = 0;
    WorkingSet.indexFixed[i] = 0;
  }
  colOffsetATw = 1;
  mUB = 0;
  mFixed = 0;
  guard1 = false;
  if ((!std::isinf(lb[0])) && (!std::isnan(lb[0]))) {
    if (std::abs(lb[0] - ub[0]) < 1.0E-6) {
      mFixed = 1;
      WorkingSet.indexFixed[0] = 1;
    } else {
      colOffsetATw = 2;
      WorkingSet.indexLB[0] = 1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1 && ((!std::isinf(ub[0])) && (!std::isnan(ub[0])))) {
    mUB = 1;
    WorkingSet.indexUB[0] = 1;
  }
  guard1 = false;
  if ((!std::isinf(lb[1])) && (!std::isnan(lb[1]))) {
    if (std::abs(lb[1] - ub[1]) < 1.0E-6) {
      mFixed++;
      WorkingSet.indexFixed[mFixed - 1] = 2;
    } else {
      colOffsetATw++;
      WorkingSet.indexLB[colOffsetATw - 2] = 2;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1 && ((!std::isinf(ub[1])) && (!std::isnan(ub[1])))) {
    mUB++;
    WorkingSet.indexUB[mUB - 1] = 2;
  }
  guard1 = false;
  if ((!std::isinf(lb[2])) && (!std::isnan(lb[2]))) {
    if (std::abs(lb[2] - ub[2]) < 1.0E-6) {
      mFixed++;
      WorkingSet.indexFixed[mFixed - 1] = 3;
    } else {
      colOffsetATw++;
      WorkingSet.indexLB[colOffsetATw - 2] = 3;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1 && ((!std::isinf(ub[2])) && (!std::isnan(ub[2])))) {
    mUB++;
    WorkingSet.indexUB[mUB - 1] = 3;
  }
  guard1 = false;
  if ((!std::isinf(lb[3])) && (!std::isnan(lb[3]))) {
    if (std::abs(lb[3] - ub[3]) < 1.0E-6) {
      mFixed++;
      WorkingSet.indexFixed[mFixed - 1] = 4;
    } else {
      colOffsetATw++;
      WorkingSet.indexLB[colOffsetATw - 2] = 4;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1 && ((!std::isinf(ub[3])) && (!std::isnan(ub[3])))) {
    mUB++;
    WorkingSet.indexUB[mUB - 1] = 4;
  }
  i = (colOffsetATw + mUB) + mFixed;
  WorkingSet.mConstr = i;
  WorkingSet.mConstrOrig = i;
  WorkingSet.mConstrMax = 11;
  obj_tmp[0] = static_cast<signed char>(mFixed);
  obj_tmp[1] = 0;
  obj_tmp[2] = 1;
  obj_tmp[3] = static_cast<signed char>(colOffsetATw - 1);
  obj_tmp[4] = static_cast<signed char>(mUB);
  for (i = 0; i < 5; i++) {
    b_i = obj_tmp[i];
    WorkingSet.sizes[i] = b_i;
    WorkingSet.sizesNormal[i] = b_i;
  }
  obj_tmp[0] = static_cast<signed char>(mFixed);
  obj_tmp[1] = 0;
  obj_tmp[2] = 1;
  obj_tmp[3] = static_cast<signed char>(colOffsetATw);
  obj_tmp[4] = static_cast<signed char>(mUB);
  WorkingSet.sizesRegPhaseOne[0] = mFixed;
  WorkingSet.sizesRegPhaseOne[1] = 0;
  WorkingSet.sizesRegPhaseOne[2] = 1;
  WorkingSet.sizesRegPhaseOne[3] = colOffsetATw + 1;
  WorkingSet.sizesRegPhaseOne[4] = mUB;
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[4] = colOffsetATw - 1;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    b_i = obj_tmp[i];
    WorkingSet.sizesPhaseOne[i] = b_i;
    WorkingSet.sizesRegularized[i] = b_i;
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (i1 = 0; i1 < 6; i1++) {
    i2 = WorkingSet.isActiveIdxRegPhaseOne[i1];
    WorkingSet.isActiveIdx[i1] = i2;
    WorkingSet.isActiveIdxNormal[i1] = i2;
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[4] = colOffsetATw;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (i1 = 0; i1 < 6; i1++) {
    i2 = WorkingSet.isActiveIdxRegPhaseOne[i1];
    WorkingSet.isActiveIdxPhaseOne[i1] = i2;
    WorkingSet.isActiveIdxRegularized[i1] = i2;
  }
  WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
  WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
  WorkingSet.isActiveIdxRegPhaseOne[3] = 1;
  WorkingSet.isActiveIdxRegPhaseOne[4] = colOffsetATw + 1;
  WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
  for (i = 0; i < 5; i++) {
    WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
        WorkingSet.isActiveIdxRegPhaseOne[i];
  }
  for (idx = 0; idx <= colOffsetATw - 2; idx++) {
    i1 = WorkingSet.indexLB[idx];
    TrialState.xstarsqp[i1 - 1] =
        std::fmax(TrialState.xstarsqp[i1 - 1], lb[i1 - 1]);
  }
  for (idx = 0; idx < mUB; idx++) {
    i1 = WorkingSet.indexUB[idx];
    TrialState.xstarsqp[i1 - 1] =
        std::fmin(TrialState.xstarsqp[i1 - 1], ub[i1 - 1]);
  }
  for (idx = 0; idx < mFixed; idx++) {
    i1 = WorkingSet.indexFixed[idx];
    TrialState.xstarsqp[i1 - 1] = ub[i1 - 1];
  }
  b_TrialState[0] = TrialState.xstarsqp[0];
  b_TrialState[1] = TrialState.xstarsqp[1];
  b_TrialState[2] = TrialState.xstarsqp[2];
  b_TrialState[3] = TrialState.xstarsqp[3];
  coder::optim::coder::utils::ObjNonlinEvaluator::
      evalObjAndConstrAndDerivatives(&FcnEvaluator.objfun,
                                     &FcnEvaluator.nonlcon, b_TrialState,
                                     TrialState.grad, &TrialState.cIneq,
                                     WorkingSet.Aineq, &TrialState.sqpFval, &i);
  TrialState.FunctionEvaluations = 1;
  WorkingSet.bineq = -TrialState.cIneq;
  for (idx = 0; idx <= colOffsetATw - 2; idx++) {
    WorkingSet.lb[WorkingSet.indexLB[idx] - 1] =
        -lb[WorkingSet.indexLB[idx] - 1] + x0[WorkingSet.indexLB[idx] - 1];
  }
  for (idx = 0; idx < mUB; idx++) {
    WorkingSet.ub[WorkingSet.indexUB[idx] - 1] =
        ub[WorkingSet.indexUB[idx] - 1] - x0[WorkingSet.indexUB[idx] - 1];
  }
  for (idx = 0; idx < mFixed; idx++) {
    scale =
        ub[WorkingSet.indexFixed[idx] - 1] - x0[WorkingSet.indexFixed[idx] - 1];
    WorkingSet.ub[WorkingSet.indexFixed[idx] - 1] = scale;
    WorkingSet.bwset[idx] = scale;
  }
  coder::optim::coder::qpactiveset::WorkingSet::setProblemType(&WorkingSet, 3);
  i = WorkingSet.isActiveIdx[2];
  for (idx = i; idx < 12; idx++) {
    WorkingSet.isActiveConstr[idx - 1] = false;
  }
  WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
  WorkingSet.nWConstr[1] = 0;
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
  i = WorkingSet.sizes[0];
  for (idx = 0; idx < i; idx++) {
    WorkingSet.Wid[idx] = 1;
    WorkingSet.Wlocalidx[idx] = idx + 1;
    WorkingSet.isActiveConstr[idx] = true;
    colOffsetATw = 6 * idx;
    i1 = WorkingSet.indexFixed[idx];
    if (0 <= i1 - 2) {
      std::memset(&WorkingSet.ATwset[colOffsetATw], 0,
                  (((i1 + colOffsetATw) - colOffsetATw) + -1) * sizeof(double));
    }
    WorkingSet.ATwset[(WorkingSet.indexFixed[idx] + colOffsetATw) - 1] = 1.0;
    i1 = WorkingSet.indexFixed[idx] + 1;
    i2 = WorkingSet.nVar;
    if (i1 <= i2) {
      std::memset(&WorkingSet.ATwset[(i1 + colOffsetATw) + -1], 0,
                  ((((i2 + colOffsetATw) - i1) - colOffsetATw) + 1) *
                      sizeof(double));
    }
    WorkingSet.bwset[idx] = WorkingSet.ub[WorkingSet.indexFixed[idx] - 1];
  }
  MeritFunction.initFval = TrialState.sqpFval;
  MeritFunction.penaltyParam = 1.0;
  MeritFunction.threshold = 0.0001;
  MeritFunction.nPenaltyDecreases = 0;
  MeritFunction.linearizedConstrViol = 0.0;
  MeritFunction.initConstrViolationEq = 0.0;
  MeritFunction.initConstrViolationIneq = 0.0;
  if (TrialState.cIneq > 0.0) {
    MeritFunction.initConstrViolationIneq = TrialState.cIneq;
  }
  MeritFunction.phi = 0.0;
  MeritFunction.phiPrimePlus = 0.0;
  MeritFunction.phiFullStep = 0.0;
  MeritFunction.feasRelativeFactor = 0.0;
  MeritFunction.nlpPrimalFeasError = 0.0;
  MeritFunction.nlpDualFeasError = 0.0;
  MeritFunction.nlpComplError = 0.0;
  MeritFunction.firstOrderOpt = 0.0;
  MeritFunction.hasObjective = true;
  QRManager.ldq = 11;
  std::memset(&QRManager.QR[0], 0, 121U * sizeof(double));
  std::memset(&QRManager.Q[0], 0, 121U * sizeof(double));
  QRManager.mrows = 0;
  QRManager.ncols = 0;
  std::memset(&QRManager.tau[0], 0, 11U * sizeof(double));
  for (i = 0; i < 11; i++) {
    QRManager.jpvt[i] = 0;
  }
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  for (i = 0; i < 6; i++) {
    QPObjective.grad[i] = 0.0;
  }
  for (i = 0; i < 5; i++) {
    QPObjective.Hx[i] = 0.0;
  }
  QPObjective.hasLinear = true;
  QPObjective.nvar = 4;
  QPObjective.maxVar = 6;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.objtype = 3;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  coder::optim::coder::fminconsqp::driver(
      lb, ub, &TrialState, &MeritFunction, &FcnEvaluator, &memspace,
      &WorkingSet, &QRManager, &QPObjective, Hessian, &CholManager);
  xOpt[0] = TrialState.xstarsqp[0];
  xOpt[1] = TrialState.xstarsqp[1];
  xOpt[2] = TrialState.xstarsqp[2];
  xOpt[3] = TrialState.xstarsqp[3];
  *fval = TrialState.sqpFval;
  *eflag = TrialState.sqpExitFlag;
  output->iterations = TrialState.sqpIterations;
  output->funcCount = TrialState.FunctionEvaluations;
  output->algorithm[0] = 's';
  output->algorithm[1] = 'q';
  output->algorithm[2] = 'p';
  output->constrviolation = MeritFunction.nlpPrimalFeasError;
  scale = 3.3121686421112381E-170;
  absxk = std::abs(TrialState.delta_x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = std::abs(TrialState.delta_x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(TrialState.delta_x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(TrialState.delta_x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  output->stepsize = scale * std::sqrt(y);
  output->lssteplength = TrialState.steplength;
  output->firstorderopt = MeritFunction.firstOrderOpt;
  mFixed = WorkingSet.sizes[0];
  colOffsetATw = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  lambda->lower[0] = 0.0;
  lambda->upper[0] = 0.0;
  lambda->lower[1] = 0.0;
  lambda->upper[1] = 0.0;
  lambda->lower[2] = 0.0;
  lambda->upper[2] = 0.0;
  lambda->lower[3] = 0.0;
  lambda->upper[3] = 0.0;
  lambda->ineqnonlin = TrialState.lambdasqp[WorkingSet.sizes[0]];
  for (idx = 0; idx < mFixed; idx++) {
    lambda->lower[WorkingSet.indexFixed[idx] - 1] = -TrialState.lambdasqp[idx];
  }
  for (idx = 0; idx < colOffsetATw; idx++) {
    lambda->lower[WorkingSet.indexLB[idx] - 1] =
        TrialState.lambdasqp[(WorkingSet.sizes[0] + idx) + 1];
  }
  i = (WorkingSet.sizes[0] + WorkingSet.sizes[3]) + 1;
  for (idx = 0; idx < mUB; idx++) {
    lambda->upper[WorkingSet.indexUB[idx] - 1] = TrialState.lambdasqp[i + idx];
  }
}

//
// File trailer for qcqp.cpp
//
// [EOF]
//
