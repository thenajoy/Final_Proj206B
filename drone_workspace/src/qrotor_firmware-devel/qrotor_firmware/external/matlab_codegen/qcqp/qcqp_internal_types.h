//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qcqp_internal_types.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef QCQP_INTERNAL_TYPES_H
#define QCQP_INTERNAL_TYPES_H

// Include Files
#include "anonymous_function.h"
#include "qcqp_types.h"
#include "rtwtypes.h"

// Type Definitions
struct struct_T {
  double grad[6];
  double Hx[5];
  bool hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  bool prev_hasLinear;
  double gammaScalar;
};

struct b_struct_T {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  bool hasObjective;
};

struct c_struct_T {
  bool gradOK;
  bool fevalOK;
  bool done;
  bool stepAccepted;
  bool failedLineSearch;
  int stepType;
};

struct f_struct_T {
  double workspace_double[66];
  int workspace_int[11];
  int workspace_sort[11];
};

struct g_struct_T {
  int ldq;
  double QR[121];
  double Q[121];
  int jpvt[11];
  int mrows;
  int ncols;
  double tau[11];
  int minRowCol;
  bool usedPivoting;
};

struct h_struct_T {
  double FMat[121];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  bool ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
};

struct i_struct_T {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double OptimalityTolerance;
  double ConstraintTolerance;
  double ObjectiveLimit;
  double PricingTolerance;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  bool RemainFeasible;
  bool IterDisplayQP;
};

struct j_struct_T {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[4];
  double xstarsqp_old[4];
  double cIneq;
  double cIneq_old;
  double grad[6];
  double grad_old[6];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[11];
  double lambdasqp_old[11];
  double steplength;
  double delta_x[6];
  double socDirection[6];
  double lambda_old[11];
  int workingset_old[11];
  double JacCineqTrans_old[6];
  double gradLag[6];
  double delta_gradLag[6];
  double xstar[6];
  double fstar;
  double firstorderopt;
  double lambda[11];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[6];
};

struct k_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double Aineq[6];
  double bineq;
  double lb[6];
  double ub[6];
  int indexLB[6];
  int indexUB[6];
  int indexFixed[6];
  int mEqRemoved;
  double ATwset[66];
  double bwset[11];
  int nActiveConstr;
  double maxConstrWorkspace[11];
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  bool isActiveConstr[11];
  int Wid[11];
  int Wlocalidx[11];
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct l_struct_T {
  coder::anonymous_function objfun;
  coder::b_anonymous_function nonlcon;
  int nVar;
  int mCineq;
  int mCeq;
  bool NonFiniteSupport;
  bool SpecifyObjectiveGradient;
  bool SpecifyConstraintGradient;
  bool ScaleProblem;
};

#endif
//
// File trailer for qcqp_internal_types.h
//
// [EOF]
//
