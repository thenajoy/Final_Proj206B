//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: relaxed.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "relaxed.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double Hessian[16]
//                const double grad[6]
//                j_struct_T *TrialState
//                b_struct_T *MeritFunction
//                f_struct_T *memspace
//                k_struct_T *WorkingSet
//                g_struct_T *b_QRManager
//                h_struct_T *b_CholManager
//                struct_T *QPObjective
//                i_struct_T *qpoptions
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
void relaxed(const double Hessian[16], const double grad[6],
             j_struct_T *TrialState, b_struct_T *MeritFunction,
             f_struct_T *memspace, k_struct_T *WorkingSet,
             g_struct_T *b_QRManager, h_struct_T *b_CholManager,
             struct_T *QPObjective, i_struct_T *qpoptions)
{
  i_struct_T b_qpoptions;
  double beta;
  double rho;
  double s;
  double smax;
  int idx;
  int idx_max;
  int k;
  int nVarOrig_tmp_tmp;
  int temp;
  bool tf;
  nVarOrig_tmp_tmp = WorkingSet->nVar;
  beta = 0.0;
  for (idx = 0; idx < nVarOrig_tmp_tmp; idx++) {
    beta += Hessian[idx + (idx << 2)];
  }
  beta /= static_cast<double>(WorkingSet->nVar);
  if (TrialState->sqpIterations <= 1) {
    temp = QPObjective->nvar;
    if (QPObjective->nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective->nvar > 1) {
        smax = std::abs(grad[0]);
        for (k = 2; k <= temp; k++) {
          s = std::abs(grad[k - 1]);
          if (s > smax) {
            idx_max = k;
            smax = s;
          }
        }
      }
    }
    rho = 100.0 * std::fmax(1.0, std::abs(grad[idx_max - 1]));
  } else {
    temp = WorkingSet->mConstr;
    idx_max = 1;
    if (WorkingSet->mConstr > 1) {
      smax = std::abs(TrialState->lambdasqp[0]);
      for (k = 2; k <= temp; k++) {
        s = std::abs(TrialState->lambdasqp[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
    rho = std::abs(TrialState->lambdasqp[idx_max - 1]);
  }
  QPObjective->nvar = WorkingSet->nVar;
  QPObjective->beta = beta;
  QPObjective->rho = rho;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 4;
  qpactiveset::WorkingSet::setProblemType(WorkingSet, 2);
  memspace->workspace_double[0] = WorkingSet->bineq;
  memspace->workspace_double[0] = -memspace->workspace_double[0];
  smax = 0.0;
  for (temp = 1; temp <= nVarOrig_tmp_tmp; temp++) {
    smax += WorkingSet->Aineq[temp - 1] * TrialState->xstar[temp - 1];
  }
  memspace->workspace_double[0] += smax;
  TrialState->xstar[nVarOrig_tmp_tmp] =
      static_cast<double>(memspace->workspace_double[0] > 0.0) *
      memspace->workspace_double[0];
  if (memspace->workspace_double[0] <= 1.0E-6) {
    qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(WorkingSet, 4,
                                                        WorkingSet->sizes[3]);
  }
  temp = qpoptions->MaxIterations;
  qpoptions->MaxIterations =
      (qpoptions->MaxIterations + WorkingSet->nVar) - nVarOrig_tmp_tmp;
  b_qpoptions = *qpoptions;
  ::coder::optim::coder::qpactiveset::driver(
      Hessian, grad, TrialState, memspace, WorkingSet, b_QRManager,
      b_CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
  qpoptions->MaxIterations = temp;
  tf =
      WorkingSet
          ->isActiveConstr[(WorkingSet->isActiveIdx[3] + WorkingSet->sizes[3]) -
                           2];
  memspace->workspace_int[0] = tf;
  if (TrialState->state != -6) {
    double constrViolationIneq;
    temp = nVarOrig_tmp_tmp + 1;
    smax = 0.0;
    s = 0.0;
    if (5 - nVarOrig_tmp_tmp >= 1) {
      for (k = temp; k <= temp; k++) {
        smax += std::abs(TrialState->xstar[k - 1]);
      }
      s = TrialState->xstar[nVarOrig_tmp_tmp] *
          TrialState->xstar[nVarOrig_tmp_tmp];
    }
    rho = (TrialState->fstar - rho * smax) - beta / 2.0 * s;
    smax = TrialState->cIneq;
    beta = MeritFunction->penaltyParam;
    constrViolationIneq = 0.0;
    if (smax > 0.0) {
      constrViolationIneq = smax;
    }
    smax = MeritFunction->linearizedConstrViol;
    s = 0.0;
    if (5 - nVarOrig_tmp_tmp >= 1) {
      for (k = temp; k <= temp; k++) {
        s += std::abs(TrialState->xstar[k - 1]);
      }
    }
    MeritFunction->linearizedConstrViol = s;
    smax = (constrViolationIneq + smax) - s;
    if ((smax > 2.2204460492503131E-16) && (rho > 0.0)) {
      if (TrialState->sqpFval == 0.0) {
        beta = 1.0;
      } else {
        beta = 1.5;
      }
      beta = beta * rho / smax;
    }
    if (beta < MeritFunction->penaltyParam) {
      MeritFunction->phi = TrialState->sqpFval + beta * constrViolationIneq;
      if ((MeritFunction->initFval +
           beta * MeritFunction->initConstrViolationIneq) -
              MeritFunction->phi >
          static_cast<double>(MeritFunction->nPenaltyDecreases) *
              MeritFunction->threshold) {
        MeritFunction->nPenaltyDecreases++;
        if ((MeritFunction->nPenaltyDecreases << 1) >
            TrialState->sqpIterations) {
          MeritFunction->threshold *= 10.0;
        }
        MeritFunction->penaltyParam = std::fmax(beta, 1.0E-10);
      } else {
        MeritFunction->phi = TrialState->sqpFval +
                             MeritFunction->penaltyParam * constrViolationIneq;
      }
    } else {
      MeritFunction->penaltyParam = std::fmax(beta, 1.0E-10);
      MeritFunction->phi = TrialState->sqpFval +
                           MeritFunction->penaltyParam * constrViolationIneq;
    }
    MeritFunction->phiPrimePlus =
        std::fmin(rho - MeritFunction->penaltyParam * constrViolationIneq, 0.0);
    temp = WorkingSet->isActiveIdx[2];
    idx_max = WorkingSet->nActiveConstr;
    for (idx = temp; idx <= idx_max; idx++) {
      if (WorkingSet->Wid[idx - 1] == 3) {
        TrialState->lambda[idx - 1] *= static_cast<double>(
            memspace->workspace_int[WorkingSet->Wlocalidx[idx - 1] - 1]);
      }
    }
  }
  temp = tf;
  idx = WorkingSet->nActiveConstr - 1;
  while ((idx + 1 > WorkingSet->sizes[0]) && (temp > 0)) {
    if ((WorkingSet->Wid[idx] == 4) &&
        (WorkingSet->Wlocalidx[idx] > WorkingSet->sizes[3] - 1)) {
      smax = TrialState->lambda[WorkingSet->nActiveConstr - 1];
      TrialState->lambda[WorkingSet->nActiveConstr - 1] = 0.0;
      TrialState->lambda[idx] = smax;
      temp = WorkingSet->Wid[idx] - 1;
      WorkingSet
          ->isActiveConstr[(WorkingSet->isActiveIdx[WorkingSet->Wid[idx] - 1] +
                            WorkingSet->Wlocalidx[idx]) -
                           2] = false;
      WorkingSet->Wid[idx] = WorkingSet->Wid[WorkingSet->nActiveConstr - 1];
      WorkingSet->Wlocalidx[idx] =
          WorkingSet->Wlocalidx[WorkingSet->nActiveConstr - 1];
      idx_max = WorkingSet->nVar;
      for (k = 0; k < idx_max; k++) {
        WorkingSet->ATwset[k + 6 * idx] =
            WorkingSet->ATwset[k + 6 * (WorkingSet->nActiveConstr - 1)];
      }
      WorkingSet->bwset[idx] = WorkingSet->bwset[WorkingSet->nActiveConstr - 1];
      WorkingSet->nActiveConstr--;
      WorkingSet->nWConstr[temp]--;
      temp = 0;
    }
    idx--;
  }
  QPObjective->nvar = nVarOrig_tmp_tmp;
  QPObjective->hasLinear = true;
  QPObjective->objtype = 3;
  qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
  qpactiveset::parseoutput::sortLambdaQP(
      TrialState->lambda, WorkingSet->nActiveConstr, WorkingSet->sizes,
      WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
      memspace->workspace_double);
}

} // namespace step
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for relaxed.cpp
//
// [EOF]
//
