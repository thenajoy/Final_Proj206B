//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: step.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "step.h"
#include "addAineqConstr.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "qcqp_internal_types.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : int *STEP_TYPE
//                double Hessian[16]
//                const double lb[4]
//                const double ub[4]
//                j_struct_T *TrialState
//                b_struct_T *MeritFunction
//                f_struct_T *memspace
//                k_struct_T *WorkingSet
//                g_struct_T *b_QRManager
//                h_struct_T *b_CholManager
//                struct_T *QPObjective
//                i_struct_T *qpoptions
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool b_step(int *STEP_TYPE, double Hessian[16], const double lb[4],
            const double ub[4], j_struct_T *TrialState,
            b_struct_T *MeritFunction, f_struct_T *memspace,
            k_struct_T *WorkingSet, g_struct_T *b_QRManager,
            h_struct_T *b_CholManager, struct_T *QPObjective,
            i_struct_T *qpoptions)
{
  i_struct_T b_qpoptions;
  double dv[6];
  double c;
  double penaltyParamTrial;
  int idx;
  int idx_Aineq;
  int idx_lower;
  int idx_upper;
  int nVar_tmp_tmp;
  bool checkBoundViolation;
  bool stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet->nVar - 1;
  if (*STEP_TYPE != 3) {
    if (0 <= nVar_tmp_tmp) {
      std::copy(&TrialState->xstarsqp[0],
                &TrialState->xstarsqp[nVar_tmp_tmp + 1], &TrialState->xstar[0]);
    }
  } else if (0 <= nVar_tmp_tmp) {
    std::copy(&TrialState->xstar[0], &TrialState->xstar[nVar_tmp_tmp + 1],
              &TrialState->searchDir[0]);
  }
  int exitg1;
  bool guard1{false};
  do {
    exitg1 = 0;
    guard1 = false;
    switch (*STEP_TYPE) {
    case 1: {
      b_qpoptions = *qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, TrialState->grad, TrialState, memspace, WorkingSet,
          b_QRManager, b_CholManager, QPObjective, &b_qpoptions,
          qpoptions->MaxIterations);
      if (TrialState->state > 0) {
        double constrViolationIneq;
        c = TrialState->cIneq;
        penaltyParamTrial = MeritFunction->penaltyParam;
        constrViolationIneq = 0.0;
        if (c > 0.0) {
          constrViolationIneq = c;
        }
        c = MeritFunction->linearizedConstrViol;
        MeritFunction->linearizedConstrViol = 0.0;
        c += constrViolationIneq;
        if ((c > 2.2204460492503131E-16) && (TrialState->fstar > 0.0)) {
          if (TrialState->sqpFval == 0.0) {
            penaltyParamTrial = 1.0;
          } else {
            penaltyParamTrial = 1.5;
          }
          penaltyParamTrial = penaltyParamTrial * TrialState->fstar / c;
        }
        if (penaltyParamTrial < MeritFunction->penaltyParam) {
          MeritFunction->phi =
              TrialState->sqpFval + penaltyParamTrial * constrViolationIneq;
          if ((MeritFunction->initFval +
               penaltyParamTrial * MeritFunction->initConstrViolationIneq) -
                  MeritFunction->phi >
              static_cast<double>(MeritFunction->nPenaltyDecreases) *
                  MeritFunction->threshold) {
            MeritFunction->nPenaltyDecreases++;
            if ((MeritFunction->nPenaltyDecreases << 1) >
                TrialState->sqpIterations) {
              MeritFunction->threshold *= 10.0;
            }
            MeritFunction->penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          } else {
            MeritFunction->phi =
                TrialState->sqpFval +
                MeritFunction->penaltyParam * constrViolationIneq;
          }
        } else {
          MeritFunction->penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          MeritFunction->phi =
              TrialState->sqpFval +
              MeritFunction->penaltyParam * constrViolationIneq;
        }
        MeritFunction->phiPrimePlus =
            std::fmin(TrialState->fstar -
                          MeritFunction->penaltyParam * constrViolationIneq,
                      0.0);
      }
      qpactiveset::parseoutput::sortLambdaQP(
          TrialState->lambda, WorkingSet->nActiveConstr, WorkingSet->sizes,
          WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
          memspace->workspace_double);
      if ((TrialState->state <= 0) && (TrialState->state != -6)) {
        *STEP_TYPE = 2;
      } else {
        if (0 <= nVar_tmp_tmp) {
          std::copy(&TrialState->xstar[0], &TrialState->xstar[nVar_tmp_tmp + 1],
                    &TrialState->delta_x[0]);
        }
        guard1 = true;
      }
    } break;
    case 2:
      idx_lower = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
      idx_Aineq = WorkingSet->nActiveConstr;
      for (int idx_Partition{idx_lower}; idx_Partition <= idx_Aineq;
           idx_Partition++) {
        WorkingSet->isActiveConstr
            [(WorkingSet->isActiveIdx[WorkingSet->Wid[idx_Partition - 1] - 1] +
              WorkingSet->Wlocalidx[idx_Partition - 1]) -
             2] = false;
      }
      WorkingSet->nWConstr[2] = 0;
      WorkingSet->nWConstr[3] = 0;
      WorkingSet->nWConstr[4] = 0;
      WorkingSet->nActiveConstr =
          WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
      for (idx_lower = 0; idx_lower < 6; idx_lower++) {
        dv[idx_lower] = TrialState->xstar[idx_lower];
      }
      idx_upper = WorkingSet->sizes[3];
      idx_Aineq = WorkingSet->sizes[4];
      for (idx = 0; idx < idx_upper; idx++) {
        c = WorkingSet->lb[WorkingSet->indexLB[idx] - 1];
        if (-dv[WorkingSet->indexLB[idx] - 1] > c) {
          if (std::isinf(ub[WorkingSet->indexLB[idx] - 1])) {
            dv[WorkingSet->indexLB[idx] - 1] = -c + std::abs(c);
          } else {
            dv[WorkingSet->indexLB[idx] - 1] =
                (WorkingSet->ub[WorkingSet->indexLB[idx] - 1] - c) / 2.0;
          }
        }
      }
      for (idx = 0; idx < idx_Aineq; idx++) {
        c = WorkingSet->ub[WorkingSet->indexUB[idx] - 1];
        if (dv[WorkingSet->indexUB[idx] - 1] > c) {
          if (std::isinf(lb[WorkingSet->indexUB[idx] - 1])) {
            dv[WorkingSet->indexUB[idx] - 1] = c - std::abs(c);
          } else {
            dv[WorkingSet->indexUB[idx] - 1] =
                (c - WorkingSet->lb[WorkingSet->indexUB[idx] - 1]) / 2.0;
          }
        }
      }
      for (idx_lower = 0; idx_lower < 6; idx_lower++) {
        TrialState->xstar[idx_lower] = dv[idx_lower];
      }
      step::relaxed(Hessian, TrialState->grad, TrialState, MeritFunction,
                    memspace, WorkingSet, b_QRManager, b_CholManager,
                    QPObjective, qpoptions);
      if (0 <= nVar_tmp_tmp) {
        std::copy(&TrialState->xstar[0], &TrialState->xstar[nVar_tmp_tmp + 1],
                  &TrialState->delta_x[0]);
      }
      guard1 = true;
      break;
    default: {
      int i;
      int i1;
      int idxIneqOffset;
      int idx_Partition;
      int nWIneq_old;
      int nWLower_old;
      int nWUpper_old;
      nWIneq_old = WorkingSet->nWConstr[2];
      nWLower_old = WorkingSet->nWConstr[3];
      nWUpper_old = WorkingSet->nWConstr[4];
      i = WorkingSet->nVar - 1;
      if (0 <= i) {
        std::copy(&TrialState->xstarsqp_old[0],
                  &TrialState->xstarsqp_old[i + 1], &TrialState->xstarsqp[0]);
        std::copy(&TrialState->xstar[0], &TrialState->xstar[i + 1],
                  &TrialState->socDirection[0]);
      }
      std::copy(&TrialState->lambda[0], &TrialState->lambda[11],
                &TrialState->lambda_old[0]);
      idxIneqOffset = WorkingSet->isActiveIdx[2];
      WorkingSet->bineq = -TrialState->cIneq;
      i1 = WorkingSet->nVar;
      c = 0.0;
      for (idx_Aineq = 1; idx_Aineq <= i1; idx_Aineq++) {
        c += WorkingSet->Aineq[idx_Aineq - 1] *
             TrialState->searchDir[idx_Aineq - 1];
      }
      c += WorkingSet->bineq;
      WorkingSet->bineq = c;
      idx_Aineq = 1;
      idx_lower = 2;
      idx_upper = WorkingSet->sizes[3] + 2;
      i1 = WorkingSet->nActiveConstr;
      for (idx = idxIneqOffset; idx <= i1; idx++) {
        switch (WorkingSet->Wid[idx - 1]) {
        case 3:
          idx_Partition = idx_Aineq;
          idx_Aineq++;
          WorkingSet->bwset[idx - 1] = c;
          break;
        case 4:
          idx_Partition = idx_lower;
          idx_lower++;
          break;
        default:
          idx_Partition = idx_upper;
          idx_upper++;
          break;
        }
        TrialState->workingset_old[idx_Partition - 1] =
            WorkingSet->Wlocalidx[idx - 1];
      }
      if (0 <= i) {
        std::copy(&TrialState->xstarsqp[0], &TrialState->xstarsqp[i + 1],
                  &TrialState->xstar[0]);
      }
      for (idx_lower = 0; idx_lower < 6; idx_lower++) {
        dv[idx_lower] = TrialState->grad[idx_lower];
      }
      b_qpoptions = *qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, dv, TrialState, memspace, WorkingSet, b_QRManager,
          b_CholManager, QPObjective, &b_qpoptions, qpoptions->MaxIterations);
      for (idx = 0; idx <= i; idx++) {
        c = TrialState->socDirection[idx];
        TrialState->socDirection[idx] =
            TrialState->xstar[idx] - TrialState->socDirection[idx];
        TrialState->xstar[idx] = c;
      }
      stepSuccess =
          (::coder::internal::blas::xnrm2(i + 1, TrialState->socDirection) <=
           2.0 * ::coder::internal::blas::xnrm2(i + 1, TrialState->xstar));
      idx_upper = WorkingSet->sizes[3];
      WorkingSet->bineq = -TrialState->cIneq;
      if (!stepSuccess) {
        idx_lower = (WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1]) + 1;
        i = WorkingSet->nActiveConstr;
        for (idx_Partition = idx_lower; idx_Partition <= i; idx_Partition++) {
          WorkingSet
              ->isActiveConstr[(WorkingSet->isActiveIdx
                                    [WorkingSet->Wid[idx_Partition - 1] - 1] +
                                WorkingSet->Wlocalidx[idx_Partition - 1]) -
                               2] = false;
        }
        WorkingSet->nWConstr[2] = 0;
        WorkingSet->nWConstr[3] = 0;
        WorkingSet->nWConstr[4] = 0;
        WorkingSet->nActiveConstr =
            WorkingSet->nWConstr[0] + WorkingSet->nWConstr[1];
        for (idx = 0; idx < nWIneq_old; idx++) {
          qpactiveset::WorkingSet::addAineqConstr(
              WorkingSet, TrialState->workingset_old[idx]);
        }
        for (idx = 0; idx < nWLower_old; idx++) {
          qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
              WorkingSet, 4, TrialState->workingset_old[idx + 1]);
        }
        for (idx = 0; idx < nWUpper_old; idx++) {
          qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
              WorkingSet, 5, TrialState->workingset_old[(idx + idx_upper) + 1]);
        }
        std::copy(&TrialState->lambda_old[0], &TrialState->lambda_old[11],
                  &TrialState->lambda[0]);
      } else {
        qpactiveset::parseoutput::sortLambdaQP(
            TrialState->lambda, WorkingSet->nActiveConstr, WorkingSet->sizes,
            WorkingSet->isActiveIdx, WorkingSet->Wid, WorkingSet->Wlocalidx,
            memspace->workspace_double);
      }
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState->state != -6)) {
        for (idx = 0; idx <= nVar_tmp_tmp; idx++) {
          TrialState->delta_x[idx] =
              TrialState->xstar[idx] + TrialState->socDirection[idx];
        }
      }
      guard1 = true;
    } break;
    }
    if (guard1) {
      if (TrialState->state != -6) {
        exitg1 = 1;
      } else {
        c = std::fmax(
            2.2204460492503131E-16,
            std::fmax(
                std::fmax(
                    std::fmax(std::fmax(0.0, std::abs(TrialState->grad[0])),
                              std::abs(TrialState->grad[1])),
                    std::abs(TrialState->grad[2])),
                std::abs(TrialState->grad[3])) /
                std::fmax(
                    std::fmax(
                        std::fmax(
                            std::fmax(1.0, std::abs(TrialState->xstar[0])),
                            std::abs(TrialState->xstar[1])),
                        std::abs(TrialState->xstar[2])),
                    std::abs(TrialState->xstar[3])));
        for (idx_upper = 0; idx_upper < 4; idx_upper++) {
          idx_Aineq = idx_upper << 2;
          for (idx_lower = 0; idx_lower < idx_upper; idx_lower++) {
            Hessian[idx_Aineq + idx_lower] = 0.0;
          }
          idx_Aineq += idx_upper;
          Hessian[idx_Aineq] = c;
          idx_lower = 2 - idx_upper;
          if (0 <= idx_lower) {
            std::memset(&Hessian[idx_Aineq + 1], 0,
                        (((idx_lower + idx_Aineq) - idx_Aineq) + 1) *
                            sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    idx_upper = WorkingSet->sizes[3];
    idx_Aineq = WorkingSet->sizes[4];
    for (idx_lower = 0; idx_lower < 6; idx_lower++) {
      dv[idx_lower] = TrialState->delta_x[idx_lower];
    }
    for (idx = 0; idx < idx_upper; idx++) {
      c = dv[WorkingSet->indexLB[idx] - 1];
      penaltyParamTrial =
          (TrialState->xstarsqp[WorkingSet->indexLB[idx] - 1] + c) -
          lb[WorkingSet->indexLB[idx] - 1];
      if (penaltyParamTrial < 0.0) {
        dv[WorkingSet->indexLB[idx] - 1] = c - penaltyParamTrial;
        TrialState->xstar[WorkingSet->indexLB[idx] - 1] -= penaltyParamTrial;
      }
    }
    for (idx = 0; idx < idx_Aineq; idx++) {
      c = dv[WorkingSet->indexUB[idx] - 1];
      penaltyParamTrial = (ub[WorkingSet->indexUB[idx] - 1] -
                           TrialState->xstarsqp[WorkingSet->indexUB[idx] - 1]) -
                          c;
      if (penaltyParamTrial < 0.0) {
        dv[WorkingSet->indexUB[idx] - 1] = c + penaltyParamTrial;
        TrialState->xstar[WorkingSet->indexUB[idx] - 1] += penaltyParamTrial;
      }
    }
    for (idx_lower = 0; idx_lower < 6; idx_lower++) {
      TrialState->delta_x[idx_lower] = dv[idx_lower];
    }
  }
  return stepSuccess;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for step.cpp
//
// [EOF]
//
