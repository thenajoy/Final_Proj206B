//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: test_exit.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "test_exit.h"
#include "computeComplError.h"
#include "computeGradLag.h"
#include "computeQ_.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xgemv.h"
#include "xgeqp3.h"
#include <algorithm>
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : c_struct_T *Flags
//                f_struct_T *memspace
//                b_struct_T *MeritFunction
//                k_struct_T *WorkingSet
//                j_struct_T *TrialState
//                g_struct_T *b_QRManager
//                const double lb[4]
//                const double ub[4]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(c_struct_T *Flags, f_struct_T *memspace,
                 b_struct_T *MeritFunction, k_struct_T *WorkingSet,
                 j_struct_T *TrialState, g_struct_T *b_QRManager,
                 const double lb[4], const double ub[4])
{
  double optimRelativeFactor;
  double s;
  double smax;
  int idx_max;
  int ix;
  int mFixed;
  int mLB;
  int mLambda;
  int mUB;
  int minDim;
  int nVar;
  bool dxTooSmall;
  bool exitg1;
  bool isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda =
      (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  stopping::computeGradLag(
      TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
      WorkingSet->Aineq, WorkingSet->indexFixed, WorkingSet->sizes[0],
      WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
      WorkingSet->sizes[4], TrialState->lambdasqp);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = std::abs(TrialState->grad[0]);
      for (minDim = 2; minDim <= nVar; minDim++) {
        s = std::abs(TrialState->grad[minDim - 1]);
        if (s > smax) {
          idx_max = minDim;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor = std::fmax(1.0, std::abs(TrialState->grad[idx_max - 1]));
  if (std::isinf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  smax = std::fmax(0.0, TrialState->cIneq);
  for (ix = 0; ix < mLB; ix++) {
    idx_max = WorkingSet->indexLB[ix] - 1;
    smax = std::fmax(smax, lb[idx_max] - TrialState->xstarsqp[idx_max]);
  }
  for (ix = 0; ix < mUB; ix++) {
    idx_max = WorkingSet->indexUB[ix] - 1;
    smax = std::fmax(smax, TrialState->xstarsqp[idx_max] - ub[idx_max]);
  }
  MeritFunction->nlpPrimalFeasError = smax;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = std::fmax(1.0, smax);
  }
  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix <= WorkingSet->nVar - 1)) {
    dxTooSmall = ((!std::isinf(TrialState->gradLag[ix])) &&
                  (!std::isnan(TrialState->gradLag[ix])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs(TrialState->gradLag[ix]));
      ix++;
    }
  }
  Flags->gradOK = dxTooSmall;
  MeritFunction->nlpDualFeasError = smax;
  if (!dxTooSmall) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = stopping::computeComplError(
        TrialState->xstarsqp, TrialState->cIneq, WorkingSet->indexLB,
        WorkingSet->sizes[3], lb, WorkingSet->indexUB, WorkingSet->sizes[4], ub,
        TrialState->lambdasqp, WorkingSet->sizes[0] + 1);
    MeritFunction->firstOrderOpt =
        std::fmax(smax, MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      stopping::b_computeGradLag(
          memspace->workspace_double, WorkingSet->nVar, TrialState->grad,
          WorkingSet->Aineq, WorkingSet->indexFixed, WorkingSet->sizes[0],
          WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
          WorkingSet->sizes[4], TrialState->lambdasqp_old);
      s = 0.0;
      ix = 0;
      while ((ix <= WorkingSet->nVar - 1) &&
             ((!std::isinf(memspace->workspace_double[ix])) &&
              (!std::isnan(memspace->workspace_double[ix])))) {
        s = std::fmax(s, std::abs(memspace->workspace_double[ix]));
        ix++;
      }
      nlpComplErrorTmp = stopping::computeComplError(
          TrialState->xstarsqp, TrialState->cIneq, WorkingSet->indexLB,
          WorkingSet->sizes[3], lb, WorkingSet->indexUB, WorkingSet->sizes[4],
          ub, TrialState->lambdasqp_old, WorkingSet->sizes[0] + 1);
      d = std::fmax(s, nlpComplErrorTmp);
      if (d < std::fmax(smax, MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = s;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        if (0 <= mLambda) {
          std::copy(&TrialState->lambdasqp_old[0],
                    &TrialState->lambdasqp_old[mLambda + 1],
                    &TrialState->lambdasqp[0]);
        }
      } else if (0 <= mLambda) {
        std::copy(&TrialState->lambdasqp[0],
                  &TrialState->lambdasqp[mLambda + 1],
                  &TrialState->lambdasqp_old[0]);
      }
    } else if (0 <= mLambda) {
      std::copy(&TrialState->lambdasqp[0], &TrialState->lambdasqp[mLambda + 1],
                &TrialState->lambdasqp_old[0]);
    }
    if (isFeasible &&
        (MeritFunction->nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction->nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        bool guard1{false};
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          ix = 0;
          exitg1 = false;
          while ((!exitg1) && (ix <= nVar - 1)) {
            if (1.0E-6 * std::fmax(1.0, std::abs(TrialState->xstarsqp[ix])) <=
                std::abs(TrialState->delta_x[ix])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              ix++;
            }
          }
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              int nActiveConstr;
              nActiveConstr = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr > 0) {
                int rankR;
                internal::updateWorkingSetForNewQP(
                    TrialState->xstarsqp, WorkingSet, TrialState->cIneq,
                    WorkingSet->sizes[3], lb, WorkingSet->sizes[4], ub,
                    WorkingSet->sizes[0]);
                for (minDim = 0; minDim < nActiveConstr; minDim++) {
                  TrialState->lambda[minDim] = 0.0;
                  idx_max = 6 * minDim;
                  ix = 11 * minDim;
                  for (rankR = 0; rankR < nVar; rankR++) {
                    b_QRManager->QR[ix + rankR] =
                        WorkingSet->ATwset[idx_max + rankR];
                  }
                }
                b_QRManager->usedPivoting = true;
                b_QRManager->mrows = nVar;
                b_QRManager->ncols = nActiveConstr;
                if (nVar < nActiveConstr) {
                  minDim = nVar;
                } else {
                  minDim = nActiveConstr;
                }
                b_QRManager->minRowCol = minDim;
                ::coder::internal::lapack::xgeqp3(
                    b_QRManager->QR, nVar, nActiveConstr, b_QRManager->jpvt,
                    b_QRManager->tau);
                QRManager::computeQ_(b_QRManager, nVar);
                if (nVar > nActiveConstr) {
                  idx_max = nVar;
                } else {
                  idx_max = nActiveConstr;
                }
                smax = std::abs(b_QRManager->QR[0]) *
                       std::fmin(1.4901161193847656E-8,
                                 static_cast<double>(idx_max) *
                                     2.2204460492503131E-16);
                rankR = 0;
                idx_max = 0;
                while ((rankR < minDim) &&
                       (std::abs(b_QRManager->QR[idx_max]) > smax)) {
                  rankR++;
                  idx_max += 12;
                }
                ::coder::internal::blas::xgemv(nVar, nVar, b_QRManager->Q,
                                               TrialState->grad,
                                               memspace->workspace_double);
                if (rankR != 0) {
                  for (int j{rankR}; j >= 1; j--) {
                    idx_max = (j + (j - 1) * 11) - 1;
                    memspace->workspace_double[j - 1] /=
                        b_QRManager->QR[idx_max];
                    for (int i{0}; i <= j - 2; i++) {
                      ix = (j - i) - 2;
                      memspace->workspace_double[ix] -=
                          memspace->workspace_double[j - 1] *
                          b_QRManager->QR[(idx_max - i) - 1];
                    }
                  }
                }
                if (nActiveConstr < minDim) {
                  minDim = nActiveConstr;
                }
                for (ix = 0; ix < minDim; ix++) {
                  TrialState->lambda[b_QRManager->jpvt[ix] - 1] =
                      memspace->workspace_double[ix];
                }
                idx_max = mFixed + 1;
                for (ix = idx_max; ix <= mFixed; ix++) {
                  TrialState->lambda[ix - 1] = -TrialState->lambda[ix - 1];
                }
                qpactiveset::parseoutput::sortLambdaQP(
                    TrialState->lambda, WorkingSet->nActiveConstr,
                    WorkingSet->sizes, WorkingSet->isActiveIdx, WorkingSet->Wid,
                    WorkingSet->Wlocalidx, memspace->workspace_double);
                stopping::b_computeGradLag(
                    memspace->workspace_double, nVar, TrialState->grad,
                    WorkingSet->Aineq, WorkingSet->indexFixed, mFixed,
                    WorkingSet->indexLB, mLB, WorkingSet->indexUB, mUB,
                    TrialState->lambda);
                smax = 0.0;
                ix = 0;
                while ((ix <= nVar - 1) &&
                       ((!std::isinf(memspace->workspace_double[ix])) &&
                        (!std::isnan(memspace->workspace_double[ix])))) {
                  smax =
                      std::fmax(smax, std::abs(memspace->workspace_double[ix]));
                  ix++;
                }
                s = stopping::computeComplError(
                    TrialState->xstarsqp, TrialState->cIneq,
                    WorkingSet->indexLB, mLB, lb, WorkingSet->indexUB, mUB, ub,
                    TrialState->lambda, mFixed + 1);
                if ((smax <= 1.0E-6 * optimRelativeFactor) &&
                    (s <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = smax;
                  MeritFunction->nlpComplError = s;
                  MeritFunction->firstOrderOpt = std::fmax(smax, s);
                  if (0 <= mLambda) {
                    std::copy(&TrialState->lambda[0],
                              &TrialState->lambda[mLambda + 1],
                              &TrialState->lambdasqp[0]);
                  }
                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else if (TrialState->FunctionEvaluations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

//
// Arguments    : b_struct_T *MeritFunction
//                const k_struct_T *WorkingSet
//                j_struct_T *TrialState
//                const double lb[4]
//                const double ub[4]
//                bool *Flags_gradOK
//                bool *Flags_fevalOK
//                bool *Flags_done
//                bool *Flags_stepAccepted
//                bool *Flags_failedLineSearch
//                int *Flags_stepType
// Return Type  : void
//
void test_exit(b_struct_T *MeritFunction, const k_struct_T *WorkingSet,
               j_struct_T *TrialState, const double lb[4], const double ub[4],
               bool *Flags_gradOK, bool *Flags_fevalOK, bool *Flags_done,
               bool *Flags_stepAccepted, bool *Flags_failedLineSearch,
               int *Flags_stepType)
{
  double s;
  double smax;
  int idx_max;
  int mLB;
  int mLambda;
  int mUB;
  int nVar;
  bool exitg1;
  bool isFeasible;
  *Flags_fevalOK = true;
  *Flags_done = false;
  *Flags_stepAccepted = false;
  *Flags_failedLineSearch = false;
  *Flags_stepType = 1;
  nVar = WorkingSet->nVar;
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda =
      (WorkingSet->sizes[0] + WorkingSet->sizes[3]) + WorkingSet->sizes[4];
  stopping::computeGradLag(
      TrialState->gradLag, WorkingSet->nVar, TrialState->grad,
      WorkingSet->Aineq, WorkingSet->indexFixed, WorkingSet->sizes[0],
      WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
      WorkingSet->sizes[4], TrialState->lambdasqp);
  if (WorkingSet->nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet->nVar > 1) {
      smax = std::abs(TrialState->grad[0]);
      for (int k{2}; k <= nVar; k++) {
        s = std::abs(TrialState->grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  s = std::fmax(1.0, std::abs(TrialState->grad[idx_max - 1]));
  if (std::isinf(s)) {
    s = 1.0;
  }
  smax = std::fmax(0.0, TrialState->cIneq);
  for (idx_max = 0; idx_max < mLB; idx_max++) {
    nVar = WorkingSet->indexLB[idx_max] - 1;
    smax = std::fmax(smax, lb[nVar] - TrialState->xstarsqp[nVar]);
  }
  for (idx_max = 0; idx_max < mUB; idx_max++) {
    nVar = WorkingSet->indexUB[idx_max] - 1;
    smax = std::fmax(smax, TrialState->xstarsqp[nVar] - ub[nVar]);
  }
  MeritFunction->nlpPrimalFeasError = smax;
  MeritFunction->feasRelativeFactor = std::fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction->feasRelativeFactor);
  *Flags_gradOK = true;
  smax = 0.0;
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= WorkingSet->nVar - 1)) {
    *Flags_gradOK = ((!std::isinf(TrialState->gradLag[idx_max])) &&
                     (!std::isnan(TrialState->gradLag[idx_max])));
    if (!*Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs(TrialState->gradLag[idx_max]));
      idx_max++;
    }
  }
  MeritFunction->nlpDualFeasError = smax;
  if (!*Flags_gradOK) {
    *Flags_done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = smax;
    if (0 <= mLambda) {
      std::copy(&TrialState->lambdasqp[0], &TrialState->lambdasqp[mLambda + 1],
                &TrialState->lambdasqp_old[0]);
    }
    if (isFeasible && (smax <= 1.0E-6 * s)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = 1;
    } else if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
      *Flags_done = true;
      TrialState->sqpExitFlag = -3;
    }
  }
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for test_exit.cpp
//
// [EOF]
//
