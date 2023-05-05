//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleX0ForWorkingSet.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "qcqp_internal_types.h"
#include "qcqp_rtwutil.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : double workspace[66]
//                double xCurrent[6]
//                k_struct_T *workingset
//                g_struct_T *qrmanager
// Return Type  : bool
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
bool feasibleX0ForWorkingSet(double workspace[66], double xCurrent[6],
                             k_struct_T *workingset, g_struct_T *qrmanager)
{
  double B[66];
  int mWConstr;
  int nVar_tmp_tmp;
  bool nonDegenerateWset;
  mWConstr = workingset->nActiveConstr;
  nVar_tmp_tmp = workingset->nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int br;
    int i;
    int i1;
    int iAcol;
    int idx;
    int jBcol;
    int k;
    int mLB;
    for (idx = 0; idx < mWConstr; idx++) {
      workspace[idx] = workingset->bwset[idx];
      workspace[idx + 11] = workingset->bwset[idx];
    }
    if (mWConstr != 0) {
      i = 6 * (mWConstr - 1) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 6) {
        c = 0.0;
        i1 = (iAcol + nVar_tmp_tmp) - 1;
        for (idx = iAcol; idx <= i1; idx++) {
          c += workingset->ATwset[idx - 1] * xCurrent[idx - iAcol];
        }
        i1 = div_nde_s32_floor(iAcol - 1, 6);
        workspace[i1] += -c;
      }
    }
    if (mWConstr >= nVar_tmp_tmp) {
      qrmanager->usedPivoting = false;
      qrmanager->mrows = mWConstr;
      qrmanager->ncols = nVar_tmp_tmp;
      for (mLB = 0; mLB < nVar_tmp_tmp; mLB++) {
        iAcol = 11 * mLB;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager->QR[jBcol + iAcol] = workingset->ATwset[mLB + 6 * jBcol];
        }
        qrmanager->jpvt[mLB] = mLB + 1;
      }
      if (mWConstr < nVar_tmp_tmp) {
        i = mWConstr;
      } else {
        i = nVar_tmp_tmp;
      }
      qrmanager->minRowCol = i;
      std::memset(&qrmanager->tau[0], 0, 11U * sizeof(double));
      if (i >= 1) {
        std::memset(&qrmanager->tau[0], 0, 11U * sizeof(double));
        internal::reflapack::qrf(qrmanager->QR, mWConstr, nVar_tmp_tmp, i,
                                 qrmanager->tau);
      }
      QRManager::computeQ_(qrmanager, mWConstr);
      std::copy(&workspace[0], &workspace[66], &B[0]);
      for (k = 0; k <= 11; k += 11) {
        i = k + 1;
        i1 = k + nVar_tmp_tmp;
        if (i <= i1) {
          std::memset(&workspace[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
        }
      }
      br = -1;
      for (k = 0; k <= 11; k += 11) {
        jBcol = -1;
        i = k + 1;
        i1 = k + nVar_tmp_tmp;
        for (int ic{i}; ic <= i1; ic++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager->Q[(iAcol + jBcol) + 1] * B[(iAcol + br) + 1];
          }
          workspace[ic - 1] += c;
          jBcol += 11;
        }
        br += 11;
      }
      for (idx = 0; idx < 2; idx++) {
        jBcol = 11 * idx - 1;
        for (k = nVar_tmp_tmp; k >= 1; k--) {
          iAcol = 11 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager->QR[k + iAcol];
            for (br = 0; br <= k - 2; br++) {
              i1 = (br + jBcol) + 1;
              workspace[i1] -= workspace[i] * qrmanager->QR[(br + iAcol) + 1];
            }
          }
        }
      }
    } else {
      QRManager::factorQR(qrmanager, workingset->ATwset, nVar_tmp_tmp,
                          mWConstr);
      QRManager::computeQ_(qrmanager, qrmanager->minRowCol);
      for (idx = 0; idx < 2; idx++) {
        jBcol = 11 * idx;
        for (br = 0; br < mWConstr; br++) {
          iAcol = 11 * br;
          mLB = br + jBcol;
          c = workspace[mLB];
          for (k = 0; k < br; k++) {
            c -= qrmanager->QR[k + iAcol] * workspace[k + jBcol];
          }
          workspace[mLB] = c / qrmanager->QR[br + iAcol];
        }
      }
      std::copy(&workspace[0], &workspace[66], &B[0]);
      for (k = 0; k <= 11; k += 11) {
        i = k + 1;
        i1 = k + nVar_tmp_tmp;
        if (i <= i1) {
          std::memset(&workspace[i + -1], 0, ((i1 - i) + 1) * sizeof(double));
        }
      }
      br = 0;
      for (k = 0; k <= 11; k += 11) {
        jBcol = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (idx = i; idx <= i1; idx++) {
          iAcol = k + 1;
          mLB = k + nVar_tmp_tmp;
          for (int ic{iAcol}; ic <= mLB; ic++) {
            workspace[ic - 1] += B[idx - 1] * qrmanager->Q[(jBcol + ic) - k];
          }
          jBcol += 11;
        }
        br += 11;
      }
    }
    idx = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar_tmp_tmp - 1) {
        if (std::isinf(workspace[idx]) || std::isnan(workspace[idx])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + 11];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        double constrViolation_minnormX;
        iAcol = nVar_tmp_tmp - 1;
        for (k = 0; k <= iAcol; k++) {
          workspace[k] += xCurrent[k];
        }
        mLB = workingset->sizes[3];
        jBcol = workingset->sizes[4];
        br = workingset->sizes[0];
        switch (workingset->probType) {
        case 2:
          workingset->maxConstrWorkspace[0] = workingset->bineq;
          workingset->maxConstrWorkspace[0] =
              -workingset->maxConstrWorkspace[0];
          workingset->maxConstrWorkspace[0] +=
              ((workingset->Aineq[0] * workspace[0] +
                workingset->Aineq[1] * workspace[1]) +
               workingset->Aineq[2] * workspace[2]) +
              workingset->Aineq[3] * workspace[3];
          workingset->maxConstrWorkspace[0] -= workspace[4];
          constrViolation_minnormX =
              std::fmax(0.0, workingset->maxConstrWorkspace[0]);
          break;
        default:
          workingset->maxConstrWorkspace[0] = workingset->bineq;
          workingset->maxConstrWorkspace[0] =
              -workingset->maxConstrWorkspace[0];
          c = 0.0;
          i = workingset->nVar;
          for (idx = 1; idx <= i; idx++) {
            c += workingset->Aineq[idx - 1] * workspace[idx - 1];
          }
          workingset->maxConstrWorkspace[0] += c;
          constrViolation_minnormX =
              std::fmax(0.0, workingset->maxConstrWorkspace[0]);
          break;
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            iAcol = workingset->indexLB[idx] - 1;
            constrViolation_minnormX =
                std::fmax(constrViolation_minnormX,
                          -workspace[iAcol] - workingset->lb[iAcol]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < jBcol; idx++) {
            iAcol = workingset->indexUB[idx] - 1;
            constrViolation_minnormX =
                std::fmax(constrViolation_minnormX,
                          workspace[iAcol] - workingset->ub[iAcol]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < br; idx++) {
            constrViolation_minnormX = std::fmax(
                constrViolation_minnormX,
                std::abs(workspace[workingset->indexFixed[idx] - 1] -
                         workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        mLB = workingset->sizes[3];
        jBcol = workingset->sizes[4];
        br = workingset->sizes[0];
        switch (workingset->probType) {
        case 2:
          workingset->maxConstrWorkspace[0] = workingset->bineq;
          workingset->maxConstrWorkspace[0] =
              -workingset->maxConstrWorkspace[0];
          workingset->maxConstrWorkspace[0] +=
              ((workingset->Aineq[0] * workspace[11] +
                workingset->Aineq[1] * workspace[12]) +
               workingset->Aineq[2] * workspace[13]) +
              workingset->Aineq[3] * workspace[14];
          workingset->maxConstrWorkspace[0] -= workspace[15];
          c = std::fmax(0.0, workingset->maxConstrWorkspace[0]);
          break;
        default:
          workingset->maxConstrWorkspace[0] = workingset->bineq;
          workingset->maxConstrWorkspace[0] =
              -workingset->maxConstrWorkspace[0];
          c = 0.0;
          i = workingset->nVar;
          for (idx = 1; idx <= i; idx++) {
            c += workingset->Aineq[idx - 1] * workspace[idx + 10];
          }
          workingset->maxConstrWorkspace[0] += c;
          c = std::fmax(0.0, workingset->maxConstrWorkspace[0]);
          break;
        }
        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < mLB; idx++) {
            iAcol = workingset->indexLB[idx];
            c = std::fmax(c,
                          -workspace[iAcol + 10] - workingset->lb[iAcol - 1]);
          }
        }
        if (workingset->sizes[4] > 0) {
          for (idx = 0; idx < jBcol; idx++) {
            iAcol = workingset->indexUB[idx];
            c = std::fmax(c, workspace[iAcol + 10] - workingset->ub[iAcol - 1]);
          }
        }
        if (workingset->sizes[0] > 0) {
          for (idx = 0; idx < br; idx++) {
            c = std::fmax(
                c, std::abs(workspace[workingset->indexFixed[idx] + 10] -
                            workingset->ub[workingset->indexFixed[idx] - 1]));
          }
        }
        if ((constrViolation_minnormX <= 2.2204460492503131E-16) ||
            (constrViolation_minnormX < c)) {
          if (0 <= nVar_tmp_tmp - 1) {
            std::copy(&workspace[0], &workspace[nVar_tmp_tmp], &xCurrent[0]);
          }
        } else if (0 <= nVar_tmp_tmp - 1) {
          std::copy(&workspace[11], &workspace[11 + nVar_tmp_tmp],
                    &xCurrent[0]);
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for feasibleX0ForWorkingSet.cpp
//
// [EOF]
//
