//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: updateWorkingSetForNewQP.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "updateWorkingSetForNewQP.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const double xk[4]
//                k_struct_T *WorkingSet
//                double cIneq
//                int mLB
//                const double lb[4]
//                int mUB
//                const double ub[4]
//                int mFixed
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace internal {
void updateWorkingSetForNewQP(const double xk[4], k_struct_T *WorkingSet,
                              double cIneq, int mLB, const double lb[4],
                              int mUB, const double ub[4], int mFixed)
{
  int idx;
  int nVar;
  nVar = WorkingSet->nVar;
  WorkingSet->bineq = -cIneq;
  for (idx = 0; idx < mLB; idx++) {
    WorkingSet->lb[WorkingSet->indexLB[idx] - 1] =
        -lb[WorkingSet->indexLB[idx] - 1] + xk[WorkingSet->indexLB[idx] - 1];
  }
  for (idx = 0; idx < mUB; idx++) {
    WorkingSet->ub[WorkingSet->indexUB[idx] - 1] =
        ub[WorkingSet->indexUB[idx] - 1] - xk[WorkingSet->indexUB[idx] - 1];
  }
  for (idx = 0; idx < mFixed; idx++) {
    WorkingSet->ub[WorkingSet->indexFixed[idx] - 1] =
        ub[WorkingSet->indexFixed[idx] - 1] -
        xk[WorkingSet->indexFixed[idx] - 1];
    WorkingSet->bwset[idx] = ub[WorkingSet->indexFixed[idx] - 1] -
                             xk[WorkingSet->indexFixed[idx] - 1];
  }
  if (WorkingSet->nActiveConstr > mFixed) {
    int i;
    int ineqStart;
    if (1 < mFixed + 1) {
      ineqStart = mFixed + 1;
    } else {
      ineqStart = 1;
    }
    i = WorkingSet->nActiveConstr;
    for (idx = ineqStart; idx <= i; idx++) {
      switch (WorkingSet->Wid[idx - 1]) {
      case 4:
        WorkingSet->bwset[idx - 1] =
            WorkingSet
                ->lb[WorkingSet->indexLB[WorkingSet->Wlocalidx[idx - 1] - 1] -
                     1];
        break;
      case 5:
        WorkingSet->bwset[idx - 1] =
            WorkingSet
                ->ub[WorkingSet->indexUB[WorkingSet->Wlocalidx[idx - 1] - 1] -
                     1];
        break;
      default: {
        int iy0;
        WorkingSet->bwset[idx - 1] = WorkingSet->bineq;
        iy0 = 6 * (idx - 1);
        for (int k{0}; k < nVar; k++) {
          WorkingSet->ATwset[iy0 + k] = WorkingSet->Aineq[k];
        }
      } break;
      }
    }
  }
}

} // namespace internal
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for updateWorkingSetForNewQP.cpp
//
// [EOF]
//
