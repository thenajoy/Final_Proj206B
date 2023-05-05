//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: setProblemType.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "setProblemType.h"
#include "modifyOverheadPhaseOne_.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : k_struct_T *obj
//                int PROBLEM_TYPE
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(k_struct_T *obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj->nVar = 4;
    obj->mConstr = obj->mConstrOrig;
    if (obj->nWConstr[4] > 0) {
      i = obj->sizesNormal[4];
      for (int colOffsetATw{0}; colOffsetATw < i; colOffsetATw++) {
        obj->isActiveConstr[(obj->isActiveIdxNormal[4] + colOffsetATw) - 1] =
            obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetATw) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
    }
  } break;
  case 1: {
    int i;
    obj->nVar = 5;
    obj->mConstr = obj->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
  } break;
  case 2: {
    int i;
    obj->nVar = 5;
    obj->mConstr = 10;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegularized[i];
    }
    if (obj->probType != 4) {
      int colOffsetATw;
      int i1;
      int idx_col;
      int idx_lb;
      i = obj->sizes[0];
      for (idx_col = 0; idx_col < i; idx_col++) {
        obj->ATwset[6 * idx_col + 4] = 0.0;
      }
      obj->Aineq[4] = -1.0;
      idx_lb = 4;
      i = obj->sizesNormal[3] + 1;
      i1 = obj->sizesRegularized[3];
      for (colOffsetATw = i; colOffsetATw <= i1; colOffsetATw++) {
        idx_lb++;
        obj->indexLB[colOffsetATw - 1] = idx_lb;
      }
      if (obj->nWConstr[4] > 0) {
        i = obj->sizesRegularized[4];
        for (colOffsetATw = 0; colOffsetATw < i; colOffsetATw++) {
          obj->isActiveConstr[obj->isActiveIdxRegularized[4] + colOffsetATw] =
              obj->isActiveConstr[(obj->isActiveIdx[4] + colOffsetATw) - 1];
        }
      }
      i = obj->isActiveIdx[4];
      i1 = obj->isActiveIdxRegularized[4] - 1;
      if (i <= i1) {
        std::memset(&obj->isActiveConstr[i + -1], 0,
                    ((i1 - i) + 1) * sizeof(bool));
      }
      obj->lb[4] = 0.0;
      idx_lb = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (idx_col = idx_lb; idx_col <= i; idx_col++) {
        colOffsetATw = 6 * (idx_col - 1) - 1;
        switch (obj->Wid[idx_col - 1]) {
        case 3:
          i1 = obj->Wlocalidx[idx_col - 1] + 3;
          if (5 <= i1) {
            std::memset(&obj->ATwset[colOffsetATw + 5], 0,
                        (((i1 + colOffsetATw) - colOffsetATw) + -4) *
                            sizeof(double));
          }
          obj->ATwset[(obj->Wlocalidx[idx_col - 1] + colOffsetATw) + 4] = -1.0;
          i1 = obj->Wlocalidx[idx_col - 1] + 5;
          if (i1 <= 5) {
            std::memset(&obj->ATwset[i1 + colOffsetATw], 0,
                        (((colOffsetATw - i1) - colOffsetATw) + 6) *
                            sizeof(double));
          }
          break;
        default:
          obj->ATwset[colOffsetATw + 5] = 0.0;
          break;
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
    }
  } break;
  default: {
    int i;
    obj->nVar = 6;
    obj->mConstr = 11;
    for (i = 0; i < 5; i++) {
      obj->sizes[i] = obj->sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
  } break;
  }
  obj->probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for setProblemType.cpp
//
// [EOF]
//
