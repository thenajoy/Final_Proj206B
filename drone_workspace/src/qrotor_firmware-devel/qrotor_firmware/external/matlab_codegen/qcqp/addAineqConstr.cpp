//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: addAineqConstr.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "addAineqConstr.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : k_struct_T *obj
//                int idx_local
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void addAineqConstr(k_struct_T *obj, int idx_local)
{
  int i;
  int iAineq0;
  int iAw0;
  obj->nWConstr[2]++;
  obj->isActiveConstr[(obj->isActiveIdx[2] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = 3;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  iAineq0 = 6 * (idx_local - 1);
  iAw0 = 6 * (obj->nActiveConstr - 1);
  i = obj->nVar - 1;
  for (int idx{0}; idx <= i; idx++) {
    obj->ATwset[iAw0 + idx] = obj->Aineq[iAineq0 + idx];
  }
  obj->bwset[obj->nActiveConstr - 1] = obj->bineq;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for addAineqConstr.cpp
//
// [EOF]
//
