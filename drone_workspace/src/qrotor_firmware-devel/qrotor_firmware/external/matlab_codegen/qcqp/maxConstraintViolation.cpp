//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: maxConstraintViolation.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "maxConstraintViolation.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : k_struct_T *obj
//                const double x[6]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(k_struct_T *obj, const double x[6])
{
  double v;
  int ia;
  int idxLB;
  int mFixed;
  int mLB;
  int mUB;
  mLB = obj->sizes[3];
  mUB = obj->sizes[4];
  mFixed = obj->sizes[0];
  switch (obj->probType) {
  case 2:
    obj->maxConstrWorkspace[0] = obj->bineq;
    obj->maxConstrWorkspace[0] = -obj->maxConstrWorkspace[0];
    obj->maxConstrWorkspace[0] +=
        ((obj->Aineq[0] * x[0] + obj->Aineq[1] * x[1]) + obj->Aineq[2] * x[2]) +
        obj->Aineq[3] * x[3];
    obj->maxConstrWorkspace[0] -= x[4];
    v = std::fmax(0.0, obj->maxConstrWorkspace[0]);
    break;
  default: {
    double c;
    obj->maxConstrWorkspace[0] = obj->bineq;
    obj->maxConstrWorkspace[0] = -obj->maxConstrWorkspace[0];
    c = 0.0;
    idxLB = obj->nVar;
    for (ia = 1; ia <= idxLB; ia++) {
      c += obj->Aineq[ia - 1] * x[ia - 1];
    }
    obj->maxConstrWorkspace[0] += c;
    v = std::fmax(0.0, obj->maxConstrWorkspace[0]);
  } break;
  }
  if (obj->sizes[3] > 0) {
    for (ia = 0; ia < mLB; ia++) {
      idxLB = obj->indexLB[ia] - 1;
      v = std::fmax(v, -x[idxLB] - obj->lb[idxLB]);
    }
  }
  if (obj->sizes[4] > 0) {
    for (ia = 0; ia < mUB; ia++) {
      idxLB = obj->indexUB[ia] - 1;
      v = std::fmax(v, x[idxLB] - obj->ub[idxLB]);
    }
  }
  if (obj->sizes[0] > 0) {
    for (ia = 0; ia < mFixed; ia++) {
      v = std::fmax(v, std::abs(x[obj->indexFixed[ia] - 1] -
                                obj->ub[obj->indexFixed[ia] - 1]));
    }
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for maxConstraintViolation.cpp
//
// [EOF]
//
