//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFval_ReuseHx.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "computeFval_ReuseHx.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <string.h>

// Function Definitions
//
// Arguments    : const struct_T *obj
//                double workspace[66]
//                const double f[6]
//                const double x[6]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const struct_T *obj, double workspace[66],
                           const double f[6], const double x[6])
{
  double val;
  switch (obj->objtype) {
  case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;
  case 3: {
    if (obj->hasLinear) {
      int ixlast;
      int k;
      ixlast = obj->nvar;
      for (k = 0; k < ixlast; k++) {
        workspace[k] = 0.5 * obj->Hx[k] + f[k];
      }
      val = 0.0;
      if (obj->nvar >= 1) {
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj->nvar >= 1) {
        int ixlast;
        ixlast = obj->nvar;
        for (int k{0}; k < ixlast; k++) {
          val += x[k] * obj->Hx[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    if (obj->hasLinear) {
      int ixlast;
      int k;
      ixlast = obj->nvar;
      if (0 <= ixlast - 1) {
        std::copy(&f[0], &f[ixlast], &workspace[0]);
      }
      ixlast = 4 - obj->nvar;
      for (k = 0; k <= ixlast; k++) {
        workspace[obj->nvar + k] = obj->rho;
      }
      val = 0.0;
      for (k = 0; k < 5; k++) {
        workspace[k] += 0.5 * obj->Hx[k];
        val += x[k] * workspace[k];
      }
    } else {
      int ixlast;
      int k;
      val = 0.0;
      for (k = 0; k < 5; k++) {
        val += x[k] * obj->Hx[k];
      }
      val *= 0.5;
      ixlast = obj->nvar + 1;
      for (k = ixlast; k < 6; k++) {
        val += x[k - 1] * obj->rho;
      }
    }
  } break;
  }
  return val;
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFval_ReuseHx.cpp
//
// [EOF]
//
