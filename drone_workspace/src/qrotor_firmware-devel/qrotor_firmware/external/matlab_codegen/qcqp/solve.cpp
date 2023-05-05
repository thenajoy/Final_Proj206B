//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: solve.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "solve.h"
#include "qcqp_internal_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const h_struct_T *obj
//                double rhs[6]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace CholManager {
void solve(const h_struct_T *obj, double rhs[6])
{
  int n_tmp;
  n_tmp = obj->ndims;
  if (obj->ndims != 0) {
    int i;
    int j;
    int jA;
    for (j = 0; j < n_tmp; j++) {
      double temp;
      jA = j * 11;
      temp = rhs[j];
      for (i = 0; i < j; i++) {
        temp -= obj->FMat[jA + i] * rhs[i];
      }
      rhs[j] = temp / obj->FMat[jA + j];
    }
    for (j = n_tmp; j >= 1; j--) {
      jA = (j + (j - 1) * 11) - 1;
      rhs[j - 1] /= obj->FMat[jA];
      for (i = 0; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs[ix] -= rhs[j - 1] * obj->FMat[(jA - i) - 1];
      }
    }
  }
}

} // namespace CholManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for solve.cpp
//
// [EOF]
//
