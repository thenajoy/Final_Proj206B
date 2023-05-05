//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "xgemv.h"
#include "qcqp_rtwutil.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : int m
//                int n
//                const double A[121]
//                const double x[6]
//                double y[66]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xgemv(int m, int n, const double A[121], const double x[6], double y[66])
{
  if (m != 0) {
    int i;
    if (0 <= n - 1) {
      std::memset(&y[0], 0, n * sizeof(double));
    }
    i = 11 * (n - 1) + 1;
    for (int iac{1}; iac <= i; iac += 11) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      i1 = div_nde_s32_floor(iac - 1, 11);
      y[i1] += c;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgemv.cpp
//
// [EOF]
//
