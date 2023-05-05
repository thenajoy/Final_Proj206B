//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemm.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef XGEMM_H
#define XGEMM_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgemm(int m, int n, int k, const double A[16], int lda,
           const double B[121], int ib0, double C[66]);

void xgemm(int m, int n, int k, const double A[121], int ia0,
           const double B[66], double C[121]);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xgemm.h
//
// [EOF]
//
