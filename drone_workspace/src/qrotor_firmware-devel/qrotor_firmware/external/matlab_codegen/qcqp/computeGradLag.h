//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeGradLag.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void b_computeGradLag(double workspace[66], int nVar, const double grad[6],
                      const double AineqTrans[6], const int finiteFixed[6],
                      int mFixed, const int finiteLB[6], int mLB,
                      const int finiteUB[6], int mUB, const double lambda[11]);

void computeGradLag(double workspace[6], int nVar, const double grad[6],
                    const double AineqTrans[6], const int finiteFixed[6],
                    int mFixed, const int finiteLB[6], int mLB,
                    const int finiteUB[6], int mUB, const double lambda[11]);

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computeGradLag.h
//
// [EOF]
//
