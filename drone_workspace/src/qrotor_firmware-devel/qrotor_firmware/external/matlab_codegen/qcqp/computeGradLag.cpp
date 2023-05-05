//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeGradLag.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "computeGradLag.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <string.h>

// Function Definitions
//
// Arguments    : double workspace[66]
//                int nVar
//                const double grad[6]
//                const double AineqTrans[6]
//                const int finiteFixed[6]
//                int mFixed
//                const int finiteLB[6]
//                int mLB
//                const int finiteUB[6]
//                int mUB
//                const double lambda[11]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void b_computeGradLag(double workspace[66], int nVar, const double grad[6],
                      const double AineqTrans[6], const int finiteFixed[6],
                      int mFixed, const int finiteLB[6], int mLB,
                      const int finiteUB[6], int mUB, const double lambda[11])
{
  int iL0;
  int ia;
  int idx;
  if (0 <= nVar - 1) {
    std::copy(&grad[0], &grad[nVar], &workspace[0]);
  }
  for (idx = 0; idx < mFixed; idx++) {
    ia = finiteFixed[idx];
    workspace[ia - 1] += lambda[idx];
  }
  for (ia = 1; ia <= nVar; ia++) {
    workspace[ia - 1] += AineqTrans[ia - 1] * lambda[mFixed];
  }
  for (idx = 0; idx < mLB; idx++) {
    ia = finiteLB[idx];
    workspace[ia - 1] -= lambda[(mFixed + idx) + 1];
  }
  iL0 = (mFixed + mLB) + 1;
  for (idx = 0; idx < mUB; idx++) {
    ia = finiteUB[idx];
    workspace[ia - 1] += lambda[iL0 + idx];
  }
}

//
// Arguments    : double workspace[6]
//                int nVar
//                const double grad[6]
//                const double AineqTrans[6]
//                const int finiteFixed[6]
//                int mFixed
//                const int finiteLB[6]
//                int mLB
//                const int finiteUB[6]
//                int mUB
//                const double lambda[11]
// Return Type  : void
//
void computeGradLag(double workspace[6], int nVar, const double grad[6],
                    const double AineqTrans[6], const int finiteFixed[6],
                    int mFixed, const int finiteLB[6], int mLB,
                    const int finiteUB[6], int mUB, const double lambda[11])
{
  int iL0;
  int ia;
  int idx;
  if (0 <= nVar - 1) {
    std::copy(&grad[0], &grad[nVar], &workspace[0]);
  }
  for (idx = 0; idx < mFixed; idx++) {
    ia = finiteFixed[idx];
    workspace[ia - 1] += lambda[idx];
  }
  for (ia = 1; ia <= nVar; ia++) {
    workspace[ia - 1] += AineqTrans[ia - 1] * lambda[mFixed];
  }
  for (idx = 0; idx < mLB; idx++) {
    ia = finiteLB[idx];
    workspace[ia - 1] -= lambda[(mFixed + idx) + 1];
  }
  iL0 = (mFixed + mLB) + 1;
  for (idx = 0; idx < mUB; idx++) {
    ia = finiteUB[idx];
    workspace[ia - 1] += lambda[iL0 + idx];
  }
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeGradLag.cpp
//
// [EOF]
//
