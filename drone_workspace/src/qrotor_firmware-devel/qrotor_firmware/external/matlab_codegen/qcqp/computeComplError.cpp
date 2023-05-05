//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeComplError.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "computeComplError.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double xCurrent[4]
//                double cIneq
//                const int finiteLB[6]
//                int mLB
//                const double lb[4]
//                const int finiteUB[6]
//                int mUB
//                const double ub[4]
//                const double lambda[11]
//                int iL0
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
double computeComplError(const double xCurrent[4], double cIneq,
                         const int finiteLB[6], int mLB, const double lb[4],
                         const int finiteUB[6], int mUB, const double ub[4],
                         const double lambda[11], int iL0)
{
  double lbDelta;
  double lbLambda;
  double nlpComplError;
  int i;
  int idx;
  int ubOffset;
  lbDelta = lambda[iL0 - 1];
  nlpComplError =
      std::fmax(0.0, std::fmin(std::abs(cIneq * lbDelta),
                               std::fmin(std::abs(cIneq), lbDelta)));
  ubOffset = iL0 + mLB;
  for (idx = 0; idx < mLB; idx++) {
    i = finiteLB[idx];
    lbDelta = xCurrent[i - 1] - lb[i - 1];
    lbLambda = lambda[iL0 + idx];
    nlpComplError = std::fmax(
        nlpComplError, std::fmin(std::abs(lbDelta * lbLambda),
                                 std::fmin(std::abs(lbDelta), lbLambda)));
  }
  for (idx = 0; idx < mUB; idx++) {
    i = finiteUB[idx];
    lbDelta = ub[i - 1] - xCurrent[i - 1];
    lbLambda = lambda[ubOffset + idx];
    nlpComplError = std::fmax(
        nlpComplError, std::fmin(std::abs(lbDelta * lbLambda),
                                 std::fmin(std::abs(lbDelta), lbLambda)));
  }
  return nlpComplError;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeComplError.cpp
//
// [EOF]
//
