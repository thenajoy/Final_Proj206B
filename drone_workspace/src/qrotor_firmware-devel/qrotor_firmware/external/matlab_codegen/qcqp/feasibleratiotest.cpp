//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleratiotest.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "feasibleratiotest.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double solution_xstar[6]
//                const double solution_searchDir[6]
//                double workspace[66]
//                int workingset_nVar
//                const double workingset_Aineq[6]
//                double workingset_bineq
//                const double workingset_lb[6]
//                const double workingset_ub[6]
//                const int workingset_indexLB[6]
//                const int workingset_indexUB[6]
//                const int workingset_sizes[5]
//                const int workingset_isActiveIdx[6]
//                const bool workingset_isActiveConstr[11]
//                const int workingset_nWConstr[5]
//                bool isPhaseOne
//                double *alpha
//                bool *newBlocking
//                int *constrType
//                int *constrIdx
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void feasibleratiotest(
    const double solution_xstar[6], const double solution_searchDir[6],
    double workspace[66], int workingset_nVar, const double workingset_Aineq[6],
    double workingset_bineq, const double workingset_lb[6],
    const double workingset_ub[6], const int workingset_indexLB[6],
    const int workingset_indexUB[6], const int workingset_sizes[5],
    const int workingset_isActiveIdx[6],
    const bool workingset_isActiveConstr[11], const int workingset_nWConstr[5],
    bool isPhaseOne, double *alpha, bool *newBlocking, int *constrType,
    int *constrIdx)
{
  double b_c;
  double c;
  double denomTol;
  double phaseOneCorrectionP;
  double ratio;
  int ia;
  int idx;
  int totalUB;
  totalUB = workingset_sizes[4];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir);
  if (workingset_nWConstr[2] < 1) {
    workspace[0] = workingset_bineq;
    workspace[0] = -workspace[0];
    b_c = 0.0;
    workspace[11] = 0.0;
    c = 0.0;
    for (ia = 1; ia <= workingset_nVar; ia++) {
      phaseOneCorrectionP = workingset_Aineq[ia - 1];
      b_c += phaseOneCorrectionP * solution_xstar[ia - 1];
      c += phaseOneCorrectionP * solution_searchDir[ia - 1];
    }
    workspace[0] += b_c;
    workspace[11] += c;
    if ((workspace[11] > denomTol) &&
        (!workingset_isActiveConstr[workingset_isActiveIdx[2] - 1])) {
      b_c = std::fmin(std::abs(workspace[0]), 1.0E-6 - workspace[0]) /
            workspace[11];
      if (b_c < 1.0E+30) {
        *alpha = b_c;
        *constrType = 3;
        *constrIdx = 1;
        *newBlocking = true;
      }
    }
  }
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    c = static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    ia = workingset_sizes[3];
    for (idx = 0; idx <= ia - 2; idx++) {
      int i;
      i = workingset_indexLB[idx];
      b_c = -solution_searchDir[i - 1] - phaseOneCorrectionP;
      if ((b_c > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio = (-solution_xstar[i - 1] - workingset_lb[i - 1]) - c;
        b_c = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / b_c;
        if (b_c < *alpha) {
          *alpha = b_c;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
    ia = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    phaseOneCorrectionP = -solution_searchDir[ia];
    if ((phaseOneCorrectionP > denomTol) &&
        (!workingset_isActiveConstr
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar[ia] - workingset_lb[ia];
      b_c = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / phaseOneCorrectionP;
      if (b_c < *alpha) {
        *alpha = b_c;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    c = static_cast<double>(isPhaseOne) * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir[workingset_nVar - 1];
    for (idx = 0; idx < totalUB; idx++) {
      ia = workingset_indexUB[idx];
      b_c = solution_searchDir[ia - 1] - phaseOneCorrectionP;
      if ((b_c > denomTol) &&
          (!workingset_isActiveConstr[(workingset_isActiveIdx[4] + idx) - 1])) {
        ratio = (solution_xstar[ia - 1] - workingset_ub[ia - 1]) - c;
        b_c = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / b_c;
        if (b_c < *alpha) {
          *alpha = b_c;
          *constrType = 5;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }
      }
    }
  }
  if (!isPhaseOne) {
    if ((*newBlocking) && (*alpha > 1.0)) {
      *newBlocking = false;
    }
    *alpha = std::fmin(*alpha, 1.0);
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for feasibleratiotest.cpp
//
// [EOF]
//
