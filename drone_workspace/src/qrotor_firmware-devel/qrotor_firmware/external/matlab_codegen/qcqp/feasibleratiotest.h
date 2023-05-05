//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleratiotest.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
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
    int *constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for feasibleratiotest.h
//
// [EOF]
//
