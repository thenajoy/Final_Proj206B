//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: updateWorkingSetForNewQP.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef UPDATEWORKINGSETFORNEWQP_H
#define UPDATEWORKINGSETFORNEWQP_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct k_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace internal {
void updateWorkingSetForNewQP(const double xk[4], k_struct_T *WorkingSet,
                              double cIneq, int mLB, const double lb[4],
                              int mUB, const double ub[4], int mFixed);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for updateWorkingSetForNewQP.h
//
// [EOF]
//
