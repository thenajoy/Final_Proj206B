//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: driver.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef DRIVER_H
#define DRIVER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct j_struct_T;

struct b_struct_T;

struct l_struct_T;

struct f_struct_T;

struct k_struct_T;

struct g_struct_T;

struct struct_T;

struct h_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void driver(const double lb[4], const double ub[4], j_struct_T *TrialState,
            b_struct_T *MeritFunction, const l_struct_T *FcnEvaluator,
            f_struct_T *memspace, k_struct_T *WorkingSet,
            g_struct_T *b_QRManager, struct_T *QPObjective, double Hessian[16],
            h_struct_T *b_CholManager);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for driver.h
//
// [EOF]
//
