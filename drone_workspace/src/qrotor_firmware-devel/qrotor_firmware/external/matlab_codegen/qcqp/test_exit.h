//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: test_exit.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef TEST_EXIT_H
#define TEST_EXIT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct b_struct_T;

struct k_struct_T;

struct j_struct_T;

struct c_struct_T;

struct f_struct_T;

struct g_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(c_struct_T *Flags, f_struct_T *memspace,
                 b_struct_T *MeritFunction, k_struct_T *WorkingSet,
                 j_struct_T *TrialState, g_struct_T *b_QRManager,
                 const double lb[4], const double ub[4]);

void test_exit(b_struct_T *MeritFunction, const k_struct_T *WorkingSet,
               j_struct_T *TrialState, const double lb[4], const double ub[4],
               bool *Flags_gradOK, bool *Flags_fevalOK, bool *Flags_done,
               bool *Flags_stepAccepted, bool *Flags_failedLineSearch,
               int *Flags_stepType);

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for test_exit.h
//
// [EOF]
//
