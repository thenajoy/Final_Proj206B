//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qcqp.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef QCQP_H
#define QCQP_H

// Include Files
#include "qcqp_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void qcqp(const double Q[16], const double f[4], double c,
                 const double H[16], const double k[4], double d,
                 const double lb[4], const double ub[4], const double x0[4],
                 double xOpt[4], double *fval, double *eflag, struct0_T *output,
                 struct1_T *lambda);

#endif
//
// File trailer for qcqp.h
//
// [EOF]
//
