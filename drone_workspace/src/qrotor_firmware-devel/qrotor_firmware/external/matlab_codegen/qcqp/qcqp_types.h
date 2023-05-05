//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qcqp_types.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef QCQP_TYPES_H
#define QCQP_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct struct0_T {
  double iterations;
  double funcCount;
  char algorithm[3];
  double constrviolation;
  double stepsize;
  double lssteplength;
  double firstorderopt;
};

struct struct1_T {
  double ineqnonlin;
  double lower[4];
  double upper[4];
};

#endif
//
// File trailer for qcqp_types.h
//
// [EOF]
//
