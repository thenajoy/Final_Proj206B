//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_qcqp_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef _CODER_QCQP_API_H
#define _CODER_QCQP_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
typedef struct {
  real_T iterations;
  real_T funcCount;
  char_T algorithm[3];
  real_T constrviolation;
  real_T stepsize;
  real_T lssteplength;
  real_T firstorderopt;
} struct0_T;

typedef struct {
  real_T ineqnonlin;
  real_T lower[4];
  real_T upper[4];
} struct1_T;

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void qcqp(real_T Q[16], real_T f[4], real_T c, real_T H[16], real_T k[4],
          real_T d, real_T lb[4], real_T ub[4], real_T x0[4], real_T xOpt[4],
          real_T *fval, real_T *eflag, struct0_T *output, struct1_T *lambda);

void qcqp_api(const mxArray *const prhs[9], int32_T nlhs,
              const mxArray *plhs[5]);

void qcqp_atexit();

void qcqp_initialize();

void qcqp_terminate();

void qcqp_xil_shutdown();

void qcqp_xil_terminate();

#endif
//
// File trailer for _coder_qcqp_api.h
//
// [EOF]
//
