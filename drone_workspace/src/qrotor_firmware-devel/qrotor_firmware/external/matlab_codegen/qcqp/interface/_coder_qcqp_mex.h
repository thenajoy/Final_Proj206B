//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_qcqp_mex.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef _CODER_QCQP_MEX_H
#define _CODER_QCQP_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_qcqp_mexFunction(int32_T nlhs, mxArray *plhs[5], int32_T nrhs,
                             const mxArray *prhs[9]);

#endif
//
// File trailer for _coder_qcqp_mex.h
//
// [EOF]
//
