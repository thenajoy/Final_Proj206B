//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_qcqp_api.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "_coder_qcqp_api.h"
#include "_coder_qcqp_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131610U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "qcqp",                                               // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *f,
                                   const char_T *identifier))[4];

static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *c,
                                 const char_T *identifier);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16];

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Q,
                                 const char_T *identifier))[16];

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[16];

static const mxArray *emlrt_marshallOut(const real_T u[4]);

static const mxArray *emlrt_marshallOut(const struct1_T *u);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct0_T u);

static const mxArray *emlrt_marshallOut(const real_T u);

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

// Function Definitions
//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[4]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *f
//                const char_T *identifier
// Return Type  : real_T (*)[4]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *f,
                                   const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(f), &thisId);
  emlrtDestroyArray(&f);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T
//
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *c
//                const char_T *identifier
// Return Type  : real_T
//
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *c,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(c), &thisId);
  emlrtDestroyArray(&c);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[16]
//
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16]
{
  static const int32_T dims[2]{4, 4};
  real_T(*ret)[16];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 2U, (void *)&dims[0]);
  ret = (real_T(*)[16])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[4]
//
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims{4};
  real_T(*ret)[4];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[16]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[16]
{
  real_T(*y)[16];
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *Q
//                const char_T *identifier
// Return Type  : real_T (*)[16]
//
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Q,
                                 const char_T *identifier))[16]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[16];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(Q), &thisId);
  emlrtDestroyArray(&Q);
  return y;
}

//
// Arguments    : const real_T u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const struct0_T u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const emlrtStack *sp, const struct0_T u)
{
  static const int32_T iv[2]{1, 3};
  static const char_T *sv[7]{"iterations",      "funcCount", "algorithm",
                             "constrviolation", "stepsize",  "lssteplength",
                             "firstorderopt"};
  const mxArray *b_y;
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 7, (const char_T **)&sv[0]));
  emlrtSetFieldR2017b(y, 0, (const char_T *)"iterations",
                      emlrt_marshallOut(u.iterations), 0);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"funcCount",
                      emlrt_marshallOut(u.funcCount), 1);
  b_y = nullptr;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtCTX)sp, 3, m, &u.algorithm[0]);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"algorithm", b_y, 2);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"constrviolation",
                      emlrt_marshallOut(u.constrviolation), 3);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"stepsize",
                      emlrt_marshallOut(u.stepsize), 4);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"lssteplength",
                      emlrt_marshallOut(u.lssteplength), 5);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"firstorderopt",
                      emlrt_marshallOut(u.firstorderopt), 6);
  return y;
}

//
// Arguments    : const struct1_T *u
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const struct1_T *u)
{
  static const int32_T i{0};
  static const int32_T i1{0};
  static const int32_T i2{0};
  static const int32_T i3{4};
  static const int32_T i4{4};
  static const char_T *sv[6]{"eqlin",      "eqnonlin", "ineqlin",
                             "ineqnonlin", "lower",    "upper"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"eqlin", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"eqnonlin", c_y, 1);
  d_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"ineqlin", d_y, 2);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"ineqnonlin",
                      emlrt_marshallOut(u->ineqnonlin), 3);
  e_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i3, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->lower[0];
  pData[1] = u->lower[1];
  pData[2] = u->lower[2];
  pData[3] = u->lower[3];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"lower", e_y, 4);
  f_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i4, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u->upper[0];
  pData[1] = u->upper[1];
  pData[2] = u->upper[2];
  pData[3] = u->upper[3];
  emlrtAssign(&f_y, m);
  emlrtSetFieldR2017b(y, 0, (const char_T *)"upper", f_y, 5);
  return y;
}

//
// Arguments    : const real_T u[4]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[4])
{
  static const int32_T i{0};
  static const int32_T i1{4};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack *sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T
//
static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const mxArray * const prhs[9]
//                int32_T nlhs
//                const mxArray *plhs[5]
// Return Type  : void
//
void qcqp_api(const mxArray *const prhs[9], int32_T nlhs,
              const mxArray *plhs[5])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T output;
  struct1_T lambda;
  real_T(*H)[16];
  real_T(*Q)[16];
  real_T(*f)[4];
  real_T(*k)[4];
  real_T(*lb)[4];
  real_T(*ub)[4];
  real_T(*x0)[4];
  real_T(*xOpt)[4];
  real_T c;
  real_T d;
  real_T eflag;
  real_T fval;
  st.tls = emlrtRootTLSGlobal;
  xOpt = (real_T(*)[4])mxMalloc(sizeof(real_T[4]));
  // Marshall function inputs
  Q = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Q");
  f = b_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "f");
  c = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "c");
  H = emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "H");
  k = b_emlrt_marshallIn(&st, emlrtAlias(prhs[4]), "k");
  d = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "d");
  lb = b_emlrt_marshallIn(&st, emlrtAlias(prhs[6]), "lb");
  ub = b_emlrt_marshallIn(&st, emlrtAlias(prhs[7]), "ub");
  x0 = b_emlrt_marshallIn(&st, emlrtAlias(prhs[8]), "x0");
  // Invoke the target function
  qcqp(*Q, *f, c, *H, *k, d, *lb, *ub, *x0, *xOpt, &fval, &eflag, &output,
       &lambda);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(*xOpt);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(fval);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(eflag);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(&st, output);
  }
  if (nlhs > 4) {
    plhs[4] = emlrt_marshallOut(&lambda);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void qcqp_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  qcqp_xil_terminate();
  qcqp_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void qcqp_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void qcqp_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_qcqp_api.cpp
//
// [EOF]
//
