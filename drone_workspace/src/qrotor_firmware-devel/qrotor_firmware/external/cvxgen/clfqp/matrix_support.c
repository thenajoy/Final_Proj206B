/* Produced by CVXGEN, 2021-01-20 17:22:48 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.A[0])-rhs[1]*(-params.A[1])-rhs[2]*(-params.A[2])-rhs[3]*(-params.A[3]);
  lhs[1] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2])-rhs[3]*(params.A[3]);
  lhs[2] = -rhs[0]*(-1);
  lhs[3] = -rhs[1]*(-1);
  lhs[4] = -rhs[2]*(-1);
  lhs[5] = -rhs[3]*(-1);
  lhs[6] = -rhs[0]*(1);
  lhs[7] = -rhs[1]*(1);
  lhs[8] = -rhs[2]*(1);
  lhs[9] = -rhs[3]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.A[0])-rhs[1]*(params.A[0])-rhs[2]*(-1)-rhs[6]*(1);
  lhs[1] = -rhs[0]*(-params.A[1])-rhs[1]*(params.A[1])-rhs[3]*(-1)-rhs[7]*(1);
  lhs[2] = -rhs[0]*(-params.A[2])-rhs[1]*(params.A[2])-rhs[4]*(-1)-rhs[8]*(1);
  lhs[3] = -rhs[0]*(-params.A[3])-rhs[1]*(params.A[3])-rhs[5]*(-1)-rhs[9]*(1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.H[0])+rhs[1]*(2*params.H[4])+rhs[2]*(2*params.H[8])+rhs[3]*(2*params.H[12]);
  lhs[1] = rhs[0]*(2*params.H[1])+rhs[1]*(2*params.H[5])+rhs[2]*(2*params.H[9])+rhs[3]*(2*params.H[13]);
  lhs[2] = rhs[0]*(2*params.H[2])+rhs[1]*(2*params.H[6])+rhs[2]*(2*params.H[10])+rhs[3]*(2*params.H[14]);
  lhs[3] = rhs[0]*(2*params.H[3])+rhs[1]*(2*params.H[7])+rhs[2]*(2*params.H[11])+rhs[3]*(2*params.H[15]);
}
void fillq(void) {
  work.q[0] = params.g[0];
  work.q[1] = params.g[1];
  work.q[2] = params.g[2];
  work.q[3] = params.g[3];
}
void fillh(void) {
  work.h[0] = -params.lbA[0];
  work.h[1] = params.ubA[0];
  work.h[2] = -params.lb[0];
  work.h[3] = -params.lb[1];
  work.h[4] = -params.lb[2];
  work.h[5] = -params.lb[3];
  work.h[6] = params.ub[0];
  work.h[7] = params.ub[1];
  work.h[8] = params.ub[2];
  work.h[9] = params.ub[3];
}
void fillb(void) {
}
void pre_ops(void) {
}
