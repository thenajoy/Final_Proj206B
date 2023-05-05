//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: anonymous_function.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef ANONYMOUS_FUNCTION_H
#define ANONYMOUS_FUNCTION_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct d_struct_T {
  double Q[16];
  double f[4];
  double c;
};

struct e_struct_T {
  double H[16];
  double k[4];
  double d;
};

namespace coder {
class anonymous_function {
public:
  d_struct_T workspace;
};

class b_anonymous_function {
public:
  e_struct_T workspace;
};

} // namespace coder

#endif
//
// File trailer for anonymous_function.h
//
// [EOF]
//
