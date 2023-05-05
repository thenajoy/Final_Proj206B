//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: evalObjAndConstr.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "evalObjAndConstr.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const anonymous_function *obj_objfun
//                const b_anonymous_function *obj_nonlcon
//                const double x[4]
//                double *Cineq_workspace
//                double *fval
//                int *status
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
void evalObjAndConstr(const anonymous_function *obj_objfun,
                      const b_anonymous_function *obj_nonlcon,
                      const double x[4], double *Cineq_workspace, double *fval,
                      int *status)
{
  double b_obj_objfun;
  double d;
  double y_tmp;
  double y_tmp_idx_0;
  double y_tmp_idx_1;
  double y_tmp_idx_2;
  double y_tmp_idx_3;
  int i;
  int idx_current;
  //
  y_tmp_idx_0 = 0.5 * x[0];
  y_tmp_idx_1 = 0.5 * x[1];
  y_tmp_idx_2 = 0.5 * x[2];
  y_tmp_idx_3 = 0.5 * x[3];
  y_tmp = 0.0;
  b_obj_objfun = 0.0;
  for (i = 0; i < 4; i++) {
    idx_current = i << 2;
    d = x[i];
    y_tmp += (((y_tmp_idx_0 * obj_objfun->workspace.Q[idx_current] +
                y_tmp_idx_1 * obj_objfun->workspace.Q[idx_current + 1]) +
               y_tmp_idx_2 * obj_objfun->workspace.Q[idx_current + 2]) +
              y_tmp_idx_3 * obj_objfun->workspace.Q[idx_current + 3]) *
             d;
    b_obj_objfun += obj_objfun->workspace.f[i] * d;
  }
  *fval = (y_tmp + b_obj_objfun) + obj_objfun->workspace.c;
  *status = 1;
  if (std::isinf(*fval) || std::isnan(*fval)) {
    if (std::isnan(*fval)) {
      *status = -3;
    } else if (*fval < 0.0) {
      *status = -1;
    } else {
      *status = -2;
    }
  }
  if (*status == 1) {
    bool allFinite;
    y_tmp = 0.0;
    b_obj_objfun = 0.0;
    for (i = 0; i < 4; i++) {
      idx_current = i << 2;
      d = x[i];
      y_tmp += (((y_tmp_idx_0 * obj_nonlcon->workspace.H[idx_current] +
                  y_tmp_idx_1 * obj_nonlcon->workspace.H[idx_current + 1]) +
                 y_tmp_idx_2 * obj_nonlcon->workspace.H[idx_current + 2]) +
                y_tmp_idx_3 * obj_nonlcon->workspace.H[idx_current + 3]) *
               d;
      b_obj_objfun += obj_nonlcon->workspace.k[i] * d;
    }
    d = (y_tmp + b_obj_objfun) + obj_nonlcon->workspace.d;
    *Cineq_workspace = d;
    *status = 1;
    allFinite = true;
    idx_current = 1;
    while (allFinite && (idx_current <= 1)) {
      allFinite = ((!std::isinf(d)) && (!std::isnan(d)));
      idx_current = 2;
    }
    if (!allFinite) {
      if (std::isnan(d)) {
        *status = -3;
      } else if (d < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for evalObjAndConstr.cpp
//
// [EOF]
//
