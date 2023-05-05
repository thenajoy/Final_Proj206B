//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: evalObjAndConstrAndDerivatives.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

// Include Files
#include "evalObjAndConstrAndDerivatives.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const anonymous_function *obj_objfun
//                const b_anonymous_function *obj_nonlcon
//                const double x[4]
//                double grad_workspace[6]
//                double *Cineq_workspace
//                double JacIneqTrans_workspace[6]
//                double *fval
//                int *status
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
void evalObjAndConstrAndDerivatives(const anonymous_function *obj_objfun,
                                    const b_anonymous_function *obj_nonlcon,
                                    const double x[4], double grad_workspace[6],
                                    double *Cineq_workspace,
                                    double JacIneqTrans_workspace[6],
                                    double *fval, int *status)
{
  double varargout_2[4];
  double b_obj_objfun;
  double d;
  double y_tmp;
  double y_tmp_idx_0;
  double y_tmp_idx_1;
  double y_tmp_idx_2;
  double y_tmp_idx_3;
  int col;
  int idx_current;
  bool allFinite;
  //
  y_tmp_idx_0 = 0.5 * x[0];
  y_tmp_idx_1 = 0.5 * x[1];
  y_tmp_idx_2 = 0.5 * x[2];
  y_tmp_idx_3 = 0.5 * x[3];
  y_tmp = 0.0;
  b_obj_objfun = 0.0;
  for (idx_current = 0; idx_current < 4; idx_current++) {
    d = x[idx_current];
    b_obj_objfun += obj_objfun->workspace.f[idx_current] * d;
    col = idx_current << 2;
    y_tmp += (((y_tmp_idx_0 * obj_objfun->workspace.Q[col] +
                y_tmp_idx_1 * obj_objfun->workspace.Q[col + 1]) +
               y_tmp_idx_2 * obj_objfun->workspace.Q[col + 2]) +
              y_tmp_idx_3 * obj_objfun->workspace.Q[col + 3]) *
             d;
    grad_workspace[idx_current] =
        (((obj_objfun->workspace.Q[idx_current] * x[0] +
           obj_objfun->workspace.Q[idx_current + 4] * x[1]) +
          obj_objfun->workspace.Q[idx_current + 8] * x[2]) +
         obj_objfun->workspace.Q[idx_current + 12] * x[3]) +
        obj_objfun->workspace.f[idx_current];
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
  } else {
    allFinite = true;
    idx_current = 0;
    while (allFinite && (idx_current + 1 <= 4)) {
      allFinite = ((!std::isinf(grad_workspace[idx_current])) &&
                   (!std::isnan(grad_workspace[idx_current])));
      idx_current++;
    }
    if (!allFinite) {
      idx_current--;
      if (std::isnan(grad_workspace[idx_current])) {
        *status = -3;
      } else if (grad_workspace[idx_current] < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    }
  }
  if (*status == 1) {
    y_tmp = 0.0;
    b_obj_objfun = 0.0;
    for (col = 0; col < 4; col++) {
      d = x[col];
      b_obj_objfun += obj_nonlcon->workspace.k[col] * d;
      idx_current = col << 2;
      y_tmp += (((y_tmp_idx_0 * obj_nonlcon->workspace.H[idx_current] +
                  y_tmp_idx_1 * obj_nonlcon->workspace.H[idx_current + 1]) +
                 y_tmp_idx_2 * obj_nonlcon->workspace.H[idx_current + 2]) +
                y_tmp_idx_3 * obj_nonlcon->workspace.H[idx_current + 3]) *
               d;
      varargout_2[col] = (((obj_nonlcon->workspace.H[col] * x[0] +
                            obj_nonlcon->workspace.H[col + 4] * x[1]) +
                           obj_nonlcon->workspace.H[col + 8] * x[2]) +
                          obj_nonlcon->workspace.H[col + 12] * x[3]) +
                         obj_nonlcon->workspace.k[col];
    }
    b_obj_objfun = (y_tmp + b_obj_objfun) + obj_nonlcon->workspace.d;
    JacIneqTrans_workspace[0] = varargout_2[0];
    JacIneqTrans_workspace[1] = varargout_2[1];
    JacIneqTrans_workspace[2] = varargout_2[2];
    JacIneqTrans_workspace[3] = varargout_2[3];
    *status = 1;
    allFinite = true;
    idx_current = 1;
    while (allFinite && (idx_current <= 1)) {
      allFinite = ((!std::isinf(b_obj_objfun)) && (!std::isnan(b_obj_objfun)));
      idx_current = 2;
    }
    if (!allFinite) {
      if (std::isnan(b_obj_objfun)) {
        *status = -3;
      } else if (b_obj_objfun < 0.0) {
        *status = -1;
      } else {
        *status = -2;
      }
    } else {
      allFinite = true;
      idx_current = 0;
      col = -1;
      while (allFinite && (col + 2 <= 1)) {
        idx_current = 0;
        while (allFinite && (idx_current + 1 <= 4)) {
          allFinite = ((!std::isinf(JacIneqTrans_workspace[idx_current])) &&
                       (!std::isnan(JacIneqTrans_workspace[idx_current])));
          idx_current++;
        }
        col = 0;
      }
      if (!allFinite) {
        idx_current = (idx_current + 6 * col) - 1;
        if (std::isnan(JacIneqTrans_workspace[idx_current])) {
          *status = -3;
        } else if (JacIneqTrans_workspace[idx_current] < 0.0) {
          *status = -1;
        } else {
          *status = -2;
        }
      }
    }
    *Cineq_workspace = b_obj_objfun;
  }
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for evalObjAndConstrAndDerivatives.cpp
//
// [EOF]
//
