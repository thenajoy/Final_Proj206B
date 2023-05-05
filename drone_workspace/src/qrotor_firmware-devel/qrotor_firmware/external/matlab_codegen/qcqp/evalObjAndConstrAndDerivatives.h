//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: evalObjAndConstrAndDerivatives.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef EVALOBJANDCONSTRANDDERIVATIVES_H
#define EVALOBJANDCONSTRANDDERIVATIVES_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

class b_anonymous_function;

} // namespace coder

// Function Declarations
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
                                    double *fval, int *status);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for evalObjAndConstrAndDerivatives.h
//
// [EOF]
//
