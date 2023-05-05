//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: BFGSUpdate.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool BFGSUpdate(int nvar, double Bk[16], const double sk[6], double yk[6],
                double workspace[66]);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for BFGSUpdate.h
//
// [EOF]
//
