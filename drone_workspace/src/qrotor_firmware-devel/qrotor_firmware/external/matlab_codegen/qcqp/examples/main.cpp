//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Apr-2021 14:35:35
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "qcqp.h"
#include "qcqp_terminate.h"
#include "qcqp_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Declarations
static void argInit_4x1_real_T(double result[4]);

static void argInit_4x4_real_T(double result[16]);

static double argInit_real_T();

static void main_qcqp();

// Function Definitions
//
// Arguments    : double result[4]
// Return Type  : void
//
static void argInit_4x1_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[16]
// Return Type  : void
//
static void argInit_4x4_real_T(double result[16])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 4; idx0++) {
    for (int idx1{0}; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (idx1 << 2)] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_qcqp()
{
  struct0_T output;
  struct1_T lambda;
  double Q_tmp[16];
  double f_tmp[4];
  double xOpt[4];
  double c_tmp;
  double eflag;
  double fval;
  // Initialize function 'qcqp' input arguments.
  // Initialize function input argument 'Q'.
  argInit_4x4_real_T(Q_tmp);
  // Initialize function input argument 'f'.
  argInit_4x1_real_T(f_tmp);
  c_tmp = argInit_real_T();
  // Initialize function input argument 'H'.
  // Initialize function input argument 'k'.
  // Initialize function input argument 'lb'.
  // Initialize function input argument 'ub'.
  // Initialize function input argument 'x0'.
  // Call the entry-point 'qcqp'.
  qcqp(Q_tmp, f_tmp, c_tmp, Q_tmp, f_tmp, c_tmp, f_tmp, f_tmp, f_tmp, xOpt,
       &fval, &eflag, &output, &lambda);
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_qcqp();
  // Terminate the application.
  // You do not need to do this more than one time.
  qcqp_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
