//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: atan2.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Apr-2018 14:45:54
//

// Include Files
#include "rt_nonfinite.h"
#include "messure_update.h"
#include "messure_update_g.h"
#include "q2euler.h"
#include "time_update.h"
#include "atan2.h"

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : const double y_data[]
//                const int y_size[2]
//                const double x_data[]
//                const int x_size[2]
//                double r_data[]
//                int r_size[2]
// Return Type  : void
//
void b_atan2(const double y_data[], const int y_size[2], const double x_data[],
             const int x_size[2], double r_data[], int r_size[2])
{
  r_size[0] = 1;
  if (y_size[1] <= x_size[1]) {
    r_size[1] = (signed char)y_size[1];
  } else {
    r_size[1] = 0;
  }

  if (1 <= y_size[1]) {
    r_data[0] = rt_atan2d_snf(y_data[0], x_data[0]);
  }
}

//
// File trailer for atan2.cpp
//
// [EOF]
//
