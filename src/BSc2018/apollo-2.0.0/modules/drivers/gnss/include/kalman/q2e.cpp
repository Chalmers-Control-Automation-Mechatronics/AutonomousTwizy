//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: q2e.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 16-Apr-2018 14:15:47
//

// Include Files
#include "rt_nonfinite.h"
#include "q2e.h"

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
// Arguments    : const double q[4]
//                double eul[3]
// Return Type  : void
//
void q2e(const double q[4], double eul[3])
{
  int ak;
  double aSinInput;
  double y[4];
  int ck;
  for (ak = 0; ak < 4; ak++) {
    y[ak] = q[ak] * q[ak];
  }

  aSinInput = y[0];
  for (ak = 0; ak < 3; ak++) {
    aSinInput += y[ak + 1];
  }

  aSinInput = 1.0 / std::sqrt(aSinInput);
  ak = 0;
  for (ck = 0; ck < 4; ck++) {
    y[ck] = q[ak] * aSinInput;
    ak++;
  }

  aSinInput = -2.0 * (y[1] * y[3] - y[0] * y[2]);
  if (aSinInput > 1.0) {
    aSinInput = 1.0;
  }

  eul[0] = rt_atan2d_snf(2.0 * (y[1] * y[2] + y[0] * y[3]), ((y[0] * y[0] + y[1]
    * y[1]) - y[2] * y[2]) - y[3] * y[3]);
  eul[1] = std::asin(aSinInput);
  eul[2] = rt_atan2d_snf(2.0 * (y[2] * y[3] + y[0] * y[1]), ((y[0] * y[0] - y[1]
    * y[1]) - y[2] * y[2]) + y[3] * y[3]);
}

//
// File trailer for q2e.cpp
//
// [EOF]
//
