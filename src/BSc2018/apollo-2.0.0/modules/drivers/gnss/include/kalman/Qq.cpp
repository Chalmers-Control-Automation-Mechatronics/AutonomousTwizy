//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Qq.cpp
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
#include "Qq.h"

// Function Definitions

//
// The matrix Q(q)
// Arguments    : const double q[4]
//                double Q[9]
// Return Type  : void
//
void Qq(const double q[4], double Q[9])
{
  Q[0] = 2.0 * (q[0] * q[0] + q[1] * q[1]) - 1.0;
  Q[3] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  Q[6] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  Q[1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  Q[4] = 2.0 * (q[0] * q[0] + q[2] * q[2]) - 1.0;
  Q[7] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  Q[2] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  Q[5] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  Q[8] = 2.0 * (q[0] * q[0] + q[3] * q[3]) - 1.0;
}

//
// File trailer for Qq.cpp
//
// [EOF]
//
