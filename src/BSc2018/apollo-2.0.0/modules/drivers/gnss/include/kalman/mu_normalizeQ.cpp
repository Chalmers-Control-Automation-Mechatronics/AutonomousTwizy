//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mu_normalizeQ.cpp
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
#include "mu_normalizeQ.h"

// Function Definitions

//
// MU_NORMALIZEQ  Normalize the quaternion
// Arguments    : double x[4]
// Return Type  : void
//
void mu_normalizeQ(double x[4])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 4; k++) {
    absxk = std::abs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  y = scale * std::sqrt(y);
  for (k = 0; k < 4; k++) {
    x[k] /= y;
  }

  if (x[0] < 0.0) {
    for (k = 0; k < 4; k++) {
      x[k] = -x[k];
    }
  }
}

//
// File trailer for mu_normalizeQ.cpp
//
// [EOF]
//
