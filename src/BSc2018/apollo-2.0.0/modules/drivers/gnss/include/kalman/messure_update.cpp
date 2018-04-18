//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: messure_update.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 12-Apr-2018 18:19:50
//

// Include Files
#include "rt_nonfinite.h"
#include "messure_update.h"

// Function Definitions

//
// Arguments    : double x[8]
//                double P[64]
//                const double pos[3]
//                const double Rp[9]
// Return Type  : void
//
void messure_update(double x[8], double P[64], const double pos[3], const double
                    Rp[9])
{
  int rtemp;
  int k;
  double A[9];
  double St[9];
  double a[24];
  int r1;
  int r2;
  double y[24];
  double maxval;
  int r3;
  static const signed char b_a[24] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double a21;
  static const signed char b[24] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0 };

  double Kt[24];
  double b_pos[3];
  double absxk;
  double t;
  double b_x[4];
  if ((pos[0] != 0.0) && (pos[1] != 0.0)) {
    for (rtemp = 0; rtemp < 3; rtemp++) {
      for (k = 0; k < 8; k++) {
        a[rtemp + 3 * k] = 0.0;
        for (r1 = 0; r1 < 8; r1++) {
          a[rtemp + 3 * k] += (double)b_a[rtemp + 3 * r1] * P[r1 + (k << 3)];
        }
      }

      for (k = 0; k < 3; k++) {
        maxval = 0.0;
        for (r1 = 0; r1 < 8; r1++) {
          maxval += a[rtemp + 3 * r1] * (double)b[r1 + (k << 3)];
        }

        St[rtemp + 3 * k] = maxval + Rp[rtemp + 3 * k];
      }
    }

    for (rtemp = 0; rtemp < 8; rtemp++) {
      for (k = 0; k < 3; k++) {
        y[rtemp + (k << 3)] = 0.0;
        for (r1 = 0; r1 < 8; r1++) {
          y[rtemp + (k << 3)] += P[rtemp + (r1 << 3)] * (double)b[r1 + (k << 3)];
        }
      }
    }

    memcpy(&A[0], &St[0], 9U * sizeof(double));
    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = std::abs(St[0]);
    a21 = std::abs(St[1]);
    if (a21 > maxval) {
      maxval = a21;
      r1 = 1;
      r2 = 0;
    }

    if (std::abs(St[2]) > maxval) {
      r1 = 2;
      r2 = 1;
      r3 = 0;
    }

    A[r2] = St[r2] / St[r1];
    A[r3] /= A[r1];
    A[3 + r2] -= A[r2] * A[3 + r1];
    A[3 + r3] -= A[r3] * A[3 + r1];
    A[6 + r2] -= A[r2] * A[6 + r1];
    A[6 + r3] -= A[r3] * A[6 + r1];
    if (std::abs(A[3 + r3]) > std::abs(A[3 + r2])) {
      rtemp = r2;
      r2 = r3;
      r3 = rtemp;
    }

    A[3 + r3] /= A[3 + r2];
    A[6 + r3] -= A[3 + r3] * A[6 + r2];
    for (k = 0; k < 8; k++) {
      Kt[k + (r1 << 3)] = y[k] / A[r1];
      Kt[k + (r2 << 3)] = y[8 + k] - Kt[k + (r1 << 3)] * A[3 + r1];
      Kt[k + (r3 << 3)] = y[16 + k] - Kt[k + (r1 << 3)] * A[6 + r1];
      Kt[k + (r2 << 3)] /= A[3 + r2];
      Kt[k + (r3 << 3)] -= Kt[k + (r2 << 3)] * A[6 + r2];
      Kt[k + (r3 << 3)] /= A[6 + r3];
      Kt[k + (r2 << 3)] -= Kt[k + (r3 << 3)] * A[3 + r3];
      Kt[k + (r1 << 3)] -= Kt[k + (r3 << 3)] * A[r3];
      Kt[k + (r1 << 3)] -= Kt[k + (r2 << 3)] * A[r2];
    }

    for (rtemp = 0; rtemp < 3; rtemp++) {
      b_pos[rtemp] = pos[rtemp] - x[rtemp];
    }

    for (rtemp = 0; rtemp < 8; rtemp++) {
      maxval = 0.0;
      for (k = 0; k < 3; k++) {
        maxval += Kt[rtemp + (k << 3)] * b_pos[k];
        a[rtemp + (k << 3)] = 0.0;
        for (r1 = 0; r1 < 3; r1++) {
          a[rtemp + (k << 3)] += Kt[rtemp + (r1 << 3)] * St[r1 + 3 * k];
        }
      }

      x[rtemp] += maxval;
      for (k = 0; k < 8; k++) {
        maxval = 0.0;
        for (r1 = 0; r1 < 3; r1++) {
          maxval += a[rtemp + (r1 << 3)] * Kt[k + (r1 << 3)];
        }

        P[rtemp + (k << 3)] -= maxval;
      }
    }

    //  MU_NORMALIZEQ  Normalize the quaternion

    maxval = 0.0;
    a21 = 2.2250738585072014E-308;
    for (k = 0; k < 4; k++) {
      absxk = std::abs(x[k + 4]);
      if (absxk > a21) {
        t = a21 / absxk;
        maxval = 1.0 + maxval * t * t;
        a21 = absxk;
      } else {
        t = absxk / a21;
        maxval += t * t;
      }
    }

    maxval = a21 * std::sqrt(maxval);
    for (rtemp = 0; rtemp < 4; rtemp++) {
      b_x[rtemp] = x[rtemp + 4] / maxval;
    }

    if (b_x[0] < 0.0) {
      for (rtemp = 0; rtemp < 4; rtemp++) {
        b_x[rtemp] = -b_x[rtemp];
      }
    }

    for (rtemp = 0; rtemp < 4; rtemp++) {
      x[rtemp + 4] = b_x[rtemp];
    }
  }
}

//
// File trailer for messure_update.cpp
//
// [EOF]
//
