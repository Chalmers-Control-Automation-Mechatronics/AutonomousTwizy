//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: messure_update_g.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 12-Apr-2018 13:42:11
//

// Include Files
#include "rt_nonfinite.h"
#include "messure_update_g.h"

// Function Definitions

//
// Measurement update according to (13.19)
// Arguments    : double x[8]
//                const double P[64]
//                const double yacc[3]
//                const double Ra[9]
// Return Type  : void
//
void messure_update_g(double x[8], const double [64], const double yacc[3],
                      const double Ra[9])
{
  double S[9];
  double dv2[9];
  double dv3[9];
  double dv4[9];
  int r2;
  double b_yacc[3];
  double dv5[3];
  int rtemp;
  double dv6[3];
  double dv7[3];
  int r1;
  double maxval;
  int r3;
  double y[12];
  double H[12];
  static const double b[3] = { 0.0, 0.0, 9.82 };

  double K[12];
  double a21;
  static const double b_b[16] = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1 };

  double b_y;
  double scale;
  double b_x[4];

  //  assumption
  //  The matrix Q(q)
  //  The derivative of Q(q) wrt qi, i={0,1,2,3}
  S[0] = 2.0 * (2.0 * x[4]);
  S[1] = 2.0 * -x[7];
  S[2] = 2.0 * x[6];
  S[3] = 2.0 * x[7];
  S[4] = 2.0 * (2.0 * x[4]);
  S[5] = 2.0 * -x[5];
  S[6] = 2.0 * -x[6];
  S[7] = 2.0 * x[5];
  S[8] = 2.0 * (2.0 * x[4]);
  dv2[0] = 2.0 * (2.0 * x[5]);
  dv2[1] = 2.0 * x[6];
  dv2[2] = 2.0 * x[7];
  dv2[3] = 2.0 * x[6];
  dv2[4] = 0.0;
  dv2[5] = 2.0 * -x[4];
  dv2[6] = 2.0 * x[7];
  dv2[7] = 2.0 * x[4];
  dv2[8] = 0.0;
  dv3[0] = 0.0;
  dv3[1] = 2.0 * x[5];
  dv3[2] = 2.0 * x[4];
  dv3[3] = 2.0 * x[5];
  dv3[4] = 2.0 * (2.0 * x[6]);
  dv3[5] = 2.0 * x[7];
  dv3[6] = 2.0 * -x[4];
  dv3[7] = 2.0 * x[7];
  dv3[8] = 0.0;
  dv4[0] = 0.0;
  dv4[1] = 2.0 * -x[4];
  dv4[2] = 2.0 * x[5];
  dv4[3] = 2.0 * x[4];
  dv4[4] = 0.0;
  dv4[5] = 2.0 * x[6];
  dv4[6] = 2.0 * x[5];
  dv4[7] = 2.0 * x[6];
  dv4[8] = 2.0 * (2.0 * x[7]);

  //  Vilket tillstånd påverkas av mätvärdet.
  for (r2 = 0; r2 < 3; r2++) {
    b_yacc[r2] = 0.0;
    dv5[r2] = 0.0;
    dv6[r2] = 0.0;
    dv7[r2] = 0.0;
    for (rtemp = 0; rtemp < 3; rtemp++) {
      b_yacc[r2] += S[r2 + 3 * rtemp] * b[rtemp];
      dv5[r2] += dv2[r2 + 3 * rtemp] * b[rtemp];
      dv6[r2] += dv3[r2 + 3 * rtemp] * b[rtemp];
      dv7[r2] += dv4[r2 + 3 * rtemp] * b[rtemp];
    }

    H[r2] = b_yacc[r2];
    H[3 + r2] = dv5[r2];
    H[6 + r2] = dv6[r2];
    H[9 + r2] = dv7[r2];
    for (rtemp = 0; rtemp < 4; rtemp++) {
      K[r2 + 3 * rtemp] = 0.0;
      for (r1 = 0; r1 < 4; r1++) {
        K[r2 + 3 * rtemp] += H[r2 + 3 * r1] * b_b[r1 + (rtemp << 2)];
      }
    }
  }

  for (r2 = 0; r2 < 3; r2++) {
    for (rtemp = 0; rtemp < 3; rtemp++) {
      maxval = 0.0;
      for (r1 = 0; r1 < 4; r1++) {
        maxval += K[r2 + 3 * r1] * H[rtemp + 3 * r1];
      }

      S[r2 + 3 * rtemp] = maxval + Ra[r2 + 3 * rtemp];
    }
  }

  for (r2 = 0; r2 < 4; r2++) {
    for (rtemp = 0; rtemp < 3; rtemp++) {
      y[r2 + (rtemp << 2)] = 0.0;
      for (r1 = 0; r1 < 4; r1++) {
        y[r2 + (rtemp << 2)] += b_b[r2 + (r1 << 2)] * H[rtemp + 3 * r1];
      }
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(S[0]);
  a21 = std::abs(S[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(S[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  S[r2] /= S[r1];
  S[r3] /= S[r1];
  S[3 + r2] -= S[r2] * S[3 + r1];
  S[3 + r3] -= S[r3] * S[3 + r1];
  S[6 + r2] -= S[r2] * S[6 + r1];
  S[6 + r3] -= S[r3] * S[6 + r1];
  if (std::abs(S[3 + r3]) > std::abs(S[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  S[3 + r3] /= S[3 + r2];
  S[6 + r3] -= S[3 + r3] * S[6 + r2];
  for (rtemp = 0; rtemp < 4; rtemp++) {
    K[rtemp + (r1 << 2)] = y[rtemp] / S[r1];
    K[rtemp + (r2 << 2)] = y[4 + rtemp] - K[rtemp + (r1 << 2)] * S[3 + r1];
    K[rtemp + (r3 << 2)] = y[8 + rtemp] - K[rtemp + (r1 << 2)] * S[6 + r1];
    K[rtemp + (r2 << 2)] /= S[3 + r2];
    K[rtemp + (r3 << 2)] -= K[rtemp + (r2 << 2)] * S[6 + r2];
    K[rtemp + (r3 << 2)] /= S[6 + r3];
    K[rtemp + (r2 << 2)] -= K[rtemp + (r3 << 2)] * S[3 + r3];
    K[rtemp + (r1 << 2)] -= K[rtemp + (r3 << 2)] * S[r3];
    K[rtemp + (r1 << 2)] -= K[rtemp + (r2 << 2)] * S[r2];
  }

  //  K is kalman gain
  //  P_ = P_ - K*H*P_;  %
  // P(5:8,5:8) = P_;
  S[0] = 2.0 * (x[4] * x[4] + x[5] * x[5]) - 1.0;
  S[1] = 2.0 * (x[5] * x[6] - x[4] * x[7]);
  S[2] = 2.0 * (x[5] * x[7] + x[4] * x[6]);
  S[3] = 2.0 * (x[5] * x[6] + x[4] * x[7]);
  S[4] = 2.0 * (x[4] * x[4] + x[6] * x[6]) - 1.0;
  S[5] = 2.0 * (x[6] * x[7] - x[4] * x[5]);
  S[6] = 2.0 * (x[5] * x[7] - x[4] * x[6]);
  S[7] = 2.0 * (x[6] * x[7] + x[4] * x[5]);
  S[8] = 2.0 * (x[4] * x[4] + x[7] * x[7]) - 1.0;
  for (r2 = 0; r2 < 3; r2++) {
    maxval = 0.0;
    for (rtemp = 0; rtemp < 3; rtemp++) {
      maxval += S[r2 + 3 * rtemp] * b[rtemp];
    }

    b_yacc[r2] = yacc[r2] - maxval;
  }

  //  MU_NORMALIZEQ  Normalize the quaternion
  b_y = 0.0;
  scale = 2.2250738585072014E-308;
  for (rtemp = 0; rtemp < 4; rtemp++) {
    maxval = 0.0;
    for (r2 = 0; r2 < 3; r2++) {
      maxval += K[rtemp + (r2 << 2)] * b_yacc[r2];
    }

    x[4 + rtemp] += maxval;
    maxval = std::abs(x[rtemp + 4]);
    if (maxval > scale) {
      a21 = scale / maxval;
      b_y = 1.0 + b_y * a21 * a21;
      scale = maxval;
    } else {
      a21 = maxval / scale;
      b_y += a21 * a21;
    }
  }

  b_y = scale * std::sqrt(b_y);
  for (rtemp = 0; rtemp < 4; rtemp++) {
    b_x[rtemp] = x[rtemp + 4] / b_y;
  }

  if (b_x[0] < 0.0) {
    for (r2 = 0; r2 < 4; r2++) {
      b_x[r2] = -b_x[r2];
    }
  }

  for (rtemp = 0; rtemp < 4; rtemp++) {
    x[rtemp + 4] = b_x[rtemp];
  }
}

//
// File trailer for messure_update_g.cpp
//
// [EOF]
//
