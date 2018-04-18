//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: q2euler.cpp
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
#include "power.h"

// Function Declarations
static double rt_remd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_remd_snf(double u0, double u1)
{
  double y;
  double b_u1;
  double q;
  if (!((!rtIsNaN(u0)) && (!rtIsInf(u0)) && ((!rtIsNaN(u1)) && (!rtIsInf(u1)))))
  {
    y = rtNaN;
  } else {
    if (u1 < 0.0) {
      b_u1 = std::ceil(u1);
    } else {
      b_u1 = std::floor(u1);
    }

    if ((u1 != 0.0) && (u1 != b_u1)) {
      q = std::abs(u0 / u1);
      if (std::abs(q - std::floor(q + 0.5)) <= DBL_EPSILON * q) {
        y = 0.0 * u0;
      } else {
        y = std::fmod(u0, u1);
      }
    } else {
      y = std::fmod(u0, u1);
    }
  }

  return y;
}

//
// Q2EULER  Convert quaternions to Euler angles
// Arguments    : const double q[4]
//                double euler[3]
// Return Type  : void
//
void q2euler(const double q[4], double euler[3])
{
  int i;
  double xzpwy;
  int trueCount;
  int q_size[2];
  int i0;
  int b_q_size[2];
  double q_data[1];
  double b_q_data[1];
  double b_data[1];
  int b_size[2];
  int c_q_size[2];
  int d_q_size[2];
  boolean_T IN;
  boolean_T IS;
  int b_trueCount;
  int e_q_size[2];
  int f_q_size[2];
  double tmp_data[1];
  int tmp_size[2];
  int b_tmp_size[2];
  int g_q_size[2];
  int h_q_size[2];
  int c_tmp_size[2];
  int d_tmp_size[2];
  for (i = 0; i < 3; i++) {
    euler[i] = 0.0;
  }

  xzpwy = q[1] * q[3] + q[0] * q[2];

  //  Handle the north pole
  trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (xzpwy + 1.4901161193847656E-8 > 0.5) {
      trueCount++;
    }
  }

  q_size[0] = 1;
  q_size[1] = trueCount;
  for (i0 = 0; i0 < trueCount; i0++) {
    q_data[i0] = q[1];
  }

  b_q_size[0] = 1;
  b_q_size[1] = trueCount;
  for (i0 = 0; i0 < trueCount; i0++) {
    b_q_data[i0] = q[0];
  }

  b_atan2(q_data, q_size, b_q_data, b_q_size, b_data, b_size);
  i = b_size[1];
  for (i0 = 0; i0 < i; i0++) {
    euler[0] = 2.0 * b_data[b_size[0] * i0];
  }

  //  Handle the south pole
  trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (xzpwy - 1.4901161193847656E-8 < -0.5) {
      trueCount++;
    }
  }

  c_q_size[0] = 1;
  c_q_size[1] = trueCount;
  for (i0 = 0; i0 < trueCount; i0++) {
    q_data[i0] = q[1];
  }

  d_q_size[0] = 1;
  d_q_size[1] = trueCount;
  for (i0 = 0; i0 < trueCount; i0++) {
    b_q_data[i0] = q[0];
  }

  b_atan2(q_data, c_q_size, b_q_data, d_q_size, b_data, b_size);
  i = b_size[1];
  for (i0 = 0; i0 < i; i0++) {
    euler[0] = -2.0 * b_data[b_size[0] * i0];
  }

  IN = (xzpwy + 1.4901161193847656E-8 > 0.5);
  IS = (xzpwy - 1.4901161193847656E-8 < -0.5);

  //  Handle the default case
  trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (!(IN || IS)) {
      trueCount++;
    }
  }

  b_trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (!(IN || IS)) {
      b_trueCount++;
    }
  }

  e_q_size[0] = 1;
  e_q_size[1] = b_trueCount;
  for (i0 = 0; i0 < b_trueCount; i0++) {
    q_data[i0] = q[2];
  }

  power(q_data, e_q_size, b_data, b_size);
  b_trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (!(IN || IS)) {
      b_trueCount++;
    }
  }

  f_q_size[0] = 1;
  f_q_size[1] = b_trueCount;
  for (i0 = 0; i0 < b_trueCount; i0++) {
    q_data[i0] = q[3];
  }

  power(q_data, f_q_size, tmp_data, q_size);
  tmp_size[0] = 1;
  tmp_size[1] = trueCount;
  for (i0 = 0; i0 < trueCount; i0++) {
    b_q_data[i0] = -2.0 * (q[0] * q[2] - q[0] * q[3]);
  }

  b_tmp_size[0] = 1;
  b_tmp_size[1] = b_size[1];
  i = b_size[0] * b_size[1];
  for (i0 = 0; i0 < i; i0++) {
    q_data[i0] = 1.0 - 2.0 * (b_data[i0] + tmp_data[i0]);
  }

  b_atan2(b_q_data, tmp_size, q_data, b_tmp_size, b_data, b_size);
  i = b_size[1];
  for (i0 = 0; i0 < i; i0++) {
    euler[0] = b_data[b_size[0] * i0];
  }

  trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (!(IN || IS)) {
      trueCount++;
    }
  }

  b_trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (!(IN || IS)) {
      b_trueCount++;
    }
  }

  g_q_size[0] = 1;
  g_q_size[1] = b_trueCount;
  for (i0 = 0; i0 < b_trueCount; i0++) {
    q_data[i0] = q[1];
  }

  power(q_data, g_q_size, b_data, b_size);
  b_trueCount = 0;
  for (i = 0; i < 1; i++) {
    if (!(IN || IS)) {
      b_trueCount++;
    }
  }

  h_q_size[0] = 1;
  h_q_size[1] = b_trueCount;
  for (i0 = 0; i0 < b_trueCount; i0++) {
    q_data[i0] = q[2];
  }

  power(q_data, h_q_size, tmp_data, q_size);
  c_tmp_size[0] = 1;
  c_tmp_size[1] = trueCount;
  for (i0 = 0; i0 < trueCount; i0++) {
    b_q_data[i0] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  }

  d_tmp_size[0] = 1;
  d_tmp_size[1] = b_size[1];
  i = b_size[0] * b_size[1];
  for (i0 = 0; i0 < i; i0++) {
    q_data[i0] = 1.0 - 2.0 * (b_data[i0] + tmp_data[i0]);
  }

  b_atan2(b_q_data, c_tmp_size, q_data, d_tmp_size, b_data, b_size);
  i = b_size[1];
  for (i0 = 0; i0 < i; i0++) {
    euler[2] = b_data[b_size[0] * i0];
  }

  euler[1] = std::asin(2.0 * xzpwy);
  euler[0] = rt_remd_snf(euler[0], 6.2831853071795862);
}

//
// File trailer for q2euler.cpp
//
// [EOF]
//
