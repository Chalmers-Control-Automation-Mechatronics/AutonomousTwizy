//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: P2EulerCov.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 16-Apr-2018 16:31:31
//

// Include Files
#include "rt_nonfinite.h"
#include "P2EulerCov.h"

// Function Definitions

//
// P2EulerCov Converts Full state covariance matrix to euler angle covariance
//    q = [q1 q2 q3 q4] euler = [psi theta phi]
// Arguments    : const double P[64]
//                const double q[4]
//                double eulerCov[9]
// Return Type  : void
//
void P2EulerCov(const double P[64], const double q[4], double eulerCov[9])
{
  double a;
  double b_a;
  double c_a;
  double d_a;
  double e_a;
  double f_a;
  double g_a;
  double h_a;
  double i_a;
  double j_a;
  double k_a;
  double l_a;
  double m_a;
  double n_a;
  double o_a;
  double p_a;
  double q_a;
  double r_a;
  double s_a;
  double t_a;
  double u_a;
  double v_a;
  double w_a;
  double x_a;
  double y_a;
  double ab_a;
  double bb_a;
  double cb_a;
  double db_a;
  double eb_a;
  double fb_a;
  double gb_a;
  double hb_a;
  double ib_a;
  double jb_a;
  double kb_a;
  double G[12];
  double b_G[12];
  int i0;
  int i1;
  int i2;
  a = q[2] + q[1];
  b_a = q[3] + q[0];
  c_a = q[2] - q[1];
  d_a = q[3] - q[0];
  e_a = q[2] + q[1];
  f_a = q[3] + q[0];
  g_a = q[2] - q[1];
  h_a = q[3] - q[0];
  i_a = q[2] + q[1];
  j_a = q[3] + q[0];
  k_a = q[2] - q[1];
  l_a = q[3] - q[0];
  m_a = q[2] + q[1];
  n_a = q[3] + q[0];
  o_a = q[2] - q[1];
  p_a = q[3] - q[0];
  q_a = q[1] * q[2] + q[0] * q[3];
  r_a = q[1] * q[2] + q[0] * q[3];
  s_a = q[1] * q[2] + q[0] * q[3];
  t_a = q[1] * q[2] + q[0] * q[3];
  u_a = q[2] + q[1];
  v_a = q[3] + q[0];
  w_a = q[2] - q[1];
  x_a = q[3] - q[0];
  y_a = q[2] + q[1];
  ab_a = q[3] + q[0];
  bb_a = q[2] - q[1];
  cb_a = q[3] - q[0];
  db_a = q[2] + q[1];
  eb_a = q[3] + q[0];
  fb_a = q[2] - q[1];
  gb_a = q[3] - q[0];
  hb_a = q[2] + q[1];
  ib_a = q[3] + q[0];
  jb_a = q[2] - q[1];
  kb_a = q[3] - q[0];
  G[0] = -(q[2] + q[1]) / (a * a + b_a * b_a) + (q[2] - q[1]) / (c_a * c_a + d_a
    * d_a);
  G[3] = (q[3] + q[0]) / (e_a * e_a + f_a * f_a) - (q[3] - q[0]) / (g_a * g_a +
    h_a * h_a);
  G[6] = (q[3] + q[0]) / (i_a * i_a + j_a * j_a) + (q[3] - q[0]) / (k_a * k_a +
    l_a * l_a);
  G[9] = -(q[2] + q[1]) / (m_a * m_a + n_a * n_a) - (q[2] - q[1]) / (o_a * o_a +
    p_a * p_a);
  G[1] = 2.0 * q[3] / std::sqrt(1.0 - 4.0 * (q_a * q_a));
  G[4] = 2.0 * q[2] / std::sqrt(1.0 - 4.0 * (r_a * r_a));
  G[7] = 2.0 * q[1] / std::sqrt(1.0 - 4.0 * (s_a * s_a));
  G[10] = 2.0 * q[0] / std::sqrt(1.0 - 4.0 * (t_a * t_a));
  G[2] = -(q[2] + q[1]) / (u_a * u_a + v_a * v_a) - (q[2] - q[1]) / (w_a * w_a +
    x_a * x_a);
  G[5] = (q[3] + q[0]) / (y_a * y_a + ab_a * ab_a) + (q[3] - q[0]) / (bb_a *
    bb_a + cb_a * cb_a);
  G[8] = (q[3] + q[0]) / (db_a * db_a + eb_a * eb_a) - (q[3] - q[0]) / (fb_a *
    fb_a + gb_a * gb_a);
  G[11] = -(q[2] + q[1]) / (hb_a * hb_a + ib_a * ib_a) + (q[2] - q[1]) / (jb_a *
    jb_a + kb_a * kb_a);
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      b_G[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        b_G[i0 + 3 * i1] += G[i0 + 3 * i2] * P[(i2 + ((4 + i1) << 3)) + 4];
      }
    }

    for (i1 = 0; i1 < 3; i1++) {
      eulerCov[i0 + 3 * i1] = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        eulerCov[i0 + 3 * i1] += b_G[i0 + 3 * i2] * G[i1 + 3 * i2];
      }
    }
  }
}

//
// File trailer for P2EulerCov.cpp
//
// [EOF]
//
