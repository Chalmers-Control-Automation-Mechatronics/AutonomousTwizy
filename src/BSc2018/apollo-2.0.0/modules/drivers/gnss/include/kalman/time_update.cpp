//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: time_update.cpp
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
#include "Ft.h"
#include "mu_normalizeQ.h"
#include "norm.h"
#include "Qq.h"


#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>

// Function Definitions

//
// Time update step
// Arguments    : double x[8]
//                double P[64]
//                double T
//                const double gyr[3]
//                const double acc[3]
//                const double Ra[9]
//                const double Rw[9]
// Return Type  : void
//
void time_update(double x[8], double P[64], double T, const double gyr[3], const
                 double acc[3], const double Ra[9], const double Rw[9])
{

  // debug
/*ROS_WARN_STREAM("x");
for(int i = 0; i<8; i++){
  ROS_WARN_STREAM(i << ": " << x[i]);
}
ROS_WARN_STREAM("P");
for(int i = 0; i<64; i++){
  ROS_WARN_STREAM(i << ": " << P[i]);
}
ROS_WARN_STREAM("T : " << T);*/


  double Rnb[9];
  int i;
  int i2;
  double c;
  double Rbn[9];
  double y;
  double v;
  double b_x;
  static const signed char a[3] = { 0, 1, 0 };

  static const double b[3] = { 0.0, 0.0, 9.82 };

  double q[4];
  double b_y[3];
  double dv17[3];
  double b_Rnb[3];
  double c_x[8];
  double d_x[4];
  double Q[64];
  double dv18[64];
  double dv19[64];
  double dv20[64];
  int i3;

  //  calc acc
  //  Gravity vector in navigation-frame
  Qq(*(double (*)[4])&x[4], Rnb);

  //  Rotation matrix from b-frame to n-frame
  for (i = 0; i < 3; i++) {
    for (i2 = 0; i2 < 3; i2++) {
      Rbn[i2 + 3 * i] = Rnb[i + 3 * i2];
    }
  }

  //  Rotaion matrix from n-frame to b-frame
  //  Update pos
  c = T * T;

  //  Update v
  y = 0.0;
  for (i = 0; i < 3; i++) {
    b_x = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      b_x += Rbn[i + 3 * i2] * b[i2];
    }

    y += (double)a[i] * (acc[i] - b_x);
  }

  v = x[3] + y * T;

  //  Update quaternion orientation
  if (genpnorm(gyr) > 0.001) {
    y = T / 2.0;
    for (i = 0; i < 3; i++) {
      b_y[i] = y * gyr[i];
    }

    y = genpnorm(b_y);
    b_x = std::sin(genpnorm(b_y));
    q[0] = std::cos(genpnorm(b_y));
    for (i = 0; i < 3; i++) {
      q[i + 1] = b_y[i] / y * b_x;
    }

    mu_normalizeQ(q);
    y = 0.0;
    b_x = x[4];
    b_y[0] = x[6] * q[3] - x[7] * q[2];
    b_y[1] = x[7] * q[1] - x[5] * q[3];
    b_y[2] = x[5] * q[2] - x[6] * q[1];
    for (i = 0; i < 3; i++) {
      y += x[5 + i] * q[i + 1];
      d_x[i + 1] = (b_x * q[1 + i] + q[0] * x[i + 5]) + b_y[i];
    }

    d_x[0] = x[4] * q[0] - y;
    for (i = 0; i < 4; i++) {
      q[i] = d_x[i];
    }

    mu_normalizeQ(q);
  } else {
    for (i = 0; i < 4; i++) {
      q[i] = x[i + 4];
    }

    mu_normalizeQ(q);
  }

  //  Update the state vector
  dv17[0] = 0.0;
  dv17[1] = x[3];
  dv17[2] = 0.0;
  y = 0.0;
  for (i = 0; i < 3; i++) {
    b_Rnb[i] = 0.0;
    b_x = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      b_x += Rbn[i + 3 * i2] * b[i2];
      b_Rnb[i] += Rnb[i + 3 * i2] * dv17[i2];
    }

    b_y[i] = acc[i] - b_x;
    y += (double)a[i] * b_y[i];
  }

  dv17[0] = 0.0;
  dv17[1] = y;
  dv17[2] = 0.0;
  for (i = 0; i < 3; i++) {
    b_y[i] = 0.0;
    for (i2 = 0; i2 < 3; i2++) {
      b_y[i] += Rnb[i + 3 * i2] * dv17[i2];
    }

    c_x[i] = (x[i] + b_Rnb[i] * T) + b_y[i] * c / 2.0;
  }

  c_x[3] = v;
  for (i = 0; i < 4; i++) {
    c_x[i + 4] = q[i];
  }

  memcpy(&x[0], &c_x[0], sizeof(double) << 3);

  //  Update covariance
  memset(&Q[0], 0, sizeof(double) << 6);
  for (i = 0; i < 8; i++) {
    Q[i + (i << 3)] = 1.0;
  }

  Q[27] = Ra[4];
  Q[36] = Rw[0];
  for (i = 0; i < 3; i++) {
    for (i2 = 0; i2 < 3; i2++) {
      Q[i2 + (i << 3)] = Ra[i2 + 3 * i];
      Q[(i2 + ((5 + i) << 3)) + 5] = Rw[i2 + 3 * i];
    }
  }

  Ft(T, acc, gyr, q, v, dv18);
  Ft(T, acc, gyr, q, v, dv19);
  for (i = 0; i < 8; i++) {
    for (i2 = 0; i2 < 8; i2++) {
      dv20[i + (i2 << 3)] = 0.0;
      for (i3 = 0; i3 < 8; i3++) {
        dv20[i + (i2 << 3)] += dv18[i + (i3 << 3)] * P[i3 + (i2 << 3)];
      }
    }
  }

  for (i = 0; i < 8; i++) {
    for (i2 = 0; i2 < 8; i2++) {
      y = 0.0;
      for (i3 = 0; i3 < 8; i3++) {
        y += dv20[i + (i3 << 3)] * dv19[i2 + (i3 << 3)];
      }

      P[i + (i2 << 3)] = y + Q[i + (i2 << 3)];
    }
  }

  //  + G*Q*G';
}

//
// File trailer for time_update.cpp
//
// [EOF]
//
