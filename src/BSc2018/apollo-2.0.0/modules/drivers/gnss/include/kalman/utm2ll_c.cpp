//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: utm2ll_c.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 12-Apr-2018 14:16:40
//

// Include Files
#include "rt_nonfinite.h"
#include "ll2utm_c.h"
#include "utm2ll_c.h"
#include "ll2utm_c_rtwutil.h"

// Function Definitions

//
// UTM2LL UTM to Lat/Lon coordinates precise conversion.
//  [LAT,LON]=UTM2LL(X,Y,ZONE) converts UTM coordinates X,Y (in meters)
// Arguments    : double x
//                double y
//                double zone
//                double *lat
//                double *lon
// Return Type  : void
//
void utm2ll_c(double x, double y, double zone, double *lat, double *lon)
{
  int i;
  double zt_re;
  double C[5];
  int n;
  double zt_im;
  double p;
  double b_p[9];
  static const double c0[45] = { -0.01068115234375, -0.025634765625,
    0.03204345703125, -0.014241536458333334, 0.00240325927734375, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.01953125, -0.0439453125, 0.0439453125, -0.011393229166666666,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.046875, -0.09375, 0.05859375, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.25, -0.375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0 };

  double L;
  double p0;
  double y_im;
  double y_re;
  double b_y_im;
  double b_y_re;
  double c_y_im;
  double c_y_re;

  //  constants
  //  conversion rad to deg
  //  maximum iteration for latitude computation
  //  minimum residue for latitude computation
  //  UTM scale factor
  //  UTM false East (m)
  //  UTM false North (m)
  //  UTM origin latitude (rad)
  //  UTM origin longitude (rad)
  //  ellpsoid excentricity
  //  computing parameters for Mercator Transverse projection
  // COEF Projection coefficients
  // 	COEF(E,M) returns a vector of 5 coefficients from:
  // 		E = first ellipsoid excentricity
  // 		M = 0 for transverse mercator
  // 		M = 1 for transverse mercator reverse coefficients
  // 		M = 2 for merdian arc
  // end
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for (i = 0; i < 5; i++) {
    for (n = 0; n < 9; n++) {
      b_p[n] = c0[i + 5 * n];
    }

    p = b_p[0];
    for (n = 0; n < 8; n++) {
      p = 0.081819190842621486 * p + b_p[n + 1];
    }

    C[i] = p;
  }

  zt_re = (y - (1.0E+7 * (double)(zone < 0.0) - 6.3755857452000007E+6 * ((((C[0]
    * 0.0 + C[1] * 0.0) + C[2] * 0.0) + C[3] * 0.0) + C[4] * 0.0))) /
    6.3755857452000007E+6 / 0.99832429843134363;
  zt_im = (x - 500000.0) / 6.3755857452000007E+6 / 0.99832429843134363;
  p = 2.0 * zt_re;
  L = 2.0 * zt_im;
  if (L == 0.0) {
    p = std::sin(p);
    L = 0.0;
  } else if (p == 0.0) {
    p = 0.0;
    L = std::sinh(L);
  } else {
    p0 = p;
    p = std::sin(p) * std::cosh(L);
    L = std::cos(p0) * std::sinh(L);
  }

  p0 = 4.0 * zt_re;
  y_im = 4.0 * zt_im;
  if (y_im == 0.0) {
    p0 = std::sin(p0);
    y_im = 0.0;
  } else if (p0 == 0.0) {
    p0 = 0.0;
    y_im = std::sinh(y_im);
  } else {
    y_re = p0;
    p0 = std::sin(p0) * std::cosh(y_im);
    y_im = std::cos(y_re) * std::sinh(y_im);
  }

  y_re = 6.0 * zt_re;
  b_y_im = 6.0 * zt_im;
  if (b_y_im == 0.0) {
    y_re = std::sin(y_re);
    b_y_im = 0.0;
  } else if (y_re == 0.0) {
    y_re = 0.0;
    b_y_im = std::sinh(b_y_im);
  } else {
    b_y_re = y_re;
    y_re = std::sin(y_re) * std::cosh(b_y_im);
    b_y_im = std::cos(b_y_re) * std::sinh(b_y_im);
  }

  b_y_re = 8.0 * zt_re;
  c_y_im = 8.0 * zt_im;
  if (c_y_im == 0.0) {
    b_y_re = std::sin(b_y_re);
    c_y_im = 0.0;
  } else if (b_y_re == 0.0) {
    b_y_re = 0.0;
    c_y_im = std::sinh(c_y_im);
  } else {
    c_y_re = b_y_re;
    b_y_re = std::sin(b_y_re) * std::cosh(c_y_im);
    c_y_im = std::cos(c_y_re) * std::sinh(c_y_im);
  }

  zt_re = (((zt_re - 0.0008377321642861037 * p) - 5.9058690849780855E-8 * p0) -
           1.6734091622846454E-10 * y_re) - 2.1388357130731985E-13 * b_y_re;
  zt_im = (((zt_im - 0.0008377321642861037 * L) - 5.9058690849780855E-8 * y_im)
           - 1.6734091622846454E-10 * b_y_im) - 2.1388357130731985E-13 * c_y_im;
  L = std::log(std::tan(0.78539816339744828 + std::asin(std::sin(zt_re) / std::
    cosh(zt_im)) / 2.0));

  //  calculates latitude from the isometric latitude
  p = 2.0 * std::atan(std::exp(L)) - 1.5707963267948966;
  p0 = rtNaN;
  n = 0;
  while ((rtIsNaN(p0) || (std::abs(p - p0) > 1.0E-11)) && (n < 100)) {
    p0 = p;
    p = 0.081819190842621486 * std::sin(p);
    p = 2.0 * std::atan(rt_powd_snf((1.0 + p) / (1.0 - p), 0.040909595421310743)
                        * std::exp(L)) - 1.5707963267948966;
    n++;
  }

  //  if nargout < 2
  //  	lat = D0*[p(:),l(:)];
  //  else
  *lat = p * 57.295779513082323;
  *lon = ((6.0 * std::abs(zone) - 183.0) / 57.295779513082323 + std::atan(std::
           sinh(zt_im) / std::cos(zt_re))) * 57.295779513082323;
}

//
// File trailer for utm2ll_c.cpp
//
// [EOF]
//
