//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ll2utm_c.cpp
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
// LL2UTM Lat/Lon to UTM coordinates precise conversion.
//  [X,Y]=LL2UTM2(LAT,LON) or LL2UTM([LAT,LON]) converts coordinates
//  LAT,LON (in degrees) to UTM X and Y (in meters). Default datum is WGS84.
// Arguments    : double lat
//                double lon
//                double zone
//                double *x
//                double *y
// Return Type  : void
//
void ll2utm_c(double lat, double lon, double zone, double *x, double *y)
{
  double p1;
  double l1;
  double L0;
  int i;
  double L;
  int k;
  double z_re;
  double p[9];
  static const double c0[45] = { -0.01068115234375, -0.025634765625,
    0.03204345703125, -0.014241536458333334, 0.00240325927734375, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.01953125, -0.0439453125, 0.0439453125, -0.011393229166666666,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.046875, -0.09375, 0.05859375, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.25, -0.375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0 };

  double C[5];
  double y_re;
  double y_im;
  double b_y_re;
  double b_y_im;
  double c_y_re;
  double c_y_im;
  double d_y_re;

  // 	LL2UTM(...,ZONE) forces the UTM ZONE (scalar integer or same size as
  //    LAT and LON) instead of automatic set.
  //  constants
  //  conversion rad to deg
  //  UTM scale factor
  //  UTM false East (m)
  p1 = lat / 57.295779513082323;

  //  Phi = Latitude (rad)
  l1 = lon / 57.295779513082323;

  //  Lambda = Longitude (rad)
  //  UTM zone automatic setting
  //  if isempty(zone)
  //  	F0 = round((l1*D0 + 183)/6);
  //  else
  //  	F0 = zone;
  //  end
  L0 = (6.0 * zone - 183.0) / 57.295779513082323;

  //  UTM origin longitude (rad)
  //  UTM false northern (m)
  // COEF Projection coefficients
  // 	COEF(E,M) returns a vector of 5 coefficients from:
  // 		E = first ellipsoid excentricity
  // 		M = 0 for transverse mercator
  // 		M = 1 for transverse mercator reverse coefficients
  // 		M = 2 for merdian arc
  //  end
  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for (i = 0; i < 5; i++) {
    for (k = 0; k < 9; k++) {
      p[k] = c0[i + 5 * k];
    }

    *y = p[0];
    for (k = 0; k < 8; k++) {
      *y = 0.081819190842621486 * *y + p[k + 1];
    }

    C[i] = *y;
  }

  L = std::log(std::tan(0.78539816339744828 + p1 / 2.0) * rt_powd_snf((1.0 -
    0.081819190842621486 * std::sin(p1)) / (1.0 + 0.081819190842621486 * std::
    sin(p1)), 0.040909595421310743));
  z_re = std::atan(std::sinh(L) / std::cos(l1 - L0));
  L = std::log(std::tan(0.78539816339744828 + std::asin(std::sin(l1 - L0) / std::
    cosh(L)) / 2.0));
  l1 = 2.0 * z_re;
  L0 = 2.0 * L;
  if (L0 == 0.0) {
    l1 = std::sin(l1);
    L0 = 0.0;
  } else if (l1 == 0.0) {
    l1 = 0.0;
    L0 = std::sinh(L0);
  } else {
    y_re = l1;
    l1 = std::sin(l1) * std::cosh(L0);
    L0 = std::cos(y_re) * std::sinh(L0);
  }

  y_re = 4.0 * z_re;
  y_im = 4.0 * L;
  if (y_im == 0.0) {
    y_re = std::sin(y_re);
    y_im = 0.0;
  } else if (y_re == 0.0) {
    y_re = 0.0;
    y_im = std::sinh(y_im);
  } else {
    b_y_re = y_re;
    y_re = std::sin(y_re) * std::cosh(y_im);
    y_im = std::cos(b_y_re) * std::sinh(y_im);
  }

  b_y_re = 6.0 * z_re;
  b_y_im = 6.0 * L;
  if (b_y_im == 0.0) {
    b_y_re = std::sin(b_y_re);
    b_y_im = 0.0;
  } else if (b_y_re == 0.0) {
    b_y_re = 0.0;
    b_y_im = std::sinh(b_y_im);
  } else {
    c_y_re = b_y_re;
    b_y_re = std::sin(b_y_re) * std::cosh(b_y_im);
    b_y_im = std::cos(c_y_re) * std::sinh(b_y_im);
  }

  c_y_re = 8.0 * z_re;
  c_y_im = 8.0 * L;
  if (c_y_im == 0.0) {
    c_y_re = std::sin(c_y_re);
    c_y_im = 0.0;
  } else if (c_y_re == 0.0) {
    c_y_re = 0.0;
    c_y_im = std::sinh(c_y_im);
  } else {
    d_y_re = c_y_re;
    c_y_re = std::sin(c_y_re) * std::cosh(c_y_im);
    c_y_im = std::cos(d_y_re) * std::sinh(c_y_im);
  }

  z_re = 6.3649021661656657E+6 * z_re + 6.3755857452000007E+6 *
    (((0.00083632803213595363 * l1 + 7.5957782755955371E-7 * y_re) +
      1.1956312986940848E-9 * b_y_re) + 2.4107991079286062E-12 * c_y_re);
  L = 6.3649021661656657E+6 * L + 6.3755857452000007E+6 *
    (((0.00083632803213595363 * L0 + 7.5957782755955371E-7 * y_im) +
      1.1956312986940848E-9 * b_y_im) + 2.4107991079286062E-12 * c_y_im);

  //  outputs zone if needed: scalar value if unique, or vector/matrix of the
  //  same size as x/y in case of crossed zones
  //  if nargout > 2
  //     	f = F0.*sign(lat);
  //  	fu = unique(f);
  //  	if isscalar(fu)
  //  		f = fu;
  //  	end
  //  end
  //  if nargout < 2
  //  	x = [xs(:),ys(:)];
  //  else
  *x = L + 500000.0;
  *y = z_re + (1.0E+7 * (double)(p1 < 0.0) - 6.3755857452000007E+6 * ((((C[0] *
    0.0 + C[1] * 0.0) + C[2] * 0.0) + C[3] * 0.0) + C[4] * 0.0));
}

//
// File trailer for ll2utm_c.cpp
//
// [EOF]
//
