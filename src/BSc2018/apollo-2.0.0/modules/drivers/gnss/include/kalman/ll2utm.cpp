//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ll2utm.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Apr-2018 16:31:04
//

// Include Files
#include "rt_nonfinite.h"
#include "ll2utm.h"

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d0;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d0 = std::abs(u0);
    d1 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (d0 == 1.0) {
        y = 1.0;
      } else if (d0 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// LL2UTM Lat/Lon to UTM coordinates precise conversion.
//  [X,Y]=LL2UTM2(LAT,LON) or LL2UTM([LAT,LON]) converts coordinates
//  LAT,LON (in degrees) to UTM X and Y (in meters). Default datum is WGS84.
//
//  LAT and LON can be scalars, vectors or matrix. Outputs X and Y will
//  have the same size as inputs.
//
//  LL2UTM(...,DATUM) uses specific DATUM for conversion. DATUM can be one
//  of the following char strings:
//   'wgs84': World Geodetic System 1984 (default)
//   'nad27': North American Datum 1927
//   'clk66': Clarke 1866
//   'nad83': North American Datum 1983
//   'grs80': Geodetic Reference System 1980
//   'int24': International 1924 / Hayford 1909
//  or DATUM can be a 2-element vector [A,F] where A is semimajor axis (in
//  meters) and F is flattening of the user-defined ellipsoid.
//
//  LL2UTM(...,ZONE) forces the UTM ZONE (scalar integer or same size as
//    LAT and LON) instead of automatic set.
//
//  [X,Y,ZONE]=LL2UTM(...) returns also the computed UTM ZONE (negative
//  value for southern hemisphere points).
//
//
//  XY=LL2UTM(...) or without any output argument returns a 2-column
//  matrix [X,Y].
//
//  Note:
//   - LL2UTM does not perform cross-datum conversion.
//   - precision is near a millimeter.
//
//
//  Reference:
//   I.G.N., Projection cartographique Mercator Transverse: Algorithmes,
//      Notes Techniques NT/G 76, janvier 1995.
//
//  Acknowledgments: Mathieu, Frederic Christen.
//
//
//  Author: Francois Beauducel, <beauducel@ipgp.fr>
//  Created: 2003-12-02
//  Updated: 2015-01-29
// Arguments    : double varargin_1
//                double varargin_2
//
//                double *x
//                double *y
// Return Type  : void
//
void ll2utm(double varargin_1, double varargin_2, double *x, double *y)
{
  double p1;
  double l1;
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

  double y_im;
  double C[5];
  double y_re;
  double b_y_im;
  double b_y_re;
  double c_y_im;
  double c_y_re;
  double d_y_im;
  double d_y_re;

  // 	Copyright (c) 2001-2015, Fran�ois Beauducel, covered by BSD License.
  // 	All rights reserved.
  //
  // 	Redistribution and use in source and binary forms, with or without
  // 	modification, are permitted provided that the following conditions are
  // 	met:
  //
  // 	   * Redistributions of source code must retain the above copyright
  // 	     notice, this list of conditions and the following disclaimer.
  // 	   * Redistributions in binary form must reproduce the above copyright
  // 	     notice, this list of conditions and the following disclaimer in
  // 	     the documentation and/or other materials provided with the distribution
  //
  // 	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  // 	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  // 	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  // 	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  // 	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  // 	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  // 	SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  // 	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  // 	CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  // 	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  // 	POSSIBILITY OF SUCH DAMAGE.
  //  Available datums
  //  constants
  //  conversion rad to deg
  //  UTM scale factor
  //  UTM false East (m)
  //  defaults
  p1 = varargin_1 / 57.295779513082323;

  //  Phi = Latitude (rad)
  l1 = varargin_2 / 57.295779513082323;

  //  Lambda = Longitude (rad)
  //  UTM zone automatic setting
  //  UTM origin longitude (rad)
  //  UTM false northern (m)
  // COEF Projection coefficients
  // 	COEF(E,M) returns a vector of 5 coefficients from:
  // 		E = first ellipsoid excentricity
  // 		M = 0 for transverse mercator
  // 		M = 1 for transverse mercator reverse coefficients
  // 		M = 2 for merdian arc
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
  z_re = std::atan(std::sinh(L) / std::cos(l1 - 0.15707963267948966));
  L = std::log(std::tan(0.78539816339744828 + std::asin(std::sin(l1 -
    0.15707963267948966) / std::cosh(L)) / 2.0));
  l1 = 2.0 * z_re;
  y_im = 2.0 * L;
  if (y_im == 0.0) {
    l1 = std::sin(l1);
    y_im = 0.0;
  } else if (l1 == 0.0) {
    l1 = 0.0;
    y_im = std::sinh(y_im);
  } else {
    y_re = l1;
    l1 = std::sin(l1) * std::cosh(y_im);
    y_im = std::cos(y_re) * std::sinh(y_im);
  }

  y_re = 4.0 * z_re;
  b_y_im = 4.0 * L;
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

  b_y_re = 6.0 * z_re;
  c_y_im = 6.0 * L;
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

  c_y_re = 8.0 * z_re;
  d_y_im = 8.0 * L;
  if (d_y_im == 0.0) {
    c_y_re = std::sin(c_y_re);
    d_y_im = 0.0;
  } else if (c_y_re == 0.0) {
    c_y_re = 0.0;
    d_y_im = std::sinh(d_y_im);
  } else {
    d_y_re = c_y_re;
    c_y_re = std::sin(c_y_re) * std::cosh(d_y_im);
    d_y_im = std::cos(d_y_re) * std::sinh(d_y_im);
  }

  z_re = 6.3649021661656657E+6 * z_re + 6.3755857452000007E+6 *
    (((0.00083632803213595363 * l1 + 7.5957782755955371E-7 * y_re) +
      1.1956312986940848E-9 * b_y_re) + 2.4107991079286062E-12 * c_y_re);
  L = 6.3649021661656657E+6 * L + 6.3755857452000007E+6 *
    (((0.00083632803213595363 * y_im + 7.5957782755955371E-7 * b_y_im) +
      1.1956312986940848E-9 * c_y_im) + 2.4107991079286062E-12 * d_y_im);

  //  outputs zone if needed: scalar value if unique, or vector/matrix of the
  //  same size as x/y in case of crossed zones
  *x = L + 500000.0;
  *y = z_re + (1.0E+7 * (double)(p1 < 0.0) - 6.3755857452000007E+6 * ((((C[0] *
    0.0 + C[1] * 0.0) + C[2] * 0.0) + C[3] * 0.0) + C[4] * 0.0));
}

//
// File trailer for ll2utm.cpp
//
// [EOF]
//
