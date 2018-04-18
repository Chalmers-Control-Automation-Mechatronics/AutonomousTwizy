//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: power.cpp
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
#include "power.h"

// Function Definitions

//
// Arguments    : const double a_data[]
//                const int a_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
void power(const double a_data[], const int a_size[2], double y_data[], int
           y_size[2])
{
  y_size[0] = 1;
  y_size[1] = (signed char)a_size[1];
  if (1 <= a_size[1]) {
    y_data[0] = a_data[0] * a_data[0];
  }
}

//
// File trailer for power.cpp
//
// [EOF]
//
