/*
 * File: AeroMEst.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
/*#include "controller.h"
#include "AeroMEst.h"
#include "norm.h"*/
#include "norm.c"
#include "controller_rtwutil.h"

/* Function Definitions */

/*
 * UNTITLED2 Summary of this function goes here
 *    Detailed explanation goes here
 * Arguments    : const double eul[3]
 *                const double x_dot[3]
 *                const double omega[3]
 *                const double Fa[3]
 *                double Moment_aero[3]
 * Return Type  : void
 */
void AeroMEst(const double eul[3], const double x_dot[3], const double omega[3],
              const double Fa[3], double Moment_aero[3])
{
  int i;
  double V;
  double dv11[9];
  double a[3];
  int i3;
  double xw_dot[3];
  static const signed char b_a[9] = { 0, 0, -1, 0, 1, 0, 1, 0, 0 };
  //const double epsilon=0.0;
  double b_xw_dot;
  for (i = 0; i < 3; i++) {
    Moment_aero[i] = 0.0;
  }

  V = norm(x_dot);
  if (fabs(V) > 0.0000001)
  {
    dv11[0] = cos(eul[1]) * cos(eul[0]);
    dv11[1] = -cos(eul[2]) * sin(eul[0]) + sin(eul[2]) * sin(eul[1]) * cos(eul[0]);
    dv11[2] = sin(eul[2]) * sin(eul[0]) + cos(eul[2]) * sin(eul[1]) * cos(eul[0]);
    dv11[3] = cos(eul[1]) * sin(eul[0]);
    dv11[4] = cos(eul[2]) * cos(eul[0]) + sin(eul[2]) * sin(eul[1]) * sin(eul[0]);
    dv11[5] = -sin(eul[2]) * cos(eul[0]) + cos(eul[2]) * sin(eul[1]) * sin(eul[0]);
    dv11[6] = -sin(eul[1]);
    dv11[7] = sin(eul[2]) * cos(eul[1]);
    dv11[8] = cos(eul[2]) * cos(eul[1]);
    for (i = 0; i < 3; i++) {
      a[i] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        a[i] += dv11[i + 3 * i3] * x_dot[i3];
      }
    }
  }
  else {return;}

    for (i = 0; i < 3; i++) {
      xw_dot[i] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        xw_dot[i] += (double)b_a[i + 3 * i3] * a[i3];
      }
    }

    /*  wing frame */
    /*  r = BQ.x_ac - BQ.x_cg; */
    /*  M_cg = M_ac + cross(r',(Rq2w*Fa)')'; */
    /*  L = -Fa(1); */
    /*  M_cg = M_ac + [0;r*L;0]; % wing frame */
    /*  Moment_aero = BQ.wing_n*Rq2w'*M_cg; % body frame */
    for (i = 0; i < 3; i++) {
      a[i] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        a[i] += (double)b_a[i + 3 * i3] * omega[i3];
      }
    }

    Moment_aero[0] = 0.0;
    if (fabs(xw_dot[0]) > 0.00001) {
      b_xw_dot = atan2(-xw_dot[2], xw_dot[0]);
    } else {
      b_xw_dot = 0.0;
    }

    Moment_aero[1] = 2.0 * (0.6125 * (V * V) * 0.377 * 0.29648527679623088 *
      ((-0.0156 + 0.995 * b_xw_dot) + -0.51 * a[1] * 0.29648527679623088 / (2.0 *
      V))) + -0.0732 * Fa[0];
    Moment_aero[2] = 0.0;
 
}

/*
 * File trailer for AeroMEst.c
 *
 * [EOF]
 */
