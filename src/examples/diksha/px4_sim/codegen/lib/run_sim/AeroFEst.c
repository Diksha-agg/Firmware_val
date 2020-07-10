/*
 * File: AeroFEst.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "AeroFEst.h"
#include "AeroMEst.h"
#include "norm.h"
#include "run_sim_rtwutil.h"

/* Function Definitions */

/*
 * UNTITLED Summary of this function goes here
 *    Detailed explanation goes here
 * Arguments    : const double eul[3]
 *                const double x_dot[3]
 *                const double omega[3]
 *                double Fa[3]
 * Return Type  : void
 */
void AeroFEst(const double eul[3], const double x_dot[3], const double omega[3],
              double Fa[3])
{
  int i;
  double V;
  double dv12[9];
  double dv13[3];
  int i2;
  double xw_dot[3];
  double alpha;
  static const signed char a[9] = { 0, 0, -1, 0, 1, 0, 1, 0, 0 };

  double beta;
  double sigma_a;
  double x;
  double b_alpha;
  double CL;
  double dv14[9];
  double dv15[3];
  for (i = 0; i < 3; i++) {
    Fa[i] = 0.0;
  }

  /*  Eul=quat2eul((quat/norm(quat))'); */
  /*  psi=Eul(1,1); */
  /*  theta=Eul(1,2); */
  /*  phi=Eul(1,3); */
  V = norm(x_dot);
  if (V == 0.0) {
  } else {
    dv12[0] = cos(eul[1]) * cos(eul[0]);
    dv12[1] = -cos(eul[2]) * sin(eul[0]) + sin(eul[2]) * sin(eul[1]) * cos(eul[0]);
    dv12[2] = sin(eul[2]) * sin(eul[0]) + cos(eul[2]) * sin(eul[1]) * cos(eul[0]);
    dv12[3] = cos(eul[1]) * sin(eul[0]);
    dv12[4] = cos(eul[2]) * cos(eul[0]) + sin(eul[2]) * sin(eul[1]) * sin(eul[0]);
    dv12[5] = -sin(eul[2]) * cos(eul[0]) + cos(eul[2]) * sin(eul[1]) * sin(eul[0]);
    dv12[6] = -sin(eul[1]);
    dv12[7] = sin(eul[2]) * cos(eul[1]);
    dv12[8] = cos(eul[2]) * cos(eul[1]);
    for (i = 0; i < 3; i++) {
      dv13[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv13[i] += dv12[i + 3 * i2] * x_dot[i2];
      }
    }

    for (i = 0; i < 3; i++) {
      xw_dot[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        xw_dot[i] += (double)a[i + 3 * i2] * dv13[i2];
      }
    }

    /*  alpha=atan2(-xw_dot(3),xw_dot(1)); */
    /*  beta=atan2(xw_dot(2),xw_dot(1)); */
    if (xw_dot[0] != 0.0) {
      alpha = rt_atan2d_snf(-xw_dot[2], xw_dot[0]);
      beta = rt_atan2d_snf(xw_dot[1], xw_dot[0]);
    } else {
      alpha = 0.0;
      beta = 0.0;
    }

    /*  alpha=atan2(xb_dot(1),xb_dot(3)); */
    /*  beta=atan2(xb_dot(2),xb_dot(3)); */
    /*  alpha = abs(alpha); */
    /*  beta = abs(beta); */
    sigma_a = ((1.0 + exp(-50.0 * (alpha - 0.3490658503988659))) + exp(50.0 *
                (alpha + 0.3490658503988659))) / ((1.0 + exp(-50.0 * (alpha -
      0.3490658503988659))) * (1.0 + exp(50.0 * (alpha + 0.3490658503988659))));
    x = sin(alpha);

    /*      CLofalpha = (2*sign(alpha)*(sin(alpha)^2)*cos(alpha)); */
    if (alpha < 0.0) {
      b_alpha = -1.0;
    } else if (alpha > 0.0) {
      b_alpha = 1.0;
    } else if (alpha == 0.0) {
      b_alpha = 0.0;
    } else {
      b_alpha = alpha;
    }

    CL = ((1.0 - sigma_a) * (0.4918 + 4.695 * alpha) + sigma_a * (2.0 * b_alpha *
           (x * x) * cos(alpha))) + 0.0 * (omega[1] * 0.29648527679623088 / 2.0 *
      V);
    x = sin(alpha);
    dv14[0] = sin(alpha) * cos(beta);
    dv14[3] = -sin(alpha) * sin(beta);
    dv14[6] = cos(alpha);
    dv14[1] = sin(beta);
    dv14[4] = cos(beta);
    dv14[7] = 0.0;
    dv14[2] = -cos(alpha) * cos(beta);
    dv14[5] = cos(alpha) * sin(beta);
    dv14[8] = sin(alpha);
    for (i = 0; i < 3; i++) {
      xw_dot[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv12[i2 + 3 * i] = 2.0 * dv14[i2 + 3 * i];
        xw_dot[i] += (double)a[i + 3 * i2] * omega[i2];
      }
    }

    dv15[0] = -(0.6125 * (V * V) * 0.377 * ((0.009 + 0.057664834453585265 * (CL *
      CL)) + x * x));
    dv15[1] = 0.6125 * (V * V) * 0.377 * ((-0.951 * beta + 0.0 * (xw_dot[0] *
      1.145 / (2.0 * V))) + 0.008 * (xw_dot[2] * 1.145 / (2.0 * V)));
    dv15[2] = -(0.6125 * (V * V) * 0.377 * CL);
    for (i = 0; i < 3; i++) {
      Fa[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        Fa[i] += dv12[i + 3 * i2] * dv15[i2];
      }
    }

    Fa[1] = -Fa[1];
    Fa[2] = -Fa[2];
  }
}

/*
 * File trailer for AeroFEst.c
 *
 * [EOF]
 */
