/*
 * File: controller.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

/* Include Files */
#include "rt_nonfinite.h"
//#include "run_sim.h"
#include "controller.h"
#include "AeroMEst.h"
#include "norm.h"
#include "AeroFEst.h"

/* Function Definitions */

/*
 * Arguments    : const double posc[3]
 *                const double velc[3]
 *                const double rotc[3]
 *                const double omegac[3]
 *                const double posd[3]
 *                const double veld[3]
 *                const double rotd[3]
 *                const double omegad[3]
 *                const double controld[2]
 *                double *Thrust
 *                double M[3]
 * Return Type  : void
 */
void controller(const double posc[3], const double velc[3], const double rotc[3],
                const double omegac[3], const double posd[3], const double veld
                [3], const double rotd[3], const double omegad[3], const double
                controld[2], double M[3], double *Thrust)
{
  double Rd[9];
  double b_omegac[3];
  double R[9];
  double b_omegad[3];
  int i;
  double dv5[3];
  double omega_curr[3];
  double omega_des[3];
  int i0;
  double b_velc[3];
  double Fa[3];
  double b_R[9];
  double dv6[3];
  double dv7[3];
  double b_veld[3];
  double dv8[9];
  double acc_net[3];
  double y;
  double b3[3];
  double c2[3];
  double b_c2[3];
  static const double dv9[3] = { 0.0, 0.0, 9.81 };

  static const signed char a[9] = { 50, 0, 0, 0, 6, 0, 0, 0, 5 };

  static const signed char b_a[9] = { 5, 0, 0, 0, 5, 0, 0, 0, 50 };

  double b1[3];
  double b_Rd[9];
  double c_R[9];
  double erm[9];
  int i1;
  double dv10[3];
  double c_velc[3];
  double b_erm[3];
  double dv11[3];
  double b_omega_curr[3];
  double c_c2[3];
  static const double c_a[9] = { 1.86, 0.0, 0.0, 0.0, 2.031, 0.0, 0.0, 0.0,
    3.617 };

  static const signed char d_a[9] = { 3, 0, 0, 0, 100, 0, 0, 0, 5 };

  static const short e_a[9] = { 5, 0, 0, 0, 1500, 0, 0, 0, 1 };

  /* desired inputs:[xd yd zd xd_dot yd_dot zd_dot phid thetad psid phidotd thetadotd psidotd ud(1)---Tfwd */
  /* ud(2)---Mfwd */
  /* kg */
  Rd[0] = 1.0;
  Rd[3] = 0.0;
  Rd[6] = -sin(rotc[1]);
  Rd[1] = 0.0;
  Rd[4] = cos(rotc[2]);
  Rd[7] = cos(rotc[1]) * sin(rotc[2]);
  Rd[2] = 0.0;
  Rd[5] = -sin(rotc[2]);
  Rd[8] = cos(rotc[1]) * cos(rotc[2]);
  b_omegac[0] = omegac[2];
  b_omegac[1] = omegac[1];
  b_omegac[2] = omegac[0];
  omega_curr[0] = omegac[0];
  omega_curr[1] = omegac[1];
  omega_curr[2] = omegac[2];
  R[0] = 1.0;
  R[3] = 0.0;
  R[6] = -sin(rotd[1]);
  R[1] = 0.0;
  R[4] = cos(rotd[2]);
  R[7] = cos(rotd[1]) * sin(rotd[2]);
  R[2] = 0.0;
  R[5] = -sin(rotd[2]);
  R[8] = cos(rotd[1]) * cos(rotd[2]);
  b_omegad[0] = omegad[2];
  b_omegad[1] = omegad[1];
  b_omegad[2] = omegad[0];
  for (i = 0; i < 3; i++) {
    //omega_curr[i] = 0.0;
    omega_des[i] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      //omega_curr[i] += Rd[i + 3 * i0] * b_omegac[i0];
      omega_des[i] += R[i + 3 * i0] * b_omegad[i0];
    }
  }

  dv5[0] = 0.0;
  dv5[1] = rotc[1];
  dv5[2] = 0.0;
  b_velc[0] = velc[0];
  b_velc[1] = 0.0;
  b_velc[2] = velc[2];
  AeroFEst(dv5, b_velc, omega_curr, Fa);
  b_R[0] = cos(rotd[1]) * cos(rotd[0]);
  b_R[3] = -cos(rotd[2]) * sin(rotd[0]) + sin(rotd[2]) * sin(rotd[1]) * cos
    (rotd[0]);
  b_R[6] = sin(rotd[2]) * sin(rotd[0]) + cos(rotd[2]) * sin(rotd[1]) * cos(rotd
    [0]);
  b_R[1] = cos(rotd[1]) * sin(rotd[0]);
  b_R[4] = cos(rotd[2]) * cos(rotd[0]) + sin(rotd[2]) * sin(rotd[1]) * sin(rotd
    [0]);
  b_R[7] = -sin(rotd[2]) * cos(rotd[0]) + cos(rotd[2]) * sin(rotd[1]) * sin
    (rotd[0]);
  b_R[2] = -sin(rotd[1]);
  b_R[5] = sin(rotd[2]) * cos(rotd[1]);
  b_R[8] = cos(rotd[2]) * cos(rotd[1]);
  dv6[0] = 0.0;
  dv6[1] = 0.0;
  dv6[2] = controld[0];
  dv7[0] = 0.0;
  dv7[1] = 0.0;
  dv7[2] = posd[2] - posc[2];
  b_veld[0] = veld[0] - velc[0];
  b_veld[1] = -velc[1];
  b_veld[2] = veld[2] - velc[2];
  dv8[0] = cos(rotc[1]) * cos(rotc[0]);
  dv8[3] = -cos(rotc[2]) * sin(rotc[0]) + sin(rotc[2]) * sin(rotc[1]) * cos
    (rotc[0]);
  dv8[6] = sin(rotc[2]) * sin(rotc[0]) + cos(rotc[2]) * sin(rotc[1]) * cos(rotc
    [0]);
  dv8[1] = cos(rotc[1]) * sin(rotc[0]);
  dv8[4] = cos(rotc[2]) * cos(rotc[0]) + sin(rotc[2]) * sin(rotc[1]) * sin(rotc
    [0]);
  dv8[7] = -sin(rotc[2]) * cos(rotc[0]) + cos(rotc[2]) * sin(rotc[1]) * sin
    (rotc[0]);
  dv8[2] = -sin(rotc[1]);
  dv8[5] = sin(rotc[2]) * cos(rotc[1]);
  dv8[8] = cos(rotc[2]) * cos(rotc[1]);
  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += b_R[i + 3 * i0] * dv6[i0];
    }

    b_omegad[i] = y / 12.0;
    b_velc[i] = 0.0;
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += (double)a[i + 3 * i0] * b_veld[i0];
      b_velc[i] += (double)b_a[i + 3 * i0] * dv7[i0];
    }

    dv5[i] = ((b_omegad[i] + b_velc[i]) + y) + dv9[i];
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += dv8[i + 3 * i0] * Fa[i0];
    }

    b_omegac[i] = y / 12.0;
    acc_net[i] = dv5[i] - b_omegac[i];
  }

  /*  acc_net */
  /*  calculation of current euler angles */
  y = norm(acc_net);
  for (i = 0; i < 3; i++) {
    b3[i] = acc_net[i] / y;
  }

  c2[0] = -sin(rotd[0]);
  c2[1] = cos(rotd[0]);
  b_c2[0] = c2[1] * b3[2] - 0.0 * b3[1];
  b_c2[1] = 0.0 * b3[0] - c2[0] * b3[2];
  b_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];
  y = norm(b_c2);
  b1[0] = (c2[1] * b3[2] - 0.0 * b3[1]) / y;
  b1[1] = (0.0 * b3[0] - c2[0] * b3[2]) / y;
  b1[2] = (c2[0] * b3[1] - c2[1] * b3[0]) / y;
  b_Rd[3] = b3[1] * b1[2] - b3[2] * b1[1];
  b_Rd[4] = b3[2] * b1[0] - b3[0] * b1[2];
  b_Rd[5] = b3[0] * b1[1] - b3[1] * b1[0];
  for (i = 0; i < 3; i++) {
    b_Rd[i] = b1[i];
    b_Rd[6 + i] = b3[i];
  }

  /* thrust control input */
  /* flag=sign([0 0 1]*Rb'*acc_net); */
  *Thrust = 12.0 * norm(acc_net);
  c_R[0] = cos(rotc[1]) * cos(rotc[0]);
  c_R[3] = -cos(rotc[2]) * sin(rotc[0]) + sin(rotc[2]) * sin(rotc[1]) * cos
    (rotc[0]);
  c_R[6] = sin(rotc[2]) * sin(rotc[0]) + cos(rotc[2]) * sin(rotc[1]) * cos(rotc
    [0]);
  c_R[1] = cos(rotc[1]) * sin(rotc[0]);
  c_R[4] = cos(rotc[2]) * cos(rotc[0]) + sin(rotc[2]) * sin(rotc[1]) * sin(rotc
    [0]);
  c_R[7] = -sin(rotc[2]) * cos(rotc[0]) + cos(rotc[2]) * sin(rotc[1]) * sin
    (rotc[0]);
  c_R[2] = -sin(rotc[1]);
  c_R[5] = sin(rotc[2]) * cos(rotc[1]);
  c_R[8] = cos(rotc[2]) * cos(rotc[1]);
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      Rd[i + 3 * i0] = 0.0;
      R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        Rd[i + 3 * i0] += b_Rd[i1 + 3 * i] * c_R[i1 + 3 * i0];
        R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      erm[i0 + 3 * i] = 0.5 * (Rd[i0 + 3 * i] - R[i0 + 3 * i]);
      b_R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }

      y += b_R[i + 3 * i0] * omega_des[i0];
    }

    c2[i] = omega_curr[i] - y;
  }

  dv10[0] = 0.0;
  dv10[1] = rotc[1];
  dv10[2] = 0.0;
  c_velc[0] = velc[0];
  c_velc[1] = 0.0;
  c_velc[2] = velc[2];
  AeroMEst(dv10, c_velc, omega_curr, Fa, b1);
  b_erm[0] = erm[5];
  b_erm[1] = erm[6];
  b_erm[2] = erm[1];
  dv11[0] = 0.0;
  dv11[1] = controld[1];
  dv11[2] = 0.0;
  for (i = 0; i < 3; i++) {
    acc_net[i] = 0.0;
    b3[i] = 0.0;
    b_velc[i] = 0.0;
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      acc_net[i] += c_a[i + 3 * i0] * omega_curr[i0];
      R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }

      b3[i] += R[i + 3 * i0] * omega_des[i0];
      b_velc[i] += (double)d_a[i + 3 * i0] * b_erm[i0];
      y += (double)e_a[i + 3 * i0] * c2[i0];
    }

    dv5[i] = (dv11[i] - b_velc[i]) - y;
  }

  b_omega_curr[0] = omega_curr[1] * acc_net[2] - omega_curr[2] * acc_net[1];
  b_omega_curr[1] = omega_curr[2] * acc_net[0] - omega_curr[0] * acc_net[2];
  b_omega_curr[2] = omega_curr[0] * acc_net[1] - omega_curr[1] * acc_net[0];
  c_c2[0] = c2[1] * b3[2] - c2[2] * b3[1];
  c_c2[1] = c2[2] * b3[0] - c2[0] * b3[2];
  c_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];
  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += c_a[i + 3 * i0] * c_c2[i0];
    }

    M[i] = ((dv5[i] + b_omega_curr[i]) - y) - b1[i];
  }

  /* moment input */
}

/*
 * File trailer for controller.c
 *
 * [EOF]
 */
