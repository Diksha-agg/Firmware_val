/*
 * controller.c
 *
 * Code generation for function 'controller'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "controller.h"
#include "AeroFEst.h"
#include "AeroMEst.h"
#include "norm.h"
#include "run_sim_data.h"

/* Function Definitions */
void controller(const real_T posc[3], const real_T velc[3], const real_T rotc[3],
                const real_T omegac[3], const real_T posd[3], const real_T veld
                [3], const real_T rotd[3], const real_T omegad[3], const real_T
                controld[2], real_T *F, real_T M[3])
{
  real_T Rd[9];
  real_T b_omegac[3];
  real_T Rb[9];
  real_T b_omegad[3];
  int32_T i0;
  real_T omega_curr[3];
  real_T omega_des[3];
  int32_T k;
  real_T b_Rb[9];
  real_T dv5[3];
  real_T b_velc[3];
  real_T Fa[3];
  real_T c_Rb[9];
  real_T dv6[3];
  real_T dv7[3];
  real_T b_veld[3];
  real_T acc_net[3];
  real_T y;
  real_T b3[3];
  real_T c2[3];
  real_T b_c2[3];
  static const real_T dv8[3] = { 0.0, 0.0, 9.81 };

  static const int8_T a[9] = { 50, 0, 0, 0, 6, 0, 0, 0, 5 };

  static const int8_T b_a[9] = { 5, 0, 0, 0, 5, 0, 0, 0, 50 };

  real_T b1[3];
  real_T b_Rd[9];
  real_T b_y[3];
  static const int8_T c_a[3] = { 0, 0, 1 };

  real_T erm[9];
  int32_T i1;
  real_T dv9[3];
  real_T c_velc[3];
  real_T b_erm[3];
  real_T dv10[3];
  real_T b_omega_curr[3];
  real_T c_c2[3];
  static const real_T d_a[9] = { 1.86, 0.0, 0.0, 0.0, 2.031, 0.0, 0.0, 0.0,
    3.617 };

  static const int8_T e_a[9] = { 3, 0, 0, 0, 100, 0, 0, 0, 5 };

  static const int16_T f_a[9] = { 5, 0, 0, 0, 1500, 0, 0, 0, 1 };

  covrtLogFcn(&emlrtCoverageInstance, 3U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 3U, 0);

  /* desired inputs:[xd yd zd xd_dot yd_dot zd_dot phid thetad psid phidotd thetadotd psidotd ud(1)---Tfwd */
  /* ud(2)---Mfwd */
  /* kg */
  Rd[0] = 1.0;
  Rd[3] = 0.0;
  Rd[6] = -muDoubleScalarSin(rotc[1]);
  Rd[1] = 0.0;
  Rd[4] = muDoubleScalarCos(rotc[2]);
  Rd[7] = muDoubleScalarCos(rotc[1]) * muDoubleScalarSin(rotc[2]);
  Rd[2] = 0.0;
  Rd[5] = -muDoubleScalarSin(rotc[2]);
  Rd[8] = muDoubleScalarCos(rotc[1]) * muDoubleScalarCos(rotc[2]);
  b_omegac[0] = omegac[2];
  b_omegac[1] = omegac[1];
  b_omegac[2] = omegac[0];
  Rb[0] = 1.0;
  Rb[3] = 0.0;
  Rb[6] = -muDoubleScalarSin(rotd[1]);
  Rb[1] = 0.0;
  Rb[4] = muDoubleScalarCos(rotd[2]);
  Rb[7] = muDoubleScalarCos(rotd[1]) * muDoubleScalarSin(rotd[2]);
  Rb[2] = 0.0;
  Rb[5] = -muDoubleScalarSin(rotd[2]);
  Rb[8] = muDoubleScalarCos(rotd[1]) * muDoubleScalarCos(rotd[2]);
  b_omegad[0] = omegad[2];
  b_omegad[1] = omegad[1];
  b_omegad[2] = omegad[0];
  for (i0 = 0; i0 < 3; i0++) {
    omega_curr[i0] = 0.0;
    omega_des[i0] = 0.0;
    for (k = 0; k < 3; k++) {
      omega_curr[i0] += Rd[i0 + 3 * k] * b_omegac[k];
      omega_des[i0] += Rb[i0 + 3 * k] * b_omegad[k];
    }
  }

  covrtLogFcn(&emlrtCoverageInstance, 4U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0);
  b_Rb[0] = muDoubleScalarCos(rotc[1]) * muDoubleScalarCos(rotc[0]);
  b_Rb[3] = -muDoubleScalarCos(rotc[2]) * muDoubleScalarSin(rotc[0]) +
    muDoubleScalarSin(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarCos
    (rotc[0]);
  b_Rb[6] = muDoubleScalarSin(rotc[2]) * muDoubleScalarSin(rotc[0]) +
    muDoubleScalarCos(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarCos
    (rotc[0]);
  b_Rb[1] = muDoubleScalarCos(rotc[1]) * muDoubleScalarSin(rotc[0]);
  b_Rb[4] = muDoubleScalarCos(rotc[2]) * muDoubleScalarCos(rotc[0]) +
    muDoubleScalarSin(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarSin
    (rotc[0]);
  b_Rb[7] = -muDoubleScalarSin(rotc[2]) * muDoubleScalarCos(rotc[0]) +
    muDoubleScalarCos(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarSin
    (rotc[0]);
  b_Rb[2] = -muDoubleScalarSin(rotc[1]);
  b_Rb[5] = muDoubleScalarSin(rotc[2]) * muDoubleScalarCos(rotc[1]);
  b_Rb[8] = muDoubleScalarCos(rotc[2]) * muDoubleScalarCos(rotc[1]);
  covrtLogFcn(&emlrtCoverageInstance, 4U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0);
  dv5[0] = 0.0;
  dv5[1] = rotc[1];
  dv5[2] = 0.0;
  b_velc[0] = velc[0];
  b_velc[1] = 0.0;
  b_velc[2] = velc[2];
  AeroFEst(dv5, b_velc, omega_curr, Fa);
  c_Rb[0] = muDoubleScalarCos(rotd[1]) * muDoubleScalarCos(rotd[0]);
  c_Rb[3] = -muDoubleScalarCos(rotd[2]) * muDoubleScalarSin(rotd[0]) +
    muDoubleScalarSin(rotd[2]) * muDoubleScalarSin(rotd[1]) * muDoubleScalarCos
    (rotd[0]);
  c_Rb[6] = muDoubleScalarSin(rotd[2]) * muDoubleScalarSin(rotd[0]) +
    muDoubleScalarCos(rotd[2]) * muDoubleScalarSin(rotd[1]) * muDoubleScalarCos
    (rotd[0]);
  c_Rb[1] = muDoubleScalarCos(rotd[1]) * muDoubleScalarSin(rotd[0]);
  c_Rb[4] = muDoubleScalarCos(rotd[2]) * muDoubleScalarCos(rotd[0]) +
    muDoubleScalarSin(rotd[2]) * muDoubleScalarSin(rotd[1]) * muDoubleScalarSin
    (rotd[0]);
  c_Rb[7] = -muDoubleScalarSin(rotd[2]) * muDoubleScalarCos(rotd[0]) +
    muDoubleScalarCos(rotd[2]) * muDoubleScalarSin(rotd[1]) * muDoubleScalarSin
    (rotd[0]);
  c_Rb[2] = -muDoubleScalarSin(rotd[1]);
  c_Rb[5] = muDoubleScalarSin(rotd[2]) * muDoubleScalarCos(rotd[1]);
  c_Rb[8] = muDoubleScalarCos(rotd[2]) * muDoubleScalarCos(rotd[1]);
  dv6[0] = 0.0;
  dv6[1] = 0.0;
  dv6[2] = controld[0];
  dv7[0] = 0.0;
  dv7[1] = 0.0;
  dv7[2] = posd[2] - posc[2];
  b_veld[0] = veld[0] - velc[0];
  b_veld[1] = -velc[1];
  b_veld[2] = veld[2] - velc[2];
  for (i0 = 0; i0 < 3; i0++) {
    y = 0.0;
    for (k = 0; k < 3; k++) {
      y += c_Rb[i0 + 3 * k] * dv6[k];
    }

    b_omegad[i0] = y / 12.0;
    b_velc[i0] = 0.0;
    y = 0.0;
    for (k = 0; k < 3; k++) {
      y += (real_T)a[i0 + 3 * k] * b_veld[k];
      b_velc[i0] += (real_T)b_a[i0 + 3 * k] * dv7[k];
    }

    dv5[i0] = ((b_omegad[i0] + b_velc[i0]) + y) + dv8[i0];
    y = 0.0;
    for (k = 0; k < 3; k++) {
      y += b_Rb[i0 + 3 * k] * Fa[k];
    }

    b_omegac[i0] = y / 12.0;
    acc_net[i0] = dv5[i0] - b_omegac[i0];
  }

  /*  acc_net */
  /*  calculation of current euler angles */
  y = norm(acc_net);
  for (k = 0; k < 3; k++) {
    b3[k] = acc_net[k] / y;
  }

  c2[0] = -muDoubleScalarSin(rotd[0]);
  c2[1] = muDoubleScalarCos(rotd[0]);
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

  /* thrust control input */
  y = 0.0;
  for (k = 0; k < 3; k++) {
    b_Rd[k] = b1[k];
    b_Rd[6 + k] = b3[k];
    b_y[k] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      b_y[k] += (real_T)c_a[i0] * b_Rb[k + 3 * i0];
    }

    y += b_y[k] * acc_net[k];
  }

  *F = muDoubleScalarSign(y) * (12.0 * norm(acc_net));
  covrtLogFcn(&emlrtCoverageInstance, 4U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0);
  b_Rb[0] = muDoubleScalarCos(rotc[1]) * muDoubleScalarCos(rotc[0]);
  b_Rb[3] = -muDoubleScalarCos(rotc[2]) * muDoubleScalarSin(rotc[0]) +
    muDoubleScalarSin(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarCos
    (rotc[0]);
  b_Rb[6] = muDoubleScalarSin(rotc[2]) * muDoubleScalarSin(rotc[0]) +
    muDoubleScalarCos(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarCos
    (rotc[0]);
  b_Rb[1] = muDoubleScalarCos(rotc[1]) * muDoubleScalarSin(rotc[0]);
  b_Rb[4] = muDoubleScalarCos(rotc[2]) * muDoubleScalarCos(rotc[0]) +
    muDoubleScalarSin(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarSin
    (rotc[0]);
  b_Rb[7] = -muDoubleScalarSin(rotc[2]) * muDoubleScalarCos(rotc[0]) +
    muDoubleScalarCos(rotc[2]) * muDoubleScalarSin(rotc[1]) * muDoubleScalarSin
    (rotc[0]);
  b_Rb[2] = -muDoubleScalarSin(rotc[1]);
  b_Rb[5] = muDoubleScalarSin(rotc[2]) * muDoubleScalarCos(rotc[1]);
  b_Rb[8] = muDoubleScalarCos(rotc[2]) * muDoubleScalarCos(rotc[1]);
  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 3; k++) {
      Rd[i0 + 3 * k] = 0.0;
      Rb[i0 + 3 * k] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        Rd[i0 + 3 * k] += b_Rd[i1 + 3 * i0] * b_Rb[i1 + 3 * k];
        Rb[i0 + 3 * k] += b_Rb[i1 + 3 * i0] * b_Rd[i1 + 3 * k];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    y = 0.0;
    for (k = 0; k < 3; k++) {
      erm[k + 3 * i0] = 0.5 * (Rd[k + 3 * i0] - Rb[k + 3 * i0]);
      c_Rb[i0 + 3 * k] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        c_Rb[i0 + 3 * k] += b_Rb[i1 + 3 * i0] * b_Rd[i1 + 3 * k];
      }

      y += c_Rb[i0 + 3 * k] * omega_des[k];
    }

    c2[i0] = omega_curr[i0] - y;
  }

  dv9[0] = 0.0;
  dv9[1] = rotc[1];
  dv9[2] = 0.0;
  c_velc[0] = velc[0];
  c_velc[1] = 0.0;
  c_velc[2] = velc[2];
  AeroMEst(dv9, c_velc, omega_curr, Fa, b1);
  b_erm[0] = erm[5];
  b_erm[1] = erm[6];
  b_erm[2] = erm[1];
  dv10[0] = 0.0;
  dv10[1] = controld[1];
  dv10[2] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    acc_net[i0] = 0.0;
    b3[i0] = 0.0;
    b_velc[i0] = 0.0;
    y = 0.0;
    for (k = 0; k < 3; k++) {
      acc_net[i0] += d_a[i0 + 3 * k] * omega_curr[k];
      Rb[i0 + 3 * k] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        Rb[i0 + 3 * k] += b_Rb[i1 + 3 * i0] * b_Rd[i1 + 3 * k];
      }

      b3[i0] += Rb[i0 + 3 * k] * omega_des[k];
      b_velc[i0] += (real_T)e_a[i0 + 3 * k] * b_erm[k];
      y += (real_T)f_a[i0 + 3 * k] * c2[k];
    }

    dv5[i0] = (dv10[i0] - b_velc[i0]) - y;
  }

  b_omega_curr[0] = omega_curr[1] * acc_net[2] - omega_curr[2] * acc_net[1];
  b_omega_curr[1] = omega_curr[2] * acc_net[0] - omega_curr[0] * acc_net[2];
  b_omega_curr[2] = omega_curr[0] * acc_net[1] - omega_curr[1] * acc_net[0];
  c_c2[0] = c2[1] * b3[2] - c2[2] * b3[1];
  c_c2[1] = c2[2] * b3[0] - c2[0] * b3[2];
  c_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];
  for (i0 = 0; i0 < 3; i0++) {
    y = 0.0;
    for (k = 0; k < 3; k++) {
      y += d_a[i0 + 3 * k] * c_c2[k];
    }

    M[i0] = ((dv5[i0] + b_omega_curr[i0]) - y) - b1[i0];
  }

  /* moment input */
}

/* End of code generation (controller.c) */
