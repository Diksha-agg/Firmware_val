/*
 * AeroFEst.c
 *
 * Code generation for function 'AeroFEst'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "AeroFEst.h"
#include "mpower.h"
#include "norm.h"
#include "run_sim_data.h"

/* Function Definitions */
void AeroFEst(const real_T eul[3], const real_T x_dot[3], const real_T omega[3],
              real_T Fa[3])
{
  int32_T i;
  real_T V;
  real_T dv11[9];
  real_T dv12[3];
  int32_T i2;
  real_T xw_dot[3];
  static const int8_T a[9] = { 0, 0, -1, 0, 1, 0, 1, 0, 0 };

  real_T alpha;
  real_T beta;
  real_T sigma_a;
  real_T CLofalpha;
  real_T CL;
  real_T CD;
  real_T L;
  real_T D;
  real_T Y;
  real_T dv13[9];
  real_T b_D[3];
  covrtLogFcn(&emlrtCoverageInstance, 5U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 5U, 0);

  /* UNTITLED Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i = 0; i < 3; i++) {
    Fa[i] = 0.0;
  }

  /*  Eul=quat2eul((quat/norm(quat))'); */
  /*  psi=Eul(1,1); */
  /*  theta=Eul(1,2); */
  /*  phi=Eul(1,3); */
  covrtLogFcn(&emlrtCoverageInstance, 4U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0);
  V = norm(x_dot);
  if (covrtLogIf(&emlrtCoverageInstance, 5U, 0U, 0, V == 0.0)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 5U, 1);
  } else {
    covrtLogBasicBlock(&emlrtCoverageInstance, 5U, 2);
    dv11[0] = muDoubleScalarCos(eul[1]) * muDoubleScalarCos(eul[0]);
    dv11[1] = -muDoubleScalarCos(eul[2]) * muDoubleScalarSin(eul[0]) +
      muDoubleScalarSin(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarCos
      (eul[0]);
    dv11[2] = muDoubleScalarSin(eul[2]) * muDoubleScalarSin(eul[0]) +
      muDoubleScalarCos(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarCos
      (eul[0]);
    dv11[3] = muDoubleScalarCos(eul[1]) * muDoubleScalarSin(eul[0]);
    dv11[4] = muDoubleScalarCos(eul[2]) * muDoubleScalarCos(eul[0]) +
      muDoubleScalarSin(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarSin
      (eul[0]);
    dv11[5] = -muDoubleScalarSin(eul[2]) * muDoubleScalarCos(eul[0]) +
      muDoubleScalarCos(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarSin
      (eul[0]);
    dv11[6] = -muDoubleScalarSin(eul[1]);
    dv11[7] = muDoubleScalarSin(eul[2]) * muDoubleScalarCos(eul[1]);
    dv11[8] = muDoubleScalarCos(eul[2]) * muDoubleScalarCos(eul[1]);
    for (i = 0; i < 3; i++) {
      dv12[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        dv12[i] += dv11[i + 3 * i2] * x_dot[i2];
      }
    }

    for (i = 0; i < 3; i++) {
      xw_dot[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        xw_dot[i] += (real_T)a[i + 3 * i2] * dv12[i2];
      }
    }

    /*  alpha=atan2(-xw_dot(3),xw_dot(1)); */
    /*  beta=atan2(xw_dot(2),xw_dot(1)); */
    if (covrtLogIf(&emlrtCoverageInstance, 5U, 0U, 1, xw_dot[0] != 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 5U, 3);
      alpha = muDoubleScalarAtan2(-xw_dot[2], xw_dot[0]);
      beta = muDoubleScalarAtan2(xw_dot[1], xw_dot[0]);
    } else {
      covrtLogBasicBlock(&emlrtCoverageInstance, 5U, 4);
      alpha = 0.0;
      beta = 0.0;
    }

    covrtLogBasicBlock(&emlrtCoverageInstance, 5U, 5);

    /*  alpha=atan2(xb_dot(1),xb_dot(3)); */
    /*  beta=atan2(xb_dot(2),xb_dot(3)); */
    /*  alpha = abs(alpha); */
    /*  beta = abs(beta); */
    sigma_a = ((1.0 + muDoubleScalarExp(-50.0 * (alpha - 0.3490658503988659))) +
               muDoubleScalarExp(50.0 * (alpha + 0.3490658503988659))) / ((1.0 +
      muDoubleScalarExp(-50.0 * (alpha - 0.3490658503988659))) * (1.0 +
      muDoubleScalarExp(50.0 * (alpha + 0.3490658503988659))));
    CLofalpha = (1.0 - sigma_a) * (0.4918 + 4.695 * alpha) + sigma_a * (2.0 *
      muDoubleScalarSign(alpha) * mpower(muDoubleScalarSin(alpha)) *
      muDoubleScalarCos(alpha));

    /*      CLofalpha = (2*sign(alpha)*(sin(alpha)^2)*cos(alpha)); */
    CL = CLofalpha + 0.0 * (omega[1] * 0.29648527679623088 / 2.0 * V);
    CD = (0.009 + 0.057664834453585265 * mpower(CL)) + mpower(muDoubleScalarSin
      (alpha));
    for (i = 0; i < 3; i++) {
      xw_dot[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        xw_dot[i] += (real_T)a[i + 3 * i2] * omega[i2];
      }
    }

    L = 0.6125 * mpower(V) * 0.377 * CL;
    D = 0.6125 * mpower(V) * 0.377 * CD;
    Y = 0.6125 * mpower(V) * 0.377 * ((-0.951 * beta + 0.0 * (xw_dot[0] * 1.145 /
      (2.0 * V))) + 0.008 * (xw_dot[2] * 1.145 / (2.0 * V)));
    dv13[0] = muDoubleScalarSin(alpha) * muDoubleScalarCos(beta);
    dv13[3] = -muDoubleScalarSin(alpha) * muDoubleScalarSin(beta);
    dv13[6] = muDoubleScalarCos(alpha);
    dv13[1] = muDoubleScalarSin(beta);
    dv13[4] = muDoubleScalarCos(beta);
    dv13[7] = 0.0;
    dv13[2] = -muDoubleScalarCos(alpha) * muDoubleScalarCos(beta);
    dv13[5] = muDoubleScalarCos(alpha) * muDoubleScalarSin(beta);
    dv13[8] = muDoubleScalarSin(alpha);
    for (i = 0; i < 3; i++) {
      for (i2 = 0; i2 < 3; i2++) {
        dv11[i2 + 3 * i] = 2.0 * dv13[i2 + 3 * i];
      }
    }

    b_D[0] = -D;
    b_D[1] = Y;
    b_D[2] = -L;
    for (i = 0; i < 3; i++) {
      Fa[i] = 0.0;
      for (i2 = 0; i2 < 3; i2++) {
        Fa[i] += dv11[i + 3 * i2] * b_D[i2];
      }
    }

    Fa[1] = -Fa[1];
    Fa[2] = -Fa[2];
  }
}

/* End of code generation (AeroFEst.c) */
