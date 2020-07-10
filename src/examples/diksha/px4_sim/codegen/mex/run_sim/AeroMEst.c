/*
 * AeroMEst.c
 *
 * Code generation for function 'AeroMEst'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "AeroMEst.h"
#include "mpower.h"
#include "norm.h"
#include "run_sim_data.h"

/* Function Definitions */
void AeroMEst(const real_T eul[3], const real_T x_dot[3], const real_T omega[3],
              const real_T Fa[3], real_T Moment_aero[3])
{
  int32_T i;
  real_T V;
  real_T dv14[9];
  real_T a[3];
  int32_T i3;
  real_T xw_dot[3];
  real_T alpha;
  static const int8_T b_a[9] = { 0, 0, -1, 0, 1, 0, 1, 0, 0 };

  real_T My_w_ac;
  covrtLogFcn(&emlrtCoverageInstance, 6U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 0);

  /* UNTITLED2 Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i = 0; i < 3; i++) {
    Moment_aero[i] = 0.0;
  }

  covrtLogFcn(&emlrtCoverageInstance, 4U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 4U, 0);
  V = norm(x_dot);
  if (covrtLogIf(&emlrtCoverageInstance, 6U, 0U, 0, V == 0.0)) {
    covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 1);
  } else {
    covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 2);
    dv14[0] = muDoubleScalarCos(eul[1]) * muDoubleScalarCos(eul[0]);
    dv14[1] = -muDoubleScalarCos(eul[2]) * muDoubleScalarSin(eul[0]) +
      muDoubleScalarSin(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarCos
      (eul[0]);
    dv14[2] = muDoubleScalarSin(eul[2]) * muDoubleScalarSin(eul[0]) +
      muDoubleScalarCos(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarCos
      (eul[0]);
    dv14[3] = muDoubleScalarCos(eul[1]) * muDoubleScalarSin(eul[0]);
    dv14[4] = muDoubleScalarCos(eul[2]) * muDoubleScalarCos(eul[0]) +
      muDoubleScalarSin(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarSin
      (eul[0]);
    dv14[5] = -muDoubleScalarSin(eul[2]) * muDoubleScalarCos(eul[0]) +
      muDoubleScalarCos(eul[2]) * muDoubleScalarSin(eul[1]) * muDoubleScalarSin
      (eul[0]);
    dv14[6] = -muDoubleScalarSin(eul[1]);
    dv14[7] = muDoubleScalarSin(eul[2]) * muDoubleScalarCos(eul[1]);
    dv14[8] = muDoubleScalarCos(eul[2]) * muDoubleScalarCos(eul[1]);
    for (i = 0; i < 3; i++) {
      a[i] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        a[i] += dv14[i + 3 * i3] * x_dot[i3];
      }
    }

    for (i = 0; i < 3; i++) {
      xw_dot[i] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        xw_dot[i] += (real_T)b_a[i + 3 * i3] * a[i3];
      }
    }

    if (covrtLogIf(&emlrtCoverageInstance, 6U, 0U, 1, xw_dot[0] != 0.0)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 3);
      alpha = muDoubleScalarAtan2(-xw_dot[2], xw_dot[0]);
    } else {
      covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 4);
      alpha = 0.0;
    }

    covrtLogBasicBlock(&emlrtCoverageInstance, 6U, 5);
    mpower(V);
    for (i = 0; i < 3; i++) {
      a[i] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        a[i] += (real_T)b_a[i + 3 * i3] * omega[i3];
      }
    }

    My_w_ac = 0.6125 * mpower(V) * 0.377 * 0.29648527679623088 * ((-0.0156 +
      0.995 * alpha) + -0.51 * a[1] * 0.29648527679623088 / (2.0 * V));
    mpower(V);

    /*  wing frame */
    /*  r = BQ.x_ac - BQ.x_cg; */
    /*  M_cg = M_ac + cross(r',(Rq2w*Fa)')'; */
    /*  L = -Fa(1); */
    /*  M_cg = M_ac + [0;r*L;0]; % wing frame */
    /*  Moment_aero = BQ.wing_n*Rq2w'*M_cg; % body frame */
    Moment_aero[0] = 0.0;
    Moment_aero[1] = 2.0 * My_w_ac + -0.0732 * Fa[0];
    Moment_aero[2] = 0.0;
  }
}

/* End of code generation (AeroMEst.c) */
