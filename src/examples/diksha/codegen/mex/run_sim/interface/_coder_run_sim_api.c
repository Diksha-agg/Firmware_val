/*
 * _coder_run_sim_api.c
 *
 * Code generation for function '_coder_run_sim_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "_coder_run_sim_api.h"
#include "run_sim_data.h"

/* Function Declarations */
static const mxArray *f_emlrt_marshallOut(const real_T u[400]);
static const mxArray *g_emlrt_marshallOut(const real_T u[1200]);

/* Function Definitions */
static const mxArray *f_emlrt_marshallOut(const real_T u[400])
{
  const mxArray *y;
  const mxArray *m5;
  static const int32_T iv3[1] = { 0 };

  static const int32_T iv4[1] = { 400 };

  y = NULL;
  m5 = emlrtCreateNumericArray(1, iv3, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m5, (void *)u);
  emlrtSetDimensions((mxArray *)m5, iv4, 1);
  emlrtAssign(&y, m5);
  return y;
}

static const mxArray *g_emlrt_marshallOut(const real_T u[1200])
{
  const mxArray *y;
  const mxArray *m6;
  static const int32_T iv5[2] = { 0, 0 };

  static const int32_T iv6[2] = { 400, 3 };

  y = NULL;
  m6 = emlrtCreateNumericArray(2, iv5, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m6, (void *)u);
  emlrtSetDimensions((mxArray *)m6, iv6, 2);
  emlrtAssign(&y, m6);
  return y;
}

void run_sim_api(const mxArray *plhs[2])
{
  real_T (*F_traj)[400];
  real_T (*M_traj)[1200];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  F_traj = (real_T (*)[400])mxMalloc(sizeof(real_T [400]));
  M_traj = (real_T (*)[1200])mxMalloc(sizeof(real_T [1200]));

  /* Invoke the target function */
  run_sim(&st, *F_traj, *M_traj);

  /* Marshall function outputs */
  plhs[0] = f_emlrt_marshallOut(*F_traj);
  plhs[1] = g_emlrt_marshallOut(*M_traj);
}

/* End of code generation (_coder_run_sim_api.c) */
