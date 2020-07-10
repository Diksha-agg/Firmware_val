/*
 * File: _coder_run_sim_api.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_run_sim_api.h"
#include "_coder_run_sim_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true, false, 131434U, NULL, "run_sim", NULL,
  false, { 2045744189U, 2170104910U, 2743257031U, 4284093946U }, NULL };

/* Function Declarations */
static const mxArray *b_emlrt_marshallOut(const real_T u[1200]);
static const mxArray *emlrt_marshallOut(const real_T u[400]);

/* Function Definitions */

/*
 * Arguments    : const real_T u[1200]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[1200])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv2[2] = { 0, 0 };

  static const int32_T iv3[2] = { 400, 3 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)u);
  emlrtSetDimensions((mxArray *)m1, iv3, 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const real_T u[400]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[400])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 0 };

  static const int32_T iv1[1] = { 400 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray *plhs[2]
 * Return Type  : void
 */
void run_sim_api(const mxArray *plhs[2])
{
  real_T (*F_traj)[400];
  real_T (*M_traj)[1200];
  F_traj = (real_T (*)[400])mxMalloc(sizeof(real_T [400]));
  M_traj = (real_T (*)[1200])mxMalloc(sizeof(real_T [1200]));

  /* Invoke the target function */
  run_sim(*F_traj, *M_traj);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*F_traj);
  plhs[1] = b_emlrt_marshallOut(*M_traj);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void run_sim_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  run_sim_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void run_sim_initialize(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void run_sim_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_run_sim_api.c
 *
 * [EOF]
 */
