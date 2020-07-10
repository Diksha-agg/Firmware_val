/*
 * _coder_run_sim_mex.c
 *
 * Code generation for function '_coder_run_sim_mex'
 *
 */

/* Include files */
#include "run_sim.h"
#include "_coder_run_sim_mex.h"
#include "run_sim_terminate.h"
#include "_coder_run_sim_api.h"
#include "run_sim_initialize.h"
#include "run_sim_data.h"

/* Function Declarations */
static void run_sim_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs);

/* Function Definitions */
static void run_sim_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs)
{
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 0) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 0, 4, 7,
                        "run_sim");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 7,
                        "run_sim");
  }

  /* Call the function. */
  run_sim_api(outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  run_sim_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  (void)prhs;
  mexAtExit(run_sim_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  run_sim_initialize();

  /* Dispatch the entry-point. */
  run_sim_mexFunction(nlhs, plhs, nrhs);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_run_sim_mex.c) */
