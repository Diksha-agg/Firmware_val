/*
 * _coder_run_sim_mex.h
 *
 * Code generation for function '_coder_run_sim_mex'
 *
 */

#ifndef _CODER_RUN_SIM_MEX_H
#define _CODER_RUN_SIM_MEX_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "covrt.h"
#include "rtwtypes.h"
#include "run_sim_types.h"

/* Function Declarations */
extern void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const
  mxArray *prhs[]);
extern emlrtCTX mexFunctionCreateRootTLS(void);

#endif

/* End of code generation (_coder_run_sim_mex.h) */
