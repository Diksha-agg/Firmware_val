/*
 * File: _coder_run_sim_api.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

#ifndef _CODER_RUN_SIM_API_H
#define _CODER_RUN_SIM_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_run_sim_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void run_sim(real_T F_traj[400], real_T M_traj[1200]);
extern void run_sim_api(const mxArray *plhs[2]);
extern void run_sim_atexit(void);
extern void run_sim_initialize(void);
extern void run_sim_terminate(void);
extern void run_sim_xil_terminate(void);

#endif

/*
 * File trailer for _coder_run_sim_api.h
 *
 * [EOF]
 */
