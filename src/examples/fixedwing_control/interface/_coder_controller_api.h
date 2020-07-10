/*
 * File: _coder_controller_api.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

#ifndef _CODER_CONTROLLER_API_H
#define _CODER_CONTROLLER_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_controller_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void controller(real_T posc[3], real_T velc[3], real_T rotc[3], real_T
  omegac[3], real_T t, real_T M[3], real_T *Thrust);
extern void controller_api(const mxArray *prhs[5], const mxArray *plhs[2]);
extern void controller_atexit(void);
extern void controller_initialize(void);
extern void controller_terminate(void);
extern void controller_xil_terminate(void);

#endif

/*
 * File trailer for _coder_controller_api.h
 *
 * [EOF]
 */
