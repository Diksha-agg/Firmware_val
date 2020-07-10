/*
 * controller.h
 *
 * Code generation for function 'controller'
 *
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

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
// #include "run_sim_types.h"

/* Function Declarations */
extern void controller(const real_T posc[3], const real_T velc[3], const real_T
  rotc[3], const real_T omegac[3], const real_T posd[3], const real_T veld[3],
  const real_T rotd[3], const real_T omegad[3], const real_T controld[2], real_T
  *F, real_T M[3]);

#endif

/* End of code generation (controller.h) */
