/*
 * run_sim.h
 *
 * Code generation for function 'run_sim'
 *
 */

#ifndef RUN_SIM_H
#define RUN_SIM_H

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
extern void run_sim(const emlrtStack *sp, real_T F_traj[400], real_T M_traj[1200]);

#endif

/* End of code generation (run_sim.h) */
