/*
 * time_trajj.c
 *
 * Code generation for function 'time_trajj'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "time_trajj.h"
#include "mpower.h"
#include "run_sim_data.h"

/* Function Definitions */
void time_trajj(real_T t, real_T posd[3], real_T veld[3])
{
  real_T z;
  covrtLogFcn(&emlrtCoverageInstance, 1U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1U, 0);
  z = 5.0 * mpower(t);
  posd[0] = 0.0;
  posd[1] = 0.0;
  posd[2] = z;
  veld[0] = 0.0;
  veld[1] = 0.0;
  veld[2] = 10.0 * t;
}

/* End of code generation (time_trajj.c) */
