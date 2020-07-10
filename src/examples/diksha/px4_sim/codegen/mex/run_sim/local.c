/*
 * local.c
 *
 * Code generation for function 'local'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "local.h"
#include "mpower.h"
#include "run_sim_data.h"

/* Function Definitions */
void local(real_T t, real_T posc[3], real_T velc[3])
{
  real_T zc;
  covrtLogFcn(&emlrtCoverageInstance, 2U, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 2U, 0);
  zc = 5.0 * mpower(t);
  posc[0] = 0.0;
  posc[1] = 0.0;
  posc[2] = zc;
  velc[0] = 0.0;
  velc[1] = 0.0;
  velc[2] = 10.0 * t;
}

/* End of code generation (local.c) */
