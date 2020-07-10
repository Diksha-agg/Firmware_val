/*
 * File: local.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "local.h"

/* Function Definitions */

/*
 * Arguments    : double t
 *                double posc[3]
 *                double velc[3]
 * Return Type  : void
 */
void local(double t, double posc[3], double velc[3])
{
  posc[0] = 0.0;
  posc[1] = 0.0;
  posc[2] = 5.0 * (t * t);
  velc[0] = 0.0;
  velc[1] = 0.0;
  velc[2] = 10.0 * t;
}

/*
 * File trailer for local.c
 *
 * [EOF]
 */
