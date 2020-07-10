/*
 * File: time_trajj.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "time_trajj.h"

/* Function Definitions */

/*
 * Arguments    : double t
 *                double posd[3]
 *                double veld[3]
 * Return Type  : void
 */
void time_trajj(double t, double posd[3], double veld[3])
{
  posd[0] = 0.0;
  posd[1] = 0.0;
  posd[2] = 5.0 * (t * t);
  veld[0] = 0.0;
  veld[1] = 0.0;
  veld[2] = 10.0 * t;
}

/*
 * File trailer for time_trajj.c
 *
 * [EOF]
 */
