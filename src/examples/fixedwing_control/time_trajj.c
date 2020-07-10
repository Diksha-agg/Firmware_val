/*
 * File: time_trajj.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
/*#include "controller.h"
#include "time_trajj.h"*/

/* Function Definitions */

/*
 * Arguments    : double t
 *                double posd[3]
 *                double veld[3]
 *                double rot_des[3]
 *                double omegad[3]
 *                double controld[2]
 * Return Type  : void
 */
void time_trajj(double t, double posd[3], double veld[3], double rot_des[3],
                double omegad[3], double controld[2])
{
  posd[0] = 0.0;
  posd[1] = 0.0;
  posd[2] = 5.0 * (t * t);
  veld[0] = 0.0;
  veld[1] = 0.0;
  veld[2] = 10.0 * t;
  rot_des[0] = 0.0;
  rot_des[1] = 0;
  rot_des[2] = 0.0;
  omegad[0] = 0.0;
  omegad[1] = 0;
  omegad[2] = 0.0;
  controld[0] = 0;
  controld[1] = 0.0;
}

/*
 * File trailer for time_trajj.c
 *
 * [EOF]
 */
