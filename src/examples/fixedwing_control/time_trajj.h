/*
 * File: time_trajj.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

#ifndef TIME_TRAJJ_H
#define TIME_TRAJJ_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "controller_types.h"

/* Function Declarations */
extern void time_trajj(double t, double posd[3], double veld[3], double rot_des
  [3], double omegad[3], double controld[2]);

#endif

/*
 * File trailer for time_trajj.h
 *
 * [EOF]
 */
