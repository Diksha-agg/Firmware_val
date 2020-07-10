/*
 * File: controller.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "controller_types.h"

/* Function Declarations */
extern void controller(const double posc[3], const double velc[3], const double
  rotc[3], const double omegac[3], double t, double M[3], double *Thrust);

#endif

/*
 * File trailer for controller.h
 *
 * [EOF]
 */
