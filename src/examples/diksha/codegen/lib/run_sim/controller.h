/*
 * File: controller.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
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
#include "run_sim_types.h"

/* Function Declarations */
extern void controller(const double posc[3], const double velc[3], const double
  rotc[3], const double omegac[3], const double posd[3], const double veld[3],
  const double rotd[3], const double omegad[3], const double controld[2], double
  *Thrust, double M[3]);

#endif

/*
 * File trailer for controller.h
 *
 * [EOF]
 */
