/*
 * File: AeroMEst.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

#ifndef AEROMEST_H
#define AEROMEST_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "run_sim_types.h"

/* Function Declarations */
extern void AeroMEst(const double eul[3], const double x_dot[3], const double
                     omega[3], const double Fa[3], double Moment_aero[3]);

#endif

/*
 * File trailer for AeroMEst.h
 *
 * [EOF]
 */
