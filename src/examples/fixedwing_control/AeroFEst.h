/*
 * File: AeroFEst.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

#ifndef AEROFEST_H
#define AEROFEST_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "controller_types.h"

/* Function Declarations */
extern void AeroFEst(const double eul[3], const double x_dot[3], const double
                     omega[3], double Fa[3]);

#endif

/*
 * File trailer for AeroFEst.h
 *
 * [EOF]
 */
