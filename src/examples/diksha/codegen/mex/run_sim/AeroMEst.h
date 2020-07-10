/*
 * AeroMEst.h
 *
 * Code generation for function 'AeroMEst'
 *
 */

#ifndef AEROMEST_H
#define AEROMEST_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mwmathutil.h"
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "covrt.h"
#include "rtwtypes.h"
#include "run_sim_types.h"

/* Function Declarations */
extern void AeroMEst(const real_T eul[3], const real_T x_dot[3], const real_T
                     omega[3], const real_T Fa[3], real_T Moment_aero[3]);

#endif

/* End of code generation (AeroMEst.h) */
