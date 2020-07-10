/*
 * AeroFEst.h
 *
 * Code generation for function 'AeroFEst'
 *
 */

#ifndef AEROFEST_H
#define AEROFEST_H

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
extern void AeroFEst(const real_T eul[3], const real_T x_dot[3], const real_T
                     omega[3], real_T Fa[3]);

#endif

/* End of code generation (AeroFEst.h) */
