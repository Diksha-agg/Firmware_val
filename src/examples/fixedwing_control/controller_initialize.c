/*
 * File: controller_initialize.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "controller.h"
#include "controller_initialize.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void controller_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/*
 * File trailer for controller_initialize.c
 *
 * [EOF]
 */
