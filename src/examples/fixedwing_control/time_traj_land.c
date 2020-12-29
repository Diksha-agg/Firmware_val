/*
 * File: time_trajj.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
#include <math.h>
static double back_t=0;
static double back_z=0;
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
void time_traj_land(double t, double posd[3], double veld[3], double rot_des[3],
                double omegad[3], double controld[2])
{
double t1;
t1=t-back_t;
posd[0]=0;
posd[2]=(back_z*(5-t1))/(5);
if(posd[2]<0)
{posd[2]=0;}
posd[1]=0;

veld[0]=0.0;
veld[1]=0;
veld[2]=0;

rot_des[0] = 0;
  rot_des[1] = 0.0;
  rot_des[2] = 0.0;
  omegad[0] = 0.0;
  omegad[1] = 0;
  omegad[2] = 0.0;

controld[0] = 0.0;


  controld[1] = 0.0;
}

/*
 * File trailer for time_trajj.c
 *
 * [EOF]
 */
