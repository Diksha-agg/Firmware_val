/*
 * File: time_trajj.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 22-Jun-2020 09:41:24
 */

/* Include Files */
#include "rt_nonfinite.h"
#include <math.h>
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
  /*posd[0] = 0.0;
  posd[1] = 0.0;
if(t<=5)
{  posd[2] = 5;
}
else
{posd[2]=5;
}
  veld[0] = 0.0;
  veld[1] = 0.0;
  veld[2] = 0;
  rot_des[0] = 0;
  rot_des[1] = 0;
  rot_des[2] = 0.0;
  omegad[0] = 0.0;
  omegad[1] = 0;
  omegad[2] = 0.0;
  controld[0] = 0;
  controld[1] = 0.0;*/
//double tc;
//double t_max = 20.0;
//if (t<t_max)
//{//t=t/t_max;
//posd[2]=10*(t/t_max);
//veld[2]=10*(t/t_max);
//posd[2]=t/t_max;
/*}
else
{//t=1;
posd[2]=10;
//veld[2]=10;
}*/

//t = max(0, min(t, t_max));
//t = t/t_max;

//posd[0] = 10*pow(tc,3) - 15*pow(tc,4) + 6*pow(tc,5);
if(t<5)
{posd[2]=t;}
else
{posd[2]=5;}

posd[0]=0;
posd[1]=0;

//posd[2]=5;
//posd[1] = 10*pow(tc,3) - 15*pow(tc,4) + 6*pow(tc,5);
//posd[2]=10*pow(1,t);
//posd[2] = 10*pow(3,tc) - 15*pow(4,tc) + 6*pow(5,tc);
//veld[0] = (30/t_max)*pow(tc,2) - (60/t_max)*pow(tc,3) + (30/t_max)*pow(tc,4);
//veld[1] = (30/t_max)*pow(tc,2) - (60/t_max)*pow(tc,3) + (30/t_max)*pow(tc,4);
veld[0]=0;
veld[1]=0;
veld[2]=0;
//veld[2] = (30/t_max)*pow(2,tc) - (60/t_max)*pow(3,tc) + (30/t_max)*pow(4,tc);
//veld[2]=0;
//veld2=6/5t-6/25t^2 should work to get the vehicle at 5m in 5sec
rot_des[0] = 0;
  rot_des[1] = 0;
  rot_des[2] = 0.0;
  omegad[0] = 0.0;
  omegad[1] = 0;
  omegad[2] = 0.0;

controld[0] = 0.0;

  //controld[0] = 40*tc*1.5;
  controld[1] = 0.0;
}

/*
 * File trailer for time_trajj.c
 *
 * [EOF]
 */
