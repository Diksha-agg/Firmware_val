/*
 * File: run_sim.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 17-Jun-2020 15:56:16
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "run_sim.h"
#include "controller.h"
#include "local.h"
#include "time_trajj.h"

/* Function Definitions */

/*
 * Arguments    : double F_traj[400]
 *                double M_traj[1200]
 * Return Type  : void
 */
void run_sim(double F_traj[400], double M_traj[1200])
{
  boolean_T b0;
  double time;
  int iter;
  double posd[3];
  double veld[3];
  double posc[3];
  double velc[3];
  double dv0[3];
  double dv1[3];
  double dv2[3];
  double dv3[3];
  double dv4[2];
  double Thrust;
  double M[3];
  int i;
  b0 = false;

  /*  this determines the time step at which the solution is given */
  /*  max iteration */
  time = 0.0;
  for (iter = 0; iter < 400; iter++) {
    time_trajj(time, posd, veld);
    local(time, posc, velc);
    if (!b0) {
      for (i = 0; i < 3; i++) {
        dv0[i] = 0.0;
        dv1[i] = 0.0;
        dv2[i] = 0.0;
        dv3[i] = 0.0;
      }

      for (i = 0; i < 2; i++) {
        dv4[i] = 0.0;
      }

      b0 = true;
    }

    controller(posc, velc, dv0, dv1, posd, veld, dv2, dv3, dv4, &Thrust, M);
    F_traj[iter] = Thrust;
    for (i = 0; i < 3; i++) {
      M_traj[iter + 400 * i] = M[i];
    }

    time += 0.01;
  }
}

/*
 * File trailer for run_sim.c
 *
 * [EOF]
 */
