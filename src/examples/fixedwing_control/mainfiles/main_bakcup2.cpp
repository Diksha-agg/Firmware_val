/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file main.c
 *
 * Example implementation of a fixed wing attitude controller. This file is a complete
 * fixed wing controller for manual attitude control or auto waypoint control.
 * There is no need to touch any other system components to extend / modify the
 * complete control architecture.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */


#include "params.h"
#include <matrix/matrix/math.hpp>
//#include <../build/px4_fmu-v2_default/NuttX/nuttx/include/nuttx/lib/math.h>       /* sin */
#include <poll.h>
#include "norm.h"

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/uORB.h>
//#include "px4_sim1/codegen/lib/controller/controller.h"
//#include "controller.h"
/* Include Files */
#include "rt_nonfinite.h"
//#include "controller.h"

#include "AeroMEst.c"

#include "AeroFEst.c"

#include "time_trajj.c"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"

#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "controller_types.h"


using namespace matrix;

vehicle_local_position_s _local_pos{};			/**< vehicle local position */
struct vehicle_attitude_s _v_att {};				/**< vehicle attitude */
struct vehicle_angular_velocity_s angular_velocity{};		/**< vehicle angular velocity */
struct actuator_controls_s actuators{};		/**< actuators */

extern "C" __EXPORT int ex_fixedwing_control_main(int argc, char *argv[]);
//extern double rotd[3],omegad[3], controld[2];
/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

static void usage(const char *reason);


static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */

static int deamon_task;				/**< Handle of deamon task / thread */


static void controller(const double posc[3], const double velc[3], const double rotc[3], const double omegac[3], double t, double M[3], double *Thrust);
static void controller(const double posc[3], const double velc[3], const double rotc[3],
                const double omegac[3], double t, double M[3], double *Thrust)
{
  double b3[3];
  double c2[3];
  double tau_a[3];
  double b1[3];
  double controld[2];
  double Rd[9];
  double b_omegac[3];
  double R[9];
  double b_b1[3];
  int i;
  double dv0[3];
  double omega_curr[3];
  double omega_des[3];
  int i0;
  double b_velc[3];
  double Fa[3];
  double b_R[9];
  double dv1[3];
  double dv2[3];
  double b_c2[3];
  double dv3[9];
  double acc_net[3];
  double y;
  double c_c2[3];
  static const double dv4[3] = { 0.0, 0.0, 9.81 };

  static const signed char a[9] = { 50, 0, 0, 0, 6, 0, 0, 0, 5 };

  static const signed char b_a[9] = { 5, 0, 0, 0, 5, 0, 0, 0, 50 };

  double b_Rd[9];
  double c_R[9];
  double erm[9];
  int i1;
  double dv5[3];
  double c_velc[3];
  double b_erm[3];
  double dv6[3];
  double b_omega_curr[3];
  double d_c2[3];
  static const double c_a[9] = { 1.86, 0.0, 0.0, 0.0, 2.031, 0.0, 0.0, 0.0,
    3.617 };

  static const signed char d_a[9] = { 3, 0, 0, 0, 100, 0, 0, 0, 5 };

  static const short e_a[9] = { 5, 0, 0, 0, 1500, 0, 0, 0, 1 };

  /* desired inputs:[xd yd zd xd_dot yd_dot zd_dot phid thetad psid phidotd thetadotd psidotd ud(1)---Tfwd */
  /* ud(2)---Mfwd */
  time_trajj(t, b3, c2, tau_a, b1, controld);

  /* kg */
  Rd[0] = 1.0;
  Rd[3] = 0.0;
  Rd[6] = -sinf(rotc[1]);  //understood sinf from controlmath.cpp in position control 
  Rd[1] = 0.0;
  Rd[4] = cosf(rotc[2]);
  Rd[7] = cosf(rotc[1]) * sinf(rotc[2]);
  Rd[2] = 0.0;
  Rd[5] = -sinf(rotc[2]);
  Rd[8] = cosf(rotc[1]) * cosf(rotc[2]);
  b_omegac[0] = omegac[2];
  b_omegac[1] = omegac[1];
  b_omegac[2] = omegac[0];
  R[0] = 1.0;
  R[3] = 0.0;
  R[6] = -sinf(tau_a[1]);
  R[1] = 0.0;
  R[4] = cosf(tau_a[2]);
  R[7] = cosf(tau_a[1]) * sinf(tau_a[2]);
  R[2] = 0.0;
  R[5] = -sinf(tau_a[2]);
  R[8] = cosf(tau_a[1]) * cosf(tau_a[2]);
  b_b1[0] = b1[2];
  b_b1[1] = b1[1];
  b_b1[2] = b1[0];
  for (i = 0; i < 3; i++) {
    omega_curr[i] = 0.0;
    omega_des[i] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      omega_curr[i] += Rd[i + 3 * i0] * b_omegac[i0];
      omega_des[i] += R[i + 3 * i0] * b_b1[i0];
    }
  }

  dv0[0] = 0.0;
  dv0[1] = rotc[1];
  dv0[2] = 0.0;
  b_velc[0] = velc[0];
  b_velc[1] = 0.0;
  b_velc[2] = velc[2];
  AeroFEst(dv0, b_velc, omega_curr, Fa);
  b_R[0] = cosf(tau_a[1]) * cosf(tau_a[0]);
  b_R[3] = -cosf(tau_a[2]) * sinf(tau_a[0]) + sinf(tau_a[2]) * sinf(tau_a[1]) * cosf
    (tau_a[0]);
  b_R[6] = sinf(tau_a[2]) * sinf(tau_a[0]) + cosf(tau_a[2]) * sinf(tau_a[1]) * cosf
    (tau_a[0]);
  b_R[1] = cosf(tau_a[1]) * sinf(tau_a[0]);
  b_R[4] = cosf(tau_a[2]) * cosf(tau_a[0]) + sinf(tau_a[2]) * sinf(tau_a[1]) * sinf
    (tau_a[0]);
  b_R[7] = -sinf(tau_a[2]) * cosf(tau_a[0]) + cosf(tau_a[2]) * sinf(tau_a[1]) * sinf
    (tau_a[0]);
  b_R[2] = -sinf(tau_a[1]);
  b_R[5] = sinf(tau_a[2]) * cosf(tau_a[1]);
  b_R[8] = cosf(tau_a[2]) * cosf(tau_a[1]);
  dv1[0] = 0.0;
  dv1[1] = 0.0;
  dv1[2] = controld[0];
  dv2[0] = 0.0;
  dv2[1] = 0.0;
  dv2[2] = b3[2] - posc[2];
  b_c2[0] = c2[0] - velc[0];
  b_c2[1] = -velc[1];
  b_c2[2] = c2[2] - velc[2];
  dv3[0] = cosf(rotc[1]) * cosf(rotc[0]);
  dv3[3] = -cosf(rotc[2]) * sinf(rotc[0]) + sinf(rotc[2]) * sinf(rotc[1]) * cosf
    (rotc[0]);
  dv3[6] = sinf(rotc[2]) * sinf(rotc[0]) + cosf(rotc[2]) * sinf(rotc[1]) * cosf(rotc
    [0]);
  dv3[1] = cosf(rotc[1]) * sinf(rotc[0]);
  dv3[4] = cosf(rotc[2]) * cosf(rotc[0]) + sinf(rotc[2]) * sinf(rotc[1]) * sinf(rotc
    [0]);
  dv3[7] = -sinf(rotc[2]) * cosf(rotc[0]) + cosf(rotc[2]) * sinf(rotc[1]) * sinf
    (rotc[0]);
  dv3[2] = -sinf(rotc[1]);
  dv3[5] = sinf(rotc[2]) * cosf(rotc[1]);
  dv3[8] = cosf(rotc[2]) * cosf(rotc[1]);
  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += b_R[i + 3 * i0] * dv1[i0];
    }

    b_b1[i] = y / 12.0;
    b_velc[i] = 0.0;
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += (double)a[i + 3 * i0] * b_c2[i0];
      b_velc[i] += (double)b_a[i + 3 * i0] * dv2[i0];
    }

    dv0[i] = ((b_b1[i] + b_velc[i]) + y) + dv4[i];
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += dv3[i + 3 * i0] * Fa[i0];
    }

    b_omegac[i] = y / 12.0;
    acc_net[i] = dv0[i] - b_omegac[i];
  }

  /*  acc_net */
  /*  calculation of current euler angles */
  y = norm(acc_net);
  for (i = 0; i < 3; i++) {
    b3[i] = acc_net[i] / y;
  }

  c2[0] = -sinf(tau_a[0]);
  c2[1] = cosf(tau_a[0]);
  c_c2[0] = c2[1] * b3[2] - 0.0 * b3[1];
  c_c2[1] = 0.0 * b3[0] - c2[0] * b3[2];
  c_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];
  y = norm(c_c2);
  b1[0] = (c2[1] * b3[2] - 0.0 * b3[1]) / y;
  b1[1] = (0.0 * b3[0] - c2[0] * b3[2]) / y;
  b1[2] = (c2[0] * b3[1] - c2[1] * b3[0]) / y;
  b_Rd[3] = b3[1] * b1[2] - b3[2] * b1[1];
  b_Rd[4] = b3[2] * b1[0] - b3[0] * b1[2];
  b_Rd[5] = b3[0] * b1[1] - b3[1] * b1[0];
  for (i = 0; i < 3; i++) {
    b_Rd[i] = b1[i];
    b_Rd[6 + i] = b3[i];
  }

  /* thrust control input */
  /* flag=sign([0 0 1]*Rb'*acc_net); */
  *Thrust = 12.0 * norm(acc_net);
  c_R[0] = cosf(rotc[1]) * cosf(rotc[0]);
  c_R[3] = -cosf(rotc[2]) * sinf(rotc[0]) + sinf(rotc[2]) * sinf(rotc[1]) * cosf
    (rotc[0]);
  c_R[6] = sinf(rotc[2]) * sinf(rotc[0]) + cosf(rotc[2]) * sinf(rotc[1]) * cosf(rotc
    [0]);
  c_R[1] = cosf(rotc[1]) * sinf(rotc[0]);
  c_R[4] = cosf(rotc[2]) * cosf(rotc[0]) + sinf(rotc[2]) * sinf(rotc[1]) * sinf(rotc
    [0]);
  c_R[7] = -sinf(rotc[2]) * cosf(rotc[0]) + cosf(rotc[2]) * sinf(rotc[1]) * sinf
    (rotc[0]);
  c_R[2] = -sinf(rotc[1]);
  c_R[5] = sinf(rotc[2]) * cosf(rotc[1]);
  c_R[8] = cosf(rotc[2]) * cosf(rotc[1]);
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      Rd[i + 3 * i0] = 0.0;
      R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        Rd[i + 3 * i0] += b_Rd[i1 + 3 * i] * c_R[i1 + 3 * i0];
        R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      erm[i0 + 3 * i] = 0.5 * (Rd[i0 + 3 * i] - R[i0 + 3 * i]);
      b_R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }

      y += b_R[i + 3 * i0] * omega_des[i0];
    }

    c2[i] = omega_curr[i] - y;
  }

  dv5[0] = 0.0;
  dv5[1] = rotc[1];
  dv5[2] = 0.0;
  c_velc[0] = velc[0];
  c_velc[1] = 0.0;
  c_velc[2] = velc[2];
  AeroMEst(dv5, c_velc, omega_curr, Fa, tau_a);
  b_erm[0] = erm[5];
  b_erm[1] = erm[6];
  b_erm[2] = erm[1];
  dv6[0] = 0.0;
  dv6[1] = controld[1];
  dv6[2] = 0.0;
  for (i = 0; i < 3; i++) {
    b1[i] = 0.0;
    b3[i] = 0.0;
    b_velc[i] = 0.0;
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      b1[i] += c_a[i + 3 * i0] * omega_curr[i0];
      R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }

      b3[i] += R[i + 3 * i0] * omega_des[i0];
      b_velc[i] += (double)d_a[i + 3 * i0] * b_erm[i0];
      y += (double)e_a[i + 3 * i0] * c2[i0];
    }

    dv0[i] = (dv6[i] - b_velc[i]) - y;
  }

  b_omega_curr[0] = omega_curr[1] * b1[2] - omega_curr[2] * b1[1];
  b_omega_curr[1] = omega_curr[2] * b1[0] - omega_curr[0] * b1[2];
  b_omega_curr[2] = omega_curr[0] * b1[1] - omega_curr[1] * b1[0];
  d_c2[0] = c2[1] * b3[2] - c2[2] * b3[1];
  d_c2[1] = c2[2] * b3[0] - c2[0] * b3[2];
  d_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];
  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += c_a[i + 3 * i0] * d_c2[i0];
    }

    M[i] = ((dv0[i] + b_omega_curr[i]) - y) - tau_a[i];
  }

  /* moment input */
}

/*
 * File trailer for controller.c
 *
 * [EOF]
 */

/*controller produces the control inputs thrust and moments and gets value from time traj as well as local position*/


//unsigned
int fixedwing_control_thread_main(int argc, char *argv[])
{

	//struct vehicle_local_position_s _local_pos_s;
	memset(&_local_pos, 0, sizeof(_local_pos));
	//struct vehicle_attitude_s _v_att_s;
	memset(&_v_att, 0, sizeof(_v_att));
	//struct vehicle_angular_velocity_s angular_velocity_s;
	memset(&angular_velocity, 0, sizeof(angular_velocity));
	

	/* output structs - this is what is sent to the mixer */
	//struct actuator_controls_s actuators_p;
	memset(&actuators, 0, sizeof(actuators));


	/* publish actuator controls with zero values */

	for (int i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}



/*
	 * Advertise that this controller will publish actuator
	 * control values 
	 */
	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	
	/* subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	//int local_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	//int omega_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	/* Setup of loop */

	struct pollfd fds[1] {};
	fds[0].fd = att_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {

		/*
		 * Wait for a sensor or param update, check for exit condition every 500 ms.
		 * This means that the execution will block here without consuming any resources,
		 * but will continue to execute the very moment a new attitude measurement or
		 * a param update is published. So no latency in contrast to the polling
		 * design pattern (do not confuse the poll() system call with polling).
		 *
		 * This design pattern makes the controller also agnostic of the attitude
		 * update speed - it runs as fast as the attitude updates with minimal latency.
		 */
		int ret = poll(fds, 1, 500);

		if (ret < 0) {
			/*
			 * Poll error, this will not really happen in practice,
			 * but its good design practice to make output an error message.
			 */
			warnx("poll error");

		} else if (ret == 0) {
			/* no return value = nothing changed for 500 ms, ignore */
		} else {


			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {


				/* Check if there is a new position measurement or position setpoint */
				bool pos_updated;
				orb_check(global_pos_sub, &pos_updated);

				// get a local copy of attitude 
				//orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

				if (pos_updated) {

					double t=_local_pos.timestamp;

					const double posc[3] = {_local_pos.x, _local_pos.y, _local_pos.z};
					const double velc[3] = {_local_pos.vx, _local_pos.vy, _local_pos.vz};
					const double rotc[3]= 										{ Eulerf(Quatf(_v_att.q)).psi(),Eulerf(Quatf(_v_att.q)).theta(),Eulerf(Quatf(_v_att.q)).phi()};
					
					const double omegac[3]={angular_velocity.xyz[0],angular_velocity.xyz[1],angular_velocity.xyz[2]};
					double Thrust;
					double M[3];
					
					controller(posc, velc, rotc, omegac, t, M, &Thrust);
					//controller(posc, velc, rotc, omegac, t, M, Thrust);
					/***
					actuators.control[0]=M[0];
					actuators.control[1]=M[1];
					actuators.control[2]=M[2];
					actuators.control[3]=Thrust;**/

				}

				
				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

					/*if (verbose) {
						warnx("published");
					}*/
				}
			}
		}
	}

	printf("[ex_fixedwing_control] exiting, stopping all motors.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	fflush(stdout);

	return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: ex_fixedwing_control {start|stop|status}\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int ex_fixedwing_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("ex_fixedwing_control already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("ex_fixedwing_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 fixedwing_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tex_fixedwing_control is running\n");

		} else {
			printf("\tex_fixedwing_control not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}

