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

#include <poll.h>

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
//#include "px4_sim/codegen/lib/run_sim/controller.c"


using namespace matrix;

vehicle_local_position_s _local_pos{};			/**< vehicle local position */
struct vehicle_attitude_s _v_att {};				/**< vehicle attitude */
struct vehicle_angular_velocity_s angular_velocity{};		/**< vehicle angular velocity */
struct actuator_controls_s actuators{};		/**< actuators */

extern "C" __EXPORT int ex_fixedwing_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

static bool thread_should_exit = false;		/**< Daemon exit flag */



double t=_local_pos.timestamp;
double posd[3] ={0};
double veld[3];
/*timetrajj generates desired trajactory*/
//float t;
	
//t=&_local_pos->timestamp;

extern void time_trajj(double t, double posd[3], double veld[3]);

//extern "C" __EXPORT void time_trajj(double t, double posd[3], double veld[3]);


/*controller produces the control inputs thrust and moments and gets value from time traj as well as local position*/
//extern void controller(const double posc[3], const double velc[3], const double rotc[3], const double omegac[3], const double posd[3], const double veld[3], const double rotd[3], const double omegad[3], const double controld[2], double *F, double M[3])

extern void controller(const double posc[3], const double velc[3], const double rotc[3], const double omegac[3], const double posd[3], double veld[3],const double rotd[3], const double omegad[3], const double controld[2], double *Thrust, double M[3]);
/*
{
	actuators.control[0]=M[1];
	actuators.control[1]=M[2];
	actuators.control[2]=M[3];
	actuators.control[3]=*Thrust;
}*/
float posc[3] = {_local_pos.x, _local_pos.y, _local_pos.z};
float velc[3] = {_local_pos.vx, _local_pos.vy, _local_pos.vz};
float rotc[3] = { Eulerf(Quatf(_v_att.q)).psi(),Eulerf(Quatf(_v_att.q)).theta(),Eulerf(Quatf(_v_att.q)).phi()};

float omegac[3]={angular_velocity.xyz[1],angular_velocity.xyz[2],angular_velocity.xyz[3]};

//unsigned
int fixedwing_control_thread_main(int argc, char *argv[])
{

	struct vehicle_local_position_s _local_pos_s;
	memset(&_local_pos_s, 0, sizeof(_local_pos_s));
	struct vehicle_attitude_s _v_att_s;
	memset(&_v_att_s, 0, sizeof(_v_att_s));
	struct vehicle_angular_velocity_s angular_velocity_s;
	memset(&angular_velocity, 0, sizeof(angular_velocity));
	

	/* output structs - this is what is sent to the mixer */
	struct actuator_controls_s actuators_p;
	memset(&actuators_p, 0, sizeof(actuators_p));


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
	int local_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int omega_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
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

			// check for parameter updates
			/*if (parameter_update_sub.updated()) {
				// clear update
				parameter_update_s pupdate;
				parameter_update_sub.copy(&pupdate);

				// if a param update occured, re-read our parameters
				parameters_update(&ph, &p);
			} */

			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {


				/* Check if there is a new position measurement or position setpoint */
				bool pos_updated;
				orb_check(global_pos_sub, &pos_updated);
				/*bool global_sp_updated;
				orb_check(global_sp_sub, &global_sp_updated);
				bool manual_sp_updated;
				orb_check(manual_sp_sub, &manual_sp_updated);*/

				// get a local copy of attitude 
				//orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

				if (pos_updated) {

					//struct position_setpoint_triplet_s triplet;
					//orb_copy(ORB_ID(position_setpoint_triplet), global_sp_sub, &triplet);
					//memcpy(&global_sp, &triplet.current, sizeof(global_sp));

					//double t=_local_pos.timestamp;
					//float posc[3] = {_local_pos_s.x, _local_pos_s.y, _local_pos_s.z};
					time_trajj(t, posd, veld);
					controller(posc[3], velc[3], rotc[3], omegac[3], posd[3], veld[3],
 					const double rotd[3], const double omegad[3], const double controld[2], double
  					*Thrust, double M[3]);

				}

				
				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

					if (verbose) {
						warnx("published");
					}
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

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, actuators{});

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

