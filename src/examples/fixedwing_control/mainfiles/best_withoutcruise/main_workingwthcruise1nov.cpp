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
#include <commander/px4_custom_mode.h>
//#include <../Tools/sitl_gazebo/src/liftdrag_plugin/liftdrag_plugin.cpp>
#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
//#include <lib/flight_tasks/FlightTasks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/weather_vane/WeatherVane.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/hover_thrust_estimate.h>

//#include <px4_config.h>
//#include <px4_platform_common/px4_tasks.h>
//#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
//#include "interpl.h"


#include "params.h"
#include <matrix/matrix/math.hpp>
//#include <../build/px4_fmu-v2_default/NuttX/nuttx/include/nuttx/lib/math.h>       /* sin */
#include <poll.h>
#include "norm.h"
#include <px4_log.h>
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
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
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
#include "time_traj_fort.c"
#include "time_traj_cruise.c"
#include "time_traj_back.c"
#include "time_traj_land.c"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_defines.h"

#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "controller_types.h"
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
using namespace matrix;

int trans=1;

double hover_z=0;
double hover_x=0;
double hover_y=0;
vehicle_local_position_s _local_pos{};			/**< vehicle local position */
struct vehicle_local_position_setpoint_s _local_pos_setp;
struct vehicle_attitude_s _v_att {};				/**< vehicle attitude */
struct vehicle_angular_velocity_s angular_velocity{};		/**< vehicle angular velocity */
struct actuator_controls_s actuators{};		/**< actuators */
vehicle_attitude_setpoint_s _att_set{};
vehicle_rates_setpoint_s _rate_set{};
struct debug_key_value_s dbg{};
vehicle_status_s status{};
vehicle_command_s vcom{};
struct debug_vect_s dbg_vect;


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
  //int flag;
  double b3[3];
  double posd[3];
  double c2[3];
  double veld[3];
  double rotd[3];
  double tau_a[3];
  double BQ_m = 0.057;
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
  double b_R[9];		//R multiplied with [0 0 u(1)]t
  double dv1[3];
  double dv2[3];
  double b_c2[3];
  double dv3[9];		//Rb multiplied by Fa
  double acc_net[3];
  double y;
  double c_c2[3];
  static double dv4[3] = { 0.0, 0.0, 9.81 };  //gravity

  static double a[9] = { 0.1, 0, 0, 0, .1, 0, 0, 0, 0.1 };  // kv derivative v

  static double b_a[9] = { .01, 0, 0, 0, .01, 0, 0, 0, 0.01 }; // kp proportional x

  double b_Rd[9];
  double c_R[9];		//R used in angle cal
  double erm[9];
  int i1;
  //double acc;
  //double dv5[3];
  //double c_velc[3];
  double b_erm[3];
  double dv6[3];
  double b_omega_curr[3];
  double d_c2[3];


  static const double c_a[9] = { 1.86, 0.0, 0.0, 0.0, 2.031, 0.0, 0.0, 0.0, 3.617 };		//as per code animesh
  //static const double c_a[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };		//as per code animesh
//static const double c_a[9] = {0.147563, 0.0, 0.0, 0.0, 0.0458929, 0.0, 0.0, 0.0, 0.1977 };		//tailsitter J

 //static const double c_a[9] = { 0.887921, 0.00073908, -7.96652e-07, 0.00073908, 0.552353, 2.99865e-06, -7.96652e-07, 2.99865e-06, 1.35587 };//j for biplane
 static double d_a[9] = { 0.07, 0, 0, 0, .07, 0, 0, 0, .06 }; // kr error in rotation

  static double e_a[9] = { .07 , 0, 0, 0, .07 , 0, 0, 0, .05  }; //kw error in omega

  /* desired inputs:[xd yd zd xd_dot yd_dot zd_dot phid thetad psid phidotd thetadotd psidotd ud(1)---Tfwd */
  /* ud(2)---Mfwd */
  //t=_local_pos.timestamp;


if (t>1 && trans ==1 && posc[2]>5 && velc[0]<0.05 && velc[1]<0.05 && velc[2]<0.9 && fabs(rotc[0])<0.02 && fabs(rotc[1])<0.02 && fabs(rotc[2])<0.03 && omegac[0]<0.1 && omegac[1]<0.1&& omegac[2]<0.1)
              { hover_t=t;
                hover_z=-posc[2];
                hover_x=posc[0];//ch
                hover_y=posc[1];//ch
                //hover_y=0;//ch
                //hover_x=0;//ch
                trans=2;
	            	warnx("hover finished");
                PX4_INFO("value of current height is %f", (double)hover_z);
                PX4_INFO("value of time is %f", (double)t);
                PX4_INFO("value of current y is %f", (double)hover_y);
                PX4_INFO("value of current x is %f", (double)hover_x);
                PX4_INFO("value of current vx is %f", (double)velc[0]);
                PX4_INFO("value of current yaw is %f", (double)rotc[2]);
                PX4_INFO("value of current pitch is %f", (double)rotc[1]);
                PX4_INFO("value of current roll is %f", (double)rotc[0]);

                }

    else if(t>(hover_t+Tff) && trans==2)
	      { trans=3;
                hover_x=posc[0];
                //hover_y=0;//ch
                hover_y=posc[1];//ch
		            fort_t=t;
		           // cruise_t=t;
		            hover_z=-posc[2];
		            warnx("forward transition finished");
                PX4_INFO("value of current height is %f", (double)hover_z);
                PX4_INFO("value of current y is %f", (double)hover_y);
                PX4_INFO("value of current x is %f", (double)hover_x);
                PX4_INFO("value of current vx is %f", (double)velc[0]);
                PX4_INFO("value of current yaw is %f", (double)rotc[2]);
                PX4_INFO("value of current pitch is %f", (double)rotc[1]);
                PX4_INFO("value of current roll is %f", (double)rotc[0]);
                }
    //else if(t>1 && trans ==3 && velc[0]>19.9 && velc[0]<20 && velc[1]<0.05 && velc[2]<0.05 && rotc[0]<0.05 && rotc[1]>1.5 && rotc[1]<1.539 && rotc[2]<0.05 && omegac[0]<0.05 && omegac[1]<0.05&& omegac[2]<0.05)
	else if (t>(1) && trans==3 && velc[0]>19 && velc[0]<20)
		{ trans=4;
                hover_x=posc[0];
                hover_y=posc[1];//ch
                //hover_y=0;//ch
		            cruise_t=t;
		            hover_z=-posc[2];
		            warnx("cruise finished");
                PX4_INFO("value of current height is %f", (double)hover_z);
                PX4_INFO("value of current y is %f", (double)hover_y);
                PX4_INFO("value of current x is %f", (double)hover_x);
                PX4_INFO("value of current vx is %f", (double)velc[0]);
                PX4_INFO("value of current yaw is %f", (double)rotc[2]);
                PX4_INFO("value of current pitch is %f", (double)rotc[1]);
                PX4_INFO("value of current roll is %f", (double)rotc[0]);
                }
	else if (t>(cruise_t+Tbf) && trans==4)
		{ trans=5;
                hover_x=posc[0];//ch
                hover_y=posc[1];//ch
		            back_t=t;
		            back_z=posc[2];
		            hover_z=-0;
		            warnx("back trans finished");
                PX4_INFO("value of current height is %f", (double)posc[2]);
                PX4_INFO("value of current x is %f", (double)posc[0]);
                PX4_INFO("value of current y is %f", (double)hover_y);
                PX4_INFO("value of current vx is %f", (double)velc[0]);
                PX4_INFO("value of current yaw is %f", (double)rotc[2]);
                PX4_INFO("value of current pitch is %f", (double)rotc[1]);
                PX4_INFO("value of current roll is %f", (double)rotc[0]);
                }


  if(trans==1)
	  {time_trajj(t, posd, veld, rotd, b1, controld);
   /*  double kv[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 0 };  // kv derivative v
     double kp[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 100  }; // kp proportional x
     double kr[9] = {1, 0, 0, 0, 2, 0, 0, 0, 0.1}; // kr error in rotation 0.04 .06
     double kw[9] = {2 , 0, 0, 0, 5 , 0, 0, 0, 0.1}; //kw error in omega 0.04 .05*/

     /*double kv[9] = { 15, 0, 0, 0, 15, 0, 0, 0, 30 };  // kv derivative v
     double kp[9] = { 10, 0, 0, 0, 1, 0, 0, 0, 40  }; // kp proportional x //latest
     double kr[9] = {3, 0, 0, 0, 15, 0, 0, 0, 15}; // kr error in rotation 0.04 .06
     double kw[9] = {5 , 0, 0, 0, 60 , 0, 0, 0, 30}; //kw error in omega 0.04 .05*/ 

    /* double kv[9] = { 15, 0, 0, 0, 15, 0, 0, 0, 30 };  // kv derivative v
     double kp[9] = { 10, 0, 0, 0, 1, 0, 0, 0, 40  }; // kp proportional x
     double kr[9] = {3, 0, 0, 0, 15, 0, 0, 0, 15}; // kr error in rotation 0.04 .06
     double kw[9] = {5 , 0, 0, 0, 60 , 0, 0, 0, 30}; //kw error in omega 0.04 .05

		memcpy(a, kv, sizeof(kv));
		memcpy(b_a, kp, sizeof(kp));
		memcpy(d_a, kr, sizeof(kr));
		memcpy(e_a, kv, sizeof(kw));*/
    }
  else if(trans == 2)
	  {
        time_traj_fort(t, posd, veld, rotd, b1, controld);
     		/*a[9] = { 0.1, 0, 0, 0, .1, 0, 0, 0, 0.1 };  // kv derivative v
     		b_a[9] = { .01, 0, 0, 0, .01, 0, 0, 0, 0.01 }; // kp proportional x		for hover
     		d_a[9] = { 0.07, 0, 0, 0, .07, 0, 0, 0, .06 }; // kr error in rotation
     		e_a[9] = { .07 , 0, 0, 0, .07 , 0, 0, 0, .05  }; //kw error in omega*/
     		/*double kv[9] = { 500, 0, 0, 0, 500, 0, 0, 0, 30 };  // kv derivative v210 210 30
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 10 }; // kp proportional x		//fort
     		double kr[9] = { 1, 0, 0, 0, 110, 0, 0, 0, 1 }; // kr error in rotation
     		double kw[9] = { 1, 0, 0, 0, 110 , 0, 0, 0, 1  }; //kw error in omega 150*/

     		/*double kv[9] = { 210, 0, 0, 0, 210, 0, 0, 0, 30 };  // kv derivative v210 210 30
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 10 }; // kp proportional x		//fort
     		double kr[9] = { 1, 0, 0, 0, 110, 0, 0, 0, 1 }; // kr error in rotation
     		double kw[9] = { 1, 0, 0, 0, 110 , 0, 0, 0, 1  }; //kw error in omega 150*/

     		/*double kv[9] = {400, 0, 0, 0, 90, 0, 0, 0, 30 };  // kv derivative v210 210 30
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 40 }; // kp proportional x		//fort
     		double kr[9] = { 1, 0, 0, 0, 80, 0, 0, 0, 1 }; // kr error in rotation 7/3 110 3
     		double kw[9] = { 5, 0, 0, 0, 100 , 0, 0, 0, 5  }; //kw error in omega 150 ...14/3 120 14/3*/
     		double kv[9] = {15, 0, 0, 0, 5, 0, 0, 0, 5 };  // kv derivative v210 210 30
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 5 }; // kp proportional x		//fort
     		double kr[9] = { 0.5, 0, 0, 0, .3, 0, 0, 0, 0.5 }; // kr error in rotation 7/3 110 3
     		double kw[9] = { 1, 0, 0, 0, .7 , 0, 0, 0, 1  }; //kw error in omega 150 ...14/3 120 14/3

     		/*double kv[9] = { 350, 0, 0, 0, 350, 0, 0, 0, 0 };  // kv derivative v	giving kv to x and y helped in removing the error of quickly going to 90deg
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 30 }; // kp proportional x		//fort
     		double kr[9] = { 0, 0, 0, 0, 60, 0, 0, 0, 0 }; // kr error in rotation
     		double kw[9] = { 0 , 0, 0, 0, 50 , 0, 0, 0, 0  }; //kw error in omega 10 10 50 0 0 10 1 30 1 1 60 1*/
		    memcpy(a, kv, sizeof(kv));
		    memcpy(b_a, kp, sizeof(kp));
		    memcpy(d_a, kr, sizeof(kr));
		    memcpy(e_a, kv, sizeof(kw));

		}
  else if (trans==3)
	  {
	      time_traj_cruise(t, posd, veld, rotd, b1, controld);
     		/*double kv[9] = { 105, 0, 0, 0, 100, 0, 0, 0, 10 };  // kv derivative v
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 50 }; // kp proportional x
     		double kr[9] = { 0.1, 0, 0, 0, 10, 0, 0, 0, 0.1 }; // kr error in rotation
     		double kw[9] = { 5 , 0, 0, 0, 10 , 0, 0, 0, 0.1 }; //kw error in omega*/
     		double kv[9] = { 60, 0, 0, 0, 5, 0, 0, 0, 10 };  // kv derivative v//15 15 30
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 10 }; // kp proportional x
     		double kr[9] = { 5, 0, 0, 0, 30, 0, 0, 0, 3 }; // kr error in rotation
     		double kw[9] = { 5 , 0, 0, 0, 60 , 0, 0, 0, 5 }; //kw error in omega
		    memcpy(a, kv, sizeof(kv));
		    memcpy(b_a, kp, sizeof(kp));
		    memcpy(d_a, kr, sizeof(kr));
		    memcpy(e_a, kv, sizeof(kw));
	  }
  else if (trans==4)
	  {
	      time_traj_back(t, posd, veld, rotd, b1, controld);
     		/*double kv[9] = { 100, 0, 0, 0, 100, 0, 0, 0, 30 };  // kv derivative v
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 10 }; // kp proportional x
     		double kr[9] = { .1, 0, 0, 0, 30, 0, 0, 0, .1 }; // kr error in rotation
     		double kw[9] = { .10 , 0, 0, 0, 30 , 0, 0, 0, .10 }; //kw error in omega*/
     		double kv[9] = {50, 0, 0, 0, 5, 0, 0, 0, 7 };  // kv derivative v210 210 30
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 5 }; // kp proportional x		//fort
     		double kr[9] = { .5, 0, 0, 0, 1, 0, 0, 0, 1 }; // kr error in rotation 7/3 110 3
     		double kw[9] = { 1, 0, 0, 0, 1 , 0, 0, 0, 1  }; //kw error in omega 150 ...14/3 120 14/3

		    memcpy(a, kv, sizeof(kv));
		    memcpy(b_a, kp, sizeof(kp));
		    memcpy(d_a, kr, sizeof(kr));
		    memcpy(e_a, kv, sizeof(kw));
	  }
  else if (trans==5)
	  {
	      time_traj_land(t, posd, veld, rotd, b1, controld);
     		/*double kv[9] = { 150, 0, 0, 0, 150, 0, 0, 0, 0 };  // kv derivative v
     		double kp[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 10 }; // kp proportional x
     		double kr[9] = { 1, 0, 0, 0, 60, 0, 0, 0, 01 }; // kr error in rotation
     		double kw[9] = { 1 , 0, 0, 0, 50 , 0, 0, 0, 01 }; //kw error in omega
		    memcpy(a, kv, sizeof(kv));
		    memcpy(b_a, kp, sizeof(kp));
		    memcpy(d_a, kr, sizeof(kr));
		    memcpy(e_a, kv, sizeof(kw));*/
	  }

    
       			_local_pos_setp.x=posd[1]+hover_y;
  					_local_pos_setp.y=posd[0]+hover_x;
					  _local_pos_setp.z=-posd[2]+hover_z;
					  _local_pos_setp.vx=veld[1];
					  _local_pos_setp.vy=veld[0];
					  _local_pos_setp.vz=-veld[2];
					_local_pos_setp.timestamp=hrt_absolute_time() ;
  _att_set.roll_body=rotd[0];
  _att_set.pitch_body=rotd[1];
  _att_set.yaw_body=rotd[2];
posd[1]=_local_pos_setp.x;
posd[0]=_local_pos_setp.y;
posd[2]=-_local_pos_setp.z;

  //_att_set.q_d=Eulerf(rotd[0], rotd[1], rotd[2]);
	Quatf q_sp = Eulerf(_att_set.roll_body, -_att_set.pitch_body, _att_set.yaw_body);
	q_sp.copyTo(_att_set.q_d);
//_att_set.q_d=Quatf(Eulerf(_att_set.roll_body, _att_set.pitch_body, _att_set.yaw_body));
  _att_set.timestamp=hrt_absolute_time() ;

  _rate_set.roll=b1[0];
  _rate_set.pitch=-b1[1];
  _rate_set.yaw=b1[2];
  _rate_set.timestamp=hrt_absolute_time() ;
if (trans == 2|| trans == 3 || trans ==4)
{					//start of non linear code

//if (controld[0]>0 || controld[0]<0)
//{dv4[2] = 0.0;		//value of g is 0 if transition
//}
//controld[1]=controld[1]/50;
controld[0]=0;
controld[1]=0;

  /* kg */
  Rd[0] = 1.0;
  Rd[3] = 0.0;
  Rd[6] = -sinf(rotc[1]);  //understood sinf from controlmath.cpp in position control
  Rd[1] = 0.0;
  Rd[4] = cosf(rotc[0]);
  Rd[7] = cosf(rotc[1]) * sinf(rotc[0]);
  Rd[2] = 0.0;
  Rd[5] = -sinf(rotc[0]);
  Rd[8] = cosf(rotc[1]) * cosf(rotc[0]);
  b_omegac[0] = omegac[2];
  b_omegac[1] = omegac[1];
  b_omegac[2] = omegac[0];
  R[0] = 1.0;
  R[3] = 0.0;
  R[6] = -sinf(rotd[1]);
  R[1] = 0.0;
  R[4] = cosf(rotd[0]);
  R[7] = cosf(rotd[1]) * sinf(rotd[0]);
  R[2] = 0.0;
  R[5] = -sinf(rotd[0]);
  R[8] = cosf(rotd[1]) * cosf(rotd[0]);
  b_b1[0] = b1[0];
  b_b1[1] = b1[1];
  b_b1[2] = b1[2];
  for (i = 0; i < 3; i++) {
    omega_curr[i] = omegac[i];
    omega_des[i] = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      //omega_curr[i] += Rd[i + 3 * i0] * b_omegac[i0];				//omaga des in body frame
      omega_des[i] += R[i + 3 * i0] * b_b1[i0];
    }
  }
  dv0[0] = 0.0;
  dv0[1] = rotc[1];
  dv0[2] = 0.0;
  b_velc[0] = velc[0];
  b_velc[1] = 0.0;
  b_velc[2] = velc[2];
  AeroFEst(rotc, velc, omega_curr, Fa);
  Fa[0]=0;
  Fa[1]=0;		//change
  Fa[2]=0;
Fa[1]=0;
  if(trans==1)
  {Fa[0]=0;
  Fa[1]=0;
  Fa[2]=0;}
  b_R[0] = cosf(rotd[1]) * cosf(rotd[2]);
  b_R[3] = -cosf(rotd[0]) * sinf(rotd[2]) + sinf(rotd[1]) * sinf(rotd[0]) * cosf(rotd[2]);
  b_R[6] = sinf(rotd[2]) * sinf(rotd[0]) + cosf(rotd[2]) * sinf(rotd[1]) * cosf(rotd[0]);
  b_R[1] = cosf(rotd[1]) * sinf(rotd[2]);
  b_R[4] = cosf(rotd[2]) * cosf(rotd[0]) + sinf(rotd[2]) * sinf(rotd[1]) * sinf(rotd[0]);
  b_R[7] = -sinf(rotd[0]) * cosf(rotd[2]) + cosf(rotd[0]) * sinf(rotd[1]) * sinf(rotd[2]);
  b_R[2] = -sinf(rotd[1]);
  b_R[5] = sinf(rotd[0]) * cosf(rotd[1]);
  b_R[8] = cosf(rotd[0]) * cosf(rotd[1]);
  dv1[0] = 0.0;
  dv1[1] = 0.0;
  dv1[2] = controld[0];
  dv2[0] = 0.0;
  dv2[1] = 0.0;                     //dv2 is delta p
  dv2[2] = posd[2] - posc[2];
  b_c2[0] = veld[0] - velc[0];      //b_c2 is delta v
  b_c2[1] = veld[1] - velc[1];
  b_c2[2] = veld[2] - velc[2];
  dv3[0] = cosf(rotc[1]) * cosf(rotc[2]);
  dv3[3] = -cosf(rotc[0]) * sinf(rotc[2]) + sinf(rotc[1]) * sinf(rotc[0]) * cosf(rotc[2]);
  dv3[6] = sinf(rotc[2]) * sinf(rotc[0]) + cosf(rotc[2]) * sinf(rotc[1]) * cosf(rotc[0]);
  dv3[1] = cosf(rotc[1]) * sinf(rotc[2]);
  dv3[4] = cosf(rotc[2]) * cosf(rotc[0]) + sinf(rotc[2]) * sinf(rotc[1]) * sinf(rotc[0]);
  dv3[7] = -sinf(rotc[0]) * cosf(rotc[2]) + cosf(rotc[0]) * sinf(rotc[1]) * sinf(rotc[2]);
  dv3[2] = -sinf(rotc[1]);
  dv3[5] = sinf(rotc[0]) * cosf(rotc[1]);
  dv3[8] = cosf(rotc[0]) * cosf(rotc[1]);
  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += b_R[i + 3 * i0] * dv1[i0];       //Ru
    }

    b_b1[i] = y / BQ_m;
    b_velc[i] = 0.0;
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += (double)a[i + 3 * i0] * b_c2[i0];        //kvdelv
      b_velc[i] += (double)b_a[i + 3 * i0] * dv2[i0];     //kpdelx
    }

    dv0[i] = ((b_b1[i] + b_velc[i]) + y)+ dv4[i] ;  //   kvdelv+kpdelx+Ru+g
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += dv3[i + 3 * i0] * Fa[i0];      //Rb*Fa
    }

    b_omegac[i] = y / BQ_m;           //Rb*Fa/m
    acc_net[i] = dv0[i] - b_omegac[i];
  }

  /*  acc_net */
  /*  calculation of current euler angles */
  y = norm(acc_net);
  for (i = 0; i < 3; i++) {
    b3[i] = acc_net[i] / y;
  }

  c2[0] = -sinf(rotd[2]);   //c2
  c2[1] = cosf(rotd[2]);
  c2[2] = 0;
  c_c2[0] = c2[1] * b3[2] - 0.0 * b3[1];		//c2xb3
  c_c2[1] = 0.0 * b3[0] - c2[0] * b3[2];
  c_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];
  y = norm(c_c2);
  b1[0] = (c2[1] * b3[2] - 0.0 * b3[1]) / y;
  b1[1] = (0.0 * b3[0] - c2[0] * b3[2]) / y;
  b1[2] = (c2[0] * b3[1] - c2[1] * b3[0]) / y;
  /*b1[0]=c2[0];
  b1[1]=c2[1];
  b1[2]=c2[2];
  c_c2[0] = b3[2] * b1[1] - b3[1] * b1[2];		//b1xb3
  c_c2[1] = b1[2] * b3[0] - b1[0] * b3[2];
  c_c2[2] = b1[0] * b3[1] - b1[1] * b3[0];
  y = norm(c_c2);*/

  b_Rd[3] = b3[1] * b1[2] - b3[2] * b1[1];		//b_Rd[3,4,5] is b2
  b_Rd[4] = b3[2] * b1[0] - b3[0] * b1[2];
  b_Rd[5] = b3[0] * b1[1] - b3[1] * b1[0];

  /*b_Rd[3] = c_c2[0]/y;					//b_Rd[3,4,5] is b2
  b_Rd[4] = c_c2[1]/y;
  b_Rd[5] = c_c2[2]/y;*/

  for (i = 0; i < 3; i++) {
    b_Rd[i] = b1[i];
    b_Rd[6 + i] = b3[i];				//b_Rd is Rd
  }


  /* thrust control input */

if(acc_net[2]<0)
{acc_net[2]=-acc_net[2];
}

 *Thrust = BQ_m * acc_net[2];

  c_R[0] = cosf(rotc[1]) * cosf(rotc[2]);							//c_R is R in attitude controller
  c_R[3] = -cosf(rotc[0]) * sinf(rotc[2]) + sinf(rotc[1]) * sinf(rotc[0]) * cosf(rotc[2]);
  c_R[6] = sinf(rotc[2]) * sinf(rotc[0]) + cosf(rotc[2]) * sinf(rotc[1]) * cosf(rotc[0]);
  c_R[1] = cosf(rotc[1]) * sinf(rotc[2]);
  c_R[4] = cosf(rotc[2]) * cosf(rotc[0]) + sinf(rotc[2]) * sinf(rotc[1]) * sinf(rotc[0]);
  c_R[7] = -sinf(rotc[0]) * cosf(rotc[2]) + cosf(rotc[0]) * sinf(rotc[1]) * sinf(rotc[2]);
  c_R[2] = -sinf(rotc[1]);
  c_R[5] = sinf(rotc[0]) * cosf(rotc[1]);
  c_R[8] = cosf(rotc[0]) * cosf(rotc[1]);

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
      erm[i0 + 3 * i] = 0.5 * (Rd[i0 + 3 * i] - R[i0 + 3 * i]);		//er=(rdTr-rTrd)v
      b_R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        b_R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }

      y += b_R[i + 3 * i0] * omega_des[i0];
    }

    c2[i] = omega_curr[i] - y;					//c2 is ew
  }

  /*dv5[0] = 0.0;
  dv5[1] = rotc[1];
  dv5[2] = 0.0;
  c_velc[0] = velc[0];
  c_velc[1] = 0.0;
  c_velc[2] = velc[2];*/
  AeroMEst(rotc, velc, omega_curr, Fa, tau_a);
  tau_a[0]=0;		//change
  tau_a[1]=0;
  tau_a[2]=0;
  if(trans==1)
  {
  tau_a[0]=0;
  tau_a[1]=0;
  tau_a[2]=0;}
  b_erm[0] = erm[5];						//b_erm is er
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
      b1[i] += c_a[i + 3 * i0] * omega_curr[i0];		//c_a is J		b1 is J*omegacurr
      R[i + 3 * i0] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        R[i + 3 * i0] += c_R[i1 + 3 * i] * b_Rd[i1 + 3 * i0];
      }

      b3[i] += R[i + 3 * i0] * omega_des[i0];
      b_velc[i] += (double)d_a[i + 3 * i0] * b_erm[i0];		//d_a is kr
      y += (double)e_a[i + 3 * i0] * c2[i0];			//e_a is kw
    }

    dv0[i] = (dv6[i] - b_velc[i]) - y;
  }

  b_omega_curr[0] = omega_curr[1] * b1[2] - omega_curr[2] * b1[1];			//=omegacurr x J*omegacurr
  b_omega_curr[1] = omega_curr[2] * b1[0] - omega_curr[0] * b1[2];
  b_omega_curr[2] = omega_curr[0] * b1[1] - omega_curr[1] * b1[0];
  d_c2[0] = c2[1] * b3[2] - c2[2] * b3[1];
  d_c2[1] = c2[2] * b3[0] - c2[0] * b3[2];
  d_c2[2] = c2[0] * b3[1] - c2[1] * b3[0];					//eω × RT Rd ωd
  for (i = 0; i < 3; i++) {
    y = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      y += c_a[i + 3 * i0] * d_c2[i0];			//c_a is J
    }

    M[i] = ((dv0[i] + b_omega_curr[i]) - y)- tau_a[i];
  if(controld[0]>0 || controld[0]<0)
  {
 // M[0]=0;
  M[1]=M[1];
  //M[2]=0;
  }
//if(M[1]<0){
//M[1]=M[1]/5;
if(rotc[1]>0.78)
{
  M[0]=M[0];
  //M[1]=M[1];
  M[2]=M[2];
}

//M[0]=0;
//M[2]=0;
 /* if (trans==3)
	{// M[0]=0;
  //M[1]=0;
  	  //M[2]=0;
	}*/

					/*dbg_vect.x = acc_net[0]/norm(acc_net);
					dbg_vect.y = acc_net[1]/norm(acc_net);
					dbg_vect.z = acc_net[2]/norm(acc_net);
					dbg_vect.timestamp = hrt_absolute_time();*/

//dbg.value = posc[0];

  }
}//ends the non-linear controller
else if (trans==1)
{

double kdx=5;	//1
double kpx=0;	//1
double kdy=5;	//1
double kpy=0;	//1
double kdz=5;	//3
double kpz=3;	//1


double des_accx=kdx*(veld[0]-velc[0])+kpx*(posd[0]-posc[0]);
double des_accy=kdy*(veld[1]-velc[1])+kpy*(posd[1]-posc[1]);
double des_accz=kdz*(veld[2]-velc[2])+kpz*(posd[2]-posc[2]);
					dbg_vect.x = posd[0];
					dbg_vect.y = posd[1];
					dbg_vect.z = des_accz;
					dbg_vect.timestamp = hrt_absolute_time();

double phides=-des_accy/9.81;
double thetades=des_accx/9.81;//ch
*Thrust = BQ_m*(9.81+des_accz);
if(*Thrust<0)
{*Thrust=-*Thrust;
}

double kpphi=0.1;	//0.1
double kdphi=1;		//1
double kptheta=0.1;	//0.1
double kdtheta=0.6;	//0.1
double kppsi=0.1;	//0.1
double kdpsi=0.1;		//0.1
M[0]=kpphi*(phides-rotc[0])+kdphi*(-omegac[0]);
M[1]=kptheta*(thetades-rotc[1])+kdtheta*(-omegac[1]);
M[2]=(kppsi*(-rotc[2])+kdpsi*(-omegac[2]));
}


else if (trans==5)
{

double kdx=2;
double kpx=1;
double kdy=2;
double kpy=1;
double kdz=2;
double kpz=1;
/*double kdx=5;	//1
double kpx=0;	//1
double kdy=5;	//1
double kpy=0;	//1
double kdz=5;	//3
double kpz=3;	//1*/

double des_accx=kdx*(veld[0]-velc[0])+kpx*(posd[0]-posc[0]);
double des_accy=kdy*(veld[1]-velc[1])+kpy*(posd[1]-posc[1]);
double des_accz=kdz*(veld[2]-velc[2])+kpz*(posd[2]-posc[2]);

double phides=-des_accy/9.81;
double thetades=des_accx/9.81;//ch

*Thrust = BQ_m*(9.81+des_accz);
if(*Thrust<0)
{*Thrust=-*Thrust;
}

double kpphi=2;
double kdphi=5;
double kptheta=2;
double kdtheta=3;
double kppsi=1;
double kdpsi=5;
M[0]=kpphi*(phides-rotc[0])+kdphi*(-omegac[0]);
M[1]=kptheta*(thetades-rotc[1])+kdtheta*(-omegac[1]);
M[2]=(kppsi*(-rotc[2])+kdpsi*(-omegac[2]));
if (posc[2]<0.5)
{*Thrust=0;
M[0]=0;
M[1]=0;
M[2]=0;}

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
	/* read arguments */
	bool verbose = false;
			        warnx("[example fixedwing control] started");
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			warnx("[example fixedwing control] started");
			verbose = true;
		}
	}

	/* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	//warnx("[example fixedwing control] started");

	//struct vehicle_local_position_s _local_pos_s;
	memset(&_local_pos, 0, sizeof(_local_pos));
        //struct vehicle_local_position_setpoint_s _local_pos_setp;			/**< vehicle local position */
	memset(&_local_pos_setp, 0, sizeof(_local_pos_setp));

	//struct vehicle_attitude_setpoint_s _att_set;
	memset(&_att_set, 0, sizeof(_att_set));

	memset(&_rate_set, 0, sizeof(_rate_set));
	//struct vehicle_attitude_s _v_att_s;
	memset(&_v_att, 0, sizeof(_v_att));
	//struct vehicle_angular_velocity_s angular_velocity_s;
	memset(&angular_velocity, 0, sizeof(angular_velocity));

	memset(&status, 0, sizeof(status));
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
	//orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators);
        orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_set);
        orb_advert_t rate_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_rate_set);
        orb_advert_t loc_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_setp);
	/* subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int local_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int omega_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    	int stat = orb_subscribe(ORB_ID(vehicle_status));

strncpy(dbg.key, "velx", 10);
//strncpy(dbg.key, "vely", 10);
dbg.value= 0.0f;

	orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);

	strncpy(dbg_vect.name, "vel3D", 10);
	dbg_vect.x = 1.0f;
	dbg_vect.y = 2.0f;
	dbg_vect.z = 3.0f;
	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

        //int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	/* Setup of loop */
	//int vcom_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct pollfd fds[1] {};
	fds[0].fd = att_sub;
	fds[0].events = POLLIN;
	orb_copy(ORB_ID(vehicle_status), stat, &status);
	//if (status.arming_state == 1)
	//{warnx("commander started");
	PX4_INFO("value of arming state is %d", (int)status.arming_state);
	int t1=hrt_absolute_time();
	int flag1=1;
	float psi=0;

					double Thrust=0;
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
			//hrt_abstime time =hrt_absolute_time();
			//hrt_abstime t;
				if (pos_updated) {
					//if(status.arming_state == 1)

					if(status.arming_state==2)
					{
					if (flag1==1)
					{t1=status.nav_state_timestamp;
					flag1=2; psi=Eulerf(Quatf(_v_att.q)).psi();}}
					
					double t=(hrt_absolute_time() - t1)* 1e-6f;
					//t=time;
					if (flag1==1)
					{t=0;psi=Eulerf(Quatf(_v_att.q)).psi();}
                			orb_copy(ORB_ID(vehicle_local_position), local_sub, &_local_pos);
                			orb_copy(ORB_ID(vehicle_angular_velocity), omega_sub, &angular_velocity);
                			orb_copy(ORB_ID(vehicle_attitude), att_sub, &_v_att);
					//t=hrt_elapsed_time(&time);

					double M[3];
					if(status.arming_state == 1){
					M[0]=0;M[1]=0;M[2]=0; Thrust=0;}
					else if(status.arming_state == 2){
					const double posc[3] = {_local_pos.y,_local_pos.x, -_local_pos.z};
					const double velc[3] = {_local_pos.vy,_local_pos.vx, -_local_pos.vz};
					const double rotc[3]=  {Eulerf(Quatf(_v_att.q)).phi(),-(Eulerf(Quatf(_v_att.q)).theta()),-(Eulerf(Quatf(_v_att.q)).psi()-psi)};
//Eulerf(Quatf(_v_att.q)).psi(),Eulerf(Quatf(_v_att.q)).theta(),Eulerf(Quatf(_v_att.q)).phi()
					/*if(trans==3)
					{rotc[0]=-rotc[0];
					 rotc[2]=-rotc[2];
						}*/

					
			  		const double omegac[3]={angular_velocity.xyz[0],-angular_velocity.xyz[1],-angular_velocity.xyz[2]};



					//PX4_INFO((double)_local_pos_setp.y);

					controller(posc, velc, rotc, omegac, t, M, &Thrust);}
					//controller(posc, velc, rotc, omegac, t, M, Thrust);



					// M[2]=0;M[0]=0; M[1]=0;      Thrust=0.6;     
					actuators.control[0]=M[0];
					actuators.control[1]=-M[1];
					actuators.control[2]=-M[2];
					actuators.control[3]=Thrust;
					actuators.timestamp=hrt_absolute_time();
					/*dbg_vect.x = M[0];
					dbg_vect.y = -M[1];
					dbg_vect.z = M[2];
					dbg_vect.timestamp = hrt_absolute_time();*/
					orb_copy(ORB_ID(vehicle_status), stat, &status);
						dbg.value = Eulerf(Quatf(_v_att.q)).psi();
					//dbg.value = t;




				}
					orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);
					orb_publish(ORB_ID(vehicle_local_position_setpoint), loc_pub,&_local_pos_setp);
					orb_publish(ORB_ID(vehicle_attitude_setpoint), att_pub,&_att_set);
					orb_publish(ORB_ID(vehicle_rates_setpoint), rate_pub,&_rate_set);
					orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);

				/* sanity check and publish actuator outputs */
				if (PX4_ISFINITE(actuators.control[0]) &&
				    PX4_ISFINITE(actuators.control[1]) &&
				    PX4_ISFINITE(actuators.control[2]) &&
				    PX4_ISFINITE(actuators.control[3])) {
					//orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
					orb_publish(ORB_ID(actuator_controls_0), actuator_pub, &actuators);

					if (verbose) {
						warnx("*published*");
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

	orb_publish(ORB_ID(actuator_controls_0), actuator_pub, &actuators);

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

