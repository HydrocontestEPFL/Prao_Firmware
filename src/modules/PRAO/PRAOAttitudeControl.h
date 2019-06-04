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

#include <px4_module.h>
#include <px4_module_params.h>
#include <drivers/drv_hrt.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <matrix/math.hpp>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/distance_sensor.h>

// Modif Fab
#include <stdio.h>
#include <mathlib/mathlib.h>
//#include <ecl.h>
//Fin modif Fab

using matrix::Eulerf;
using matrix::Quatf;

using uORB::Subscription;

//Initialise la structure de parametres
struct _params {
    float roll_p;
    float roll_i;
    float pitch_p;
    float pitch_i;
    float pitch_scl;
    float roll_scl;
    float mode;
    float roll_tc;
    float pitch_tc;
    float roll_spd_max;
    float pitch_spd_max;
    float roll_int_max;
    float pitch_int_max;
    float k_filter;
    float a_filter;
    float alpha_filter;
    float reverse;
    float lift_p;
    float lift_i;
    float lift_scl;
    float lift_int_max;
    float lift_setpoint;
    float roll_setpoint;
    float speed_takeoff_init;
    float speed_takeoff_final;
    float tol_takeoff;
    float scaler_takeoff_roll;
    float coeff_takeoff;
};

//Initialise la structure des handles de param
struct _param_handles {
    param_t roll_p;
    param_t roll_i;
    param_t pitch_p;
    param_t pitch_i;
    param_t pitch_scl;
    param_t roll_scl;
    param_t mode;
    param_t roll_tc;
    param_t pitch_tc;
    param_t roll_spd_max;
    param_t pitch_spd_max;
    param_t roll_int_max;
    param_t pitch_int_max;
    param_t k_filter;
    param_t a_filter;
    param_t alpha_filter;
    param_t reverse;
    param_t lift_p;
    param_t lift_i;
    param_t lift_scl;
    param_t lift_int_max;
    param_t lift_setpoint;
    param_t roll_setpoint;
    param_t speed_takeoff_init;
    param_t speed_takeoff_final;
    param_t tol_takeoff;
    param_t scaler_takeoff_roll;
    param_t coeff_takeoff;
};
