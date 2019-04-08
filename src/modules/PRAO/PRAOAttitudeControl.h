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

// Modif Fab
#include <stdio.h>
#include <mathlib/mathlib.h>
//#include <ecl.h>
//Fin modif Fab

using matrix::Eulerf;
using matrix::Quatf;

using uORB::Subscription;

//Initialise la structure de parametres
// Peut etre besoin de mettre params juste apres struct
struct _params {
    float yaw_p;
    float yaw_i;
    float roll_p;
    float roll_i;
    float pitch_p;
    float pitch_i;
    float int_max_pitch;
    float int_max_roll;
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
};

//Initialise la structure des handles de param
struct _param_handles {
    param_t yaw_p;
    param_t yaw_i;
    param_t roll_p;
    param_t roll_i;
    param_t pitch_p;
    param_t pitch_i;
    param_t int_max_pitch;
    param_t int_max_roll;
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
};

// MODIFIE LE HEADER FAB
struct PRAO_ControlData {
    float airspeed_min;
    float airspeed_max;
    float airspeed;
    float scaler;
    bool lock_integrator;
};
class PRAOAttitudeControl_Controller
{
// public:
    PRAOAttitudeControl_Controller(const char *name);
    virtual ~PRAOAttitudeControl_Controller() = default;
    virtual float control_attitude(const struct PRAO_ControlData &ctl_data) = 0;
    virtual float control_euler_rate(const struct PRAO_ControlData &ctl_data) = 0;
    virtual float control_bodyrate(const struct PRAO_ControlData &ctl_data) = 0;
    /* Setters */
    void set_integrator_max(float max);
    /* Getters */ /*
    float get_rate_error();
    float get_desired_rate();
    float get_desired_bodyrate();
    float get_integrator();
    void reset_integrator(); */
// protected:
    uint64_t _last_run;
    float _integrator_max;
    float _last_output;
    float _integrator;
    float constrain_airspeed(float airspeed, float minspeed, float maxspeed);
};
PRAOAttitudeControl_Controller::PRAOAttitudeControl_Controller(const char *name) :
        _last_run(0),
        _integrator_max(0.0f),
        _last_output(0.0f),
        _integrator(0.0f)
{
}
// FIN MODIF HEADER FAB