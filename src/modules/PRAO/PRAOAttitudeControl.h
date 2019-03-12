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
#include <drivers/drv_hrt.h>
//#include <ecl/attitude_fw/ecl_pitch_controller.h>
//#include <ecl/attitude_fw/ecl_roll_controller.h>
//#include <ecl/attitude_fw/ecl_wheel_controller.h>
//#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
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
#include <vtol_att_control/vtol_type.h>

#ifndef FIRMWARE_PRAOATTITUDECONTROL_H
#define FIRMWARE_PRAOATTITUDECONTROL_H

using uORB::Subscription;

class PRAOAttitudeControl : public ModuleBase<PRAOAttitudeControl>
{
public:
    PRAOAttitudeControl();

private:

    int		_att_sub{-1};				/**< vehicle attitude */
    int		_att_sp_sub{-1};			/**< vehicle attitude setpoint */
    int		_rates_sp_sub{-1};			/**< vehicle rates setpoint */
    int		_battery_status_sub{-1};		/**< battery status subscription */
    int		_global_pos_sub{-1};			/**< global position subscription */
    int		_manual_sub{-1};			/**< notification of manual control updates */
    int		_params_sub{-1};			/**< notification of parameter updates */
    int		_vcontrol_mode_sub{-1};			/**< vehicle status subscription */
    int		_vehicle_land_detected_sub{-1};		/**< vehicle land detected subscription */
    int		_vehicle_status_sub{-1};		/**< vehicle status subscription */

    //Initialise la structure de parametres
    // Peut etre besoin de mettre params juste apres struct
    struct {
        float yaw_p;
        float yaw_i;
        float roll_p;
        float roll_i;
    } _params{};

    //Initialise la structure des handles de param
    struct {
        param_t yaw_p;
        param_t yaw_i;
        param_t roll_p;
        param_t roll_i;
    } _param_handles{};
};


#endif //FIRMWARE_PRAOATTITUDECONTROL_H
