//
// Created by Johan on 01.03.2019.
//

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

#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>
#include <parameters/param.h>
#include <lib/ecl/geo/geo.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>

#include "PRAOAttitudeControl.h"

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[]);
{
    PX4_INFO("Hello water!");

}

//Mettre tous les paramètres à utiliser

//Faire toutes les subscriptions ( peut etre besoin de mettre un int devant )
// Maybe limiter l'update rate avec orb_set_interval (voir dans exemples/uuv_exemple)

int _att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
int _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
int _accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
int _vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
int _distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
int _params_sub = orb_subscribe(ORB_ID(parameter_update));
int _manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
int _global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
int _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
int _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

//Advertise to actuator_topic (voir dans exemples/uuv_exemple)

// Faire les update de paramètres

vehicle_setpoint_poll();
vehicle_accel_poll();
vehicle_control_mode_poll();
vehicle_manual_poll();
distance_sensor_poll();
vehicle_status_poll();
vehicle_land_detected_poll();

// Mettre les PID

//Envoyer les commandes au moteur
