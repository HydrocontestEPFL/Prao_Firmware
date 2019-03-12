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

extern "C" __EXPORT int PRAO_att_control_main(int argc, char *argv[]);

PX4_INFO("Hello water!");

//Mettre tous les paramètres à utiliser

//Definit certaines variables
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct params pp; // pp est le nom de la structure qui gere les params
static struct param_handles ph; // ph est le nom de la structure qui gere le param handles

//Initialise param values
int parameters_init(struct param_handles *h);
{
    _param_handles.yaw_p = param_find("PRAO_P_P");
    _param_handles.yaw_i= param_find("PRAO_P_I");
    _param_handles.roll_p = param_find("PRAO_R_P");
    _param_handles.roll_i = param_find("PRAO_R_I");
    return OK;
}

// Updater les parametres
int parameters_update(const struct param_handles *h, struct params *p)
{
    param_get(_param_handles.yaw_p, &(_params.yaw_p));
    param_get(_param_handles.yaw_i, &(_params.yaw_i));
    param_get(_param_handles.roll_p, &(_params.roll_p));
    param_get(_param_handles.roll_i, &(_params.roll_i));
    return OK;
}

// Fonction de controle appelee dans le while
void control_attitude(const struct manual_control_setpoint *manual_sp, const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators)
{
    //Les numero de channel sont tires de actuator_controls.

    // On amène le roll à 0 (peut etre un - a rajouter devant yaw_err)
    float roll_err = matrix::Eulerf(matrix::Quatf(att->q)).phi() //att est le nom de la struct qui gere vehicule_attitude
    actuators->control[0] = yaw_err * pp.yaw_p;

    // On amène le pitch à 0 (peut etre un - a rajouter devant pitch_err)
    float pitch_err = matrix::Eulerf(matrix::Quatf(att->q)).theta()
    actuators->control[1] = pitch_err * pp.pitch_p;

    //le z et y sont tires de manual_control_setpoint.msg
    //On controle le yaw avec la RC
    actuators->control[2]=manual_sp->z;

    //On controle le throttle avec la RC
    actuators->control[3]=manual_sp->y;

// Début de Johan qui fait de la merde

// Fin de Johan qui fait de la merde
}

//Main thread
int prao_control_thread_main(int argc, char *argv[])
{

    // Initialiser les structures donnees par les subscriptions
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_attitude_setpoint_s att_sp;
    memset(&att_sp, 0, sizeof(att_sp));
    struct vehicle_global_position_s global_pos;
    memset(&global_pos, 0, sizeof(global_pos));
    struct manual_control_setpoint_s manual_sp;
    memset(&manual_sp, 0, sizeof(manual_sp));
    struct vehicle_status_s vstatus;
    memset(&vstatus, 0, sizeof(vstatus));
    struct position_setpoint_s global_sp;
    memset(&global_sp, 0, sizeof(global_sp));

    // Initialisation des output structures
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));


    //Initialiser la structure d'output à 0
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }


    //Advertise that we will publish actuators values
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);


    //Faire toutes les subscriptions ( peut etre besoin de mettre un int devant )
    // Maybe limiter l'update rate avec orb_set_interval (voir dans exemples/uuv_exemple)
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    //int ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    //int accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
    //int vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    //int distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
    int param_sub = orb_subscribe(ORB_ID(parameter_update));
    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    //int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    //int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

    //Setup of loop
    struct pollfd fds[2];
    fds[0].fd = param_sub;
    fds[0].events = POLLIN;
    fds[1].fd = att_sub;
    fds[1].events = POLLIN;

    while (!thread_should_exit) {
        //poll waits 500ms to make fds ready, 2 is number of arguments in fds
        int ret = poll(fds,2,500);
            if (ret<0) {
                warnx("Error de loop")
            } else if (ret==0) {
              //Nothing has changed
            } else {
                //Only update parameters if they have changed
                if ( fds[0]).revents & POLLIN ){
                    //ecrire l update dans parameter_update
                    struct parameter_update_s update;
                    orb_copy(ORB_ID(parameter_update), param_sub, &update);
                    /* if a param update occured, re-read our parameters */
                    parameters_update(&ph, &pp);
                }
                // Only change controller if attitude changed
                if (fds[1].revents & POLLIN) {
                    //Check what is new
                    bool pos_updated;
                    orb_check(global_pos_sub, &pos_updated);
                    bool att_sp_updated;
                    orb_check(att_sp_sub, &att_sp_updated);
                    bool manual_sp_updated;
                    orb_check(manual_sp_sub, &manual_sp_updated);

                    //Get local copy of attitude
                    orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

                    //Copier l'attitude sp si il est changé
                    if (att_sp_updated) {
                        orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &_att_sp);
                    }

                    //Copier le manual sp si il est changé
                    if (manual_sp_updated){
                        orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
                    }

                    //Appeler la fonction qui controle les actuators
                    control_attitude(&manual_sp, &att, &actuators);

                    //Get vehicule status
                    orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

                    //Sanity check then publish actuators outputs
                    if (PX4_ISFINITE(actuators.control[0]) &&
                        PX4_ISFINITE(actuators.control[1]) &&
                        PX4_ISFINITE(actuators.control[2]) &&
                        PX4_ISFINITE(actuators.control[3])) {
                            orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                    }
                        // J ai pas mis de verbose
                }
            }
    }
    warnx("Exiting, stopping all motors");
    thread_running = false;

//Kill all outputs
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }

    actuators.timestamp = hrt_absolute_time();

    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
}

//Startup functions
int rover_steering_control_main(int argc, char *argv[]) {
    if (argc < 2) {
        return 1;
    }
    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            warnx("running");
            return 0;
        }
        thread_should_exit = false;
        deamon_task = px4_task_spawn_cmd("PRAOAttitudeControl",
                                         SCHED_DEFAULT,
                                         SCHED_PRIORITY_MAX - 20,
                                         2048,
                                         prao_control_thread_main,
                                         (argv) ? (char *const *) &argv[2] : (char *const *) nullptr);
        thread_running = true;
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("running");

        } else {
            warnx("not started");
        }

        return 0;
    }
    return 1;
}

//////////END COMMENTS