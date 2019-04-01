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

#include "PRAOAttitudeControl.h"

#include <poll.h>

#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_config.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

// Pour le contrôle
#include <math.h>
#include <float.h>
//#include <geo/geo.h>
#include <mathlib/mathlib.h>

/* Prototypes copiés de main.cpp dans l'example fixedwing_control */

/**
 * Initialize all parameter handles and values
 *
 */
extern "C" int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
extern "C" int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int PRAO_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int PRAO_thread_main(int argc, char *argv[]);

int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Basic control function
 */
void control_attitude(struct _params *para, const struct manual_control_setpoint_s *manual_sp,
                      const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators,
                      struct vehicle_global_position *global_pos, uint64_t last_run);

//Definit certaines variables
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct _params pp; // pp est le nom de la structure qui gere les params
static struct _param_handles ph; // ph est le nom de la structure qui gere le param handles

//Fonction d'initialisation des parametres
int parameters_init(struct _param_handles *h)
{
    h->yaw_p    =   param_find("PRAO_Y_P");
    h->yaw_i    =   param_find("PRAO_Y_I");
    h->roll_p   =   param_find("PRAO_R_P");
    h->roll_i   =   param_find("PRAO_R_I");
    h->pitch_p  =   param_find("PRAO_P_P");
    h->pitch_i  =   param_find("PRAO_P_I");
    h->int_max_pitch    =   param_find("PRAO_INT_MAX_P");
    h->int_max_roll    =   param_find("PRAO_INT_MAX_R");
    h->pitch_scl    =   param_find("PRAO_P_SCALER");
    h->roll_scl    =   param_find("PRAO_R_SCALER");
    h->mode    =   param_find("PRAO_MODE");
    h->roll_tc  =   param_find("PRAO_R_TC");
    h->pitch_tc  =   param_find("PRAO_P_TC");
    return 0;
}

// Fonction d'updating des parametres
int parameters_update(const struct _param_handles *h, struct _params *p)
{
    param_get(h->yaw_p, &(p->yaw_p));
    param_get(h->yaw_i, &(p->yaw_i));
    param_get(h->roll_p, &(p->roll_p));
    param_get(h->roll_i, &(p->roll_i));
    param_get(h->pitch_p, &(p->pitch_p));
    param_get(h->pitch_i, &(p->pitch_i));
    param_get(h->int_max_pitch, &(p->int_max_pitch));
    param_get(h->int_max_roll, &(p->int_max_roll));
    param_get(h->pitch_scl, &(p->pitch_scl));
    param_get(h->roll_scl, &(p->roll_scl));
    param_get(h->mode, &(p->mode));
    param_get(h-roll_tc, &(p->roll_tc));
    param_get(h->pitch_tc, &(p->pitch_tc));
    return 0;
}

// Fonction de controle appelee dans le while
void control_attitude(struct _params *para, const struct manual_control_setpoint_s *manual_sp,
        const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators,
                struct vehicle_global_position *global_pos, uint64_t last_run) {

    if (para->mode > 0.5f) {
        // Calcul de la vitesse
        float speed = sqrt((global_pos->vel_n)^2 + (global_pos->vel_e)^2);

        // Get le dt
        uint64_t dt_micros = hrt_elapsed_time(&last_run);
        last_run = hrt_absolute_time();
        float dt = (float)dt_micros * 1e-6f;

        // Borner la vitesse pour la mettre dans le scaler
        float speed_ctrl;
        if (para->mode > 0.5f) {
            // get le airspeed sans aller à l'infini
            if (speed < 1) {
                speed_ctrl = 1.0f;
            } else {
                speed_ctrl = speed;
            }
        }

        //Faire les scalers
        float roll_scaler = para->roll_scl / speed_ctrl;
        float pitch_scaler = para->pitch_scl / speed_ctrl;

        // Controle du roll

        // Trouver vitesse de roll
        float roll_err = matrix::Eulerf(matrix::Quatf(att->q)).phi(); //att est le nom de la struct qui gere vehicule_attitude
        float roll_spd_sp_nonsat = roll_err * para->roll_tc;

        //Saturation de la vitesse de roll
        float roll_spd_sp = math::constrain(roll_spd_sp_nonsat, - para->roll_spd_max, para->roll_spd_max);

        //Trouver error de roll speed
        float roll_spd_err = rollspeed - roll_spd_sp;

        // Terme prop de roll speed
        float roll_spd_prop = roll_spd_err * para->roll_p;

        // Terme int de roll speed
        float roll_spd_int = math::constrain(roll_spd_int + roll_spd_err*dt*para->roll_i, - para->roll_int_max, para->roll_int_max);

        // Addition des termes
        float roll_output = para->roll_scl * (roll_spd_prop + roll_spd_int);

        // Envoyer dans actuatoors ( les numeros de channel sont tires de actuator_controls )
        actuators->control[0]= roll_output;



        // Terme proportionnel (peut etre un - a rajouter devant yaw_err)
        float roll_err = matrix::Eulerf(matrix::Quatf(att->q)).phi(); //att est le nom de la struct qui gere vehicule_attitude
        float roll_prop = roll_err * para->roll_p;

        //Terme integrateur
        float roll_int = math::constrain(roll_int + roll_err * para->roll_i, - para->int_max_roll, para->int_max_roll);

        //Calcul du output final
        float roll_output = (roll_int + roll_prop) * roll_scaler;
        actuators->control[0]= roll_output;


        // Controle du pitch

        //le z et y sont tires de manual_control_setpoint.msg
        //On controle le yaw avec la RC
        actuators->control[2]=manual_sp->z;

        //On controle le throttle avec la RC
        actuators->control[3]=manual_sp->y;
    }
    else {
        //On controle le roll avec la RC
        actuators->control[0]=manual_sp->y;

        //On controle le pitch avec la RC
        actuators->control[1]=manual_sp->x;

        //On controle le yaw avec la RC
        actuators->control[2]=manual_sp->r;

        //On controle le throttle avec la RC
        actuators->control[3]=manual_sp->z;
    }
}

//Main thread
int PRAO_thread_main(int argc, char *argv[])
{
    PX4_INFO("Hello water!");

    parameters_init(&ph);
    parameters_update(&ph, &pp);

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
    //struct airspeed_s airspd;
    //memset(&airspd, 0, sizeof(airspd));

    // Initialisation des output structures
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));


    //Initialiser la structure d'output à 0
    for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
        actuators.control[i] = 0.0f;
    }


    //Advertise that we will publish actuators values
    orb_advert_t actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators);


    //Faire toutes les subscriptions ( peut etre besoin de mettre un int devant )
    // Maybe limiter l'update rate avec orb_set_interval (voir dans exemples/uuv_exemple)
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    //int ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    //int accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
    //int vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    //int distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
    int params_sub = orb_subscribe(ORB_ID(parameter_update));
    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    //int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
    //int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
    //int airspeed_sub = orb_subscribe(ORB_ID(airspeed));

    //Setup of loop
    struct pollfd fds[2];
    fds[0].fd = params_sub;
    fds[0].events = POLLIN;
    fds[1].fd = att_sub;
    fds[1].events = POLLIN;

    // Initialisation of parameters for dt
    uint64_t last_run;
    last_run = hrt_absolute_time();

    while (!thread_should_exit) {
        //poll waits 500ms to make fds ready, 2 is number of arguments in fds
        int ret = poll(fds,2,500);

        if (ret<0) {
            warnx("Error de loop");
        } else if (ret==0) {
          //Nothing has changed
        } else {
            //Only update parameters if they have changed
            if (fds[0].revents & POLLIN ) {
                //ecrire l update dans parameter_update
                struct parameter_update_s update;
                orb_copy(ORB_ID(parameter_update), params_sub, &update);
                /* if a param update occured, re-read our parameters */
                parameters_update(&ph, &pp);
            }
            //Only change controller if attitude changed
            if (fds[1].revents & POLLIN) {
                //Check what is new
                bool pos_updated;
                orb_check(global_pos_sub, &pos_updated);
                bool att_sp_updated;
                orb_check(att_sp_sub, &att_sp_updated);
                bool manual_sp_updated;
                orb_check(manual_sp_sub, &manual_sp_updated);
                bool airspeed_updated;
                orb_check(airspeed_sub, &airspeed_updated);

                //Get local copy of attitude
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

                //Copier l'attitude sp si il est change
                if (att_sp_updated) {
                    orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
                }

                //Copier le manual sp si il est change
                if (manual_sp_updated){
                    orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
                }

                //Copier aispeed si changé
                if (airspeed_updated){
                    orb_copy(ORB_ID(airspeed), airspeed_sub, &airspd);
                }

                //Appeler la fonction qui controle les actuators
                control_attitude(&pp, &manual_sp, &att, &actuators, &global_pos, last_run);

                //Get vehicule status
                orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

                //Sanity check then publish actuators outputs
                if (PX4_ISFINITE(actuators.control[0]) &&
                    PX4_ISFINITE(actuators.control[1]) &&
                    PX4_ISFINITE(actuators.control[2]) &&
                    PX4_ISFINITE(actuators.control[3])) {
                        orb_publish(ORB_ID(actuator_controls_0), actuator_pub, &actuators);
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

    return 0;
}

//Startup functions
int PRAO_main(int argc, char *argv[]) {
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
                                         PRAO_thread_main,
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