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
 * @file PRAOAttitudeControl.cpp
 * PRAO Attitude Controller
 *
 * @author Johan Poccard <johan.poccard-saudart@epfl.ch>
 * @author Fabien Benoist <f.benoist@epfl.ch>
 * @author Guillaume Rozand <guillaume.rozand@epfl.ch>
 */

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
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/distance_sensor.h>
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

/**
 * Update all parameters
 *
 */
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
                      const struct vehicle_global_position_s *global_pos, uint64_t last_run,
                      float roll_spd_int, float roll_spd_filtree, const distance_sensor_s *dist_sensor,
                      float lift_int);

//Definit certaines variables
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static struct _params pp; // pp est le nom de la structure qui gere les params
static struct _param_handles ph; // ph est le nom de la structure qui gere le param handles

//Fonction d'initialisation des parametres
int parameters_init(struct _param_handles *h)
{
    h->roll_p       =   param_find("PRAO_R_P");
    h->roll_i       =   param_find("PRAO_R_I");
    h->roll_scl     =   param_find("PRAO_R_SCALER");
    h->roll_tc      =   param_find("PRAO_R_TC");
    h->roll_spd_max =   param_find("PRAO_R_SPD_MAX");
    h->roll_int_max =   param_find("PRAO_R_INT_MAX");
    h->lift_p       =   param_find("PRAO_L_P");
    h->lift_i       =   param_find("PRAO_L_I");
    h->lift_scl     =   param_find("PRAO_L_SCALER");
    h->lift_int_max =   param_find("PRAO_L_INT_MAX");
    h->k_filter     =   param_find("PRAO_K_FILTER");
    h->a_filter     =   param_find("PRAO_A_FILTER");
    h->alpha_filter =   param_find("PRAO_ALPHA_FILT");
    h->filter       =   param_find("PRAO_FILTER");
    h->sat          =   param_find("PRAO_SAT");
    h->spd_or_pos   =   param_find("PRAO_SPD_OR_POS");
    h->plp_vtl      =   param_find("PRAO_PLP_VERTI");
    h->plp_hzl      =   param_find("PRAO_PLP_HORIZON");
    return 0;
}

// Fonction d'updating des parametres
int parameters_update(const struct _param_handles *h, struct _params *p)
{
    param_get(h->roll_p, &(p->roll_p));
    param_get(h->roll_i, &(p->roll_i));
    param_get(h->roll_scl, &(p->roll_scl));
    param_get(h->roll_tc, &(p->roll_tc));
    param_get(h->roll_spd_max, &(p->roll_spd_max));
    param_get(h->roll_int_max, &(p->roll_int_max));
    param_get(h->lift_p, &(p->lift_p));
    param_get(h->lift_i, &(p->lift_i));
    param_get(h->lift_scl, &(p->lift_scl));
    param_get(h->lift_int_max, &(p->lift_int_max));
    param_get(h->k_filter, &(p->k_filter));
    param_get(h->a_filter, &(p->a_filter));
    param_get(h->alpha_filter, &(p->alpha_filter));
    param_get(h->filter, &(p->filter));
    param_get(h->sat, &(p->sat));
    param_get(h->spd_or_pos, &(p->spd_or_pos));
    param_get(h->plp_vtl, &(p->plp_vtl));
    param_get(h->plp_hzl, &(p->plp_hzl));

    return 0;
}

// Fonction de controle appelee dans le while
void control_attitude(struct _params *para, const struct manual_control_setpoint_s *manual_sp,
                      const struct vehicle_attitude_s *att, struct actuator_controls_s *actuators,
                      const struct vehicle_global_position_s *global_pos, uint64_t last_run,
                      float roll_spd_int, float roll_spd_filtree, const distance_sensor_s *dist_sensor,
                      float lift_int) {


    // Calcul de la vitesse
    float speed = sqrt(pow(global_pos->vel_n, 2) + pow(global_pos->vel_e, 2));

    // Get le dt
    uint64_t dt_micros = hrt_elapsed_time(&last_run);
    last_run = hrt_absolute_time();
    float dt = (float) dt_micros * 1e-6f;

    // Borner la vitesse pour la mettre dans le scaler
    float speed_ctrl;
    if (speed < 1) {
        speed_ctrl = 1.0f;
    } else {
        speed_ctrl = speed;
    }

    //Faire les scalers
    float roll_scaler = para->roll_scl + (1 - para->roll_scl ) / powf(speed_ctrl, 2);
    float lift_scaler = para->lift_scl + (1 - para->lift_scl ) / powf(speed_ctrl, 2);


    if (manual_sp->aux2 > 0.0f) {
        /** Mode REVERSE : Si on est en mode reverse (marche arrière)
         * on met un moins sur le throttle car manual_sp->z est defini sur 0..1 **/

        /** Envoie dans actuators (les numeros de channel sont tires de actuator_controls) **/
        // control[0] -> roll
        // control[1] -> lift (pitch)
        // control[2] -> yaw
        // control[3] -> throttle
        // NE CORRESPOND PAS AU BRANCHEMENT SUR LE PIXHAWK (voir ordre dans le mixer pour cela)
        actuators->control[0] = -manual_sp->y;
        actuators->control[1] = -manual_sp->x;
        actuators->control[2] = manual_sp->r;
        actuators->control[3] = -manual_sp->z;

    } else {
        if (manual_sp->aux1 < -0.5f) {
            /** Mode 0 : MANUEL **/

            /** Envoie dans actuators (les numeros de channel sont tires de actuator_controls) **/
            // control[0] -> roll
            // control[1] -> lift (pitch)
            // control[2] -> yaw
            // control[3] -> throttle
            // NE CORRESPOND PAS AU BRANCHEMENT SUR LE PIXHAWK (voir ordre dans le mixer pour cela)
            actuators->control[0] = -manual_sp->y;
            actuators->control[1] = -manual_sp->x;
            actuators->control[2] = manual_sp->r;
            actuators->control[3] = manual_sp->z;

        } else if (manual_sp->aux1 > -0.5f && manual_sp->aux1 < 0.5f) {
            /** Mode 1 : STABILIZE
             * AUTO Controle ROLL sur la vitesse PI :
             * SAT par param PRAO_SAT
             * FILTRE par param PRAO_FILTER
             * Setpoint à la RC
             *
             *
             * MANUAL LIFT (pitch)
             *
             * Equivalent au mode stabilize du bifoiler
             * **/

            /** Controle du roll **/

            // Trouver vitesse de roll
            float roll_err = manual_sp->y - matrix::Eulerf(
                    matrix::Quatf(att->q)).phi(); //att est le nom de la struct qui gere vehicule_attitude
            float roll_spd_sp_nonsat = roll_err * (1 / para->roll_tc);
            float roll_spd_err = roll_err;

            // Controle en vitesse
            // Si le parametre PRAO_SPD_OR_POS est positif, controle en vitesse,
            // s'il est négatif controle en position
            if (para->spd_or_pos > 0.0f) {
                // Filtrer la vitesse de roll
                // Chaque ligne correspond à l'activation par param PRAO_FILTER = 0 ou 1 ou 2
                // 1ère ligne correspond à pas de filtre (PRAO_FILTER = 0)
                // 2ème ligne correspond à filtre PASSE-BAS 1 (PRAO_FILTER = 1)
                // 2ème ligne correspond à filtre PASSE-BAS 1 (PRAO_FILTER = 2)
                roll_spd_filtree = 0.5f * (2.0f - para->filter) * (1.0f - para->filter) * (att->rollspeed) +
                                   (2.0f - para->filter) * (para->filter) *
                                   ((para->k_filter * para->a_filter * dt * att->rollspeed + roll_spd_filtree) /
                                    (para->a_filter * dt + 1.0f)) +
                                   0.5f * (para->filter - 1.0f) * (para->filter) *
                                   (para->alpha_filter * att->rollspeed +
                                    (1.0f - para->alpha_filter) * roll_spd_filtree);


                // Saturation de la consigne de vitesse de roll et de la vitesse de roll et calcul de l'erreur
                // Activation par le paramètre PRAO_SAT
                // PRAO_SAT = 0 -> pas de saturation
                // PRAO_SAT = 1 -> saturation
                float roll_spd_sp = (1.0f - para->sat) * roll_spd_sp_nonsat +
                                    (para->sat) *
                                    (math::constrain(roll_spd_sp_nonsat, -para->roll_spd_max, para->roll_spd_max));
                float roll_spd = (1.0f - para->sat) * roll_spd_filtree +
                                 (para->sat) *
                                 (math::constrain(roll_spd_filtree, -para->roll_spd_max, para->roll_spd_max));
                roll_spd_err = roll_spd_sp - roll_spd;
            }

            // PI sur la vitesse de roll avec saturation du terme integrateur (permet de le vider)
            float roll_spd_prop = roll_spd_err * para->roll_p;
            roll_spd_int = math::constrain(roll_spd_int + roll_spd_err * dt * para->roll_i, -para->roll_int_max,
                                           para->roll_int_max);
            float roll_output = roll_scaler * (roll_spd_prop + roll_spd_int);


            /** Envoie dans actuators (les numeros de channel sont tires de actuator_controls) **/
            // control[0] -> roll
            // control[1] -> lift (pitch)
            // control[2] -> yaw
            // control[3] -> throttle
            // NE CORRESPOND PAS AU BRANCHEMENT SUR LE PIXHAWK (voir ordre dans le mixer pour cela)
            actuators->control[0] = -roll_output;
            actuators->control[1] = -manual_sp->x;
            actuators->control[2] = manual_sp->r;
            actuators->control[3] = manual_sp->z;

        } else if (manual_sp->aux1 > 0.5f) {
            /** Mode 2 : ALTITUDE
             * AUTO Controle ROLL sur la vitesse PI :
             * SAT par param PRAO_SAT
             * FILTRE par param PRAO_FILTER
             * Setpoint à la RC
             *
             *
             * AUTO Controle LIFT (altitude) sur la position de lift PI (publié sur channel pitch)
             * sans SAT, sans FILTRE
             * setpoint à la RC
             *
             * Equivalent au mode altitude du bifoiler
             * **/

            /** Controle du roll **/

            // Trouver vitesse de roll
            float roll_err = manual_sp->y - matrix::Eulerf(
                    matrix::Quatf(att->q)).phi(); //att est le nom de la struct qui gere vehicule_attitude
            float roll_spd_sp_nonsat = roll_err * (1 / para->roll_tc);
            float roll_spd_err = roll_err;

            // Controle en vitesse
            // Si le parametre PRAO_SPD_OR_POS est positif, controle en vitesse,
            // s'il est négatif controle en position
            if (para->spd_or_pos > 0.0f) {
                // Filtrer la vitesse de roll
                // Chaque ligne correspond à l'activation par param PRAO_FILTER = 0 ou 1 ou 2
                // 1ère ligne correspond à pas de filtre (PRAO_FILTER = 0)
                // 2ème ligne correspond à filtre PASSE-BAS 1 (PRAO_FILTER = 1)
                // 2ème ligne correspond à filtre PASSE-BAS 1 (PRAO_FILTER = 2)
                roll_spd_filtree = 0.5f * (2.0f - para->filter) * (1.0f - para->filter) * (att->rollspeed) +
                                   (2.0f - para->filter) * (para->filter) *
                                   ((para->k_filter * para->a_filter * dt * att->rollspeed + roll_spd_filtree) /
                                    (para->a_filter * dt + 1.0f)) +
                                   0.5f * (para->filter - 1.0f) * (para->filter) *
                                   (para->alpha_filter * att->rollspeed +
                                    (1.0f - para->alpha_filter) * roll_spd_filtree);


                // Saturation de la consigne de vitesse de roll et de la vitesse de roll et calcul de l'erreur
                // Activation par le paramètre PRAO_SAT
                // PRAO_SAT = 0 -> pas de saturation
                // PRAO_SAT = 1 -> saturation
                float roll_spd_sp = (1.0f - para->sat) * roll_spd_sp_nonsat +
                                    (para->sat) *
                                    (math::constrain(roll_spd_sp_nonsat, -para->roll_spd_max, para->roll_spd_max));
                float roll_spd = (1.0f - para->sat) * roll_spd_filtree +
                                 (para->sat) *
                                 (math::constrain(roll_spd_filtree, -para->roll_spd_max, para->roll_spd_max));
                roll_spd_err = roll_spd_sp - roll_spd;
            }


            // PI sur la vitesse de roll avec saturation du terme integrateur (permet de le vider)
            float roll_spd_prop = roll_spd_err * para->roll_p;
            roll_spd_int = math::constrain(roll_spd_int + roll_spd_err * dt * para->roll_i, -para->roll_int_max,
                                           para->roll_int_max);
            float roll_output = roll_scaler * (roll_spd_prop + roll_spd_int);


            /** Controle du lift **/


            // Calcul de l'erreur par rapport au setpoint désiré (qui vient de la RC)
            // ATTENTION: Changer si la taille du palpeur change de l=0.8m
            // (marge de sécurité pour pas que le palpeur soit vertical)

            //calibration of the palpeur

            float offset = (para->plp_hzl + para->plp_vtl) / 2;
            float range = (para->plp_hzl - para->plp_vtl) / 2;

            float lift_err = range * manual_sp->x + offset - dist_sensor->current_distance;

            // PI sur la position de lift avec saturation du terme integrateur (permet de le vider)
            float lift_prop = lift_err * para->lift_p;
            lift_int = math::constrain(lift_int + lift_err * dt * para->lift_i, -para->lift_int_max,
                                       para->lift_int_max);
            float lift_output = lift_scaler * (lift_prop + lift_int);


            /** Envoie dans actuators (les numeros de channel sont tires de actuator_controls) **/
            // control[0] -> roll
            // control[1] -> lift (pitch)
            // control[2] -> yaw
            // control[3] -> throttle
            // NE CORRESPOND PAS AU BRANCHEMENT SUR LE PIXHAWK (voir ordre dans le mixer pour cela)
            actuators->control[0] = -roll_output;
            actuators->control[1] = -lift_output;
            actuators->control[2] = manual_sp->r;
            actuators->control[3] = manual_sp->z;
        }
    }
}


//Main thread
    int PRAO_thread_main(int argc, char *argv[]) {
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
        struct distance_sensor_s dist_sensor;
        memset (&dist_sensor, 0, sizeof(dist_sensor));

        // Initialisation des output structures
        struct actuator_controls_s actuators;
        memset(&actuators, 0, sizeof(actuators));


        //Initialiser la structure d'output à 0
        for (unsigned i = 0; i < (sizeof(actuators.control) / sizeof(actuators.control[0])); i++) {
            actuators.control[i] = 0.0f;
        }


        //Advertise that we will publish actuators values
        orb_advert_t actuator_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators);


        //Faire toutes les subscriptions
        // Maybe limiter l'update rate avec orb_set_interval (voir dans exemples/uuv_exemple)
        int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
        int params_sub = orb_subscribe(ORB_ID(parameter_update));
        int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
        int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
        int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
        int dist_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));

        //Setup of loop
        struct pollfd fds[2];
        fds[0].fd = params_sub;
        fds[0].events = POLLIN;
        fds[1].fd = att_sub;
        fds[1].events = POLLIN;

        // Initialisation of parameters for dt
        uint64_t last_run;
        last_run = hrt_absolute_time();

        // Initialisation de termes reutilises
        float roll_spd_int = 0;
        float roll_spd_filtree = 0;
        float lift_int = 0;

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
                    bool dist_sensor_updated;
                    orb_check(dist_sensor_sub, &dist_sensor_updated);
                    //Get local copy of attitude
                    orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

                    //Copier le pos si il est change
                    if (pos_updated) {
                        orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
                    }

                    //Copier l'attitude sp si il est change
                    if (att_sp_updated) {
                        orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
                    }

                    //Copier le manual sp si il est change
                    if (manual_sp_updated){
                        orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
                    }

                    //Copier le distance sensor si il est changé
                    if (dist_sensor_updated) {
                        orb_copy(ORB_ID(distance_sensor), dist_sensor_sub, &dist_sensor);
                    }

                    //Appeler la fonction qui controle les actuators
                    control_attitude(&pp, &manual_sp, &att, &actuators, &global_pos, last_run,
                                     roll_spd_int, roll_spd_filtree, &dist_sensor, lift_int);

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