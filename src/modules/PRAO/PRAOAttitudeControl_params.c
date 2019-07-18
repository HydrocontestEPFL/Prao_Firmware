//
// Created by Johan on 09.03.2019.
//

/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file PRAOAttitudeControl_params.c
 * Parameters for the PRAO Attitude Controller
 *
 * @author Johan Poccard <johan.poccard-saudart@epfl.ch>
 */

/**
 * Proportionnal gain of pitch controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
 PARAM_DEFINE_FLOAT(PRAO_P_P,0.5f);

/**
* Integral gain of pitch controller
*
* @min 0.0
* @max 100.0
* @group PRAO Attitude Control
*/
PARAM_DEFINE_FLOAT(PRAO_P_I,0.5f);

/**
 * Proportionnal gain of roll controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_P,60f);

/**
 * Integral gain of roll controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_I,0.5f);

/**
 * Pitch scaler
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_P_SCALER,0.5f);

/**
 * Roll scaler
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_SCALER,0.5f);

/**
 * Mode
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_MODE,0.0f);

/**
 * Roll time constant (s)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_TC,1.0f);

/**
 * Pitch time constant (s)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_P_TC,1.0f);

/**
 * Roll integrator max
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_INT_MAX,10.0f);

/**
 * Pitch integrator max
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_P_INT_MAX,1f);

/**
 * Roll speed max (rad/s)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_SPD_MAX,10.0f);

/**
 * Pitch speed max (rad/s)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_P_SPD_MAX,10.0f);

/**
 * K filtre
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_K_FILTER,10.0f);

/**
 * A filtre
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_A_FILTER,10.0f);

/**
 * Alpha filter
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_ALPHA_FILT,10.0f);

/**
 * Reverse mode to invert throttle command
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_REVERSE,1.0f);

/**
 * Proportionnal gain of lift controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_L_P,20f);

/**
* Integral gain of lift controller
*
* @min 0.0
* @max 100.0
* @group PRAO Attitude Control
*/
PARAM_DEFINE_FLOAT(PRAO_L_I,0.5f);

/**
 * Lift scaler
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_L_SCALER,0.5f);

/**
 * Lift integrator max
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_L_INT_MAX,10.0f);

/**
 * Desired altitude of flight of the boat (m)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_ALTITUDE,0.2f);

/**
 * Roll offset of flight of the boat x1000
 *
 * @min -10000
 * @max 1000.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_ROLL_SP,-50.0f);

/**
 * Speed at which the PRAO begins take off procedure if desired altitude is not yet reached (m/s)
 *
 * @min -100.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_TO_SPD_IN,2.0f);

/**
 * Speed at which the PRAO finishes take off procedure even if desired altitude is not yet reached (m/s)
 *
 * @min -100.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_TO_SPD_FN,4.0f);

/**
 * Acceptable altitude tolerance under which takeoff procedure finishes even if speed is still too low (m)
 *
 * @min -100.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_TO_TOL,0.1f);

/**
 * Ponderation coefficient for the command sent to the small foil during takeoff (1 means no roll control, 0 means no altitude control)
 *
 * @min -100.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_TO_SCL_R,0.1f);

/**
 * Coefficient for scaling takeoff actuation (to be at full actuation at some point during takeoff, > 1)
 *
 * @min -100.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_TO_COEFF,2.0f);

/**
 * Ponderation coefficient for the command sent to all servos between RC and auto control (0 means RC control, 1 normal control)
 *
 * @min -100.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_TO_RC,0.7f);