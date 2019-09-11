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
 * @author Fabien Benoist <f.benoist@epfl.ch>
 * @author Guillaume Rozand <guillaume.rozand@epfl.ch>
 */


/**
 * Proportionnal gain of roll controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_P,15.0f);

/**
 * Integral gain of roll controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_I,0.5f);

/**
 * Roll scaler
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_SCALER,0.7f);

/**
 * Roll time constant (s)
 *
 * @min 0.0
 * @max 1.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_TC,1.0f);

/**
 * Roll speed max (rad/s)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_SPD_MAX,10.0f);

/**
 * Roll integrator max
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_R_INT_MAX,10.0f);


/**
 * Proportionnal gain of lift controller
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_L_P,30.0f);

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
PARAM_DEFINE_FLOAT(PRAO_L_SCALER,0.7f);

/**
 * Lift integrator max
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_L_INT_MAX,10.0f);

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
 * Type of low pass filter desired for roll (0->none, 1->PASSE_BAS1, 2->PASSE-BAS2)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_FILTER,0.0f);

/**
 * Saturation for roll (0->none, 1->saturation)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_SAT,0.0f);

/**
 * Switch between position and speed control for roll (negative->position control, positive->speed control)
 *
 * @min 0.0
 * @max 100.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_SPD_OR_POS,1.0f);

/**
 * Position of the palpeur at the vertical (ems22 test)
 *
 * @min -10.0
 * @max 10.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_PLP_VERTI,0.92f);

/**
 * Position of the palpeur at the horizontal (ems22 test)
 *
 * @min -10.0
 * @max 10.0
 * @group PRAO Attitude Control
 */
PARAM_DEFINE_FLOAT(PRAO_PLP_HORIZON,-0.666f);

