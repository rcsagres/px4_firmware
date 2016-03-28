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

/*
 * @file angular_acc_estimator_params.c
 *
 * Parameters for angular acceleration estimator (quaternion based)
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <systemlib/param/param.h>

/**
 * Predictive filter reference weights (roll)
 *
 * @group Angular Acc estimator
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_ROLL1, 9.90545f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_ROLL2, -0.0796173f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_ROLL3, -0.059699f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_ROLL4, -0.0397927f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_ROLL5, -0.0198944f);

/**
 * Predictive filter angular rate weights (roll)
 *
 * @group Angular Acc estimator
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_ROLL1, -1.90385f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_ROLL2, -1.92218f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_ROLL3, -1.9409f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_ROLL4, -1.96001f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_ROLL5, -1.97951f);

/**
 * Predictive filter reference weights (pitch)
 *
 * @group Angular Acc estimator
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_PITCH1, 9.90545f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_PITCH2, -0.0796173f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_PITCH3, -0.059699f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_PITCH4, -0.0397927f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_PITCH5, -0.0198944f);

/**
 * Predictive filter angular rate weights (pitch)
 *
 * @group Angular Acc estimator
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_PITCH1, -1.90385f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_PITCH2, -1.92218f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_PITCH3, -1.9409f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_PITCH4, -1.96001f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_PITCH5, -1.97951f);

/**
 * Predictive filter reference weights (yaw)
 *
 * @group Angular Acc estimator
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_YAW1, 3.98483f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_YAW2, -0.0127764f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_YAW3, -0.00958192f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_YAW4, -0.00638778f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_R_YAW5, -0.00319384f);

/**
 * Predictive filter angular rate weights (yaw)
 *
 * @group Angular Acc estimator
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_YAW1, -0.784342f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_YAW2, -0.787435f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_YAW3, -0.790552f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_YAW4, -0.793695f);
PARAM_DEFINE_FLOAT(ANG_ACC_THETA_W_YAW5, -0.796864f);

/*
