/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_flat_control_params.c
 *
 * Parameters defined by the fixed-wing flat control task
 *
 * @author Kevin Cohen <kevincohen35@gmail.com>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Acro 0th order path feedback gain.
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_FLAT_K0, 1.0f);

/**
 * Acro 1st order path feedback gain.
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_FLAT_K1, 3.0f);

/**
 * Acro 2nd order path feedback gain.
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_FLAT_K2, 3.0f);

/**
 * Acro gain: acceleration WRT body in thrust dir. error ==> thrust setpoint - cruising thrust.
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_AT_GAIN, 1.0f);

/**
 * Acro lift curve slope for dynamic inversion (forces are in newtons).
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_CLA, 3.98f);

/**
 * Acro 3D lift coeff. at 0 AoA for dynamic inversion (forces are in newtons).
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_CL0, 0.462f);

/**
 * Acro drag polar coefficient (cD = cD0+r*cL^2) (forces are in newtons).
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_R, 0.1f);

/**
 * Acro 3D drag coefficient at 0 lift (forces are in newtons).
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_CD0, 0.04f);

/**
 * Acro reference area for aerodynamic coefficients.
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_AREF, 0.39755f);

/**
 * Acro air density (kg/m^3).
 *
 */
PARAM_DEFINE_FLOAT(FW_ACRO_RHO, 1.2041f);