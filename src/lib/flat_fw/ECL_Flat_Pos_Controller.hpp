/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file ECL_Flat_Pos_Controller.hpp
 * Implementation of differentially flat fixed-wing position control.
 *
 * @author Kevin Cohen <kevincohen35@gmail.com>
 *
 * Acknowledgements and References:
 *
 *    This implementation has been built for PX4 based on equations and derivations from [1].
 *
 *    [1] J. Hauser and R. Hindman, "Aggressive flight maneuvers,"
 *    Proceedings of the 36th IEEE Conference on Decision and Control,
 *    San Diego, CA, USA, 1997, pp. 4186-4191 vol.5, doi: 10.1109/CDC.1997.649490.
 *
 */

#ifndef ECL_FLAT_POS_CONTROLLER_H
#define ECL_FLAT_POS_CONTROLLER_H

#include "matrix/matrix/math.hpp"

using matrix::Vector3f;
using matrix::Quatf;

/**
 * Differentially Flat Position controller
 */
class ECL_Flat_Pos_Controller{
	public:
		ECL_Flat_Pos_Controller() = default;

		/**
		 * Set the 0th, 1st, 2nd order path error feedback gain.
		 *
		 * Select k0, k1, k2 such that the polynomial
		 * s^3 + k2*s^2 + k1*s + k0 is Hurwitz for stable tracking [1].
		 */
		void set_flat_gains(float k0, float k1, float k2) { _K0_FLAT=k0; _K1_FLAT=k1; _K2_FLAT=k2;}
		void set_cLa(float cLa) { _cLa = cLa;}
		void set_cL0(float cL0) { _cL0 = cL0;}
		void set_r(float r) { _r = r;}
		void set_cD0(float cD0) { _cD0 = cD0;}
		void set_Aref(float Aref) { _Aref = Aref;}
		void set_rho(float rho) { _rho = rho;}

		void update_pos(Vector3f pos, Vector3f vel, Vector3f acc);
		void update_pos_sp(Vector3f posd, Vector3f veld, Vector3f accd, Vector3f jerd);
		void update_att(Quatf q);
		void update_attr(Vector3f wrld_w_bdy);

		void calc_all();

		Vector3f get_omega_b_setpoint() { return _omega_sp; }
		float get_s_dot() { return _s_dot; }

	private:
		// Constants
		Vector3f _g = Vector3f(0,0,9.80665);

		// Inputs
		Vector3f _pos;									///< Current position in world (m)
		Vector3f _vel;									///< Current velocity in world (m/s)
		Vector3f _acc;									///< Current acceleration in world (m/s^2)

		Vector3f _posd;									///< Desired position in world (m)
		Vector3f _veld;									///< Desired velocity in world (m/s)
		Vector3f _accd;									///< Desired acceleration in world (m/s^2)
		Vector3f _jerd;									///< Desired jerk in world (m/s^2)

		float _K0_FLAT{1.0f};							///< 0th order path error feedback gain
		float _K1_FLAT{1.0f};							///< 1st order path error feedback gain
		float _K2_FLAT{1.0f};							///< 2nd order path error feedback gain
		float _K_SIDE_SLIP{-1.0f};						///< Proportional gain in 1/s: sideslip --> sideslip rate

		Quatf _q_wrld_bdy;								///< Quaternion describing the yaw-pitch-roll, 3-2-1 intrinsic rotation from world frame to body frame
		Vector3f _wrld_w_bdy;							///< Angular velocity of body WRT world, expressed in frd coords

		float _cLa{3.98f};
		float _cL0{0.462f};
		float _r{0.1f};
		float _cD0{0.04f};
		float _Aref{0.39755f};
		float _rho{1.2041f};

		// Intermediates
		Quatf _q_bdy_wrld;								///< Inverse of _q_bdy_wrld
		float _aoa;										///< Current angle of attack in radians
		float _sideslip;								///< current sideslip in radians
		
		float _sin_aoa;									///< sin(_aoa)
		float _cos_aoa;									///< cos(_aoa)
		float _sin_sideslip;							///< sin(_sideslip)
		float _cos_sideslip;							///< cos(_sideslip)

		float _aoar_sp;									///< Time |r|ate of change of |a|ngle |o|f |a|ttack setpoint in rad/s
		float _sideslipr_sp;							///< Time |r|ate of change of sideslip setpoint in rad/s

		float _pitchr;									///< Current pitch rate of VELOCITY FRAME WRT world frame in rad
		float _yawr;									///< Current yaw rate of VELOCITY FRAME WRT world frame in rad

		float _av1r_sp;									///< Setpoint of acceleration rate in direction of velocity in m/s^2
		float _rollr_sp;								///< Setpoint of velocitry frame roll rate WRT world in rad/s
		float _av3r_sp;									///< Setpoint of acceleration rate opposite of lift direction in m/s^2

		// Outputs
		float _s_dot{1.0f};								///< Time rate of change of path parameter (1.0 when traveling at desired speed)
		Vector3f _omega_sp; 							///< Setpoint of angular velocity of body frame WRT world frame. Expressed in frd coordinates, rad/s

		// Functions
		void calc_att();
		void calc_flat_sp();
		void calc_dyn_inv_sp();
		void calc_sideslipr_sp();
		void calc_omega_sp();
};

#endif /* ECL_FLAT_POS_CONTROLLER_H */
