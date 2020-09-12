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
 * @file ECL_Flat_Pos_Controller.cpp
 * Implementation of a differentially flat system for fixed-wing position control.
 * Authors and acknowledgements in header.
 *
 */

#include "ECL_Flat_Pos_Controller.hpp"

using matrix::Eulerf;
using matrix::SquareMatrix;
using matrix::Vector2f;
using std::sin;
using std::cos;
using std::asin;
using std::acos;

void ECL_Flat_Pos_Controller::update_pos(Vector3f pos, Vector3f vel, Vector3f acc){
	_pos = pos;
	_vel = vel;
	_acc = acc;
}

void ECL_Flat_Pos_Controller::update_pos_sp(Vector3f posd, Vector3f veld, Vector3f accd, Vector3f jerd){
	_posd = posd;
	_veld = veld;
	_accd = accd;
	_jerd = jerd;
}

void ECL_Flat_Pos_Controller::update_att(Quatf q_bdy_wrld){
	_q_bdy_wrld = q_bdy_wrld;
}

void ECL_Flat_Pos_Controller::update_attr(Vector3f wrld_w_bdy){
	_wrld_w_bdy = wrld_w_bdy;// Anguler velocity of body frame WRT world frame (expressed in frd coords).
}

void ECL_Flat_Pos_Controller::calc_all(){
	// Order matters!!!
	calc_att();
	calc_flat_sp();
	calc_dyn_inv_sp();
	calc_sideslipr_sp();
	calc_omega_sp();
}

void ECL_Flat_Pos_Controller::calc_att(){
	float vnorm = _vel.norm();
	float vnorm_dot = _vel.dot(_acc)/vnorm; // Time derivative of velocity norm
	Vector3f vel_bdy = _q_bdy_wrld.conjugate_inversed(_vel);
	Vector3f acc_bdy = _q_bdy_wrld.conjugate_inversed(_acc);

	_sin_aoa = vel_bdy(2)/vnorm; 
	_aoa = asin(_sin_aoa);
	_cos_aoa = cos(_aoa);
	_sideslip = asin( vel_bdy(1)/(vnorm*_cos_aoa) );
	_sin_sideslip = _sin_sideslip;
	_cos_sideslip = cos(_sideslip);

	float aoa_dot = ( acc_bdy(2)*vnorm-vel_bdy(2)*vnorm_dot )/( _cos_aoa*vnorm*vnorm );
	float sideslip_dot = ( ( acc_bdy(1)*vnorm-vel_bdy(1)*vnorm_dot )/( vnorm*vnorm ) + _sin_sideslip*_sin_aoa*aoa_dot )/( _cos_sideslip*_cos_aoa );

	Vector3f vel_w_bdy(_sin_aoa*sideslip_dot, aoa_dot, _cos_aoa*sideslip_dot); // Anguler velocity of body WRT velocity frame (expressed in frd coords).
	Vector3f wrld_w_vel = _wrld_w_bdy - vel_w_bdy;
	_pitchr = wrld_w_vel(1);
	_yawr = wrld_w_vel(2);
}

void ECL_Flat_Pos_Controller::calc_flat_sp(){
	// s tracks path
	// float veldN = _veld.norm();
	// float veldN2 = veldN*veldN;
	// float veldN3 = veldN2*veldN;
	// float s_dot = _vel.dot(_veld)/veldN2
	// float s_ddot = ( veldN*(_acc.dot(_veld)+_vel.dot(_accd)*s_dot) - _vel.dot(_veld)*s_dot*_veld.dot(_accd)/veldN )/veldN3 - _accd.dot(_veld)*s_dot*s_dot/veldN2;

	// s tracks vehicle
	float veldN = _veld.norm();
	float s_dot = _vel.norm()/veldN;
	float s_ddot = (veldN*_acc.norm() - _vel.norm()*_accd.norm()*s_dot)/(veldN*veldN);

	Vector3f path_error_0 = _posd - _pos;
	Vector3f path_error_1 = _veld*s_dot - _vel;
	Vector3f path_error_2 = _accd*s_dot*s_dot+_veld*s_ddot - _acc;

	Quatf q_vel_bdy = Quatf(Eulerf(0, _aoa, -_sideslip)).inversed(); // 3-2-1 sequence from "frame 1" (velocity frame) to "frame 2" (body frame)
	Quatf q_vel_wrld = _q_bdy_wrld*q_vel_bdy;
	Vector3f av = q_vel_wrld.conjugate_inversed(_acc-_g);
	Vector3f rotU = q_vel_wrld.conjugate_inversed(3.0f*_accd*s_ddot*s_dot+_jerd*s_dot*s_dot*s_dot + _K2_FLAT*path_error_2+_K1_FLAT*path_error_1+_K0_FLAT*path_error_0); // TODO: why no _veld*s_dddot ?
	Vector3f rotVeld = q_vel_wrld.conjugate_inversed(_veld);

	Vector3f flatOmegas(_pitchr*av(2), _yawr*av(0), -_pitchr*av(0)); // TODO: add effect of acceleration rate in x
	float decoupleInit[9] = {rotVeld(0),0,0, rotVeld(1),av(2),0, rotVeld(2),0,-1};
	SquareMatrix<float,3> decoupleM_inv = SquareMatrix<float,3>(decoupleInit).I();

	Vector3f flatout = decoupleM_inv*(flatOmegas - rotU);
	_rollr_sp = flatout(1);
	_av3r_sp = flatout(2);
}

void ECL_Flat_Pos_Controller::calc_dyn_inv_sp(){
	Vector3f ab = _q_bdy_wrld.conjugate_inversed(_acc-_g); // acceleration WRT v frame expressed in body coordinates

	float aT = ab(0);

	_aoar_sp = -_av3r_sp/(aT*_cos_aoa + _cLa);
}

void ECL_Flat_Pos_Controller::calc_sideslipr_sp(){
	_sideslipr_sp = _sideslip*_K_SIDE_SLIP;
}

void ECL_Flat_Pos_Controller::calc_omega_sp(){
	_omega_sp(0) = _rollr_sp*_cos_sideslip*_cos_aoa+_sin_aoa*_sideslipr_sp;
	_omega_sp(1) = _rollr_sp*_sin_sideslip*_cos_aoa+_aoar_sp;
	_omega_sp(2) = _rollr_sp*_sin_aoa+_cos_aoa*_sideslipr_sp; 
}