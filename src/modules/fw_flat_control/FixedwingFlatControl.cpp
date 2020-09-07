/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * Acknowledgements:
 *
 *   Adapted pieces of the fw_att_control and fw_pos_control_l1 modules to this control algorithm. 
 */

#include "FixedwingFlatControl.hpp"
using math::max;
using math::min;
using math::constrain;
using math::radians;
using namespace time_literals;

FixedwingFlatControl::FixedwingFlatControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_fw) : ORB_ID(actuator_controls_0)),
	// _attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_flat_freq_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": diff. flat. run freq")),
	_attr_freq_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": attitude rate run freq"))
	// _launchDetector(this),
	// _runway_takeoff(this)
{
	// if (vtol) {
	// 	_param_handle_airspeed_trans = param_find("VT_ARSP_TRANS");

	// 	// VTOL parameter VTOL_TYPE
	// 	int32_t vt_type = -1;
	// 	param_get(param_find("VT_TYPE"), &vt_type);

	// 	_vtol_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
	// }

	// // limit to 50 Hz
	// _local_pos_sub.set_interval_ms(20);

	// /* fetch initial parameter values */
	parameters_update();
}

int
FixedwingFlatControl::parameters_update()
{
	/* pitch control parameters */
	_pitchr_k_p = _param_fw_pr_p.get();
	_pitchr_k_i = _param_fw_pr_i.get();
	_pitchr_k_ff = _param_fw_pr_ff.get();
	_pitchr_integrator_max = _param_fw_pr_imax.get();

	/* roll control parameters */
	_rollr_k_p = _param_fw_rr_p.get();
	_rollr_k_i = _param_fw_rr_i.get();
	_rollr_k_ff = _param_fw_rr_ff.get();
	_rollr_integrator_max = _param_fw_rr_imax.get();

	/* yaw control parameters */
	_yawr_k_p = _param_fw_yr_p.get();
	_yawr_k_i = _param_fw_yr_i.get();
	_yawr_k_ff = _param_fw_yr_ff.get();
	_yawr_integrator_max = _param_fw_yr_imax.get();

	/* rate parameters */
	_roll_max_rate = radians(_param_fw_acro_x_max.get());
	_pitch_max_rate = radians(_param_fw_acro_y_max.get());
	_yaw_max_rate = radians(_param_fw_acro_z_max.get());

	_flat_control.set_flat_gains(
								_param_fw_acro_flat_k0.get(),
								_param_fw_acro_flat_k1.get(),
								_param_fw_acro_flat_k2.get()
								);

	// /* wheel control parameters */
	// _wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	// _wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	// _wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	// _wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	// _wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));

	return PX4_OK;
}

FixedwingFlatControl::~FixedwingFlatControl()
{
	perf_free(_loop_perf);
	perf_free(_flat_freq_perf);
	perf_free(_attr_freq_perf);
}

bool
FixedwingFlatControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	_flat_control = ECL_Flat_Pos_Controller();

	PX4_INFO("Flat control initialized.\n");
	return true;
}

void
FixedwingFlatControl::Run()
{
	if(!_started){	
		PX4_INFO("FIRST RUN OF FLAT CONTROL!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		_started = true;
	}

	hrt_abstime start_stamp = hrt_absolute_time();

	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_att_sub.update(&_att)){
		vehicle_control_mode_poll();
		vehicle_manual_poll();

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		// _actuators.timestamp_sample = _att.timestamp;
		_actuators.timestamp_sample = start_stamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}
	}
	perf_end(_loop_perf);
}

void
FixedwingFlatControl::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	// if (_vehicle_status.is_vtol) {
	// 	const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	// 				 && !_vehicle_status.in_transition_mode;
	// 	const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

	// 	if (is_hovering || is_tailsitter_transition) {
	// 		_vcontrol_mode.flag_control_attitude_enabled = false;
	// 		_vcontrol_mode.flag_control_manual_enabled = false;
	// 	}
	// }
}

float FixedwingFlatControl::get_airspeed_and_update_scaling()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().equivalent_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = max(0.5f, _airspeed_validated_sub.get().equivalent_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the minimum airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_min.get();
		}
	}

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = constrain(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_max.get());

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed;
}

void
FixedwingFlatControl::vehicle_manual_poll()
{
	if(!_started_manual){	
		PX4_INFO("NO MANUAL CONTROL!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	}

	// const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_tailsitter_transition = false;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			// // Check if we are in rattitude mode and the pilot is above the threshold on pitch
			// if (_vcontrol_mode.flag_control_rattitude_enabled) {
			// 	if (fabsf(_manual_control_setpoint.y) > _param_fw_ratt_th.get()
			// 	    || fabsf(_manual_control_setpoint.x) > _param_fw_ratt_th.get()) {
			// 		_vcontrol_mode.flag_control_attitude_enabled = false;
			// 	}
			// }

			if (!_vcontrol_mode.flag_control_climb_rate_enabled &&
			    !_vcontrol_mode.flag_control_offboard_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {

					if(!_started_stable){	
						PX4_INFO("STARTED STABLE ATT CONTROL!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
						_started_stable = true;
					}
					// STABILIZED mode generate the attitude setpoint from manual user inputs

					// _att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get()) + radians(_param_fw_rsp_off.get());
					// _att_sp.roll_body = constrain(_att_sp.roll_body,
					// 			      -radians(_param_fw_man_r_max.get()), radians(_param_fw_man_r_max.get()));

					// _att_sp.pitch_body = -_manual_control_setpoint.x * radians(_param_fw_man_p_max.get())
					// 		     + radians(_param_fw_psp_off.get());
					// _att_sp.pitch_body = constrain(_att_sp.pitch_body,
					// 			       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

					// _att_sp.yaw_body = 0.0f;
					// _att_sp.thrust_body[0] = _manual_control_setpoint.z;

					// Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					// q.copyTo(_att_sp.q_d);

					// _att_sp.timestamp = hrt_absolute_time();

					// _attitude_sp_pub.publish(_att_sp);

					perf_end(_flat_freq_perf);
					perf_begin(_flat_freq_perf);

					_local_pos_sub.copy(&_local_pos);
					_vehicle_rates_sub.copy(&_ang_vel);

					Vector3f pos = Vector3f(_local_pos.x, _local_pos.y, _local_pos.z);
					Vector3f vel = Vector3f(_local_pos.vx, _local_pos.vy, _local_pos.vz);
					Vector3f acc = Vector3f(_local_pos.ax, _local_pos.ay, _local_pos.az);

					Quatf att = Quatf(_att.q);
					_attr = Vector3f(_ang_vel.xyz);

					if(_init_stable){
						PX4_INFO("Initialized stabilizer.\n");
						_init_stable = false;
						_init_spd = vel.norm();
						_init_dir = Vector3f(vel(0),vel(1),0)/_init_spd;
						_init_pos = pos;

						_int_rollr = 0.0f;
						_int_pitchr = 0.0f;
						_int_yawr = 0.0f;

						_rollr_act_sp = 0.0f;
						_pitchr_act_sp = 0.0f;
						_yawr_act_sp = 0.0f;

						_last_run = hrt_absolute_time();
					}

					Vector3f posd =  _init_dir.dot(pos-_init_pos)*_init_dir+_init_pos;
					Vector3f veld = _init_spd*_init_dir;
					Vector3f accd = Vector3f(0,0,0);
					Vector3f jerd = Vector3f(0,0,0);

					_flat_control.update_pos(pos,vel,acc);
					_flat_control.update_pos_sp(posd,veld,accd,jerd);
					_flat_control.update_att(att);
					_flat_control.update_attr(_attr);
					_flat_control.calc_all();

					_omega_b_sp = _flat_control.get_omega_b_setpoint();
					_thrustr_sp = _flat_control.get_thrustr_setpoint();

					if(!_started_stabler){	
						PX4_INFO("STARTED STABLE RATE CONTROL!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
						_started_stabler = true;
					}

					perf_end(_attr_freq_perf);
					perf_begin(_attr_freq_perf);
					get_airspeed_and_update_scaling();

					/* get the usual dt estimate */
					uint64_t dt_micros = hrt_elapsed_time(&_last_run);
					_last_run = hrt_absolute_time();
					float dt = (float)dt_micros * 1e-6f;

					/* zero time integration of this step for long intervals*/
					if(dt > 500000){
						update_roll_act(0);
						update_pitch_act(0);
						update_yaw_act(0);
					}else{
						update_roll_act(dt);
						update_pitch_act(dt);
						update_yaw_act(dt);	
					}

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// // RATE mode we need to generate the rate setpoint from manual user inputs
					// _rates_sp.timestamp = hrt_absolute_time();
					// _rates_sp.roll = _manual_control_setpoint.y * radians(_param_fw_acro_x_max.get());
					// _rates_sp.pitch = -_manual_control_setpoint.x * radians(_param_fw_acro_y_max.get());
					// _rates_sp.yaw = _manual_control_setpoint.r * radians(_param_fw_acro_z_max.get());
					// _rates_sp.thrust_body[0] = _manual_control_setpoint.z;

					// _rate_sp_pub.publish(_rates_sp);

				} else {
					perf_cancel(_flat_freq_perf);
					perf_cancel(_attr_freq_perf);

					if(!_init_stable)
						_init_stable = true;

					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] =
						_manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
					_actuators.control[actuator_controls_s::INDEX_PITCH] =
						-_manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
					_actuators.control[actuator_controls_s::INDEX_YAW] =
						_manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual_control_setpoint.z;

					if(!_started_manual){	
						PX4_INFO("STARTED MANUAL CONTROL!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
						_started_manual = true;
					}
				}
			}
		}
	}
}

void FixedwingFlatControl::update_roll_act(float dt){
	float rollr_sp = _omega_b_sp(0);

	float rollr_error = rollr_sp - _attr(0);

	if(rollr_sp > 0)
		rollr_sp = min(rollr_sp,_roll_max_rate);
	else
		rollr_sp = max(rollr_sp,-_roll_max_rate);

	float rollr_add = rollr_error*dt*_airspeed_scaling*_airspeed_scaling;
	if(_rollr_act_sp < -1)
		rollr_add = max(rollr_add,0.0f);
	else if(_rollr_act_sp > 1)
		rollr_add = min(rollr_add,0.0f);

	_int_rollr = _int_rollr + rollr_add;
	_rollr_act_sp = rollr_sp*_rollr_k_ff*_airspeed_scaling + 
					rollr_error*_rollr_k_p*_airspeed_scaling*_airspeed_scaling +
					_int_rollr;

	_actuators.control[actuator_controls_s::INDEX_ROLL] = _rollr_act_sp + _param_trim_roll.get();
}

void FixedwingFlatControl::update_pitch_act(float dt){
	float pitchr_sp = _omega_b_sp(1);

	float pitchr_error = pitchr_sp - _attr(1);

	if(pitchr_sp > 0)
		pitchr_sp = min(pitchr_sp,_pitch_max_rate);
	else
		pitchr_sp = max(pitchr_sp,-_pitch_max_rate);

	float pitchr_add = pitchr_error*dt*_airspeed_scaling*_airspeed_scaling;
	if(_pitchr_act_sp < -1)
		pitchr_add = max(pitchr_add,0.0f);
	else if(_pitchr_act_sp > 1)
		pitchr_add = min(pitchr_add,0.0f);

	_int_pitchr = _int_pitchr + pitchr_add;
	_pitchr_act_sp = pitchr_sp*_pitchr_k_ff*_airspeed_scaling + 
					pitchr_error*_pitchr_k_p*_airspeed_scaling*_airspeed_scaling +
					_int_pitchr;
	
	_actuators.control[actuator_controls_s::INDEX_PITCH] = _pitchr_act_sp + _param_trim_pitch.get();
}

void FixedwingFlatControl::update_yaw_act(float dt){
	float yawr_sp = _omega_b_sp(2);

	float yawr_error = yawr_sp - _attr(2);

	if(yawr_sp > 0)
		yawr_sp = min(yawr_sp,_yaw_max_rate);
	else
		yawr_sp = max(yawr_sp,-_yaw_max_rate);

	float yawr_add = yawr_error*dt*_airspeed_scaling*_airspeed_scaling;
	if(_yawr_act_sp < -1)
		yawr_add = max(yawr_add,0.0f);
	else if(_yawr_act_sp > 1)
		yawr_add = min(yawr_add,0.0f);

	_int_yawr = _int_yawr + yawr_add;
	_yawr_act_sp = yawr_sp*_yawr_k_ff*_airspeed_scaling + 
					yawr_error*_yawr_k_p*_airspeed_scaling*_airspeed_scaling +
					_int_yawr;

	_actuators.control[actuator_controls_s::INDEX_YAW] = _yawr_act_sp + _param_trim_yaw.get();
}

int FixedwingFlatControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingFlatControl *instance = new FixedwingFlatControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingFlatControl::print_status()
{
	PX4_INFO("Running\n");
	perf_print_counter(_loop_perf);
	perf_print_counter(_flat_freq_perf);
	perf_print_counter(_attr_freq_perf);

	// PX4_INFO("Mean run duration: %f (s)", _loop_perf.mean);
	// PX4_INFO("Mean runtime: %f (s)", _loop_perf.mean);
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int FixedwingFlatControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingFlatControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_flat_control is the fixed wing position and attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_flat_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_flat_control_main(int argc, char *argv[])
{
	return FixedwingFlatControl::main(argc, argv);
}