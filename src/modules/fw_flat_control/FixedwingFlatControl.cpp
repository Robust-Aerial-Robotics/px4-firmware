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

#include "FixedwingFlatControl.hpp"

FixedwingFlatControl::FixedwingFlatControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_fw) : ORB_ID(actuator_controls_0)),
	// _attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
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
	// parameters_update();
}

FixedwingFlatControl::~FixedwingFlatControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingFlatControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

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

	if (_att_sub.updated()){
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

		perf_end(_loop_perf);
	}
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
	PX4_INFO("Running");
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