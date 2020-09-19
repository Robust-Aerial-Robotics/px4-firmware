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

#ifndef FIXEDWINGFLATCONTROL_HPP_
#define FIXEDWINGFLATCONTROL_HPP_

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <lib/flat_fw/ECL_Flat_Pos_Controller.hpp>
#include <lib/poly_path/poly_path.cpp>

class FixedwingFlatControl final : public ModuleBase<FixedwingFlatControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	FixedwingFlatControl(bool vtol = false);
	~FixedwingFlatControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	bool init();

private:
	void Run() override;

	void vehicle_control_mode_poll();

	void vehicle_manual_poll();

	int parameters_update();

	float get_airspeed_and_update_scaling();

	void update_roll_act(float dt);
	void update_pitch_act(float dt);
	void update_yaw_act(float dt);
	void update_thrust(float dt, Quatf att, Vector3f acc);

	orb_advert_t	_mavlink_log_pub{nullptr};

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */
	
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};		/**< local position subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};					///< control mode subscription
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	///< notification of manual control updates
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};						///< vehicle status subscription

	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub;

	vehicle_attitude_s			_att {};
	vehicle_local_position_s	_local_pos {};
	vehicle_angular_velocity_s	_ang_vel {};

	actuator_controls_s			_actuators {};											///< actuator control inputs
	manual_control_setpoint_s	_manual_control_setpoint {};							///< r/c channel data
	vehicle_control_mode_s		_vcontrol_mode {};										///< control mode
	vehicle_status_s		_vehicle_status {};											///< vehicle status

	perf_counter_t	_loop_perf;															///< loop performance counter
	perf_counter_t	_flat_freq_perf;													///< perfomance counter for run frequency of diff. flat. algo. (attitude,pos=>attitude-rate s.p.)
	perf_counter_t	_attr_freq_perf;													///< perfomance counter for run frequency of attitude-rate s.p.=>actuator controller

	bool _started{false};
	bool _started_manual{false};
	bool _started_stable{false};
	bool _started_stabler{false};
	bool _init_stable{false};

	float _pitchr_k_p{0.0f};
	float _pitchr_k_i{0.0f};
	float _pitchr_k_ff{0.0f};
	float _pitchr_integrator_max{0.0f};
	float _pitchr_act_sp{0.0f};
	float _int_pitchr{0.0f};

	float _rollr_k_p{0.0f};
	float _rollr_k_i{0.0f};
	float _rollr_k_ff{0.0f};
	float _rollr_integrator_max{0.0f};
	float _rollr_act_sp{0.0f};
	float _int_rollr{0.0f};

	float _yawr_k_p{0.0f};
	float _yawr_k_i{0.0f};
	float _yawr_k_ff{0.0f};
	float _yawr_integrator_max{0.0f};
	float _yawr_act_sp{0.0f};
	float _int_yawr{0.0f};

	// float _aT_sp{0.0f};

	float _roll_max_rate{0.0f};
	float _pitch_max_rate{0.0f};
	float _yaw_max_rate{0.0f};

	float _s{0.0f};
	hrt_abstime _last_run;
	float _thrustr_sp;

	float _airspeed_scaling{1.0f};

	Vector3f _g = Vector3f(0.0f,0.0f,9.80665f);

	Vector3f _attr;
	Vector3f _omega_b_sp;

	Vector3f _init_vel;
	Vector3f _init_pos;
	#define NUM_LEGS 8
	Poly_Path<7,NUM_LEGS> _x_path;
	Poly_Path<7,NUM_LEGS> _y_path;
	Poly_Path<7,NUM_LEGS> _z_path;

	ECL_Flat_Pos_Controller _flat_control;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_ACRO_HLD_LEN>) _param_fw_acro_hld_len,
		(ParamFloat<px4::params::FW_ACRO_CLA>) _param_fw_acro_cLa,
		(ParamFloat<px4::params::FW_ACRO_CL0>) _param_fw_acro_cL0,
		(ParamFloat<px4::params::FW_ACRO_R>) _param_fw_acro_r,
		(ParamFloat<px4::params::FW_ACRO_CD0>) _param_fw_acro_cD0,
		(ParamFloat<px4::params::FW_ACRO_AREF>) _param_fw_acro_Aref,
		(ParamFloat<px4::params::FW_ACRO_RHO>) _param_fw_acro_rho,

		(ParamFloat<px4::params::FW_THR_CRUISE>) _param_fw_thr_cruise,
		// (ParamFloat<px4::params::FW_ACRO_AT_GAIN>) _param_fw_acro_at_gain,

		(ParamFloat<px4::params::FW_ACRO_FLAT_K0>) _param_fw_acro_flat_k0,
		(ParamFloat<px4::params::FW_ACRO_FLAT_K1>) _param_fw_acro_flat_k1,
		(ParamFloat<px4::params::FW_ACRO_FLAT_K2>) _param_fw_acro_flat_k2,

		(ParamFloat<px4::params::FW_ACRO_X_MAX>) _param_fw_acro_x_max,
		(ParamFloat<px4::params::FW_ACRO_Y_MAX>) _param_fw_acro_y_max,
		(ParamFloat<px4::params::FW_ACRO_Z_MAX>) _param_fw_acro_z_max,

		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		(ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,

		// (ParamBool<px4::params::FW_BAT_SCALE_EN>) _param_fw_bat_scale_en,

		// (ParamFloat<px4::params::FW_DTRIM_P_FLPS>) _param_fw_dtrim_p_flps,
		// (ParamFloat<px4::params::FW_DTRIM_P_VMAX>) _param_fw_dtrim_p_vmax,
		// (ParamFloat<px4::params::FW_DTRIM_P_VMIN>) _param_fw_dtrim_p_vmin,
		// (ParamFloat<px4::params::FW_DTRIM_R_FLPS>) _param_fw_dtrim_r_flps,
		// (ParamFloat<px4::params::FW_DTRIM_R_VMAX>) _param_fw_dtrim_r_vmax,
		// (ParamFloat<px4::params::FW_DTRIM_R_VMIN>) _param_fw_dtrim_r_vmin,
		// (ParamFloat<px4::params::FW_DTRIM_Y_VMAX>) _param_fw_dtrim_y_vmax,
		// (ParamFloat<px4::params::FW_DTRIM_Y_VMIN>) _param_fw_dtrim_y_vmin,

		// (ParamFloat<px4::params::FW_FLAPERON_SCL>) _param_fw_flaperon_scl,
		// (ParamFloat<px4::params::FW_FLAPS_LND_SCL>) _param_fw_flaps_lnd_scl,
		// (ParamFloat<px4::params::FW_FLAPS_SCL>) _param_fw_flaps_scl,
		// (ParamFloat<px4::params::FW_FLAPS_TO_SCL>) _param_fw_flaps_to_scl,

		// (ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_P_SC>) _param_fw_man_p_sc,
		// (ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,
		(ParamFloat<px4::params::FW_MAN_R_SC>) _param_fw_man_r_sc,
		(ParamFloat<px4::params::FW_MAN_Y_SC>) _param_fw_man_y_sc,

		// (ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
		// (ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos,
		// (ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
		(ParamFloat<px4::params::FW_PR_FF>) _param_fw_pr_ff,
		(ParamFloat<px4::params::FW_PR_I>) _param_fw_pr_i,
		(ParamFloat<px4::params::FW_PR_IMAX>) _param_fw_pr_imax,
		(ParamFloat<px4::params::FW_PR_P>) _param_fw_pr_p,
		// (ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,

		// (ParamFloat<px4::params::FW_RATT_TH>) _param_fw_ratt_th,

		// (ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
		// (ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
		// (ParamFloat<px4::params::FW_RLL_TO_YAW_FF>) _param_fw_rll_to_yaw_ff,
		(ParamFloat<px4::params::FW_RR_FF>) _param_fw_rr_ff,
		(ParamFloat<px4::params::FW_RR_I>) _param_fw_rr_i,
		(ParamFloat<px4::params::FW_RR_IMAX>) _param_fw_rr_imax,
		(ParamFloat<px4::params::FW_RR_P>) _param_fw_rr_p,
		// (ParamFloat<px4::params::FW_RSP_OFF>) _param_fw_rsp_off,

		// (ParamBool<px4::params::FW_W_EN>) _param_fw_w_en,
		// (ParamFloat<px4::params::FW_W_RMAX>) _param_fw_w_rmax,
		// (ParamFloat<px4::params::FW_WR_FF>) _param_fw_wr_ff,
		// (ParamFloat<px4::params::FW_WR_I>) _param_fw_wr_i,
		// (ParamFloat<px4::params::FW_WR_IMAX>) _param_fw_wr_imax,
		// (ParamFloat<px4::params::FW_WR_P>) _param_fw_wr_p,

		// (ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax,
		(ParamFloat<px4::params::FW_YR_FF>) _param_fw_yr_ff,
		(ParamFloat<px4::params::FW_YR_I>) _param_fw_yr_i,
		(ParamFloat<px4::params::FW_YR_IMAX>) _param_fw_yr_imax,
		(ParamFloat<px4::params::FW_YR_P>) _param_fw_yr_p,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)
};

#endif // FIXEDWINGFLATCONTROL_HPP_