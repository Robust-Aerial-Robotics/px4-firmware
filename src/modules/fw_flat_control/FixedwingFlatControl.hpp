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
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>

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

	orb_advert_t	_mavlink_log_pub{nullptr};

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};	/**< vehicle attitude */

	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};					///< control mode subscription
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	///< notification of manual control updates
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};						///< vehicle status subscription

	uORB::Publication<actuator_controls_s>		_actuators_0_pub;

	actuator_controls_s			_actuators {};											///< actuator control inputs
	manual_control_setpoint_s	_manual_control_setpoint {};							///< r/c channel data
	vehicle_control_mode_s		_vcontrol_mode {};										///< control mode
	vehicle_status_s		_vehicle_status {};											///< vehicle status

	perf_counter_t	_loop_perf;															///< loop performance counter

	bool _started{false};
	bool _started_manual{false};

	DEFINE_PARAMETERS(
		// (ParamFloat<px4::params::FW_ACRO_X_MAX>) _param_fw_acro_x_max,
		// (ParamFloat<px4::params::FW_ACRO_Y_MAX>) _param_fw_acro_y_max,
		// (ParamFloat<px4::params::FW_ACRO_Z_MAX>) _param_fw_acro_z_max,

		// (ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		// (ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		// (ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		// (ParamInt<px4::params::FW_ARSP_MODE>) _param_fw_arsp_mode,

		// (ParamInt<px4::params::FW_ARSP_SCALE_EN>) _param_fw_arsp_scale_en,

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
		// (ParamFloat<px4::params::FW_PR_FF>) _param_fw_pr_ff,
		// (ParamFloat<px4::params::FW_PR_I>) _param_fw_pr_i,
		// (ParamFloat<px4::params::FW_PR_IMAX>) _param_fw_pr_imax,
		// (ParamFloat<px4::params::FW_PR_P>) _param_fw_pr_p,
		// (ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,

		// (ParamFloat<px4::params::FW_RATT_TH>) _param_fw_ratt_th,

		// (ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
		// (ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
		// (ParamFloat<px4::params::FW_RLL_TO_YAW_FF>) _param_fw_rll_to_yaw_ff,
		// (ParamFloat<px4::params::FW_RR_FF>) _param_fw_rr_ff,
		// (ParamFloat<px4::params::FW_RR_I>) _param_fw_rr_i,
		// (ParamFloat<px4::params::FW_RR_IMAX>) _param_fw_rr_imax,
		// (ParamFloat<px4::params::FW_RR_P>) _param_fw_rr_p,
		// (ParamFloat<px4::params::FW_RSP_OFF>) _param_fw_rsp_off,

		// (ParamBool<px4::params::FW_W_EN>) _param_fw_w_en,
		// (ParamFloat<px4::params::FW_W_RMAX>) _param_fw_w_rmax,
		// (ParamFloat<px4::params::FW_WR_FF>) _param_fw_wr_ff,
		// (ParamFloat<px4::params::FW_WR_I>) _param_fw_wr_i,
		// (ParamFloat<px4::params::FW_WR_IMAX>) _param_fw_wr_imax,
		// (ParamFloat<px4::params::FW_WR_P>) _param_fw_wr_p,

		// (ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax,
		// (ParamFloat<px4::params::FW_YR_FF>) _param_fw_yr_ff,
		// (ParamFloat<px4::params::FW_YR_I>) _param_fw_yr_i,
		// (ParamFloat<px4::params::FW_YR_IMAX>) _param_fw_yr_imax,
		// (ParamFloat<px4::params::FW_YR_P>) _param_fw_yr_p,

		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)
};

#endif // FIXEDWINGFLATCONTROL_HPP_