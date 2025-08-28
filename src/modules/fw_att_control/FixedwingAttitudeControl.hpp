/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include "fw_pitch_controller.h"
#include "fw_roll_controller.h"
#include "fw_wheel_controller.h"
#include "fw_yaw_controller.h"
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/landing_gear_wheel.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/target_location.h>
#include <uORB/topics/visual_location.h>
#include <uORB/topics/intercept_roll_pitch.h>

using matrix::Eulerf;
using matrix::Quatf;

using uORB::SubscriptionData;

using namespace time_literals;
#define FOCAL_LENGTH 2.75f // mm
#define SENSOR_WIDTH 6.45f // mm
#define SENSOR_HEIGHT 3.63f // mm
#define RESOLUTION_WIDTH 1536
#define RESOLUTION_HEIGHT 864
class FixedwingAttitudeControl final : public ModuleBase<FixedwingAttitudeControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FixedwingAttitudeControl(bool vtol = false);
	~FixedwingAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;
	float roll_pitch_ref[2];
	float x_cam{0.f};
	float y_cam{0.f};
	float z_cam{0.f};
	float target_x{0.f};
	float target_y{0.f};
	float target_roll{0.f};
	float target_pitch{0.f};
	float x_err_center{0.f};
	float y_err_center{0.f};
	float norm{0.f};
	hrt_abstime _last_time_att_control_called{0};
	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};		/**< vehicle attitude */

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};			/**< vehicle attitude setpoint */
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};			/**< local position subscription */
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};	/**< notification of manual control updates */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};			/**< vehicle status subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};		/**< vehicle land detected subscription */
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};				/**< vehicle status subscription */
	uORB::Subscription _vehicle_rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _target_location_sub{ORB_ID(target_location)};			/**< target location subscription */
	uORB::Subscription _intercept_roll_pitch_sub{ORB_ID(intercept_roll_pitch)};		/**< intercept roll pitch subscription */
	uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};

	uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub;
	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<landing_gear_wheel_s>		_landing_gear_wheel_pub{ORB_ID(landing_gear_wheel)};
	uORB::Publication<visual_location_s>	_visual_location_pub{ORB_ID(visual_location)};

	manual_control_setpoint_s		_manual_control_setpoint{};
	vehicle_attitude_setpoint_s		_att_sp{};
	vehicle_control_mode_s			_vcontrol_mode{};
	vehicle_rates_setpoint_s		_rates_sp{};
	vehicle_status_s			_vehicle_status{};
	landing_gear_wheel_s			_landing_gear_wheel{};
	target_location_s			_target_location{};
	visual_location_s			_visual_location{};
	intercept_roll_pitch_s			_intercept_roll_pitch{};
	matrix::Dcmf _R{matrix::eye<float, 3>()};

	perf_counter_t _loop_perf;

	hrt_abstime _last_run{0};

	bool _landed{true};
	float _groundspeed{0.f};
	bool _in_fw_or_transition_wo_tailsitter_transition{false}; // only run the FW attitude controller in these states

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamBool<px4::params::FW_USE_AIRSPD>) _param_fw_use_airspd,

		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,

		(ParamFloat<px4::params::FW_ROLL_SCALE>) _param_fw_roll_scale,
		(ParamFloat<px4::params::FW_PITCH_SCALE>) _param_fw_pitch_scale,

		(ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
		(ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos,
		(ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
		(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
		(ParamFloat<px4::params::FW_PN_R_SLEW_MAX>) _param_fw_pn_r_slew_max,
		(ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
		(ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
		(ParamBool<px4::params::FW_VIS_NAV_EN>) _param_fw_vis_nav_en,
		(ParamBool<px4::params::FW_W_EN>) _param_fw_w_en,
		(ParamFloat<px4::params::FW_W_RMAX>) _param_fw_w_rmax,
		(ParamFloat<px4::params::FW_WR_FF>) _param_fw_wr_ff,
		(ParamFloat<px4::params::FW_WR_I>) _param_fw_wr_i,
		(ParamFloat<px4::params::FW_WR_IMAX>) _param_fw_wr_imax,
		(ParamFloat<px4::params::FW_WR_P>) _param_fw_wr_p,

		(ParamFloat<px4::params::FW_Y_RMAX>) _param_fw_y_rmax,
		(ParamFloat<px4::params::FW_MAN_YR_MAX>) _param_man_yr_max,
		(ParamFloat<px4::params::X_ERROR_KP>) _param_x_error_kp,
		(ParamFloat<px4::params::X_ERROR_KI>) _param_x_error_ki,
		(ParamFloat<px4::params::X_INT_LIM>) _param_x_int_lim,
		(ParamBool<px4::params::INT_RESET_X>) _param_int_reset_x,
		(ParamFloat<px4::params::ROLL_LIM_X>) _param_roll_lim_x,
		(ParamFloat<px4::params::Y_ERROR_KP>) _param_y_error_kp,
		(ParamFloat<px4::params::Y_ERROR_KI>) _param_y_error_ki,
		(ParamFloat<px4::params::Y_INT_LIM>) _param_y_int_lim,
		(ParamBool<px4::params::INT_RESET_Y>) _param_int_reset_y,
		(ParamFloat<px4::params::ROLL_LIM_Y>) _param_roll_lim_y

	)
	static constexpr float MIN_AUTO_TIMESTEP = 0.01f;
	// [s] maximum time step between auto control updates
	static constexpr float MAX_AUTO_TIMESTEP = 0.05f;
	RollController _roll_ctrl;
	PitchController _pitch_ctrl;
	YawController _yaw_ctrl;
	WheelController _wheel_ctrl;
	SlewRate<float> _roll_vis_slew_rate;
	SlewRate<float> _pitch_vis_slew_rate;
	void parameters_update();
	void vehicle_manual_poll(const float yaw_body);
	void vision_based_nav(const float yaw_body,const float control_interval,const float current_roll, const float current_pitch);
	void computeDirectionVector(const float dt, const float target_x, const float target_y,const float current_roll, const float current_pitch);
	void vehicle_attitude_setpoint_poll();
	void vehicle_land_detected_poll();
	float get_airspeed_constrained();
};
