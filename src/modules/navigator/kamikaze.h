/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file kamikaze.cpp
 *
 * Kamikaze mission for a given target position. This mode developed for one of the mission in TEKNOFEST SAVAŞAN İHA UAV Competition
 * held in Türkiye. So applied logic specific for competition conditions. Detailed information for the competition and kamikaze mission
 *
 * https://cdn.teknofest.org/media/upload/userFormUpload/2025_SAVASAN_%C4%B0HA_YARISMA_SARTNAMES%C4%B0_TR_nbdHb.pdf
 * For up-do-date information please check the official website of the competition.
 * https://teknofest.org/tr/competitions/competition/33
 */

#pragma once
#include "navigator_mode.h"
#include "mission_block.h"
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
using namespace time_literals;

class Kamikaze : public	MissionBlock , public ModuleParams
{
public:
	Kamikaze(Navigator *navigator);
	~Kamikaze() = default;
	void on_activation() override;
	void on_active() override;
	//void on_inactivation() override;
private:

	vehicle_local_position_s *_local_pos{nullptr};
	float loiter_lat{0.0f};
	float loiter_lon{0.0f};

	float exit_lon{0.0f};
	float exit_lat{0.0f};

	float kkz_qr_lat{0.0f};
	float kkz_qr_lon{0.0f};
	float kkz_approach_ang{0.0f};
	float kkz_approach_dist{0.0f};
	float kkz_loiter_rad{0.0f};
	float kkz_dive_alt{0.0f};
	float _theta_bearing{0.0f};
	float _distance_to_target{0.0f};
	float _d{0.0f};
	/**
	 * @brief Parameters update
	 *
	 * Check for parameter changes and update them if needed.
	 */


	/**
	 * @brief Calculate the loiter position
	 *
	 * Calculate the loiter position based on the qr position, bearing and the loiter radius.
	 */
	void calculate_loiter_position();

	/**
	 * Calculate the loiter exit position where vehicle will start approach to target position.
	 */
	void calc_loiter_exit_position();

	/**
	 * @brief Loiter counter
	 *
	 * Count loiter to switch approach phase
	 */


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::KKZ_QR_LAT>) _param_kkz_qr_lat,
		(ParamFloat<px4::params::KKZ_QR_LON>) _param_kkz_qr_lon,
		(ParamFloat<px4::params::KKZ_DIVE_ALT>) _param_kkz_dive_alt,
		(ParamFloat<px4::params::KKZ_REC_ALT>) _param_kkz_rec_alt,
		(ParamFloat<px4::params::KKZ_APPROACH_ANG>) _param_kkz_approach_ang,
		(ParamFloat<px4::params::KKZ_APP_DIST>) _param_kkz_approach_dist,
		(ParamFloat<px4::params::KKZ_LOITER_RAD>) _param_kkz_loiter_rad,
		(ParamInt<px4::params::KKZ_LOITER_DIR>) _param_kkz_loiter_dir,
		(ParamFloat<px4::params::KKZ_DIVE_ANG>) _param_kkz_dive_ang,
		(ParamFloat<px4::params::KKZ_RET_LAT>) _param_kkz_ret_lat,
		(ParamFloat<px4::params::KKZ_RET_LON>) _param_kkz_ret_lon,
		(ParamFloat<px4::params::HEADING_RANGE>) _param_heading_range,
		(ParamFloat<px4::params::TARGET_DIST_SP>) _param_target_dist_sp
	)

	void parameters_update();
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
};
