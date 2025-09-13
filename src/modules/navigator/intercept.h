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
 * @file intercept.cpp
 * @brief Intercept mode for a given target position. This mode developed for one of the mission in TEKNOFEST SAVAŞAN İHA UAV Competition
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
#include <uORB/topics/target_location_lla.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/multi_vehicle_location.h>
#include <uORB/topics/target_uav_location.h>
#include <uORB/Publication.hpp>


using namespace time_literals;
class Intercept : public MissionBlock , public ModuleParams
{
public:
	Intercept(Navigator *navigator);
	~Intercept() = default;
	void on_activation() override;
	void on_active() override;
private:

	vehicle_local_position_s *_local_pos{nullptr};
	multi_vehicle_location_s _multi_vehicle_location{};
	float target_lat{0.0f};
	float target_lon{0.0f};
	float target_alt{0.0f};
	float current_latitude{0.0f};
	float current_longitude{0.0f};
	float current_altitude{0.0f};

	/**
	 * @brief Parameters update
	 *
	 * Check for parameter changes and update them if needed.
	 */

	/**
	 * @brief Loiter counter
	 *
	 * Count loiter to switch approach phase
	 */


	// DEFINE_PARAMETERS(
	// 	//(ParamFloat<px4::params::KKZ_QR_LAT>) _param_kkz_qr_lat,
	// )

	void parameters_update();
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::Subscription _target_location_lla_sub{ORB_ID(target_location_lla)};	/**< target location subscription */
	uORB::Subscription _multi_vehicle_location_sub{ORB_ID(multi_vehicle_location)};	/**< target location subscription */
	uORB::Subscription _target_uav_location_sub{ORB_ID(target_uav_location)};

};
