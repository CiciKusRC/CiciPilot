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
 * @file swarm.cpp
 *
 * Swarm mode implementation
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
#include <uORB/Publication.hpp>
#include <uORB/topics/multi_vehicle_location.h>

using namespace time_literals;

class Swarm : public MissionBlock , public ModuleParams
{
public:
	Swarm(Navigator *navigator);
	~Swarm() = default;
	void on_activation() override;
	void on_active() override;
	//void on_inactivation() override;
private:
	float vehicle1_lat{0.0f}; // Vehicle 1 latitude in degrees
	float vehicle1_lon{0.0f}; // Vehicle 1 longitude in degrees
	float vehicle1_alt{0.0f}; // Vehicle 1 altitude in meters
	/**
	 * @brief Parameters update
	 *
	 * Check for parameter changes and update them if needed.
	 */


	matrix::Vector3f calculate_wingman_position(const matrix::Vector3f &leader_position, float offset);
	void calculate_loiter_position();
	void calc_loiter_exit_position();

   	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SW_FOLLOW_OFFSET>) _param_sw_follow_offset,
		(ParamFloat<px4::params::SW_FOLLOW_SIDE>) _param_sw_follow_side
	)

	void parameters_update();
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _multi_vehicle_location_sub{ORB_ID(multi_vehicle_location)};
};
