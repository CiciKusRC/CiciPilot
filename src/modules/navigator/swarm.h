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
#include <uORB/topics/wingman_position.h>


using matrix::Vector3f;
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
	float vehicle1_heading{0.0f}; // Vehicle 1 heading in degrees
	matrix::Vector3f _leader_position{}; // Vehicle 1 position: x=lat, y=lon, z=alt

	/**
	 * @brief Parameters update
	 *
	 * Check for parameter changes and update them if needed.
	 */


	/**
	 * @brief Calculates the wingman position based on leader's position, offset, side, and heading.
	 *
	 * @param leader_pos The position of the leader (x=lat, y=lon, z=alt).
	 * @param offset The distance offset from the leader.
	 * @param follow_side The side to follow (-1 for left, 1 for right).
	 * @param leader_heading_deg The heading of the leader in degrees.
	 * @param wingman_position Output parameter for the calculated wingman position.
	 */
	void calculate_wingman_position(const Vector3f &leader_pos, float offset, int follow_side, float leader_heading_deg, Vector3f &wingman_position);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SW_FOLLOW_OFFSET>) _param_sw_follow_offset,
		(ParamFloat<px4::params::SW_FOLLOW_SIDE>) _param_sw_follow_side,
		(ParamFloat<px4::params::SW_WAIT_LAT>) _param_sw_wait_lat,
		(ParamFloat<px4::params::SW_WAIT_LON>) _param_sw_wait_lon,
		(ParamFloat<px4::params::SW_ZONE1_LAT>) _param_sw_zone1_lat,
		(ParamFloat<px4::params::SW_ZONE1_LON>) _param_sw_zone1_lon,
		(ParamFloat<px4::params::SW_ZONE2_LAT>) _param_sw_zone2_lat,
		(ParamFloat<px4::params::SW_ZONE2_LON>) _param_sw_zone2_lon,
		(ParamFloat<px4::params::SW_ZONE3_LAT>) _param_sw_zone3_lat,
		(ParamFloat<px4::params::SW_ZONE3_LON>) _param_sw_zone3_lon,
		(ParamFloat<px4::params::SW_ZONE4_LAT>) _param_sw_zone4_lat,
		(ParamFloat<px4::params::SW_ZONE4_LON>) _param_sw_zone4_lon,
		(ParamFloat<px4::params::SW_AREA1_LAT>) _param_sw_area1_lat,
		(ParamFloat<px4::params::SW_AREA1_LON>) _param_sw_area1_lon,
		(ParamFloat<px4::params::SW_AREA2_LAT>) _param_sw_area2_lat,
		(ParamFloat<px4::params::SW_AREA2_LON>) _param_sw_area2_lon,
		(ParamFloat<px4::params::SW_AREA3_LAT>) _param_sw_area3_lat,
		(ParamFloat<px4::params::SW_AREA3_LON>) _param_sw_area3_lon,
		(ParamFloat<px4::params::SW_AREA4_LAT>) _param_sw_area4_lat,
		(ParamFloat<px4::params::SW_AREA4_LON>) _param_sw_comp_area4_lon
	)

	void parameters_update();
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _multi_vehicle_location_sub{ORB_ID(multi_vehicle_location)};
	uORB::Publication<wingman_position_s> _wingman_position_pub{ORB_ID(wingman_position)};
};
