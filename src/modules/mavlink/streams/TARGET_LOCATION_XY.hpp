/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef TARGET_LOCATION_XY_HPP
#define TARGET_LOCATION_XY_HPP


#include <uORB/topics/target_location.h>


class MavlinkStreamTargetLocation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTargetLocation(mavlink); }

	static constexpr const char *get_name_static() { return "TARGET_LOCATION_XY"; }
	static constexpr uint16_t get_id_static() { return 443; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _target_location_xy_sub.advertised() ? MAVLINK_MSG_ID_TARGET_LOCATION_XY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}
private:
	explicit MavlinkStreamTargetLocation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _target_location_xy_sub{ORB_ID(target_location)};

	bool send() override
	{
		target_location_s msg;
		if (_target_location_xy_sub.update(&msg)) {
			mavlink_target_location_xy_t mav_msg{};
			mav_msg.timestamp = msg.timestamp;
			mav_msg.target_x = msg.target_x;
			mav_msg.target_y = msg.target_y;
			// Convert bool to int8_t for MAVLink
			mav_msg.tracker_status = static_cast<int8_t>(msg.tracker_status);

			mavlink_msg_target_location_xy_send_struct(_mavlink->get_channel(), &mav_msg);
			return true;
		}
		return false;
	}
};
#endif // TARGET_LOCATION_XY_HPP
