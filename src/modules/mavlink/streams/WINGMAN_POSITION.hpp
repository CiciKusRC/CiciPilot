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

#ifndef WINGMAN_POSITION_HPP
#define WINGMAN_POSITION_HPP

#include <uORB/topics/wingman_position.h>

class MavlinkStreamWingmanPosition : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamWingmanPosition(mavlink); }

	static constexpr const char *get_name_static() { return "WINGMAN_POSITION"; }
	static constexpr uint16_t get_id_static() { return 58; } // Replace with MAVLINK_MSG_ID_WINGMAN_POSITION if defined

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _wingman_position_sub.advertised() ? MAVLINK_MSG_ID_WINGMAN_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamWingmanPosition(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _wingman_position_sub{ORB_ID(wingman_position)};

	bool send() override
	{
		wingman_position_s wingman_position;

		if (_wingman_position_sub.update(&wingman_position)) {
			mavlink_wingman_position_t msg{};
			msg.latitude  = static_cast<float>(wingman_position.wingman_latitude);
			msg.longitude = static_cast<float>(wingman_position.wingman_longitude);
			msg.altitude  = static_cast<float>(wingman_position.wingman_altitude);
			msg.heading   = static_cast<float>(wingman_position.wingman_heading);

			mavlink_msg_wingman_position_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // WINGMAN_POSITION_HPP
