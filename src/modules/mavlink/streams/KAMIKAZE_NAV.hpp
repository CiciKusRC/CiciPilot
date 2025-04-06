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

#ifndef KAMIKAZE_NAV_HPP
#define KAMIKAZE_NAV_HPP
#include <uORB/topics/kamikaze_visualize.h>

class MavlinkStreamKamikazeNav : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKamikazeNav(mavlink); }

	static constexpr const char *get_name_static() { return "KAMIKAZE_NAV"; }
	static constexpr uint16_t get_id_static() { return 442; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _kamikaze_visualize_sub.advertised() ? MAVLINK_MSG_ID_KAMIKAZE_NAV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamKamikazeNav(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _kamikaze_visualize_sub{ORB_ID(kamikaze_visualize)};

	bool send() override
	{
		kamikaze_visualize_s kamikaze_visualize;

		if (_kamikaze_visualize_sub.update(&kamikaze_visualize)) {
			mavlink_kamikaze_nav_t msg{};
			msg.timestamp = kamikaze_visualize.timestamp;
			msg.exit_lat = kamikaze_visualize.exit_lat;
			msg.exit_lon = kamikaze_visualize.exit_lon;
			msg.loiter_lat = kamikaze_visualize.loiter_lat;
			msg.loiter_lon = kamikaze_visualize.loiter_lon;
			msg.kamikaze_status = kamikaze_visualize.kamikaze_status;

			mavlink_msg_kamikaze_nav_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		//return false;
	}
};

#endif // KAMIKAZE_NAV_HPP
