#include "kamikaze.h"
#include <mathlib/mathlib.h>
#include "navigator.h"

using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;

Kamikaze::Kamikaze(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_KAMIKAZE),
	ModuleParams(navigator)
{
}
void Kamikaze::on_activation()
{
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();
	kkz_qr_lat = _param_kkz_qr_lat.get()*M_DEG_TO_RAD_F;
	kkz_qr_lon = _param_kkz_qr_lon.get()*M_DEG_TO_RAD_F;
	kkz_approach_ang = _param_kkz_approach_ang.get();
	kkz_approach_dist = _param_kkz_approach_dist.get();
	kkz_loiter_rad = _param_kkz_loiter_rad.get();
	kkz_dive_alt = _param_kkz_dive_alt.get();
	//_local_pos = _navigator->get_local_position();
	calculate_loiter_position();
	PX4_INFO("Kamikaze mission activated");

}

void Kamikaze::on_active()
{
	parameters_update();
}


void Kamikaze::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

	}
}

void Kamikaze::calculate_loiter_position()
{
	calc_loiter_exit_position();
	float bearing = kkz_approach_ang - (atan(kkz_loiter_rad/kkz_approach_dist) * 180.0f/M_PI);
	float theta = bearing * M_DEG_TO_RAD_F;
	float distance = sqrt(kkz_approach_dist * kkz_approach_dist + kkz_loiter_rad * kkz_loiter_rad);
	float d = distance / float(CONSTANTS_RADIUS_OF_EARTH);

	loiter_lat = asin(sin(kkz_qr_lat) * cos(d) + cos(kkz_qr_lat) * sin(d) * cos(theta));
	loiter_lon = kkz_qr_lon + atan2(sin(theta) * sin(d) * cos(kkz_qr_lat),
								   cos(d) - sin(kkz_qr_lat) * sin(loiter_lat));
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.lat = exit_lat*M_RAD_TO_DEG_F;
	pos_sp_triplet->previous.lon = exit_lon*M_RAD_TO_DEG_F;
	pos_sp_triplet->previous.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	pos_sp_triplet->previous.alt = kkz_dive_alt+_navigator->get_home_position()->alt;
	pos_sp_triplet->current.lat = loiter_lat*M_RAD_TO_DEG_F;
	pos_sp_triplet->current.lon = loiter_lon*M_RAD_TO_DEG_F;
	pos_sp_triplet->current.alt = kkz_dive_alt+_navigator->get_home_position()->alt;
	pos_sp_triplet->current.loiter_radius = kkz_loiter_rad;
	pos_sp_triplet->current.loiter_direction_counter_clockwise = _param_kkz_loiter_dir.get();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	pos_sp_triplet->next.lat = kkz_qr_lat*M_RAD_TO_DEG_F;
	pos_sp_triplet->next.lon = kkz_qr_lon*M_RAD_TO_DEG_F;
	pos_sp_triplet->next.alt = kkz_dive_alt+_navigator->get_home_position()->alt;
	pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

}

void
Kamikaze::calc_loiter_exit_position(){
	float theta = _param_kkz_approach_ang.get() * M_DEG_TO_RAD_F;
	float d = _param_kkz_approach_dist.get() / float(CONSTANTS_RADIUS_OF_EARTH);
	exit_lat = asin(sin(_param_kkz_qr_lat.get()*M_DEG_TO_RAD_F) * cos(d) + cos(_param_kkz_qr_lat.get()*M_DEG_TO_RAD_F) * sin(d) * cos(theta));
	exit_lon = _param_kkz_qr_lon.get()*M_DEG_TO_RAD_F + atan2(sin(theta) * sin(d) * cos(_param_kkz_qr_lat.get()*M_DEG_TO_RAD_F),
								   cos(d) - sin(_param_kkz_qr_lat.get()*M_DEG_TO_RAD_F) * sin(exit_lat));
}

