#include "swarm.h"
#include <mathlib/mathlib.h>
#include "navigator.h"


using matrix::Vector2f;
using matrix::Vector2d;
using matrix::Vector3f;

Swarm::Swarm(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_SWARM),
	ModuleParams(navigator)
{
}
void Swarm::on_activation()
{
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();
	calculate_loiter_position();
	PX4_INFO("Swarm mode on activation");

}

void Swarm::on_active()
{
	PX4_INFO("Swarm mode on active");
	parameters_update();
	multi_vehicle_location_s multi_vehicle_location;

	if (_multi_vehicle_location_sub.update(&multi_vehicle_location)) {
		// Vehicle 1 latitude and longitude are in degE7, altitude in mm
		vehicle1_lat = multi_vehicle_location.vehicle_1_latitude * 1e-7;
		vehicle1_lon = multi_vehicle_location.vehicle_1_longitude * 1e-7;
		vehicle1_alt = multi_vehicle_location.vehicle_1_altitude * 0.001f; // convert mm to meters

	}
}

void Swarm::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

	}
}

void
Swarm::calculate_loiter_position()
{
	calc_loiter_exit_position();
	float bearing = 5.0f - (atan(50.0f/200.0f) * 180.0f/M_PI);
	float theta = bearing * M_DEG_TO_RAD_F;
	float distance = sqrt(200.0f * 200.0f + 50.0f * 50.0f);
	float d = distance / float(CONSTANTS_RADIUS_OF_EARTH);

	float loiter_lat = asin(sin(40.8030093) * cos(d) + cos(40.8030093) * sin(d) * cos(theta));
	float loiter_lon = 29.7374621 + atan2(sin(theta) * sin(d) * cos(40.8030093),
								   cos(d) - sin(40.8030093) * sin(loiter_lat));
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.lat = 40.8030093*M_RAD_TO_DEG_F;
	pos_sp_triplet->previous.lon = 29.7374621*M_RAD_TO_DEG_F;
	pos_sp_triplet->previous.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	pos_sp_triplet->previous.alt = 100.0f+_navigator->get_home_position()->alt;
	pos_sp_triplet->current.lat = loiter_lat*M_RAD_TO_DEG_F;
	pos_sp_triplet->current.lon = loiter_lon*M_RAD_TO_DEG_F;
	pos_sp_triplet->current.alt = 100.0f+_navigator->get_home_position()->alt;
	pos_sp_triplet->current.loiter_radius = 50.0f;
	pos_sp_triplet->current.loiter_direction_counter_clockwise = 0.0f;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	pos_sp_triplet->next.lat = 40.8030093*M_RAD_TO_DEG_F;
	pos_sp_triplet->next.lon = 29.7374621*M_RAD_TO_DEG_F;
	pos_sp_triplet->next.alt = 100.0f+_navigator->get_home_position()->alt;
	pos_sp_triplet->next.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();
}

void
Swarm::calc_loiter_exit_position(){
	float theta = 5.0f * M_DEG_TO_RAD_F;
	float d = 200.0f / float(CONSTANTS_RADIUS_OF_EARTH);
}

