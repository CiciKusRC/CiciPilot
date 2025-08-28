#include "intercept.h"
#include "navigator.h"


Intercept::Intercept(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_INTERCEPT),
	ModuleParams(navigator)
{
}


void Intercept::on_activation()
{
	PX4_INFO("Intercept mode active");
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();

}

void Intercept::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

	}
}


void Intercept::on_active()
{


	parameters_update();
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// If any parameter updated, call updateParams() to check if
		// this class attributes need updating (and do so).
		updateParams();

	}
	if(_global_pos_sub.updated()){
		vehicle_global_position_s global_pos;
		_global_pos_sub.copy(&global_pos);
		current_latitude = global_pos.lat;
		current_longitude = global_pos.lon;
		current_altitude = global_pos.alt;
	}
	if(_target_location_lla_sub.updated()){
		target_location_lla_s target_location;
		_target_location_lla_sub.copy(&target_location);
		target_lat = target_location.target_lat;
		target_lon = target_location.target_lon;
		target_alt = target_location.target_alt;
	}
	if(_multi_vehicle_location_sub.updated()){
		_multi_vehicle_location_sub.copy(&_multi_vehicle_location);
	}

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->timestamp = hrt_absolute_time();


	// Set the previous setpoint to the current position
	pos_sp_triplet->previous.lat = current_latitude;
	pos_sp_triplet->previous.lon = current_longitude;
	pos_sp_triplet->previous.alt = current_altitude;
	pos_sp_triplet->previous.valid = true;

	// Set the current setpoint to the current position
	pos_sp_triplet->current.lat = _multi_vehicle_location.vehicle_1_latitude / 1e7;
	pos_sp_triplet->current.lon = _multi_vehicle_location.vehicle_1_longitude / 1e7;
	pos_sp_triplet->current.alt = (_multi_vehicle_location.vehicle_1_altitude * 0.001f) + _navigator->get_home_position()->alt;
	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	double offset_distance = 50.0; // meters
	double heading_rad = 0.0; // 0 degree in radians

	// Earth's radius in meters
	constexpr double earth_radius = 6378137.0;

	// Convert current lat/lon to radians
	double lat_rad = pos_sp_triplet->current.lat * M_PI / 180.0;
	double lon_rad = pos_sp_triplet->current.lon * M_PI / 180.0;

	// Calculate new latitude
	double new_lat_rad = lat_rad + (offset_distance / earth_radius) * cos(heading_rad);
	// Calculate new longitude
	double new_lon_rad = lon_rad + (offset_distance / (earth_radius * cos(lat_rad))) * sin(heading_rad);

	// Convert back to degrees
	double offset_lat = new_lat_rad * 180.0 / M_PI;
	double offset_lon = new_lon_rad * 180.0 / M_PI;
	double offset_alt = pos_sp_triplet->current.alt; // keep altitude same

	pos_sp_triplet->next.valid = true;
	pos_sp_triplet->next.lat = offset_lat;
	pos_sp_triplet->next.lon = offset_lon;
	pos_sp_triplet->next.alt = offset_alt;


	_navigator->set_position_setpoint_triplet_updated();
}


