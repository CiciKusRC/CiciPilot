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


	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Set the previous setpoint to the current position
	pos_sp_triplet->previous.timestamp = hrt_absolute_time();
	pos_sp_triplet->previous.lat = current_latitude;
	pos_sp_triplet->previous.lon = current_longitude;
	pos_sp_triplet->previous.alt = current_altitude;
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->previous.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	// Set the current setpoint to the current position
	pos_sp_triplet->current.timestamp = hrt_absolute_time();
	pos_sp_triplet->current.lat = 40.2342084f;
	pos_sp_triplet->current.lon = 29.0197628f;
	pos_sp_triplet->current.alt = 567.0f;
	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	_navigator->set_position_setpoint_triplet_updated();
}


