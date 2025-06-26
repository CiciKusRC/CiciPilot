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

	multi_vehicle_location_s multi_vehicle_location;
	if (_multi_vehicle_location_sub.update(&multi_vehicle_location)) {
		// Assume vehicle_1 is the leader
		vehicle1_lat = multi_vehicle_location.vehicle_1_latitude * 1e-7;
		vehicle1_lon = multi_vehicle_location.vehicle_1_longitude * 1e-7;
		vehicle1_alt = multi_vehicle_location.vehicle_1_altitude * 0.001f; // mm to meters
		vehicle1_heading = multi_vehicle_location.vehicle_1_heading * 0.01f; // cdeg to degrees
		// Store or use leader position as needed
		_leader_position(0) = vehicle1_lat;
		_leader_position(1) = vehicle1_lon;
		_leader_position(2) = vehicle1_alt;
	}
	// Calculate wingman position based on leader position and parameters
	float follow_offset = _param_sw_follow_offset.get();
	int follow_side = static_cast<int>(_param_sw_follow_side.get()); // 0=left, 1=right, 2=back
	float leader_heading_deg = vehicle1_heading; // Use vehicle 1 heading
	Vector3f wingman_position;
	// Calculate the wingman position based on the leader's position, heading, and follow parameters
	calculate_wingman_position(_leader_position, follow_offset, follow_side, leader_heading_deg, wingman_position);
	_navigator->reset_cruising_speed();
	_navigator->set_cruising_throttle();
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

void Swarm::calculate_wingman_position(const Vector3f &leader_position, float offset, int follow_side, float leader_heading_deg, Vector3f &wingman_position)
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	// Constants
	const float earth_radius = 6378137.0f; // meters

	// Convert leader lat/lon from degrees to radians
	float lat_rad = math::radians(leader_position(0));
	float lon_rad = math::radians(leader_position(1));

	// Offset direction: 0=left, 1=right, 2=back
	float bearing_offset_deg = 0.0f;
	switch (follow_side) {
		case 0: // left
			bearing_offset_deg = -90.0f;
			break;
		case 1: // right
			bearing_offset_deg = 90.0f;
			break;
		case 2: // back
			bearing_offset_deg = 180.0f;
			break;
		default:
			bearing_offset_deg = 180.0f;
			break;
	}

	// Use provided leader heading
	float bearing = math::radians(leader_heading_deg + bearing_offset_deg);

	// Calculate new lat/lon using simple equirectangular approximation
	float d_lat = (offset * cosf(bearing)) / earth_radius;
	float d_lon = (offset * sinf(bearing)) / (earth_radius * cosf(lat_rad));

	float wingman_lat = lat_rad + d_lat;
	float wingman_lon = lon_rad + d_lon;
	float wingman_alt = leader_position(2); // same AGL altitude

	// Convert back to degrees
	wingman_position(0) = math::degrees(wingman_lat);
	wingman_position(1) = math::degrees(wingman_lon);
	wingman_position(2) = wingman_alt;
	pos_sp_triplet->timestamp = hrt_absolute_time();

	// Fill current (previous) with current vehicle location
	pos_sp_triplet->previous.valid = true;
	pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;

	// Fill current (target) with calculated wingman position
	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.lat = wingman_position(0);
	pos_sp_triplet->current.lon = wingman_position(1);
	pos_sp_triplet->current.alt = wingman_position(2);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	// Fill next with leader position
	pos_sp_triplet->next.valid = true;
	pos_sp_triplet->next.lat = leader_position(0);
	pos_sp_triplet->next.lon = leader_position(1);
	pos_sp_triplet->next.alt = leader_position(2);
	_navigator->set_position_setpoint_triplet_updated();

	wingman_position_s wingman_position_msg{};
	wingman_position_msg.timestamp = hrt_absolute_time();
	wingman_position_msg.wingman_latitude = wingman_position(0) * 1e7; // Convert to degE7
	wingman_position_msg.wingman_longitude = wingman_position(1) * 1e7; // Convert to degE7
	wingman_position_msg.wingman_altitude = wingman_position(2) * 1000.0f; // Convert to mm
	// Publish the wingman position
	_wingman_position_pub.publish(wingman_position_msg);
	// Publish the setpoint triplet
}
