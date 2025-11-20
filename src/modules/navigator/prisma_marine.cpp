#include "prisma_marine.h"
#include "navigator.h"

#include <cmath>

Marine::Marine(Navigator *navigator) : MissionBlock(navigator), ModuleParams(nullptr)
{
	// Use defaults from parameters
	// _speed_sp = _p_speed_def.get();
	// _acceptance_radius = _p_wp_radius.get();
    PX4_INFO("Marine mode started");
}

void Marine::on_activation()
{
    PX4_INFO("Marine mode activated");
	// Reset state and publish the very first guidance sample
	_reached = false;
	_last_pub = 0;
	publish_guidance();
}

void Marine::on_active()
{
	// Update current global position
	vehicle_global_position_s gpos{};
	if (_gpos_sub.update(&gpos)) {
		if (PX4_ISFINITE(_lat_target_deg) && PX4_ISFINITE(_lon_target_deg) &&
		    PX4_ISFINITE(gpos.lat) && PX4_ISFINITE(gpos.lon)) {

			const float dist_m = get_distance_to_next_waypoint(
				gpos.lat, gpos.lon, _lat_target_deg, _lon_target_deg);

			// Check acceptance
			if (dist_m <= _acceptance_radius) {
				_reached = true;
			}
		}
	}

	// Throttle publication (e.g. 5 Hz)
	if (hrt_elapsed_time(&_last_pub) > 200_ms) {
		publish_guidance();
	}
}

void Marine::on_inactive()
{
	// Send a final message marking autonomy off (optional)
	// vehicle_marine_setpoint_s sp{};
	// sp.timestamp = hrt_absolute_time();
	// sp.autonomous = false;
	// sp.lat = _lat_target_deg;
	// sp.lon = _lon_target_deg;
	// sp.speed_m_s = 0.f;
	// sp.acceptance_radius = _acceptance_radius;
	// _sp_pub.publish(sp);
	//PX4_INFO("Marine mode deactivated");
}

void Marine::set_target(double lat_deg, double lon_deg)
{
	_lat_target_deg = lat_deg;
	_lon_target_deg = lon_deg;
	_reached = false; // new target implies not reached
}

void Marine::set_speed(float speed_m_s)
{
	_speed_sp = speed_m_s;
}

void Marine::set_acceptance_radius(float radius_m)
{
	_acceptance_radius = radius_m;
}

void Marine::reset()
{
	_lat_target_deg = NAN;
	_lon_target_deg = NAN;
	_reached = false;
}

void Marine::publish_guidance()
{
	vehicle_marine_setpoint_s sp{};
	sp.timestamp = hrt_absolute_time();
	sp.autonomous = true;
	sp.lat = _lat_target_deg;
	sp.lon = _lon_target_deg;
	sp.speed_m_s = _speed_sp;
	sp.acceptance_radius = _acceptance_radius;

	_sp_pub.publish(sp);
	_last_pub = sp.timestamp;
}
