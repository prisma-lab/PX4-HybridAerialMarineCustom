#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_marine_setpoint.h>

/**
 * Marine mode
 *
 * Minimal guidance layer for autonomous marine segments.
 * It keeps publishing a marine setpoint (lat/lon, speed, acceptance radius)
 * that a dedicated marine controller module can consume.
 */

class Navigator;

class Marine : public MissionBlock, public ModuleParams
{
public:
	Marine(Navigator *navigator);
	~Marine() = default;

	// NavigatorMode interface
	void on_activation() override;
	void on_active() override;

	// Configure target
	void set_target(double lat_deg, double lon_deg);
	void set_speed(float speed_m_s);
	void set_acceptance_radius(float radius_m);

	// Report if the target has been reached (distance <= acceptance radius)
	bool reached() const { return _reached; }

	// Reset internal state
	void reset();

private:
	void publish_guidance();

	// Subscriptions
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};

	// Publications
	uORB::Publication<vehicle_marine_setpoint_s> _sp_pub{ORB_ID(vehicle_marine_setpoint)};

	// Target and params
	double _lat_target_deg{NAN};
	double _lon_target_deg{NAN};
	float  _speed_sp{0.0f};
	float  _acceptance_radius{5.0f};

	// State
	bool _reached{false};
	hrt_abstime _last_pub{0};

	// Params
	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::MAR_SPEED>)     _p_speed_def,
	// 	(ParamFloat<px4::params::MAR_WP_RADIUS>) _p_wp_radius
	// )
};
