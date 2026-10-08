// Marine mode admission: internal transitions are free of the landing/disarming constraint.
#pragma once

#include <uORB/topics/vehicle_status.h>

constexpr bool isManualMarineMode(uint8_t mode)
{
	return mode == vehicle_status_s::NAVIGATION_STATE_PRISMA_MARINE_MANUAL
	       || mode == vehicle_status_s::NAVIGATION_STATE_PRISMA_MARINE_MANUAL_TS
	       || mode == vehicle_status_s::NAVIGATION_STATE_PRISMA_MARINE_MANUAL_FF
	       || mode == vehicle_status_s::NAVIGATION_STATE_MARINE_TAKEOFF;
}

constexpr bool canEnterManualMarineMode(uint8_t current, uint8_t requested, bool armed, bool landed)
{
	return !isManualMarineMode(requested) || isManualMarineMode(current) || (!armed && landed);
}
