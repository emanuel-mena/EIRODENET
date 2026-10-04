#pragma once

#include <math.h>

#define NAV_ROVER_WIDTH_CELLS 5.0f
#define NAV_ROVER_LENGTH_CELLS 7.5f
#define NAV_OBSTACLE_SIDE_CELLS 5.0f
#define NAV_CLEARANCE_CELLS 0.4f

static inline float navigation_rover_radius(void)
{
    return hypotf(NAV_ROVER_WIDTH_CELLS / 2.0f, NAV_ROVER_LENGTH_CELLS / 2.0f);
}

static inline float navigation_square_clearance(float side)
{
    return navigation_rover_radius() + side * 0.7071067812f + NAV_CLEARANCE_CELLS;
}

static inline float navigation_peer_clearance(void)
{
    return 2.0f * navigation_rover_radius() + NAV_CLEARANCE_CELLS;
}
