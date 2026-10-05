#pragma once

#include <math.h>

#define NAV_ROVER_WIDTH_CELLS 5.0f
#define NAV_ROVER_LENGTH_CELLS 7.5f
#define NAV_OBSTACLE_SIDE_CELLS 5.0f
#define NAV_CLEARANCE_CELLS 0.4f
#define NAV_BODY_HALF_LENGTH_CELLS 2.375f
#define NAV_FRONT_EXTENT_CELLS 5.125f
#define NAV_BODY_TURN_RADIUS_CELLS 3.5f
#define NAV_PEER_ROUTE_CLEARANCE_CELLS 11.6f

// Pose is the centre of the 95 mm body; the 55 mm arms extend forwards.
static inline bool navigation_pose_inside(float col, float row, float heading,
                                          float cols, float rows)
{
    const float a = heading * 0.01745329252f;
    for (int i = 0; i < 4; ++i) {
        const float x = i & 1 ? NAV_FRONT_EXTENT_CELLS : -NAV_BODY_HALF_LENGTH_CELLS;
        const float y = i & 2 ? NAV_ROVER_WIDTH_CELLS / 2 : -NAV_ROVER_WIDTH_CELLS / 2;
        const float c = col + x * cosf(a) - y * sinf(a);
        const float r = row - x * sinf(a) - y * cosf(a);
        if (c < 0 || r < 0 || c > cols || r > rows) return false;
    }
    return true;
}

static inline bool navigation_turn_inside(float col, float row, float heading,
                                          float error, float cols, float rows)
{
    const int steps = (int)ceilf(fabsf(error) / 3.0f) + 1;
    for (int i = 0; i <= steps; ++i)
        if (!navigation_pose_inside(col, row, heading + error * i / steps, cols, rows))
            return false;
    return true;
}

static inline bool navigation_rovers_overlap(float col, float row, float heading,
                                              float peer_col, float peer_row, float peer_heading)
{
    const float a = heading * 0.01745329252f, b = peer_heading * 0.01745329252f;
    const float ax = cosf(a), ay = -sinf(a), bx = cosf(b), by = -sinf(b);
    const float offset = (NAV_FRONT_EXTENT_CELLS - NAV_BODY_HALF_LENGTH_CELLS) / 2;
    const float dx = peer_col + offset * bx - col - offset * ax;
    const float dy = peer_row + offset * by - row - offset * ay;
    const float half_length = NAV_ROVER_LENGTH_CELLS / 2;
    const float half_width = NAV_ROVER_WIDTH_CELLS / 2;
    const float axes[4][2] = {{ax, ay}, {-ay, ax}, {bx, by}, {-by, bx}};
    for (const auto &axis : axes) {
        const float x = axis[0], y = axis[1];
        const float reach = half_length * (fabsf(ax*x + ay*y) + fabsf(bx*x + by*y)) +
            half_width * (fabsf(-ay*x + ax*y) + fabsf(-by*x + bx*y)) + NAV_CLEARANCE_CELLS;
        if (fabsf(dx*x + dy*y) > reach) return false;
    }
    return true;
}

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
