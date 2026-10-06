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
#define NAV_MOTOR_AXLE_OFFSET_MM (-27.5f)
#define NAV_DEFAULT_CELL_MM 20.0f
#define NAV_SWEEP_SAMPLE_DEG 3.0f

static inline float navigation_cell_mm(float cell_mm)
{
    return isfinite(cell_mm) && cell_mm > 0.0f ? cell_mm : NAV_DEFAULT_CELL_MM;
}

/** Offset of the motor axle from the body centre in grid cells. */
static inline float navigation_axle_offset_cells(float cell_mm)
{
    return NAV_MOTOR_AXLE_OFFSET_MM / navigation_cell_mm(cell_mm);
}

static inline void navigation_axle_from_center(float center_col, float center_row,
                                               float heading, float cell_mm,
                                               float *axle_col, float *axle_row)
{
    const float a = heading * 0.01745329252f;
    const float offset = navigation_axle_offset_cells(cell_mm);
    *axle_col = center_col + offset * cosf(a);
    *axle_row = center_row - offset * sinf(a);
}

static inline void navigation_center_from_axle(float axle_col, float axle_row,
                                               float heading, float cell_mm,
                                               float *center_col, float *center_row)
{
    const float a = heading * 0.01745329252f;
    const float offset = navigation_axle_offset_cells(cell_mm);
    *center_col = axle_col - offset * cosf(a);
    *center_row = axle_row + offset * sinf(a);
}

// Pose is the centre of the 95 mm body; the 55 mm arms extend forwards.
static inline bool navigation_pose_inside_mm(float col, float row, float heading,
                                             float cols, float rows, float cell_mm)
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

static inline bool navigation_pose_inside(float col, float row, float heading,
                                          float cols, float rows)
{
    return navigation_pose_inside_mm(col, row, heading, cols, rows,
                                     NAV_DEFAULT_CELL_MM);
}

/** Checks a turn whose instantaneous centre is the motor axle, not the body centre. */
static inline bool navigation_turn_inside_mm(float col, float row, float heading,
                                             float error, float cols, float rows,
                                             float cell_mm)
{
    const int steps = (int)ceilf(fabsf(error) / NAV_SWEEP_SAMPLE_DEG) + 1;
    float axle_col, axle_row;
    navigation_axle_from_center(col, row, heading, cell_mm, &axle_col, &axle_row);
    for (int i = 0; i <= steps; ++i)
    {
        const float turn_heading = heading + error * i / steps;
        float center_col, center_row;
        navigation_center_from_axle(axle_col, axle_row, turn_heading, cell_mm,
                                    &center_col, &center_row);
        if (!navigation_pose_inside_mm(center_col, center_row, turn_heading,
                                       cols, rows, cell_mm)) return false;
    }
    return true;
}

static inline bool navigation_turn_inside(float col, float row, float heading,
                                          float error, float cols, float rows)
{
    return navigation_turn_inside_mm(col, row, heading, error, cols, rows,
                                     NAV_DEFAULT_CELL_MM);
}

static inline bool navigation_segment_inside_mm(float start_col, float start_row,
                                                float end_col, float end_row,
                                                float heading, float cols, float rows,
                                                float cell_mm)
{
    const float length = hypotf(end_col - start_col, end_row - start_row);
    const int steps = (int)ceilf(length / 0.5f) + 1;
    for (int i = 0; i <= steps; ++i) {
        const float t = steps ? (float)i / steps : 0.0f;
        if (!navigation_pose_inside_mm(start_col + t * (end_col - start_col),
                                       start_row + t * (end_row - start_row),
                                       heading, cols, rows, cell_mm)) return false;
    }
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

static inline float navigation_body_swept_radius(void)
{
    // Arms extend beyond the measured 95 mm chassis and dominate clearance
    // against cubes and obstacles when a route passes them diagonally.
    return hypotf(NAV_FRONT_EXTENT_CELLS, NAV_ROVER_WIDTH_CELLS / 2.0f);
}

static inline float navigation_square_clearance(float side)
{
    return navigation_body_swept_radius() + side * 0.7071067812f + NAV_CLEARANCE_CELLS;
}

static inline float navigation_peer_clearance(void)
{
    return 2.0f * navigation_body_swept_radius() + NAV_CLEARANCE_CELLS;
}
