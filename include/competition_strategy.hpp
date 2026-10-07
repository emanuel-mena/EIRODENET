#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define COMPETITION_FULL_PWM 1000
#define COMPETITION_HALF_PWM 700
#define COMPETITION_CUBE_DETECT_MM 60U
#define COMPETITION_CUBE_CLEARANCE_MM 70U
#define COMPETITION_CUBE_NEAR_CELLS 6.0f
// The physical cube is contacted by the front arms before its centre reaches
// 3.5 cells. Confirm capture at the measured contact envelope, otherwise the
// rover keeps driving and can push a cube out of the play area.
#define COMPETITION_CUBE_HELD_CELLS 4.2f
#define COMPETITION_DETOUR_CELLS 7.0f
#define COMPETITION_DEPOT_APPROACH_CELLS 2.5f
#define COMPETITION_DEPOT_STEP_START_CELLS 3.0f
#define COMPETITION_DEPOT_STEP_CELLS 1.0f
#define COMPETITION_DEPOT_STEP_DUTY_PERCENT 25U
#define COMPETITION_DEPOT_SLOW_PWM COMPETITION_HALF_PWM
#define COMPETITION_DELIVERY_REVERSE_MS 1000U
#define COMPETITION_CENTER_REVERSE_MS 1300U
#define COMPETITION_ASSIGNMENT_PATH_WIDTH_CELLS 5.0f
#define COMPETITION_ASSIGNMENT_CUBE_DIAMETER_CELLS 3.0f
#define COMPETITION_COMMANDER_START_DELAY_MS 3000U
#define COMPETITION_CUBE_HALF_DIAGONAL_CELLS 2.2f
#define COMPETITION_CUBE_APPROACH_OFFSET_CELLS 1.2f
#define COMPETITION_APPROACH_DECEL_CELLS_S2 35.0f
#define COMPETITION_APPROACH_REACTION_S 0.12f
#define COMPETITION_CUBE_REPOSITION_CELLS 2.5f
#define COMPETITION_REPOSITION_MS 1300U
#define COMPETITION_REPOSITION_SPEED_CELLS_S 6.3f

static inline int competition_last_cube_turn_direction(
    float rover_col, float rover_row, float heading_deg,
    float first_col, float first_row, float second_col, float second_row)
{
    const float first_distance = hypotf(first_col - rover_col, first_row - rover_row);
    const float second_distance = hypotf(second_col - rover_col, second_row - rover_row);
    if (!isfinite(first_distance) || !isfinite(second_distance) ||
        !isfinite(heading_deg)) return 0;
    const bool first_nearest = first_distance <= second_distance;
    const float nearest_col = first_nearest ? first_col : second_col;
    const float nearest_row = first_nearest ? first_row : second_row;
    const float bearing = atan2f(-(nearest_row - rover_row),
                                  nearest_col - rover_col) * 57.2957795f;
    const float relative = remainderf(bearing - heading_deg, 360.0f);
    return relative < 0.0f ? 1 : -1;
}

static inline float competition_stopping_distance(float speed_cells_s)
{
    const float speed = isfinite(speed_cells_s) ? fmaxf(0.0f, speed_cells_s) : 0.0f;
    return speed * COMPETITION_APPROACH_REACTION_S +
           speed * speed / (2.0f * COMPETITION_APPROACH_DECEL_CELLS_S2);
}

static inline bool competition_cube_acquired(float distance_cells,
                                              uint64_t clearance_ms,
                                              uint64_t near_since_ms,
                                              uint64_t ultrasonic_ms,
                                              bool ultrasonic_fresh,
                                              bool ultrasonic_valid,
                                              bool ultrasonic_timeout)
{
    if (!isfinite(distance_cells)) return false;
    if (distance_cells < COMPETITION_CUBE_HELD_CELLS) return true;
    if (distance_cells > COMPETITION_CUBE_NEAR_CELLS ||
        clearance_ms == 0 || near_since_ms == 0) return false;
    return ultrasonic_fresh && !ultrasonic_valid && ultrasonic_timeout &&
           ultrasonic_ms > clearance_ms && ultrasonic_ms >= near_since_ms;
}

static inline float competition_trim_from_yaw(float yaw_deg)
{
    if (!isfinite(yaw_deg)) return NAN;
    return fmaxf(-300.0f, fminf(300.0f, yaw_deg * 12.0f));
}

static inline bool competition_depot_needs_correction(float distance_cells)
{
    return isfinite(distance_cells) && distance_cells > COMPETITION_DEPOT_APPROACH_CELLS;
}

static inline bool competition_cube_inside(float col, float row,
                                           float cols, float rows)
{
    return isfinite(col) && isfinite(row) &&
           col >= COMPETITION_CUBE_HALF_DIAGONAL_CELLS &&
           row >= COMPETITION_CUBE_HALF_DIAGONAL_CELLS &&
           col <= cols - COMPETITION_CUBE_HALF_DIAGONAL_CELLS &&
           row <= rows - COMPETITION_CUBE_HALF_DIAGONAL_CELLS;
}

static inline bool competition_cube_push_safe(float col, float row,
                                              float push_col, float push_row,
                                              float cols, float rows)
{
    const float length = hypotf(push_col, push_row);
    if (!isfinite(length) || length <= 0.001f) return true;
    const float step = 1.5f;
    return competition_cube_inside(col + step * push_col / length,
                                  row + step * push_row / length, cols, rows);
}

static inline bool competition_vision_frame_is_new(uint32_t sequence, uint32_t last_sequence)
{
    return sequence != last_sequence;
}

static inline bool competition_assignment_line_hits_cube(
    float start_col, float start_row, float end_col, float end_row,
    float cube_col, float cube_row)
{
    const float dx = end_col - start_col;
    const float dy = end_row - start_row;
    const float length_squared = dx * dx + dy * dy;
    const float t = length_squared > 0.0f
        ? fmaxf(0.0f, fminf(1.0f,
            ((cube_col - start_col) * dx + (cube_row - start_row) * dy) /
            length_squared))
        : 0.0f;
    const float nearest_col = start_col + t * dx;
    const float nearest_row = start_row + t * dy;
    const float clearance = COMPETITION_ASSIGNMENT_PATH_WIDTH_CELLS * 0.5f +
                            COMPETITION_ASSIGNMENT_CUBE_DIAMETER_CELLS * 0.5f;
    const float offset_col = cube_col - nearest_col;
    const float offset_row = cube_row - nearest_row;
    return offset_col * offset_col + offset_row * offset_row <= clearance * clearance;
}

static inline void competition_trimmed_forward(float trim_pwm, bool reverse,
                                               int16_t *left, int16_t *right)
{
    if (left == NULL || right == NULL) return;
    const int reduction = (int)lroundf(fabsf(trim_pwm));
    int left_pwm = COMPETITION_FULL_PWM;
    int right_pwm = COMPETITION_FULL_PWM;
    if (trim_pwm >= 0) right_pwm -= reduction;
    else left_pwm -= reduction;
    *left = (int16_t)(reverse ? -left_pwm : left_pwm);
    *right = (int16_t)(reverse ? -right_pwm : right_pwm);
}
