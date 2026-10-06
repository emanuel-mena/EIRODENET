#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define COMPETITION_FULL_PWM 1000
#define COMPETITION_HALF_PWM 700
#define COMPETITION_OBSTACLE_MM 30U
#define COMPETITION_DETOUR_CELLS 7.0f
#define COMPETITION_DETOUR_MIN_DISTANCE_CELLS 5.0f
#define COMPETITION_DEPOT_APPROACH_CELLS 2.5f
#define COMPETITION_DEPOT_SLOW_PWM COMPETITION_HALF_PWM
#define COMPETITION_DELIVERY_REVERSE_MS 700U
#define COMPETITION_ASSIGNMENT_PATH_WIDTH_CELLS 5.0f
#define COMPETITION_ASSIGNMENT_CUBE_DIAMETER_CELLS 3.0f
#define COMPETITION_COMMANDER_START_DELAY_MS 3000U

static inline float competition_trim_from_yaw(float yaw_deg)
{
    if (!isfinite(yaw_deg)) return NAN;
    return fmaxf(-300.0f, fminf(300.0f, yaw_deg * 12.0f));
}

static inline bool competition_depot_needs_correction(float distance_cells)
{
    return isfinite(distance_cells) && distance_cells > COMPETITION_DEPOT_APPROACH_CELLS;
}

static inline bool competition_depot_frame_is_new(uint32_t sequence, uint32_t last_sequence)
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
