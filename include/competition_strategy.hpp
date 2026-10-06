#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define COMPETITION_FULL_PWM 1000
#define COMPETITION_HALF_PWM 700
#define COMPETITION_OBSTACLE_MM 30U
#define COMPETITION_DETOUR_CELLS 7.0f
#define COMPETITION_DETOUR_MIN_DISTANCE_CELLS 5.0f
#define COMPETITION_DEPOT_STOP_CELLS 6.0f

static inline float competition_trim_from_yaw(float yaw_deg)
{
    if (!isfinite(yaw_deg)) return NAN;
    return fmaxf(-300.0f, fminf(300.0f, yaw_deg * 12.0f));
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
