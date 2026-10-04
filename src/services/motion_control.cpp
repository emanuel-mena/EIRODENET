#include "motion_control.hpp"

#include <math.h>
#include <stddef.h>
#include <string.h>

void motion_bias_reset(motion_bias_window_t *window)
{
    if (window) memset(window, 0, sizeof(*window));
}

void motion_bias_add(motion_bias_window_t *window, uint64_t timestamp_ms,
                     const float accel_g[3], const float gyro_dps[3])
{
    if (!window || !accel_g || !gyro_dps || !timestamp_ms ||
        timestamp_ms <= window->last_timestamp_ms)
        return;
    if (window->last_timestamp_ms && timestamp_ms - window->last_timestamp_ms > 100U)
        window->start = window->count = 0;
    window->last_timestamp_ms = timestamp_ms;
    const float norm = sqrtf(accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] +
                             accel_g[2] * accel_g[2]);
    float accel_change_sq = 0;
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (!isfinite(gyro_dps[axis]) || fabsf(gyro_dps[axis]) > 3.0f) {
            window->start = window->count = 0;
            return;
        }
        const float change = accel_g[axis] - window->reference_accel[axis];
        accel_change_sq += change * change;
    }
    if (!isfinite(norm) || norm < 0.95f || norm > 1.05f ||
        (window->count && accel_change_sq > 0.0016f)) {
        window->start = window->count = 0;
        return;
    }
    if (!window->count)
        memcpy(window->reference_accel, accel_g, sizeof(window->reference_accel));
    while (window->count &&
           timestamp_ms - window->timestamps[window->start] >= 2000U) {
        window->start = (window->start + 1U) % MOTION_BIAS_WINDOW;
        --window->count;
    }
    if (window->count == MOTION_BIAS_WINDOW) {
        window->start = (window->start + 1U) % MOTION_BIAS_WINDOW;
        --window->count;
    }
    const uint16_t index = (window->start + window->count) % MOTION_BIAS_WINDOW;
    window->timestamps[index] = timestamp_ms;
    window->gyro[index] = gyro_dps[2];
    window->accel_norm[index] = norm;
    ++window->count;
}

bool motion_bias_result(const motion_bias_window_t *window, float *bias_dps)
{
    if (!window || !bias_dps || window->count < 100) return false;
    double sum_g = 0, sum_g2 = 0, sum_a = 0, sum_a2 = 0;
    for (uint16_t i = 0; i < window->count; ++i) {
        const uint16_t index = (window->start + i) % MOTION_BIAS_WINDOW;
        const double g = window->gyro[index], a = window->accel_norm[index];
        sum_g += g; sum_g2 += g * g;
        sum_a += a; sum_a2 += a * a;
    }
    const double count = window->count;
    const double mean_g = sum_g / count, mean_a = sum_a / count;
    if (fabs(mean_g) > 2.0 || sum_g2 / count - mean_g * mean_g > 0.25 ||
        sum_a2 / count - mean_a * mean_a > 0.0009) return false;
    *bias_dps = (float)mean_g;
    return true;
}

float motion_wrap_degrees(float degrees)
{
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees <= -180.0f) degrees += 360.0f;
    return degrees;
}

void motion_drive_command(float heading_error_deg, float cross_track_cells,
                          float angular_speed_dps, float trim, int *left, int *right,
                          bool *saturated)
{
    /* Positive cross track means the rover is left of the planned segment. */
    float correction = 8.0f * (heading_error_deg -
        fmaxf(-10.0f, fminf(10.0f, 3.0f * cross_track_cells))) -
        1.5f * angular_speed_dps + trim;
    *saturated = fabsf(correction) > 300.0f;
    correction = fmaxf(-300.0f, fminf(300.0f, correction));
    const int amount = (int)lroundf(fabsf(correction));
    *left = correction >= 0 ? 1000 - amount : 1000;
    *right = correction >= 0 ? 1000 : 1000 - amount;
}

int motion_turn_pwm(float error_deg, float angular_speed_dps, uint32_t tick,
                    bool *settled)
{
    const float magnitude = fabsf(error_deg);
    const float toward = copysignf(angular_speed_dps, error_deg);
    *settled = magnitude <= 3.0f && fabsf(angular_speed_dps) <= 20.0f;
    if (*settled) return 0;
    /* Initial braking estimate; physical trials will tune deceleration. */
    const float stopping_deg = toward > 0 ? toward * toward / 1000.0f : 0.0f;
    if (toward > 20.0f && magnitude <= stopping_deg + 4.0f) return 0;
    if (magnitude > stopping_deg + 20.0f) return error_deg > 0 ? 1000 : -1000;
    if ((tick % 8U) >= 2U) return 0;
    return error_deg > 0 ? 700 : -700;
}
