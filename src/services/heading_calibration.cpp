#include "heading_calibration.hpp"

#include <math.h>
#include <stddef.h>

#define MAX_HEADING_SPREAD_DEG 10.0f
#define MAX_POSITION_SPREAD_CELLS 0.5f

static constexpr float PI = 3.14159265358979323846f;

static float wrap_degrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

bool heading_calibration_calculate(
    const heading_calibration_sample_t samples[HEADING_CALIBRATION_SAMPLES],
    float *offset_deg)
{
    if (samples == NULL || offset_deg == NULL) return false;
    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (size_t i = 0; i < HEADING_CALIBRATION_SAMPLES; ++i) {
        const heading_calibration_sample_t *sample = &samples[i];
        if (sample->frame_timestamp_ms == 0 ||
            sample->age_ms > HEADING_CALIBRATION_MAX_AGE_MS ||
            !isfinite(sample->col) || !isfinite(sample->row) ||
            !isfinite(sample->theta_deg) || sample->theta_deg < 0.0f ||
            sample->theta_deg > 360.0f) return false;
        for (size_t j = 0; j < i; ++j) {
            if (sample->frame_timestamp_ms == samples[j].frame_timestamp_ms) return false;
            if (hypotf(sample->col - samples[j].col, sample->row - samples[j].row) >
                MAX_POSITION_SPREAD_CELLS) return false;
        }
        const float radians = sample->theta_deg * PI / 180.0f;
        sum_sin += sinf(radians);
        sum_cos += cosf(radians);
    }
    const float mean = atan2f(sum_sin, sum_cos) * 180.0f / PI;
    for (size_t i = 0; i < HEADING_CALIBRATION_SAMPLES; ++i)
        if (fabsf(wrap_degrees(samples[i].theta_deg - mean)) > MAX_HEADING_SPREAD_DEG)
            return false;
    *offset_deg = wrap_degrees(-mean);
    return true;
}
