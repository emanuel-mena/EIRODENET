#include "heading_calibration.hpp"

#include <assert.h>
#include <math.h>

static heading_calibration_sample_t samples[HEADING_CALIBRATION_SAMPLES] = {
    {1, 0, 3.75f, 21.5f, 358.0f},
    {2, 0, 3.76f, 21.5f, 359.0f},
    {3, 0, 3.75f, 21.51f, 0.0f},
    {4, 0, 3.74f, 21.5f, 1.0f},
    {5, 0, 3.75f, 21.5f, 2.0f},
};

int main(void)
{
    float offset = 123.0f;
    assert(heading_calibration_calculate(samples, &offset));
    assert(fabsf(offset) < 0.01f);

    for (unsigned i = 0; i < HEADING_CALIBRATION_SAMPLES; ++i)
        samples[i].theta_deg = 12.0f;
    assert(heading_calibration_calculate(samples, &offset));
    assert(fabsf(offset + 12.0f) < 0.01f);

    samples[4].frame_timestamp_ms = samples[3].frame_timestamp_ms;
    assert(!heading_calibration_calculate(samples, &offset));
    samples[4].frame_timestamp_ms = 5;

    samples[4].age_ms = HEADING_CALIBRATION_MAX_AGE_MS + 1;
    assert(!heading_calibration_calculate(samples, &offset));
    samples[4].age_ms = 0;

    samples[4].theta_deg = 40.0f;
    assert(!heading_calibration_calculate(samples, &offset));
    samples[4].theta_deg = 12.0f;

    samples[4].col = 4.4f;
    assert(!heading_calibration_calculate(samples, &offset));
    return 0;
}
