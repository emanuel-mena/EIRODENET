#pragma once

#include <stdbool.h>
#include <stdint.h>

#define HEADING_CALIBRATION_SAMPLES 5U
#define HEADING_CALIBRATION_MAX_AGE_MS 200U

typedef struct {
    uint64_t frame_timestamp_ms;
    uint32_t age_ms;
    float col;
    float row;
    float theta_deg;
} heading_calibration_sample_t;

/** Calcula el desfase del rumbo 0° de salida; rechaza capturas repetidas o inestables. */
bool heading_calibration_calculate(
    const heading_calibration_sample_t samples[HEADING_CALIBRATION_SAMPLES],
    float *offset_deg);
