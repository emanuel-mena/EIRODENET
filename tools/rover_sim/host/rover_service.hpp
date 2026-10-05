#pragma once
// Host sensor boundary: only the fields consumed by the shared control code.
#include <stdint.h>
#include "infrared_adapter.hpp"
struct rover_imu_state_t {
    uint64_t timestamp_ms; bool valid; bool calibration_valid;
    struct { float accel_g[3]; float gyro_dps[3]; } sample;
};
struct rover_sensor_state_t {
    uint64_t timestamp_ms; bool ultrasonic_valid; esp_err_t ultrasonic_error;
    uint32_t distance_mm; bool infrared_valid; infrared_adapter_state_t infrared;
};
void rover_service_get_imu(rover_imu_state_t*);
void rover_service_get_sensors(rover_sensor_state_t*);
