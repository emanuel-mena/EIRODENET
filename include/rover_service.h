#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_storage.h"
#include "color_sensor_adapter.h"
#include "esp_err.h"
#include "infrared_adapter.h"
#include "internet_adapter.h"
#include "lsm6ds3tr_c.h"

typedef struct {
    uint64_t timestamp_ms;
    bool valid;
    esp_err_t error;
    lsm6ds3tr_c_sample_t sample;
    float quaternion[4]; /* w, x, y, z */
    bool calibration_valid;
} rover_imu_state_t;

typedef struct {
    uint64_t timestamp_ms;
    bool ultrasonic_valid;
    esp_err_t ultrasonic_error;
    uint32_t distance_mm;
    bool infrared_valid;
    esp_err_t infrared_error;
    infrared_adapter_state_t infrared;
    bool color_valid;
    esp_err_t color_error;
    color_sensor_adapter_sample_t color;
} rover_sensor_state_t;

typedef enum {
    ROVER_CAL_IDLE,
    ROVER_CAL_READY,
    ROVER_CAL_CAPTURING,
    ROVER_CAL_FACE_ACCEPTED,
    ROVER_CAL_ERROR,
    ROVER_CAL_COMPLETE,
} rover_calibration_phase_t;

typedef enum {
    ROVER_CAL_REJECT_NONE = 0,
    ROVER_CAL_REJECT_ACCEL_MOVEMENT = 1U << 0,
    ROVER_CAL_REJECT_GYRO_MOVEMENT = 1U << 1,
    ROVER_CAL_REJECT_WRONG_FACE = 1U << 2,
    ROVER_CAL_REJECT_TILT = 1U << 3,
    ROVER_CAL_REJECT_GRAVITY = 1U << 4,
    ROVER_CAL_REJECT_SPAN = 1U << 5,
} rover_calibration_rejection_t;

typedef struct {
    uint32_t revision;
    rover_calibration_phase_t phase;
    uint8_t captured_mask;
    int8_t active_face;
    uint8_t rejection_mask;
    uint8_t settling_remaining;
    uint16_t samples;
    esp_err_t error;
    float accel_mean[3];
    float accel_stddev[3];
    float gyro_mean[3];
    float gyro_stddev[3];
} rover_calibration_status_t;

esp_err_t rover_service_start(void);
void rover_service_get_imu(rover_imu_state_t *state);
void rover_service_get_sensors(rover_sensor_state_t *state);
esp_err_t rover_service_get_wifi(internet_adapter_status_t *status);
void rover_service_get_network_activity(bool *reconnecting, esp_err_t *last_error);
esp_err_t rover_service_set_config(const app_storage_config_t *config, bool *wifi_reconnecting);
esp_err_t rover_service_calibration_start(void);
esp_err_t rover_service_calibration_capture(uint8_t face);
esp_err_t rover_service_calibration_commit(void);
esp_err_t rover_service_calibration_cancel(void);
void rover_service_get_calibration_status(rover_calibration_status_t *status);
