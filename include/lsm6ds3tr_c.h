#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LSM6DS3TR_C_I2C_ADDRESS_DEFAULT 0x6B
#define LSM6DS3TR_C_WHO_AM_I_VALUE 0x6A

typedef struct lsm6ds3tr_c_device *lsm6ds3tr_c_handle_t;

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint8_t i2c_address;
    uint32_t i2c_clock_hz;
} lsm6ds3tr_c_config_t;

typedef struct {
    float temperature_c;
    float gyro_dps[3];
    float accel_g[3];
} lsm6ds3tr_c_sample_t;

/**
 * Initializes the I2C bus and configures the sensor for 104 Hz, +/-2 g and
 * +/-245 dps. The returned handle owns the I2C bus.
 */
esp_err_t lsm6ds3tr_c_init(
    const lsm6ds3tr_c_config_t *config,
    lsm6ds3tr_c_handle_t *out_handle
);

/** Reads and validates the fixed WHO_AM_I value (0x6A). */
esp_err_t lsm6ds3tr_c_read_device_id(
    lsm6ds3tr_c_handle_t handle,
    uint8_t *device_id
);

/** Reads temperature, angular rate and acceleration in one I2C transaction. */
esp_err_t lsm6ds3tr_c_read_sample(
    lsm6ds3tr_c_handle_t handle,
    lsm6ds3tr_c_sample_t *sample
);

/** Releases the sensor device and its I2C bus. */
esp_err_t lsm6ds3tr_c_deinit(lsm6ds3tr_c_handle_t handle);

#ifdef __cplusplus
}
#endif
