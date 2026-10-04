#include "imu_adapter.hpp"
#include "board_pins.hpp"

#include <string.h>

static lsm6ds3tr_c_handle_t s_imu;
static imu_calibration_t s_calibration = {
    .version = IMU_CALIBRATION_VERSION,
    .accel_scale = {1.0f, 1.0f, 1.0f},
};

esp_err_t imu_adapter_init(void)
{
    const lsm6ds3tr_c_config_t config = {
        .sda_gpio = BOARD_IMU_SDA_GPIO,
        .scl_gpio = BOARD_IMU_SCL_GPIO,
        .i2c_address = BOARD_IMU_I2C_ADDRESS,
        .i2c_clock_hz = BOARD_IMU_I2C_CLOCK_HZ,
    };
    return lsm6ds3tr_c_init(&config, &s_imu);
}

esp_err_t imu_adapter_read(lsm6ds3tr_c_sample_t *sample)
{
    esp_err_t err = imu_adapter_read_raw(sample);
    if (err != ESP_OK || !s_calibration.valid) return err;
    for (size_t axis = 0; axis < 3; ++axis) {
        sample->accel_g[axis] =
            (sample->accel_g[axis] - s_calibration.accel_offset_g[axis]) *
            s_calibration.accel_scale[axis];
        sample->gyro_dps[axis] -= s_calibration.gyro_bias_dps[axis];
    }
    return ESP_OK;
}

esp_err_t imu_adapter_read_raw(lsm6ds3tr_c_sample_t *sample)
{
    if (sample == NULL) return ESP_ERR_INVALID_ARG;
    if (s_imu == NULL) return ESP_ERR_INVALID_STATE;
    esp_err_t err = lsm6ds3tr_c_read_sample(s_imu, sample);
    if (err == ESP_OK) {
        /* El chip está montado 180 grados alrededor de Z respecto al rover. */
        sample->accel_g[0] = -sample->accel_g[0];
        sample->accel_g[1] = -sample->accel_g[1];
        sample->gyro_dps[0] = -sample->gyro_dps[0];
        sample->gyro_dps[1] = -sample->gyro_dps[1];
    }
    return err;
}

esp_err_t imu_adapter_set_calibration(const imu_calibration_t *calibration)
{
    if (calibration == NULL || calibration->version != IMU_CALIBRATION_VERSION)
        return ESP_ERR_INVALID_ARG;
    memcpy(&s_calibration, calibration, sizeof(s_calibration));
    if (!s_calibration.valid) {
        memset(&s_calibration, 0, sizeof(s_calibration));
        s_calibration.version = IMU_CALIBRATION_VERSION;
        for (size_t axis = 0; axis < 3; ++axis) s_calibration.accel_scale[axis] = 1.0f;
    }
    return ESP_OK;
}

esp_err_t imu_adapter_get_calibration(imu_calibration_t *calibration)
{
    if (calibration == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(calibration, &s_calibration, sizeof(*calibration));
    return ESP_OK;
}

esp_err_t imu_adapter_deinit(void)
{
    if (s_imu == NULL) return ESP_OK;
    esp_err_t err = lsm6ds3tr_c_deinit(s_imu);
    s_imu = NULL;
    return err;
}
