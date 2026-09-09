#include "imu_adapter.h"
#include "board_pins.h"

static lsm6ds3tr_c_handle_t s_imu;

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
    return s_imu == NULL ? ESP_ERR_INVALID_STATE : lsm6ds3tr_c_read_sample(s_imu, sample);
}

esp_err_t imu_adapter_deinit(void)
{
    if (s_imu == NULL) return ESP_OK;
    esp_err_t err = lsm6ds3tr_c_deinit(s_imu);
    s_imu = NULL;
    return err;
}
