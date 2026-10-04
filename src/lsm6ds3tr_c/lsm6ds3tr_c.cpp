#include "lsm6ds3tr_c.hpp"

#include <stdbool.h>
#include <stdlib.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LSM6DS3TR_C_I2C_TIMEOUT_MS 100
#define LSM6DS3TR_C_RESET_TIMEOUT_MS 100

#define LSM6DS3TR_C_REG_WHO_AM_I 0x0F
#define LSM6DS3TR_C_REG_CTRL1_XL 0x10
#define LSM6DS3TR_C_REG_CTRL2_G 0x11
#define LSM6DS3TR_C_REG_CTRL3_C 0x12
#define LSM6DS3TR_C_REG_OUT_TEMP_L 0x20

#define LSM6DS3TR_C_CTRL3_SW_RESET (1U << 0)
#define LSM6DS3TR_C_CTRL3_IF_INC (1U << 2)
#define LSM6DS3TR_C_CTRL3_BDU (1U << 6)
#define LSM6DS3TR_C_ODR_104_HZ (4U << 4)

#define LSM6DS3TR_C_ACCEL_G_PER_LSB 0.000061f
#define LSM6DS3TR_C_GYRO_DPS_PER_LSB 0.00875f
#define LSM6DS3TR_C_TEMP_LSB_PER_C 256.0f
#define LSM6DS3TR_C_TEMP_ZERO_C 25.0f

struct lsm6ds3tr_c_device {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t device;
};

static esp_err_t read_registers(
    lsm6ds3tr_c_handle_t handle,
    uint8_t first_register,
    uint8_t *data,
    size_t data_size
)
{
    return i2c_master_transmit_receive(
        handle->device,
        &first_register,
        sizeof(first_register),
        data,
        data_size,
        LSM6DS3TR_C_I2C_TIMEOUT_MS
    );
}

static esp_err_t write_register(
    lsm6ds3tr_c_handle_t handle,
    uint8_t register_address,
    uint8_t value
)
{
    const uint8_t transaction[] = {register_address, value};
    return i2c_master_transmit(
        handle->device,
        transaction,
        sizeof(transaction),
        LSM6DS3TR_C_I2C_TIMEOUT_MS
    );
}

static int16_t decode_i16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8) | bytes[0]);
}

esp_err_t lsm6ds3tr_c_read_device_id(
    lsm6ds3tr_c_handle_t handle,
    uint8_t *device_id
)
{
    if (handle == NULL || device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_registers(handle, LSM6DS3TR_C_REG_WHO_AM_I, device_id, 1);
}

esp_err_t lsm6ds3tr_c_init(
    const lsm6ds3tr_c_config_t *config,
    lsm6ds3tr_c_handle_t *out_handle
)
{
    if (config == NULL || out_handle == NULL || config->i2c_address > 0x7F ||
        config->i2c_clock_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_handle = NULL;
    lsm6ds3tr_c_handle_t handle = static_cast<lsm6ds3tr_c_handle_t>(calloc(1, sizeof(*handle)));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)config->sda_gpio,
        .scl_io_num = (gpio_num_t)config->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {.enable_internal_pullup = true},
    };

    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = config->i2c_clock_hz,
    };
    uint8_t device_id = 0;
    bool reset_complete = false;
    esp_err_t err = i2c_new_master_bus(&bus_config, &handle->bus);
    if (err != ESP_OK) {
        free(handle);
        return err;
    }

    err = i2c_master_probe(
        handle->bus,
        config->i2c_address,
        LSM6DS3TR_C_I2C_TIMEOUT_MS
    );
    if (err != ESP_OK) {
        goto fail;
    }

    err = i2c_master_bus_add_device(handle->bus, &device_config, &handle->device);
    if (err != ESP_OK) {
        goto fail;
    }

    err = lsm6ds3tr_c_read_device_id(handle, &device_id);
    if (err != ESP_OK) {
        goto fail;
    }
    if (device_id != LSM6DS3TR_C_WHO_AM_I_VALUE) {
        err = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }

    err = write_register(
        handle,
        LSM6DS3TR_C_REG_CTRL3_C,
        LSM6DS3TR_C_CTRL3_SW_RESET
    );
    if (err != ESP_OK) {
        goto fail;
    }

    for (int elapsed_ms = 0; elapsed_ms < LSM6DS3TR_C_RESET_TIMEOUT_MS; elapsed_ms += 2) {
        uint8_t ctrl3 = 0;
        vTaskDelay(pdMS_TO_TICKS(2));
        err = read_registers(handle, LSM6DS3TR_C_REG_CTRL3_C, &ctrl3, 1);
        if (err != ESP_OK) {
            goto fail;
        }
        if ((ctrl3 & LSM6DS3TR_C_CTRL3_SW_RESET) == 0) {
            reset_complete = true;
            break;
        }
    }
    if (!reset_complete) {
        err = ESP_ERR_TIMEOUT;
        goto fail;
    }

    err = write_register(
        handle,
        LSM6DS3TR_C_REG_CTRL3_C,
        LSM6DS3TR_C_CTRL3_BDU | LSM6DS3TR_C_CTRL3_IF_INC
    );
    if (err != ESP_OK) {
        goto fail;
    }
    err = write_register(handle, LSM6DS3TR_C_REG_CTRL1_XL, LSM6DS3TR_C_ODR_104_HZ);
    if (err != ESP_OK) {
        goto fail;
    }
    err = write_register(handle, LSM6DS3TR_C_REG_CTRL2_G, LSM6DS3TR_C_ODR_104_HZ);
    if (err != ESP_OK) {
        goto fail;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    *out_handle = handle;
    return ESP_OK;

fail:
    if (handle->device != NULL) {
        i2c_master_bus_rm_device(handle->device);
    }
    i2c_del_master_bus(handle->bus);
    free(handle);
    return err;
}

esp_err_t lsm6ds3tr_c_read_sample(
    lsm6ds3tr_c_handle_t handle,
    lsm6ds3tr_c_sample_t *sample
)
{
    if (handle == NULL || sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[14];
    esp_err_t err = read_registers(
        handle,
        LSM6DS3TR_C_REG_OUT_TEMP_L,
        raw,
        sizeof(raw)
    );
    if (err != ESP_OK) {
        return err;
    }

    sample->temperature_c =
        LSM6DS3TR_C_TEMP_ZERO_C + decode_i16(&raw[0]) / LSM6DS3TR_C_TEMP_LSB_PER_C;
    for (size_t axis = 0; axis < 3; ++axis) {
        sample->gyro_dps[axis] =
            decode_i16(&raw[2 + axis * 2]) * LSM6DS3TR_C_GYRO_DPS_PER_LSB;
        sample->accel_g[axis] =
            decode_i16(&raw[8 + axis * 2]) * LSM6DS3TR_C_ACCEL_G_PER_LSB;
    }

    return ESP_OK;
}

esp_err_t lsm6ds3tr_c_deinit(lsm6ds3tr_c_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_master_bus_rm_device(handle->device);
    esp_err_t bus_err = i2c_del_master_bus(handle->bus);
    free(handle);
    return err != ESP_OK ? err : bus_err;
}
