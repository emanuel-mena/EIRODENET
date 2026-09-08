#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

#include "app_storage.h"
#include "board_pins.h"
#include "lsm6ds3tr_c.h"
#include "model_partition.h"

#define NEOPIXEL_COUNT 1

static const char *TAG = "eirodenet";
static led_strip_handle_t neopixel;
static lsm6ds3tr_c_handle_t imu;

static void persistent_data_init(void)
{
    uint32_t boot_count = 0;
    esp_err_t err = app_storage_init(&boot_count);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS listo; arranque numero %" PRIu32, boot_count);
    } else {
        ESP_LOGE(TAG, "No se pudo inicializar NVS: %s", esp_err_to_name(err));
    }

    model_partition_info_t model;
    err = model_partition_validate(&model);
    if (err == ESP_OK) {
        ESP_LOGI(
            TAG,
            "Modelo TinyML valido: version=%" PRIu32 ", bytes=%" PRIu32 ", crc32=%08" PRIx32,
            model.model_version,
            model.model_size,
            model.crc32
        );
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "Particion TinyML vacia; la aplicacion continuara sin inferencia");
    } else {
        ESP_LOGE(
            TAG,
            "Modelo TinyML invalido (%s); la aplicacion continuara sin inferencia",
            esp_err_to_name(err)
        );
    }
}

static void neopixel_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = IDEABOARD_RGB_LED,
        .max_leds = NEOPIXEL_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = false,
        },
    };

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(
            &strip_config,
            &rmt_config,
            &neopixel
        )
    );

    // Apagar inicialmente
    ESP_ERROR_CHECK(led_strip_clear(neopixel));
}

static void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    ESP_ERROR_CHECK(
        led_strip_set_pixel(neopixel, 0, r, g, b)
    );

    ESP_ERROR_CHECK(
        led_strip_refresh(neopixel)
    );
}

static esp_err_t imu_init(void)
{
    const lsm6ds3tr_c_config_t config = {
        .sda_gpio = IDEABOARD_I2C_SDA,
        .scl_gpio = IDEABOARD_I2C_SCL,
        .i2c_address = LSM6DS3TR_C_I2C_ADDRESS_DEFAULT,
        .i2c_clock_hz = 400000,
    };

    esp_err_t err = lsm6ds3tr_c_init(&config, &imu);
    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "No se pudo iniciar LSM6DS3TR-C en I2C 0x%02X (SDA=%d, SCL=%d): %s",
            config.i2c_address,
            config.sda_gpio,
            config.scl_gpio,
            esp_err_to_name(err)
        );
        return err;
    }

    uint8_t device_id = 0;
    err = lsm6ds3tr_c_read_device_id(imu, &device_id);
    if (err == ESP_OK) {
        ESP_LOGI(
            TAG,
            "LSM6DS3TR-C listo: direccion=0x%02X, WHO_AM_I=0x%02X",
            config.i2c_address,
            device_id
        );
    }
    return err;
}

void app_main(void)
{
    persistent_data_init();
    neopixel_init();
    const bool imu_ready = imu_init() == ESP_OK;

    unsigned int color_index = 0;

    while (1)
    {
        static const uint8_t colors[][3] = {
            {0, 210, 180},
            {145, 40, 255},
            {255, 95, 0},
            {0, 0, 0},
        };
        neopixel_set_rgb(
            colors[color_index][0],
            colors[color_index][1],
            colors[color_index][2]
        );
        color_index = (color_index + 1) % (sizeof(colors) / sizeof(colors[0]));

        if (imu_ready) {
            lsm6ds3tr_c_sample_t sample;
            esp_err_t err = lsm6ds3tr_c_read_sample(imu, &sample);
            if (err == ESP_OK) {
                ESP_LOGI(
                    TAG,
                    "IMU T=%.2f C | accel[g] X=%.3f Y=%.3f Z=%.3f | gyro[dps] X=%.2f Y=%.2f Z=%.2f",
                    sample.temperature_c,
                    sample.accel_g[0],
                    sample.accel_g[1],
                    sample.accel_g[2],
                    sample.gyro_dps[0],
                    sample.gyro_dps[1],
                    sample.gyro_dps[2]
                );
            } else {
                ESP_LOGE(TAG, "Fallo al leer LSM6DS3TR-C: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
