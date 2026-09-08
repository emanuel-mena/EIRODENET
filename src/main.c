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
#include "model_partition.h"

#define NEOPIXEL_COUNT 1

static const char *TAG = "eirodenet";
static led_strip_handle_t neopixel;

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

void app_main(void)
{
    persistent_data_init();
    neopixel_init();

    while (1)
    {
        // Turquesa
        neopixel_set_rgb(0, 210, 180);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Violeta
        neopixel_set_rgb(145, 40, 255);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Ambar
        neopixel_set_rgb(255, 95, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Apagar
        neopixel_set_rgb(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
