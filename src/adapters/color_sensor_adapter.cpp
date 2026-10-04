#include "color_sensor_adapter.hpp"

#include "adc1_adapter.hpp"
#include "board_pins.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#if BOARD_COLOR_PHOTORESISTOR_GPIO != 32
#error "Actualice el canal ADC cuando cambie BOARD_COLOR_PHOTORESISTOR_GPIO"
#endif

#define COLOR_ADC_CHANNEL ADC_CHANNEL_4
#define COLOR_SETTLE_MS 8
#define COLOR_LED_BRIGHTNESS 32

static bool s_adc_ready;
static led_strip_handle_t s_led;

static esp_err_t illuminate_and_read(uint8_t red, uint8_t green, uint8_t blue, uint16_t *value)
{
    esp_err_t err = led_strip_set_pixel(s_led, 0, red, green, blue);
    if (err == ESP_OK) err = led_strip_refresh(s_led);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(COLOR_SETTLE_MS));
    int raw = 0;
    err = adc1_adapter_read(COLOR_ADC_CHANNEL, &raw);
    if (err == ESP_OK) *value = (uint16_t)raw;
    return err;
}

esp_err_t color_sensor_adapter_init(void)
{
    esp_err_t err = adc1_adapter_configure(COLOR_ADC_CHANNEL);
    if (err != ESP_OK) return err;
    s_adc_ready = true;
    const led_strip_config_t strip_config = { .strip_gpio_num = BOARD_COLOR_NEOPIXEL_GPIO,
        .max_leds = 1, .led_model = LED_MODEL_WS2812, .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB };
    const led_strip_rmt_config_t rmt_config = { .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, .mem_block_symbols = 0, .flags = {.with_dma = false} };
    err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
    if (err == ESP_OK) return led_strip_clear(s_led);
    adc1_adapter_release();
    s_adc_ready = false;
    return err;
}

esp_err_t color_sensor_adapter_read(color_sensor_adapter_sample_t *sample)
{
    if (sample == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_adc_ready || s_led == NULL) return ESP_ERR_INVALID_STATE;
    esp_err_t err = illuminate_and_read(0, 0, 0, &sample->ambient);
    if (err == ESP_OK) err = illuminate_and_read(COLOR_LED_BRIGHTNESS, 0, 0, &sample->red);
    if (err == ESP_OK) err = illuminate_and_read(0, COLOR_LED_BRIGHTNESS, 0, &sample->green);
    if (err == ESP_OK) err = illuminate_and_read(0, 0, COLOR_LED_BRIGHTNESS, &sample->blue);
    led_strip_clear(s_led);
    return err;
}

esp_err_t color_sensor_adapter_deinit(void)
{
    esp_err_t err = ESP_OK;
    if (s_led != NULL) {
        led_strip_clear(s_led);
        err = led_strip_del(s_led);
        s_led = NULL;
    }
    if (s_adc_ready) {
        esp_err_t adc_err = adc1_adapter_release();
        s_adc_ready = false;
        if (err == ESP_OK) err = adc_err;
    }
    return err;
}
