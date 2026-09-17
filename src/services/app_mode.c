#include "app_mode.h"

#include <stdbool.h>

#include "app_storage.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "motor_adapter.h"

#define MODE_TASK_PERIOD_MS 20
#define BUTTON_DEBOUNCE_TICKS 3

static const char *TAG = "app_mode";
static portMUX_TYPE s_mode_lock = portMUX_INITIALIZER_UNLOCKED;
static app_mode_t s_mode = APP_MODE_TEST;
static uint8_t s_identity;
static led_strip_handle_t s_led;

app_mode_t app_mode_get(void)
{
    taskENTER_CRITICAL(&s_mode_lock);
    const app_mode_t mode = s_mode;
    taskEXIT_CRITICAL(&s_mode_lock);
    return mode;
}

const char *app_mode_name(app_mode_t mode)
{
    return mode == APP_MODE_COMPETITION ? "competition" : "test";
}

static void toggle_mode(void)
{
    taskENTER_CRITICAL(&s_mode_lock);
    s_mode = s_mode == APP_MODE_TEST ? APP_MODE_COMPETITION : APP_MODE_TEST;
    const app_mode_t mode = s_mode;
    taskEXIT_CRITICAL(&s_mode_lock);
    motor_adapter_stop();
    ESP_LOGW(TAG, "Modo cambiado con BOOT: %s", app_mode_name(mode));
}

static void set_indicator(uint8_t pulse)
{
    uint8_t red = 0, green = 0, blue = 0;
    if (app_mode_get() == APP_MODE_TEST) {
        blue = pulse;
    } else if (s_identity == APP_STORAGE_ROVER_10) {
        red = 72; green = 46;
    } else if (s_identity == APP_STORAGE_ROVER_11) {
        red = 50; blue = 72;
    } else {
        red = 72;
    }
    if (led_strip_set_pixel(s_led, 0, red, green, blue) == ESP_OK) led_strip_refresh(s_led);
}

static void mode_task(void *argument)
{
    (void)argument;
    bool stable_pressed = false;
    bool candidate = false;
    uint8_t candidate_ticks = 0;
    uint8_t pulse = 4;
    int8_t pulse_step = 2;
    while (true) {
        const bool pressed = gpio_get_level(BOARD_BOOT_BUTTON_GPIO) == 0;
        if (pressed != candidate) {
            candidate = pressed;
            candidate_ticks = 1;
        } else if (candidate_ticks < BUTTON_DEBOUNCE_TICKS) {
            candidate_ticks++;
        } else if (stable_pressed != candidate) {
            stable_pressed = candidate;
            if (stable_pressed) toggle_mode();
        }
        if (pulse >= 70) pulse_step = -2;
        else if (pulse <= 4) pulse_step = 2;
        pulse = (uint8_t)(pulse + pulse_step);
        set_indicator(pulse);
        vTaskDelay(pdMS_TO_TICKS(MODE_TASK_PERIOD_MS));
    }
}

esp_err_t app_mode_start(uint8_t rover_identity)
{
    s_identity = rover_identity;
    const gpio_config_t button = {
        .pin_bit_mask = 1ULL << BOARD_BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&button);
    const led_strip_config_t strip = {
        .strip_gpio_num = BOARD_MODE_NEOPIXEL_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    const led_strip_rmt_config_t rmt = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .mem_block_symbols = 0,
        .flags.with_dma = false,
    };
    if (err == ESP_OK) err = led_strip_new_rmt_device(&strip, &rmt, &s_led);
    if (err == ESP_OK && xTaskCreate(mode_task, "mode_button_led", 3072, NULL, 4, NULL) != pdPASS) {
        err = ESP_ERR_NO_MEM;
    }
    if (err != ESP_OK && s_led != NULL) {
        led_strip_del(s_led);
        s_led = NULL;
    }
    return err;
}
