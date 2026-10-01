#include "app_mode.h"

#include <stdbool.h>

#include "app_storage.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "motor_adapter.h"
#include "navigation_service.h"

#define MODE_TASK_PERIOD_MS 20
#define BUTTON_DEBOUNCE_TICKS 3

static const char *TAG = "app_mode";
static portMUX_TYPE s_mode_lock = portMUX_INITIALIZER_UNLOCKED;
static app_mode_t s_mode = APP_MODE_TEST;
static uint8_t s_identity;
static led_strip_handle_t s_led;
static uint8_t s_failed_step;
static bool s_competition_ready;
static bool s_heading_uncalibrated;
static int64_t s_indicator_epoch_ms;
static uint32_t s_mode_generation;

uint32_t app_mode_generation(void)
{
    taskENTER_CRITICAL(&s_mode_lock);
    const uint32_t generation = s_mode_generation;
    taskEXIT_CRITICAL(&s_mode_lock);
    return generation;
}

void app_mode_set_competition_indicator(uint8_t failed_step, bool ready)
{
    taskENTER_CRITICAL(&s_mode_lock);
    if (s_failed_step != failed_step || s_competition_ready != ready)
        s_indicator_epoch_ms = esp_timer_get_time() / 1000;
    s_failed_step = failed_step;
    s_competition_ready = ready;
    taskEXIT_CRITICAL(&s_mode_lock);
}

void app_mode_set_heading_uncalibrated(bool uncalibrated)
{
    taskENTER_CRITICAL(&s_mode_lock);
    if (s_heading_uncalibrated != uncalibrated)
        s_indicator_epoch_ms = esp_timer_get_time() / 1000;
    s_heading_uncalibrated = uncalibrated;
    taskEXIT_CRITICAL(&s_mode_lock);
}

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
    ++s_mode_generation;
    const app_mode_t mode = s_mode;
    s_failed_step = 0;
    s_competition_ready = false;
    s_heading_uncalibrated = false;
    s_indicator_epoch_ms = esp_timer_get_time() / 1000;
    taskEXIT_CRITICAL(&s_mode_lock);
    navigation_service_cancel(NAVIGATION_CANCEL_MODE);
    motor_adapter_stop();
    ESP_LOGW(TAG, "Modo cambiado con BOOT: %s", app_mode_name(mode));
}

static void set_indicator(uint8_t pulse)
{
    uint8_t red = 0, green = 0, blue = 0;
    taskENTER_CRITICAL(&s_mode_lock);
    const app_mode_t mode = s_mode;
    const uint8_t failed = s_failed_step;
    const bool ready = s_competition_ready;
    const bool uncalibrated = s_heading_uncalibrated;
    const int64_t epoch = s_indicator_epoch_ms;
    taskEXIT_CRITICAL(&s_mode_lock);
    if (mode == APP_MODE_TEST) {
        blue = pulse;
    } else if (failed != 0) {
        const uint32_t cycle = (uint32_t)failed * 400U + 1000U;
        const uint32_t elapsed = (uint32_t)((esp_timer_get_time() / 1000 - epoch) % cycle);
        if (elapsed < (uint32_t)failed * 400U && elapsed % 400U < 200U) red = 72;
    } else if (!ready) {
        /* Indicador apagado mientras la verificación está en curso. */
    } else if (uncalibrated &&
               (uint32_t)(esp_timer_get_time() / 1000 - epoch) % 1000U >= 500U) {
        /* El color del rover parpadea a 1 Hz si el rumbo no quedó calibrado. */
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
