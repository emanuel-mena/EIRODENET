#include "ultrasonic_adapter.hpp"
#include "board_pins.hpp"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define ECHO_TIMEOUT_US 30000

static constexpr gpio_num_t trigger_gpio = static_cast<gpio_num_t>(BOARD_ULTRASONIC_TRIGGER_GPIO);
static constexpr gpio_num_t echo_gpio = static_cast<gpio_num_t>(BOARD_ULTRASONIC_ECHO_GPIO);

esp_err_t ultrasonic_adapter_init(void)
{
    const gpio_config_t output = {
        .pin_bit_mask = 1ULL << BOARD_ULTRASONIC_TRIGGER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&output);
    if (err != ESP_OK) return err;
    gpio_set_level(trigger_gpio, 0);
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << BOARD_ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
    };
    return gpio_config(&input);
}

esp_err_t ultrasonic_adapter_read_mm(uint32_t *distance_mm)
{
    if (distance_mm == NULL) return ESP_ERR_INVALID_ARG;
    gpio_set_level(trigger_gpio, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trigger_gpio, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trigger_gpio, 0);
    int64_t deadline = esp_timer_get_time() + ECHO_TIMEOUT_US;
    while (gpio_get_level(echo_gpio) == 0) {
        if (esp_timer_get_time() >= deadline) return ESP_ERR_TIMEOUT;
    }
    const int64_t pulse_start = esp_timer_get_time();
    deadline = pulse_start + ECHO_TIMEOUT_US;
    while (gpio_get_level(echo_gpio) != 0) {
        if (esp_timer_get_time() >= deadline) return ESP_ERR_TIMEOUT;
    }
    *distance_mm = (uint32_t)((esp_timer_get_time() - pulse_start) * 343ULL / 2000ULL);
    return ESP_OK;
}
