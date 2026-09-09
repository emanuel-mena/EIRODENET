#include "infrared_adapter.h"
#include "board_pins.h"
#include "driver/gpio.h"

esp_err_t infrared_adapter_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << BOARD_IR_FRONT_LEFT_GPIO) | (1ULL << BOARD_IR_FRONT_RIGHT_GPIO) |
                        (1ULL << BOARD_IR_REAR_LEFT_GPIO) | (1ULL << BOARD_IR_REAR_RIGHT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    return gpio_config(&config);
}

esp_err_t infrared_adapter_read(infrared_adapter_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    state->front_left = gpio_get_level(BOARD_IR_FRONT_LEFT_GPIO) != 0;
    state->front_right = gpio_get_level(BOARD_IR_FRONT_RIGHT_GPIO) != 0;
    state->rear_left = gpio_get_level(BOARD_IR_REAR_LEFT_GPIO) != 0;
    state->rear_right = gpio_get_level(BOARD_IR_REAR_RIGHT_GPIO) != 0;
    return ESP_OK;
}
