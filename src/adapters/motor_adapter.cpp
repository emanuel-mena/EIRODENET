#include "motor_adapter.hpp"
#include <stdbool.h>
#include <stdlib.h>
#include "board_pins.hpp"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_PWM_FREQUENCY_HZ 20000
#define MOTOR_PWM_MAX_DUTY 1023
static bool s_initialized;

static esp_err_t set_channel(ledc_channel_t channel, uint32_t duty)
{
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    return err == ESP_OK ? ledc_update_duty(LEDC_LOW_SPEED_MODE, channel) : err;
}

static esp_err_t set_one_motor(int16_t command, ledc_channel_t forward, ledc_channel_t reverse)
{
    if (command < -MOTOR_ADAPTER_MAX_COMMAND || command > MOTOR_ADAPTER_MAX_COMMAND) return ESP_ERR_INVALID_ARG;
    int magnitude = abs(command);
    if (magnitude > 0 && magnitude < MOTOR_ADAPTER_MIN_COMMAND) magnitude = MOTOR_ADAPTER_MIN_COMMAND;
    const uint32_t duty = (uint32_t)magnitude * MOTOR_PWM_MAX_DUTY / MOTOR_ADAPTER_MAX_COMMAND;
    esp_err_t err = set_channel(forward, 0);
    if (err == ESP_OK) err = set_channel(reverse, 0);
    if (err != ESP_OK || command == 0) return err;
    return set_channel(command > 0 ? forward : reverse, duty);
}

esp_err_t motor_adapter_init(void)
{
    const ledc_timer_config_t timer = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = MOTOR_PWM_FREQUENCY_HZ, .clk_cfg = LEDC_AUTO_CLK };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;
    const int pins[] = {BOARD_MOTOR_1_A_GPIO, BOARD_MOTOR_1_B_GPIO,
                        BOARD_MOTOR_2_A_GPIO, BOARD_MOTOR_2_B_GPIO};
    for (int channel = 0; channel < 4; ++channel) {
        const ledc_channel_config_t config = { .gpio_num = pins[channel], .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)channel, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0,
            .duty = 0, .hpoint = 0 };
        err = ledc_channel_config(&config);
        if (err != ESP_OK) return err;
    }
    s_initialized = true;
    return motor_adapter_stop();
}

esp_err_t motor_adapter_set(int16_t motor1, int16_t motor2)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (motor1 < -MOTOR_ADAPTER_MAX_COMMAND || motor1 > MOTOR_ADAPTER_MAX_COMMAND ||
        motor2 < -MOTOR_ADAPTER_MAX_COMMAND || motor2 > MOTOR_ADAPTER_MAX_COMMAND) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = set_one_motor(motor1, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    return err == ESP_OK ? set_one_motor(motor2, LEDC_CHANNEL_2, LEDC_CHANNEL_3) : err;
}

esp_err_t motor_adapter_stop(void) { return motor_adapter_set(0, 0); }

esp_err_t motor_adapter_test_forward(void)
{
    esp_err_t err = motor_adapter_set(MOTOR_ADAPTER_TEST_COMMAND, MOTOR_ADAPTER_TEST_COMMAND);
    if (err != ESP_OK) {
        motor_adapter_stop();
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(MOTOR_ADAPTER_TEST_DURATION_MS));
    return motor_adapter_stop();
}
