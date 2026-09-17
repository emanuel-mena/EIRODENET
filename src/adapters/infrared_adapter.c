#include "infrared_adapter.h"
#include "adc1_adapter.h"
#include "board_pins.h"

#if BOARD_IR_FRONT_LEFT_GPIO != 35 || BOARD_IR_FRONT_RIGHT_GPIO != 34 || \
    BOARD_IR_REAR_LEFT_GPIO != 39 || BOARD_IR_REAR_RIGHT_GPIO != 36
#error "Actualice los canales ADC cuando cambien los GPIO infrarrojos"
#endif

#define IR_FRONT_LEFT_CHANNEL ADC_CHANNEL_7
#define IR_FRONT_RIGHT_CHANNEL ADC_CHANNEL_6
#define IR_REAR_LEFT_CHANNEL ADC_CHANNEL_3
#define IR_REAR_RIGHT_CHANNEL ADC_CHANNEL_0

esp_err_t infrared_adapter_init(void)
{
    esp_err_t err = adc1_adapter_configure(IR_FRONT_LEFT_CHANNEL);
    if (err == ESP_OK) err = adc1_adapter_configure(IR_FRONT_RIGHT_CHANNEL);
    if (err == ESP_OK) err = adc1_adapter_configure(IR_REAR_LEFT_CHANNEL);
    if (err == ESP_OK) err = adc1_adapter_configure(IR_REAR_RIGHT_CHANNEL);
    return err;
}

esp_err_t infrared_adapter_read(infrared_adapter_state_t *state)
{
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    int raw = 0;
    esp_err_t err = adc1_adapter_read(IR_FRONT_LEFT_CHANNEL, &raw);
    if (err == ESP_OK) state->front_left = (uint16_t)raw;
    if (err == ESP_OK) err = adc1_adapter_read(IR_FRONT_RIGHT_CHANNEL, &raw);
    if (err == ESP_OK) state->front_right = (uint16_t)raw;
    if (err == ESP_OK) err = adc1_adapter_read(IR_REAR_LEFT_CHANNEL, &raw);
    if (err == ESP_OK) state->rear_left = (uint16_t)raw;
    if (err == ESP_OK) err = adc1_adapter_read(IR_REAR_RIGHT_CHANNEL, &raw);
    if (err == ESP_OK) state->rear_right = (uint16_t)raw;
    return err;
}
