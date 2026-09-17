#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

esp_err_t adc1_adapter_configure(adc_channel_t channel);
esp_err_t adc1_adapter_read(adc_channel_t channel, int *raw);
esp_err_t adc1_adapter_release(void);
