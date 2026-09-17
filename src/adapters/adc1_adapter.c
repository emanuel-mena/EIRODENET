#include "adc1_adapter.h"

static adc_oneshot_unit_handle_t s_adc;
static unsigned int s_users;

esp_err_t adc1_adapter_configure(adc_channel_t channel)
{
    if (s_adc == NULL) {
        const adc_oneshot_unit_init_cfg_t unit_config = { .unit_id = ADC_UNIT_1 };
        esp_err_t err = adc_oneshot_new_unit(&unit_config, &s_adc);
        if (err != ESP_OK) return err;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t err = adc_oneshot_config_channel(s_adc, channel, &channel_config);
    if (err == ESP_OK) {
        ++s_users;
    } else if (s_users == 0) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
    return err;
}

esp_err_t adc1_adapter_read(adc_channel_t channel, int *raw)
{
    if (raw == NULL) return ESP_ERR_INVALID_ARG;
    if (s_adc == NULL) return ESP_ERR_INVALID_STATE;
    return adc_oneshot_read(s_adc, channel, raw);
}

esp_err_t adc1_adapter_release(void)
{
    if (s_users == 0 || s_adc == NULL) return ESP_ERR_INVALID_STATE;
    if (--s_users != 0) return ESP_OK;

    esp_err_t err = adc_oneshot_del_unit(s_adc);
    if (err == ESP_OK) s_adc = NULL;
    return err;
}
