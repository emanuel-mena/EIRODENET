#include "competition_service.h"

#include "esp_log.h"

static const char *TAG = "competition";

esp_err_t competition_service_start(void)
{
    ESP_LOGW(TAG, "Stub listo: estrategia de competencia y cooperacion aun no implementadas");
    return ESP_OK;
}

const char *competition_service_status(void)
{
    return "stub";
}
