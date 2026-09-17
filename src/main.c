#include <inttypes.h>
#include "app_storage.h"
#include "esp_err.h"
#include "esp_log.h"
#include "motor_adapter.h"
#include "rover_service.h"
#include "serial_protocol.h"

static const char *TAG = "rover";

/** @brief Ensambla los adapters y ejecuta el diagnóstico periódico del rover. */
void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando adapters EIRODENET");
    esp_err_t err = motor_adapter_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo asegurar la parada de motores: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Motores inicializados y detenidos");
    }

    uint32_t boot_count = 0;
    err = app_storage_init(&boot_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Storage no disponible: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Arranque NVS numero %" PRIu32, boot_count);
    }
    err = rover_service_start();
    ESP_LOGI(TAG, "Servicios del rover: %s", esp_err_to_name(err));
    if (err == ESP_OK) {
        err = serial_protocol_start();
        ESP_LOGI(TAG, "Consola de configuracion: %s", esp_err_to_name(err));
    }
}
