#include <inttypes.h>
#include "app_storage.h"
#include "esp_err.h"
#include "esp_log.h"
#include "generated_secrets.h"
#include "model_partition.h"
#include "motor_adapter.h"
#include "rover_service.h"
#include "serial_protocol.h"

static const char *TAG = "rover";

/** @brief Si NVS aún no tiene red, guarda una vez los valores generados desde .env. */
static void seed_wifi_credentials(void)
{
    app_storage_wifi_credentials_t credentials;
    esp_err_t err = app_storage_get_wifi_credentials(&credentials);
    if (err == ESP_ERR_NOT_FOUND && PROJECT_WIFI_SSID[0] != '\0') {
        err = app_storage_set_wifi_credentials(PROJECT_WIFI_SSID, PROJECT_WIFI_PASSWORD);
        ESP_LOGI(TAG, "Credenciales Wi-Fi iniciales guardadas en NVS: %s", esp_err_to_name(err));
    } else if (err == ESP_OK) {
        ESP_LOGI(TAG, "Credenciales Wi-Fi cargadas desde NVS");
    } else {
        ESP_LOGW(TAG, "No hay credenciales Wi-Fi disponibles: %s", esp_err_to_name(err));
    }
}

/** @brief Informa metadatos del modelo o mantiene la inferencia deshabilitada. */
static void report_model(void)
{
    model_partition_info_t model;
    const esp_err_t err = model_partition_validate(&model);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Modelo: version=%" PRIu32 " longitud=%" PRIu32 " CRC32=%08" PRIX32,
                 model.model_version, model.model_size, model.crc32);
    } else {
        ESP_LOGW(TAG, "Inferencia deshabilitada; modelo ausente o invalido: %s", esp_err_to_name(err));
    }
}

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
        seed_wifi_credentials();
    }
    report_model();

    err = rover_service_start();
    ESP_LOGI(TAG, "Servicios del rover: %s", esp_err_to_name(err));
    if (err == ESP_OK) {
        err = serial_protocol_start();
        ESP_LOGI(TAG, "Consola de configuracion: %s", esp_err_to_name(err));
    }
}
