#include <inttypes.h>
#include "app_mode.hpp"
#include "app_storage.hpp"
#include "competition_service.hpp"
#include "diagnostics_service.hpp"
#include "esp_err.h"
#include "esp_log.h"
#include "manual_control_service.hpp"
#include "model_partition.hpp"
#include "motor_adapter.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "serial_protocol.hpp"
#include "vision_service.hpp"

static const char *TAG = "rover";

static void report_model(void)
{
    model_partition_info_t model;
    const esp_err_t err = model_partition_validate(&model);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Modelo: version=%" PRIu32 " longitud=%" PRIu32 " CRC32=%08" PRIX32,
                 model.model_version, model.model_size, model.crc32);
    } else {
        ESP_LOGW(TAG, "Inferencia deshabilitada; modelo ausente o invalido: %s",
                 esp_err_to_name(err));
    }
}

/** @brief Ensambla modos, control, navegación y canales de diagnóstico. */
extern "C" void app_main(void)
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
    diagnostics_service_start(boot_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Storage no disponible: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Arranque NVS numero %" PRIu32, boot_count);
    }
    report_model();

    app_storage_config_t config = {0};
    if (app_storage_get_config(&config) != ESP_OK) {
        ESP_LOGW(TAG, "Identidad no disponible; indicador de competencia usara rojo");
    }
    err = app_mode_start(config.who_am_i);
    ESP_LOGI(TAG, "Selector de modo: %s", esp_err_to_name(err));
    err = rover_service_start();
    ESP_LOGI(TAG, "Servicios del rover: %s", esp_err_to_name(err));
    if (err == ESP_OK) {
        err = vision_service_start();
        ESP_LOGI(TAG, "Cliente de vision v2: %s", esp_err_to_name(err));
        err = navigation_service_start();
        ESP_LOGI(TAG, "Navegacion en core 1: %s", esp_err_to_name(err));
        err = manual_control_service_start();
        ESP_LOGI(TAG, "Control manual seguro: %s", esp_err_to_name(err));
        err = competition_service_start();
        ESP_LOGI(TAG, "Competencia: %s", esp_err_to_name(err));
        const esp_err_t peer_err = peer_comms_service_start();
        ESP_LOGI(TAG, "Enlace ESP-NOW: %s", esp_err_to_name(peer_err));
        err = serial_protocol_start();
        ESP_LOGI(TAG, "Consola de configuracion: %s", esp_err_to_name(err));
    }
}

