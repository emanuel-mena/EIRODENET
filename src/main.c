#include <inttypes.h>
#include <stdbool.h>

#include "app_storage.h"
#include "board_pins.h"
#include "color_sensor_adapter.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "generated_secrets.h"
#include "imu_adapter.h"
#include "infrared_adapter.h"
#include "internet_adapter.h"
#include "model_partition.h"
#include "motor_adapter.h"
#include "ultrasonic_adapter.h"

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

    const esp_err_t ultrasonic_init_err = ultrasonic_adapter_init();
    const bool ultrasonic_ready = ultrasonic_init_err == ESP_OK;
    const esp_err_t infrared_init_err = infrared_adapter_init();
    const bool infrared_ready = infrared_init_err == ESP_OK;
    ESP_LOGI(TAG, "Ultrasonico init: %s; infrarrojos init: %s",
             esp_err_to_name(ultrasonic_init_err), esp_err_to_name(infrared_init_err));

    const esp_err_t imu_init_err = imu_adapter_init();
    const bool imu_ready = imu_init_err == ESP_OK;
    ESP_LOGI(TAG, "IMU LSM6DS3TR-C: %s", esp_err_to_name(imu_init_err));

    const bool color_ready = color_sensor_adapter_init() == ESP_OK;

    err = internet_adapter_init();
    if (err == ESP_OK) {
        err = internet_adapter_connect(15000);
    }
    ESP_LOGI(TAG, "Conexion Wi-Fi: %s", esp_err_to_name(err));

    while (true) {
        uint32_t distance_mm = 0;
        infrared_adapter_state_t infrared = { 0 };
        lsm6ds3tr_c_sample_t imu = { 0 };
        color_sensor_adapter_sample_t color = { 0 };
        if (ultrasonic_ready) {
            const esp_err_t read_err = ultrasonic_adapter_read_mm(&distance_mm);
            if (read_err == ESP_OK) {
                ESP_LOGI(TAG, "Ultrasonico: %" PRIu32 " mm", distance_mm);
            } else {
                ESP_LOGW(TAG, "Ultrasonico sin eco: %s", esp_err_to_name(read_err));
            }
        }
        if (infrared_ready && infrared_adapter_read(&infrared) == ESP_OK) {
            ESP_LOGI(TAG, "IR FI=%d FD=%d TI=%d TD=%d", infrared.front_left,
                     infrared.front_right, infrared.rear_left, infrared.rear_right);
        }
        if (imu_ready) {
            const esp_err_t read_err = imu_adapter_read(&imu);
            if (read_err == ESP_OK) {
                ESP_LOGI(TAG, "IMU acc[g]=%.3f,%.3f,%.3f gyro[dps]=%.2f,%.2f,%.2f",
                         imu.accel_g[0], imu.accel_g[1], imu.accel_g[2],
                         imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2]);
            } else {
                ESP_LOGW(TAG, "Fallo de lectura IMU: %s", esp_err_to_name(read_err));
            }
        } else {
            ESP_LOGW(TAG, "IMU no disponible; revise GPIO%d/GPIO%d y direccion 0x%02X",
                     BOARD_IMU_SDA_GPIO, BOARD_IMU_SCL_GPIO, BOARD_IMU_I2C_ADDRESS);
        }
        if (color_ready && color_sensor_adapter_read(&color) == ESP_OK) {
            ESP_LOGI(TAG, "Color ADC ambiente=%u R=%u G=%u B=%u",
                     color.ambient, color.red, color.green, color.blue);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
