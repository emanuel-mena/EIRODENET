#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @file app_storage.hpp
 * @brief Preferencias persistentes de EIRODENET almacenadas en NVS.
 */

#define APP_STORAGE_WIFI_SSID_MAX_LENGTH 32
#define APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH 64
#define APP_STORAGE_IPV4_MAX_LENGTH 15
#define IMU_CALIBRATION_VERSION 2U
#define DRIVE_CALIBRATION_VERSION 1U
#define APP_STORAGE_ROVER_UNCONFIGURED 0U
#define APP_STORAGE_ROVER_10 10U
#define APP_STORAGE_ROVER_11 11U

/** @brief Calibración persistente del IMU, expresada en las unidades del adapter. */
typedef struct {
    uint32_t version;
    bool valid;
    float accel_offset_g[3];
    float accel_scale[3];
    float gyro_bias_dps[3];
} imu_calibration_t;

typedef struct {
    uint32_t version;
    bool valid;
    float motor_trim_pwm;
} drive_calibration_t;

/** @brief Configuración completa y persistente del rover. */
typedef struct {
    uint8_t who_am_i;
    char wifi_ssid[APP_STORAGE_WIFI_SSID_MAX_LENGTH + 1];
    char wifi_password[APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH + 1];
    bool server_configured;
    char server_ipv4[APP_STORAGE_IPV4_MAX_LENGTH + 1];
    uint16_t server_port;
    bool peer_configured;
    uint8_t peer_mac[6];
} app_storage_config_t;

/** @brief Credenciales Wi-Fi con espacio para terminadores nulos. */
typedef struct {
    char ssid[APP_STORAGE_WIFI_SSID_MAX_LENGTH + 1];
    char password[APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH + 1];
} app_storage_wifi_credentials_t;

/**
 * @brief Inicializa NVS e incrementa el contador persistente de arranques.
 * @param[out] boot_count Contador actualizado.
 * @return ESP_OK o un error de NVS.
 */
esp_err_t app_storage_init(uint32_t *boot_count);

/**
 * @brief Recupera una cadena del namespace de la aplicación.
 * @param[in] key Clave NVS.
 * @param[out] value Buffer de destino.
 * @param[in] value_size Capacidad del buffer, incluido el terminador nulo.
 * @return ESP_OK, ESP_ERR_NVS_NOT_FOUND u otro error de NVS.
 */
esp_err_t app_storage_get_string(const char *key, char *value, size_t value_size);

/**
 * @brief Guarda una cadena y confirma inmediatamente la transacción NVS.
 * @param[in] key Clave NVS.
 * @param[in] value Cadena terminada en nulo.
 * @return ESP_OK o un error de NVS.
 */
esp_err_t app_storage_set_string(const char *key, const char *value);

/**
 * @brief Lee las credenciales Wi-Fi persistentes.
 * @param[out] credentials Credenciales recuperadas.
 * @return ESP_OK, ESP_ERR_NOT_FOUND u otro error de NVS.
 */
esp_err_t app_storage_get_wifi_credentials(app_storage_wifi_credentials_t *credentials);

/**
 * @brief Guarda SSID y contraseña Wi-Fi en NVS.
 * @param[in] ssid Nombre de red, con un máximo de 32 caracteres.
 * @param[in] password Contraseña, con un máximo de 64 caracteres.
 * @return ESP_OK, ESP_ERR_INVALID_ARG u otro error de NVS.
 */
esp_err_t app_storage_set_wifi_credentials(const char *ssid, const char *password);

/** @brief Lee toda la configuración; los campos ausentes se devuelven vacíos. */
esp_err_t app_storage_get_config(app_storage_config_t *config);

/** @brief Valida y guarda toda la configuración en una única transacción NVS. */
esp_err_t app_storage_set_config(const app_storage_config_t *config);

/** @brief Lee la calibración IMU persistente y valida su versión y valores. */
esp_err_t app_storage_get_imu_calibration(imu_calibration_t *calibration);

/** @brief Valida y guarda la calibración IMU. */
esp_err_t app_storage_set_imu_calibration(const imu_calibration_t *calibration);
esp_err_t app_storage_get_drive_calibration(drive_calibration_t *calibration);
esp_err_t app_storage_set_drive_calibration(const drive_calibration_t *calibration);
