#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/** @file app_storage.h
 * @brief Preferencias persistentes de EIRODENET almacenadas en NVS.
 */

#define APP_STORAGE_WIFI_SSID_MAX_LENGTH 32
#define APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH 64

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
