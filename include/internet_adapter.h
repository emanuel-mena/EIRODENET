#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @file internet_adapter.h
 * @brief Conexión Wi-Fi en modo estación usando credenciales de app_storage.
 */

/** @brief Estado resumido de la conexión de red. */
typedef struct {
    bool connected;       /**< Verdadero cuando existe asociación con un AP. */
    int8_t rssi;          /**< Potencia recibida en dBm. */
    uint32_t ipv4_address; /**< Dirección IPv4 en orden de red. */
} internet_adapter_status_t;

/**
 * @brief Inicializa netif, el bucle de eventos y Wi-Fi STA.
 * @return ESP_OK o un error de inicialización de red.
 */
esp_err_t internet_adapter_init(void);

/**
 * @brief Conecta usando las credenciales persistentes, con hasta cinco reintentos.
 * @param[in] timeout_ms Tiempo máximo de espera del llamador en milisegundos.
 * @return ESP_OK, ESP_ERR_TIMEOUT, ESP_ERR_NOT_FOUND u otro error Wi-Fi.
 */
esp_err_t internet_adapter_connect(uint32_t timeout_ms);

/**
 * @brief Consulta asociación, RSSI y dirección IPv4 actual.
 * @param[out] status Estado de red; se inicializa a cero antes de consultar.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE u otro error Wi-Fi.
 */
esp_err_t internet_adapter_get_status(internet_adapter_status_t *status);

/**
 * @brief Solicita la desconexión de la estación Wi-Fi.
 * @return ESP_OK, ESP_ERR_INVALID_STATE u otro error Wi-Fi.
 */
esp_err_t internet_adapter_disconnect(void);

/** @brief Detiene y vuelve a iniciar la estación con las credenciales persistentes. */
esp_err_t internet_adapter_reconnect(uint32_t timeout_ms);
