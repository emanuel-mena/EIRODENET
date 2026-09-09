#pragma once
#include <stdint.h>
#include "esp_err.h"

/** @file ultrasonic_adapter.h
 * @brief Adapter del sensor ultrasónico HY-SRF05.
 */

/**
 * @brief Configura los GPIO de Trigger y Echo declarados en board_pins.h.
 * @return ESP_OK o un error del driver GPIO.
 */
esp_err_t ultrasonic_adapter_init(void);

/**
 * @brief Ejecuta una medición de distancia con timeout de 30 ms.
 * @param[out] distance_mm Distancia calculada en milímetros.
 * @return ESP_OK, ESP_ERR_TIMEOUT o ESP_ERR_INVALID_ARG.
 */
esp_err_t ultrasonic_adapter_read_mm(uint32_t *distance_mm);
