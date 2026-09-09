#pragma once
#include <stdbool.h>
#include "esp_err.h"

/** @file infrared_adapter.h
 * @brief Adapter del arreglo 2x2 de sensores infrarrojos TCRT5000.
 */

/** @brief Estado digital de los cuatro sensores según su posición física. */
typedef struct {
    bool front_left;  /**< SEN1, esquina delantera izquierda. */
    bool front_right; /**< SEN2, esquina delantera derecha. */
    bool rear_left;   /**< SEN3, esquina trasera izquierda. */
    bool rear_right;  /**< SEN4, esquina trasera derecha. */
} infrared_adapter_state_t;

/**
 * @brief Configura los cuatro GPIO como entradas sin resistencias internas.
 * @return ESP_OK o un error del driver GPIO.
 */
esp_err_t infrared_adapter_init(void);

/**
 * @brief Obtiene en una sola estructura el nivel lógico de SEN1 a SEN4.
 * @param[out] state Estado posicional del arreglo.
 * @return ESP_OK o ESP_ERR_INVALID_ARG.
 */
esp_err_t infrared_adapter_read(infrared_adapter_state_t *state);
