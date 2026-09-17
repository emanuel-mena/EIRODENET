#pragma once
#include <stdint.h>
#include "esp_err.h"

/** @file infrared_adapter.h
 * @brief Adapter del arreglo 2x2 de sensores infrarrojos TCRT5000.
 */

/** @brief Lecturas ADC de 12 bits de los cuatro sensores según su posición física. */
typedef struct {
    uint16_t front_left;  /**< SEN1, esquina delantera izquierda. */
    uint16_t front_right; /**< SEN2, esquina delantera derecha. */
    uint16_t rear_left;   /**< SEN3, esquina trasera izquierda. */
    uint16_t rear_right;  /**< SEN4, esquina trasera derecha. */
} infrared_adapter_state_t;

/**
 * @brief Configura los cuatro canales de ADC1.
 * @return ESP_OK o un error del driver ADC.
 */
esp_err_t infrared_adapter_init(void);

/**
 * @brief Obtiene en una sola estructura las lecturas ADC de SEN1 a SEN4.
 * @param[out] state Estado posicional del arreglo.
 * @return ESP_OK o ESP_ERR_INVALID_ARG.
 */
esp_err_t infrared_adapter_read(infrared_adapter_state_t *state);
