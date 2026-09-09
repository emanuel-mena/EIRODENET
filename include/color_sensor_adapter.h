#pragma once

#include <stdint.h>
#include "esp_err.h"

/** @file color_sensor_adapter.h
 * @brief Sensor de color reflectivo formado por un NeoPixel y una fotoresistencia.
 */

/** @brief Lecturas ADC con iluminación apagada, roja, verde y azul. */
typedef struct {
    uint16_t ambient; /**< Nivel ADC sin iluminación. */
    uint16_t red;     /**< Nivel ADC con iluminación roja. */
    uint16_t green;   /**< Nivel ADC con iluminación verde. */
    uint16_t blue;    /**< Nivel ADC con iluminación azul. */
} color_sensor_adapter_sample_t;

/**
 * @brief Inicializa ADC1 y el único NeoPixel del sensor.
 * @return ESP_OK o un error de ADC/RMT.
 */
esp_err_t color_sensor_adapter_init(void);

/**
 * @brief Captura la respuesta de la fotoresistencia para cada iluminación.
 * @param[out] sample Cuatro lecturas ADC de 12 bits.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE u otro error de driver.
 */
esp_err_t color_sensor_adapter_read(color_sensor_adapter_sample_t *sample);

/**
 * @brief Apaga el NeoPixel y libera ADC y RMT.
 * @return ESP_OK o el primer error encontrado al liberar recursos.
 */
esp_err_t color_sensor_adapter_deinit(void);
