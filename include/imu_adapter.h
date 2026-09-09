#pragma once

#include "esp_err.h"
#include "lsm6ds3tr_c.h"

/** @file imu_adapter.h
 * @brief Adapter singleton del acelerómetro y giroscopio LSM6DS3TR-C.
 */

/**
 * @brief Inicializa el IMU con el bus, dirección y frecuencia del mapa de pines.
 * @return ESP_OK o un error de configuración, comunicación o identificación.
 */
esp_err_t imu_adapter_init(void);

/**
 * @brief Lee temperatura, aceleración y velocidad angular.
 * @param[out] sample Muestra convertida a grados Celsius, g y grados por segundo.
 * @return ESP_OK, ESP_ERR_INVALID_STATE u otro error I2C.
 */
esp_err_t imu_adapter_read(lsm6ds3tr_c_sample_t *sample);

/**
 * @brief Libera el dispositivo y el bus I2C propiedad del adapter.
 * @return ESP_OK o un error al liberar recursos.
 */
esp_err_t imu_adapter_deinit(void);
