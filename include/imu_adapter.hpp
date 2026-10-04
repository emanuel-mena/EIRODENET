#pragma once

#include "esp_err.h"
#include "app_storage.hpp"
#include "lsm6ds3tr_c.hpp"

/** @file imu_adapter.hpp
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
 * @brief Lee una muestra sin calibrar en el marco del rover.
 *
 * El montaje físico rota el chip 180 grados alrededor de Z: X e Y se invierten,
 * mientras Z conserva su signo.
 */
esp_err_t imu_adapter_read_raw(lsm6ds3tr_c_sample_t *sample);

/** @brief Sustituye la calibración activa; una calibración no válida deja paso directo. */
esp_err_t imu_adapter_set_calibration(const imu_calibration_t *calibration);

/** @brief Obtiene una copia de la calibración activa. */
esp_err_t imu_adapter_get_calibration(imu_calibration_t *calibration);

/**
 * @brief Libera el dispositivo y el bus I2C propiedad del adapter.
 * @return ESP_OK o un error al liberar recursos.
 */
esp_err_t imu_adapter_deinit(void);
