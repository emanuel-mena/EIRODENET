#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @file lsm6ds3tr_c.h
 * @brief Driver I2C de bajo nivel para el IMU ST LSM6DS3TR-C.
 */

/** @brief Dirección I2C predeterminada con SA0 en nivel alto. */
#define LSM6DS3TR_C_I2C_ADDRESS_DEFAULT 0x6B
/** @brief Valor fijo esperado del registro WHO_AM_I. */
#define LSM6DS3TR_C_WHO_AM_I_VALUE 0x6A

/** @brief Handle opaco que posee dispositivo y bus I2C. */
typedef struct lsm6ds3tr_c_device *lsm6ds3tr_c_handle_t;

/** @brief Configuración necesaria para crear una instancia del driver. */
typedef struct {
    int sda_gpio;          /**< GPIO de datos I2C. */
    int scl_gpio;          /**< GPIO de reloj I2C. */
    uint8_t i2c_address;   /**< Dirección I2C de siete bits. */
    uint32_t i2c_clock_hz; /**< Frecuencia del bus en hercios. */
} lsm6ds3tr_c_config_t;

/** @brief Muestra física convertida a unidades de ingeniería. */
typedef struct {
    float temperature_c; /**< Temperatura en grados Celsius. */
    float gyro_dps[3];   /**< Velocidad angular XYZ en grados por segundo. */
    float accel_g[3];    /**< Aceleración XYZ en múltiplos de g. */
} lsm6ds3tr_c_sample_t;

/**
 * @brief Inicializa el bus y configura 104 Hz, ±2 g y ±245 grados/s.
 * @param[in] config Pines, dirección y frecuencia I2C.
 * @param[out] out_handle Handle que adquiere propiedad del bus.
 * @return ESP_OK o un error de argumentos, memoria, I2C o identificación.
 */
esp_err_t lsm6ds3tr_c_init(
    const lsm6ds3tr_c_config_t *config,
    lsm6ds3tr_c_handle_t *out_handle
);

/**
 * @brief Lee el registro fijo WHO_AM_I.
 * @param[in] handle Dispositivo inicializado.
 * @param[out] device_id Identificador leído; debe ser 0x6A.
 * @return ESP_OK, ESP_ERR_INVALID_ARG u otro error I2C.
 */
esp_err_t lsm6ds3tr_c_read_device_id(
    lsm6ds3tr_c_handle_t handle,
    uint8_t *device_id
);

/**
 * @brief Lee temperatura, velocidad angular y aceleración en una transacción.
 * @param[in] handle Dispositivo inicializado.
 * @param[out] sample Valores convertidos a unidades físicas.
 * @return ESP_OK, ESP_ERR_INVALID_ARG u otro error I2C.
 */
esp_err_t lsm6ds3tr_c_read_sample(
    lsm6ds3tr_c_handle_t handle,
    lsm6ds3tr_c_sample_t *sample
);

/**
 * @brief Libera el dispositivo y el bus I2C asociado.
 * @param[in] handle Dispositivo que se debe liberar.
 * @return ESP_OK, ESP_ERR_INVALID_ARG o un error al liberar recursos.
 */
esp_err_t lsm6ds3tr_c_deinit(lsm6ds3tr_c_handle_t handle);

#ifdef __cplusplus
}
#endif
