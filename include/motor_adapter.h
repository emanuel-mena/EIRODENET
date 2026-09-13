#pragma once
#include <stdint.h>
#include "esp_err.h"

/** @file motor_adapter.h
 * @brief Control PWM independiente de Motor 1 y Motor 2 mediante puentes H.
 */

/** @brief Magnitud máxima aceptada para cada comando de motor. */
#define MOTOR_ADAPTER_MAX_COMMAND 1000
#define MOTOR_ADAPTER_TEST_COMMAND 700
#define MOTOR_ADAPTER_TEST_DURATION_MS 1000

/**
 * @brief Inicializa los cuatro canales PWM con ambos motores detenidos.
 * @return ESP_OK o un error del periférico LEDC.
 */
esp_err_t motor_adapter_init(void);

/**
 * @brief Establece velocidad y sentido de ambos motores.
 * @param[in] motor1 Comando de -1000 a 1000 para GPIO12/GPIO14.
 * @param[in] motor2 Comando de -1000 a 1000 para GPIO13/GPIO15.
 * @return ESP_OK, ESP_ERR_INVALID_ARG o ESP_ERR_INVALID_STATE.
 * @note Cero deja el motor correspondiente en rueda libre.
 */
esp_err_t motor_adapter_set(int16_t motor1, int16_t motor2);

/**
 * @brief Lleva inmediatamente ambos comandos PWM a cero.
 * @return ESP_OK o un error del periférico LEDC.
 */
esp_err_t motor_adapter_stop(void);

/**
 * @brief Mueve ambos motores hacia delante al 70 % durante un segundo y los detiene.
 * @return ESP_OK o el primer error producido al activar o detener los motores.
 * @warning Ejecute la prueba con el rover suspendido y las ruedas libres.
 */
esp_err_t motor_adapter_test_forward(void);
