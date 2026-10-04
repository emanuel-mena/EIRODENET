#pragma once

#include <stdbool.h>
#include <stdint.h>

/** @file motion_control.hpp
 * @brief Cálculos puros para el control de movimiento en competencia.
 */

#define MOTION_BIAS_WINDOW 200U

typedef struct {
    uint64_t last_timestamp_ms;
    uint64_t timestamps[MOTION_BIAS_WINDOW];
    float gyro[MOTION_BIAS_WINDOW];
    float accel_norm[MOTION_BIAS_WINDOW];
    float reference_accel[3];
    uint16_t start, count;
} motion_bias_window_t;

/** Reinicia una ventana de muestras IMU corregidas. */
void motion_bias_reset(motion_bias_window_t *window);
/** Añade una muestra nueva en el marco del rover; rechaza movimiento y duplicados. */
void motion_bias_add(motion_bias_window_t *window, uint64_t timestamp_ms,
                     const float accel_g[3], const float gyro_dps[3]);
/** Devuelve el sesgo residual Z en grados/s cuando hay al menos 100 muestras estables. */
bool motion_bias_result(const motion_bias_window_t *window, float *bias_dps);

/** Envuelve un ángulo a (-180, 180] grados. */
float motion_wrap_degrees(float degrees);
/** Calcula PWM diferencial de recta; salidas no nulas permanecen entre 700 y 1000. */
void motion_drive_command(float heading_error_deg, float cross_track_cells,
                          float angular_speed_dps, float trim, int *left, int *right,
                          bool *saturated);
/** Devuelve PWM firmado de pivote, 0 para frenar o esperar entre impulsos. */
int motion_turn_pwm(float error_deg, float angular_speed_dps, uint32_t tick,
                    bool *settled);
