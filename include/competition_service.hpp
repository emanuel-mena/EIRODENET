#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMPETITION_ROLE_NONE = 0,
    COMPETITION_ROLE_COMMANDER,
    COMPETITION_ROLE_SOLDIER,
} competition_role_t;

typedef enum {
    COMPETITION_PREFLIGHT_IDLE = 0,
    COMPETITION_PREFLIGHT_WAIT_SENSORS,
    COMPETITION_PREFLIGHT_COMPLETE,
    COMPETITION_PREFLIGHT_FAILED,
} competition_preflight_phase_t;

typedef struct {
    uint8_t step;
    uint8_t failed_step;
    bool ready;
    competition_role_t role;
    esp_err_t error;
    competition_preflight_phase_t preflight_phase;
    float motor_trim_pwm;
    bool drive_calibration_running;
} competition_status_t;

/** Inicializa el punto de extensión autónomo sin activar motores. */
esp_err_t competition_service_start(void);
esp_err_t competition_service_drive_calibration_start(void);
esp_err_t competition_service_drive_calibration_stop(void);
bool competition_service_drive_calibration_running(void);

/** Devuelve el estado resumido para diagnóstico serial. */
const char *competition_service_status(void);

/** Copia el resultado de la verificación. */
void competition_service_get_status(competition_status_t *status);
