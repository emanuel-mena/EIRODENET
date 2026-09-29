#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    COMPETITION_ROLE_NONE = 0,
    COMPETITION_ROLE_COMMANDER,
    COMPETITION_ROLE_SOLDIER,
} competition_role_t;

typedef struct {
    uint8_t step;
    uint8_t failed_step;
    bool ready;
    competition_role_t role;
    esp_err_t error;
} competition_status_t;

/** Inicializa el punto de extensión autónomo sin activar motores. */
esp_err_t competition_service_start(void);

/** Devuelve el estado resumido para la interfaz web. */
const char *competition_service_status(void);

/** Copia el resultado de la verificación. */
void competition_service_get_status(competition_status_t *status);
