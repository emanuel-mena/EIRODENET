#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int16_t left;
    int16_t right;
    bool watchdog_armed;
} manual_control_status_t;

/** Inicia la parada de seguridad de comandos seriales. */
esp_err_t manual_control_service_start(void);

/** Aplica un comando por un máximo de 500 ms; sólo es válido en modo prueba. */
esp_err_t manual_control_service_set(int16_t left, int16_t right);

/** Copia el comando actualmente aplicado y el estado del watchdog. */
void manual_control_service_get_status(manual_control_status_t *status);
