#pragma once

#include "esp_err.h"

/** Inicializa el punto de extensión autónomo sin activar motores. */
esp_err_t competition_service_start(void);

/** Devuelve "stub" hasta implementar visión v2, estrategia y cooperación. */
const char *competition_service_status(void);
