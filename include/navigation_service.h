#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    NAVIGATION_IDLE = 0,
    NAVIGATION_TARGET_RECEIVED,
    NAVIGATION_STUB,
} navigation_phase_t;

typedef struct {
    navigation_phase_t phase;
    bool has_target;
    float col;
    float row;
    uint32_t request_id;
    int core_id;
} navigation_status_t;

/** Crea la cola y la tarea del stub fijada al núcleo 1. */
esp_err_t navigation_service_start(void);

/** Encola el objetivo más reciente, expresado en celdas del contrato de visión. */
esp_err_t navigation_service_submit(float col, float row, uint32_t *request_id);

/** Copia el último objetivo y la fase observable del stub. */
void navigation_service_get_status(navigation_status_t *status);
