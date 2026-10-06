#pragma once

#include <stdint.h>
#include "competition_service.hpp"

/** Ejecuta una iteración no bloqueante de la estrategia, después de la verificación. */
void competition_runtime_tick(competition_role_t role, uint32_t generation);
void competition_runtime_reset(void);
uint8_t competition_runtime_delivered_mask(void);
bool competition_runtime_available(void);

typedef struct {
    uint32_t id;
    uint8_t color;
    uint8_t result;
    uint8_t phase;
    uint8_t yield_count;
    uint8_t reassignment_count;
    uint8_t blocked_mask;
    uint8_t failure_reason;
} competition_assignment_status_t;

void competition_runtime_get_assignment_status(competition_assignment_status_t *status);
