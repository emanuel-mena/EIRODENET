#pragma once

#include <stdint.h>
#include "competition_service.hpp"

typedef struct {
    uint8_t phase;
    uint8_t target_color;
    float target_col;
    float target_row;
    float distance_remaining_cells;
    float detour_heading_deg;
    bool detour_active;
    bool soldier_paused;
    uint8_t delivered_mask;
} competition_runtime_status_t;

/** Ejecuta una iteración no bloqueante de la estrategia, después de la verificación. */
void competition_runtime_tick(competition_role_t role, uint32_t generation);
void competition_runtime_reset(void);
uint8_t competition_runtime_delivered_mask(void);
bool competition_runtime_available(void);
bool competition_runtime_moving(void);
void competition_runtime_get_status(competition_runtime_status_t *status);
