#pragma once

#include <stdint.h>
#include "competition_service.hpp"

/** Ejecuta una iteración no bloqueante de la estrategia, después de la verificación. */
void competition_runtime_tick(competition_role_t role, uint32_t generation);
void competition_runtime_reset(void);
uint8_t competition_runtime_delivered_mask(void);
bool competition_runtime_available(void);
