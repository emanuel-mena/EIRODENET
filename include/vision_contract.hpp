#pragma once
#include <stddef.h>
#include <stdint.h>

#define VISION_PROTOCOL_VERSION 3
struct cJSON;

/** Validación compartida por el cliente TCP del firmware y el host de simulación. */
bool vision_contract_validate(const cJSON *root, const cJSON **own_rover,
                              uint8_t own_id, uint16_t *cols, uint16_t *rows, float *cell_mm);
