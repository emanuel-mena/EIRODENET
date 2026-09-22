#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** Última pose propia aceptada del contrato TCP/NDJSON de visión v2. */
typedef struct {
    bool configured;
    bool connected;
    bool protocol_valid;
    bool pose_valid;
    uint32_t sequence;
    uint64_t frame_timestamp_ms;
    uint64_t received_ms;
    uint32_t age_ms;
    uint8_t rover_id;
    float col;
    float row;
    float theta_deg;
    uint16_t grid_cols;
    uint16_t grid_rows;
    float cell_mm;
    esp_err_t last_error;
} vision_status_t;

/** Inicia el cliente que reconecta al servidor configurado en NVS. */
esp_err_t vision_service_start(void);

/** Fuerza al cliente a releer IP, puerto e identidad después de configurar NVS. */
void vision_service_reload(void);

/** Copia de forma atómica el estado más reciente. */
void vision_service_get_status(vision_status_t *status);
