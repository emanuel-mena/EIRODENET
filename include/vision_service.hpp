#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define VISION_MAX_OBSTACLES 32U
#define VISION_MAX_ROVERS 2U
#define VISION_MAX_CUBES 3U

typedef enum {
    VISION_PHASE_IDLE = 0,
    VISION_PHASE_READY,
    VISION_PHASE_RUNNING,
    VISION_PHASE_FINISHED,
} vision_phase_t;

typedef enum {
    VISION_CUBE_GREEN = 0,
    VISION_CUBE_BLUE,
    VISION_CUBE_RED,
} vision_cube_color_t;

typedef struct {
    float col;
    float row;
    uint32_t age_ms;
} vision_position_t;

/** Última pose propia aceptada del contrato TCP/NDJSON de visión v3. */
typedef struct {
    bool configured;
    bool connected;
    bool protocol_valid;
    bool pose_valid;
    uint32_t sequence;
    uint64_t frame_timestamp_ms;
    uint64_t received_ms;
    uint64_t last_valid_frame_ms; /**< Recepción de la última trama v3 válida, aun sin pose propia. */
    uint32_t age_ms;
    uint8_t rover_id;
    float col;
    float row;
    float theta_deg;
    bool peer_valid;
    uint8_t peer_id;
    float peer_col;
    float peer_row;
    float peer_theta_deg;
    uint32_t peer_age_ms;
    uint8_t obstacle_count;
    vision_position_t obstacles[VISION_MAX_OBSTACLES];
    vision_phase_t phase;
    bool cube_valid[VISION_MAX_CUBES];
    bool cube_in_depot[VISION_MAX_CUBES]; /**< Veredicto del árbitro; sólo usable con detección fresca. */
    vision_position_t cubes[VISION_MAX_CUBES];
    bool depot_valid[VISION_MAX_CUBES];
    float depot_col[VISION_MAX_CUBES];
    float depot_row[VISION_MAX_CUBES];
    float depot_length;
    float depot_depth;
    float cube_side;
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
