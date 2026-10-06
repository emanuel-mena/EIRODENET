#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"

#define TINYML_POLICY_INPUTS 68U
#define TINYML_POLICY_OUTPUTS 2U
#define TINYML_POLICY_MODEL_VERSION 1U
#define TINYML_POLICY_ARENA_BYTES (32U * 1024U)

typedef enum {
    TINYML_PHASE_TRANSIT = 0,
    TINYML_PHASE_ALIGN,
    TINYML_PHASE_CAPTURE,
    TINYML_PHASE_PUSH,
    TINYML_PHASE_RETREAT,
    TINYML_PHASE_YIELD,
    TINYML_PHASE_COUNT,
} tinyml_policy_phase_t;

typedef struct {
    bool commander;
    tinyml_policy_phase_t phase;
    uint8_t color;
    bool peer_active;
    bool conflict;
    float linear_speed_cells_s;
    float previous_linear;
    float previous_angular;
} tinyml_policy_context_t;

typedef struct {
    bool available;
    uint32_t version;
    uint32_t length;
    uint32_t crc32;
    uint32_t arena_bytes;
    uint32_t arena_used_bytes;
    uint16_t input_count;
    uint16_t output_count;
    uint32_t last_latency_us;
    uint32_t inference_count;
    esp_err_t error;
} tinyml_policy_status_t;

/** Carga y valida la imagen EIRM. En host habilita el puente de inferencia. */
esp_err_t tinyml_policy_start(void);
void tinyml_policy_reset(void);
void tinyml_policy_get_status(tinyml_policy_status_t *status);
bool tinyml_policy_ready(void);

/** Construye el vector canónico normalizado que comparte firmware y simulador. */
esp_err_t tinyml_policy_build_observation(
    const tinyml_policy_context_t *context,
    const vision_status_t *vision,
    const rover_sensor_state_t *sensors,
    const rover_imu_state_t *imu,
    float observation[TINYML_POLICY_INPUTS]);

/** Ejecuta una inferencia y devuelve valores normalizados en [-1, 1]. */
esp_err_t tinyml_policy_infer(
    const tinyml_policy_context_t *context,
    const vision_status_t *vision,
    const rover_sensor_state_t *sensors,
    const rover_imu_state_t *imu,
    float *linear,
    float *angular);

#ifdef EIRO_HOST_SIM
void tinyml_policy_set_host_model(uint32_t version, uint32_t length, uint32_t crc32,
                                  bool available);
#endif
