#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint32_t id;
    uint8_t color;
    uint8_t retry;
} peer_assignment_t;

typedef enum {
    PEER_ASSIGNMENT_NONE = 0,
    PEER_ASSIGNMENT_ACTIVE,
    PEER_ASSIGNMENT_DONE,
    PEER_ASSIGNMENT_FAILED,
} peer_assignment_result_t;

typedef struct {
    bool configured;
    bool initialized;
    bool connected;
    uint8_t peer_mac[6];
    uint8_t rover_id;
    uint32_t age_ms;
    esp_err_t last_error;
    uint32_t timestamp_ms;
    uint8_t mode;
    bool vision_recent;
    uint32_t vision_frame_age_ms;
    uint32_t boot_id;
    uint8_t reset_reason;
    bool imu_valid;
    bool imu_calibrated;
    float temperature_c;
    float quaternion[4];
    bool ultrasonic_valid;
    uint32_t distance_mm;
    bool infrared_valid;
    uint16_t infrared[4];
    bool color_valid;
    uint16_t color[4];
    int8_t rssi;
    uint8_t navigation_phase;
    bool navigation_has_target;
    float navigation_col;
    float navigation_row;
    uint32_t navigation_request_id;
    int16_t navigation_cell_col;
    int16_t navigation_cell_row;
    uint8_t navigation_heading_index;
    float navigation_heading_deg;
    float vision_heading_offset_deg;
    bool vision_heading_calibrated;
    float navigation_waypoint_col;
    float navigation_waypoint_row;
    uint16_t navigation_segment_count;
    uint16_t navigation_segment_index;
    uint8_t navigation_blind_crossings;
    uint32_t navigation_replans;
    uint8_t navigation_wait_reason;
    esp_err_t navigation_error;
    uint8_t navigation_failure_reason;
    uint8_t navigation_cancel_reason;
    bool navigation_grid_calibrated;
    uint8_t navigation_grid_pattern;
    uint8_t navigation_grid_calibrated_mask;
    int16_t navigation_motor_left;
    int16_t navigation_motor_right;
    int16_t drive_left;
    int16_t drive_right;
    bool link_verified;
    bool identity_received;
    uint8_t verified_identity;
    uint32_t assignment_ack_id;
    bool assignment_ack_accepted;
    uint32_t assignment_id;
    uint8_t assignment_color;
    uint8_t assignment_result;
    uint8_t assignment_phase;
    bool model_available;
    uint32_t model_version;
    uint32_t model_crc32;
    uint8_t competition_delivered_mask;
    bool competition_available;
    uint32_t mode_ack_nonce;
    bool mode_ack_accepted;
    uint8_t mode_ack_reason;
    uint32_t target_ack_nonce;
    uint32_t target_ack_request_id;
    esp_err_t target_ack_error;
} peer_comms_status_t;

/** Inicia la comunicación ESP-NOW cuando Wi-Fi STA esté operativo. */
esp_err_t peer_comms_service_start(void);

/** Copia el último estado recibido; connected vence tras 1500 ms sin paquetes. */
void peer_comms_service_get_status(peer_comms_status_t *status);

/** Envía un comando manual al compañero, sólo si existe enlace reciente. */
esp_err_t peer_comms_service_send_drive(int16_t left, int16_t right);

/** Envía un objetivo en celdas a la navegación de prueba del compañero. */
esp_err_t peer_comms_service_send_target(float col, float row, uint32_t *request_id);
/** Solicita entrada a competencia y espera confirmación del compañero. */
esp_err_t peer_comms_service_request_competition(uint8_t *reason);

/** Reinicia los intercambios de verificación de competencia. */
void peer_comms_service_reset_verification(void);

/** Envía o reenvía una solicitud de enlace; consultar link_verified en el estado. */
esp_err_t peer_comms_service_probe_link(void);

/** Envía o reenvía WHO_AM_I; consultar identity_received en el estado. */
esp_err_t peer_comms_service_query_identity(void);
esp_err_t peer_comms_service_send_assignment(const peer_assignment_t *assignment);
bool peer_comms_service_get_assignment(peer_assignment_t *assignment);
