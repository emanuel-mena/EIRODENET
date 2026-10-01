#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define PEER_MISSION_MAX_POINTS 64U
#define PEER_MISSION_FRAGMENT_POINTS 8U

typedef struct { float col, row; } peer_mission_point_t;
typedef struct {
    uint32_t id;
    uint8_t color;
    uint8_t point_count;
    peer_mission_point_t points[PEER_MISSION_MAX_POINTS];
} peer_mission_t;

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
    int16_t drive_left;
    int16_t drive_right;
    bool link_verified;
    bool identity_received;
    uint8_t verified_identity;
    uint32_t mission_ack_id;
    uint8_t mission_ack_fragment;
    bool mission_ack_accepted;
    uint8_t competition_delivered_mask;
    bool competition_available;
} peer_comms_status_t;

/** Inicia la comunicación ESP-NOW cuando Wi-Fi STA esté operativo. */
esp_err_t peer_comms_service_start(void);

/** Copia el último estado recibido; connected vence tras 1500 ms sin paquetes. */
void peer_comms_service_get_status(peer_comms_status_t *status);

/** Envía un comando manual al compañero, sólo si existe enlace reciente. */
esp_err_t peer_comms_service_send_drive(int16_t left, int16_t right);

/** Envía un objetivo en celdas a la navegación de prueba del compañero. */
esp_err_t peer_comms_service_send_target(float col, float row);

/** Reinicia los intercambios de verificación de competencia. */
void peer_comms_service_reset_verification(void);

/** Envía o reenvía una solicitud de enlace; consultar link_verified en el estado. */
esp_err_t peer_comms_service_probe_link(void);

/** Envía o reenvía WHO_AM_I; consultar identity_received en el estado. */
esp_err_t peer_comms_service_query_identity(void);
esp_err_t peer_comms_service_send_mission_fragment(const peer_mission_t *mission, uint8_t fragment);
bool peer_comms_service_get_mission(peer_mission_t *mission);
