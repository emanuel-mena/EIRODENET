#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

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
    int16_t drive_left;
    int16_t drive_right;
} peer_comms_status_t;

/** Inicia la comunicación ESP-NOW cuando Wi-Fi STA esté operativo. */
esp_err_t peer_comms_service_start(void);

/** Copia el último estado recibido; connected vence tras 1500 ms sin paquetes. */
void peer_comms_service_get_status(peer_comms_status_t *status);

/** Envía un comando manual al compañero, sólo si existe enlace reciente. */
esp_err_t peer_comms_service_send_drive(int16_t left, int16_t right);

/** Envía un objetivo en celdas al stub de navegación del compañero. */
esp_err_t peer_comms_service_send_target(float col, float row);
