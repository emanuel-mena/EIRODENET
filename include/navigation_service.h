#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    NAVIGATION_IDLE = 0,
    NAVIGATION_WAITING_FOR_POSE,
    NAVIGATION_TURNING,
    NAVIGATION_DRIVING,
    NAVIGATION_ARRIVED,
    NAVIGATION_BLOCKED,
    NAVIGATION_CANCELLED,
    NAVIGATION_ERROR,
    NAVIGATION_PLANNING,
    NAVIGATION_REPLANNING,
    NAVIGATION_WAITING_FOR_VISION,
} navigation_phase_t;

typedef enum {
    NAVIGATION_CANCEL_USER = 0,
    NAVIGATION_CANCEL_MANUAL,
    NAVIGATION_CANCEL_MODE,
    NAVIGATION_CANCEL_REPLACED,
} navigation_cancel_reason_t;

typedef enum {
    NAVIGATION_CORRECTION_NONE = 0,
    NAVIGATION_CORRECTION_IMU,
    NAVIGATION_CORRECTION_GRID,
    NAVIGATION_CORRECTION_VISION,
} navigation_correction_t;

typedef enum {
    NAVIGATION_WAIT_NONE = 0,
    NAVIGATION_WAIT_VISION_LIMIT,
    NAVIGATION_WAIT_GRID_UNCALIBRATED,
    NAVIGATION_WAIT_PATH_OCCUPIED,
} navigation_wait_reason_t;

typedef struct {
    navigation_phase_t phase;
    bool has_target;
    float col;
    float row;
    uint32_t request_id;
    int core_id;
    bool pose_valid;
    float pose_col;
    float pose_row;
    float theta_deg;
    float linear_speed_cells_s;
    float angular_speed_dps;
    float uncertainty_cells;
    float vision_heading_offset_deg;
    bool vision_heading_calibrated;
    bool vision_configured;
    bool vision_connected;
    bool vision_fresh;
    uint32_t vision_age_ms;
    bool grid_calibrated;
    bool grid_correction_active;
    uint8_t infrared_pattern;
    uint8_t infrared_calibrated_mask;
    navigation_correction_t last_correction;
    int16_t confirmed_cell_col;
    int16_t confirmed_cell_row;
    uint8_t heading_index;
    float desired_heading_deg;
    float waypoint_col;
    float waypoint_row;
    uint16_t route_segment_count;
    uint16_t route_segment_index;
    uint8_t crossings_without_vision;
    uint32_t replan_count;
    navigation_wait_reason_t wait_reason;
    int16_t motor_left;
    int16_t motor_right;
    navigation_cancel_reason_t cancel_reason;
    esp_err_t error;
} navigation_status_t;

esp_err_t navigation_service_start(void);
esp_err_t navigation_service_submit(float col, float row, uint32_t *request_id);
/** Objetivo interno de competencia; el modo y la fase RUNNING se verifican también en cada ciclo. */
esp_err_t navigation_service_submit_competition(float col, float row, uint32_t *request_id);
void navigation_service_set_competition_cube(uint8_t color, bool allow_contact);
esp_err_t navigation_service_cancel(navigation_cancel_reason_t reason);
void navigation_service_get_status(navigation_status_t *status);
/** Instala o borra el desfase de visión para la entrada actual en competencia. */
void navigation_service_set_heading_calibration(float offset_deg, bool calibrated);
