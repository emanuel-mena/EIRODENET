#include "peer_comms_service.hpp"

#include <math.h>
#include <string.h>

#include "app_mode.hpp"
#include "app_storage.hpp"
#include "competition_runtime.hpp"
#include "diagnostics_service.hpp"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "manual_control_service.hpp"
#include "navigation_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"
#include "tinyml_policy.hpp"

#define PEER_MAGIC 0x524f4952U
#define PEER_PROTOCOL_VERSION 9U
#define PEER_STATE_PERIOD_MS 200U
#define PEER_TIMEOUT_MS 1500U

typedef enum {
    PEER_MESSAGE_STATE = 1,
    PEER_MESSAGE_DRIVE = 2,
    PEER_MESSAGE_TARGET = 3,
    PEER_MESSAGE_LINK_REQUEST = 4,
    PEER_MESSAGE_LINK_REPLY = 5,
    PEER_MESSAGE_IDENTITY_REQUEST = 6,
    PEER_MESSAGE_IDENTITY_REPLY = 7,
    PEER_MESSAGE_ASSIGNMENT = 8,
    PEER_MESSAGE_ASSIGNMENT_ACK = 9,
    PEER_MESSAGE_COMPETITION_ENTER = 10,
    PEER_MESSAGE_COMPETITION_ACK = 11,
    PEER_MESSAGE_LOG = 12,
    PEER_MESSAGE_LOG_ACK = 13,
    PEER_MESSAGE_TARGET_ACK = 14,
} peer_message_type_t;

typedef struct {
    uint32_t timestamp_ms;
    uint8_t mode;
    uint8_t flags;
    int8_t rssi;
    uint8_t navigation_phase;
    float temperature_c;
    float quaternion[4];
    uint32_t distance_mm;
    uint16_t infrared[4];
    uint16_t color[4];
    float navigation_col;
    float navigation_row;
    uint32_t navigation_request_id;
    int16_t navigation_cell_col;
    int16_t navigation_cell_row;
    uint8_t navigation_heading_index;
    uint8_t navigation_blind_crossings;
    uint8_t navigation_wait_reason;
    uint8_t navigation_cancel_reason;
    uint8_t navigation_grid_calibrated;
    uint8_t navigation_grid_pattern;
    uint8_t navigation_grid_calibrated_mask;
    int16_t navigation_motor_left;
    int16_t navigation_motor_right;
    int32_t navigation_error;
    uint8_t navigation_failure_reason;
    float navigation_heading_deg;
    float vision_heading_offset_deg;
    float navigation_waypoint_col;
    float navigation_waypoint_row;
    uint16_t navigation_segment_count;
    uint16_t navigation_segment_index;
    uint32_t navigation_replans;
    int16_t drive_left;
    int16_t drive_right;
    uint8_t competition_delivered_mask;
    uint8_t competition_available;
    uint8_t vision_recent;
    uint16_t vision_frame_age_ms;
    uint32_t boot_id;
    uint8_t reset_reason;
    uint32_t assignment_id;
    uint32_t model_version;
    uint32_t model_crc32;
    uint8_t assignment_color;
    uint8_t assignment_result;
    uint8_t assignment_phase;
} peer_state_payload_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t type;
    uint8_t source_id;
    uint32_t sequence;
    union {
        peer_state_payload_t state;
        struct { int16_t left; int16_t right; } drive;
        struct { float col; float row; uint32_t nonce; } target;
        struct { uint32_t nonce, request_id; int32_t error; } target_ack;
        struct { uint32_t nonce; uint8_t identity; } verification;
        struct { uint32_t id; uint8_t color; } assignment;
        struct { uint32_t id; uint8_t accepted; } assignment_ack;
        struct { uint32_t nonce; uint8_t accepted, reason; } mode;
        diagnostic_entry_t log;
        struct { uint32_t boot_id, sequence; } log_ack;
    } payload;
} peer_packet_t;

typedef struct {
    uint8_t source_mac[6];
    peer_packet_t packet;
} received_packet_t;

static_assert(sizeof(peer_packet_t) <= ESP_NOW_MAX_DATA_LEN, "Paquete ESP-NOW demasiado grande");

static const char *TAG = "peer_comms";
static SemaphoreHandle_t s_lock;
static QueueHandle_t s_received;
static peer_comms_status_t s_status;
static uint8_t s_own_id;
static uint32_t s_sequence;
static int64_t s_last_seen_ms;
static uint32_t s_link_nonce;
static uint32_t s_identity_nonce;
static peer_assignment_t s_assignment;
static bool s_assignment_complete;
static uint32_t s_last_mode_nonce;
static uint32_t s_last_target_nonce;
static uint32_t s_last_target_request_id;
static esp_err_t s_last_target_error;
static uint32_t s_forwarded_log_sequence;
static uint32_t s_pending_log_sequence;
static uint32_t s_pending_log_boot_id;
static int64_t s_next_log_retry_ms;

static bool recent_vision(uint32_t *age_ms)
{
    vision_status_t vision = {0};
    vision_service_get_status(&vision);
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    const uint64_t age = vision.last_valid_frame_ms != 0 && now_ms >= vision.last_valid_frame_ms
        ? now_ms - vision.last_valid_frame_ms : UINT32_MAX;
    if (age_ms != NULL) *age_ms = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    return vision.connected && vision.protocol_valid && age <= 750;
}

static bool valid_identity(uint8_t identity)
{
    return identity == APP_STORAGE_ROVER_10 || identity == APP_STORAGE_ROVER_11;
}

static void receive_callback(const esp_now_recv_info_t *info, const uint8_t *data, int length)
{
    if (info == NULL || info->src_addr == NULL || data == NULL ||
        length != sizeof(peer_packet_t) || s_received == NULL) return;
    received_packet_t received = {0};
    memcpy(received.source_mac, info->src_addr, sizeof(received.source_mac));
    memcpy(&received.packet, data, sizeof(received.packet));
    if (received.packet.magic != PEER_MAGIC || received.packet.version != PEER_PROTOCOL_VERSION) return;
    xQueueSend(s_received, &received, 0);
}

static esp_err_t send_packet(peer_packet_t *packet)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool ready = s_status.initialized && s_status.configured;
    uint8_t peer_mac[6];
    memcpy(peer_mac, s_status.peer_mac, sizeof(peer_mac));
    packet->magic = PEER_MAGIC;
    packet->version = PEER_PROTOCOL_VERSION;
    packet->source_id = s_own_id;
    packet->sequence = ++s_sequence;
    xSemaphoreGive(s_lock);
    if (!ready) return ESP_ERR_INVALID_STATE;
    const esp_err_t err = esp_now_send(peer_mac, (const uint8_t *)packet, sizeof(*packet));
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.last_error = err;
    xSemaphoreGive(s_lock);
    return err;
}

static void send_local_state(void)
{
    rover_imu_state_t imu = {0};
    rover_sensor_state_t sensors = {0};
    internet_adapter_status_t wifi = {0};
    navigation_status_t navigation = {};
    manual_control_status_t drive = {0};
    rover_service_get_imu(&imu);
    rover_service_get_sensors(&sensors);
    rover_service_get_wifi(&wifi);
    navigation_service_get_status(&navigation);
    manual_control_service_get_status(&drive);
    peer_packet_t packet = {.type = PEER_MESSAGE_STATE};
    peer_state_payload_t *state = &packet.payload.state;
    state->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    state->mode = (uint8_t)app_mode_get();
    state->flags = (imu.valid ? 1U : 0U) | (imu.calibration_valid ? 2U : 0U) |
                   (sensors.ultrasonic_valid ? 4U : 0U) |
                   (sensors.infrared_valid ? 8U : 0U) |
                   (sensors.color_valid ? 16U : 0U) |
                   (navigation.has_target ? 32U : 0U) |
                   (navigation.vision_heading_calibrated ? 64U : 0U) |
                   (tinyml_policy_ready() ? 128U : 0U);
    state->rssi = wifi.rssi;
    state->navigation_phase = (uint8_t)navigation.phase;
    state->temperature_c = imu.sample.temperature_c;
    memcpy(state->quaternion, imu.quaternion, sizeof(state->quaternion));
    state->distance_mm = sensors.distance_mm;
    state->infrared[0] = sensors.infrared.front_left;
    state->infrared[1] = sensors.infrared.front_right;
    state->infrared[2] = sensors.infrared.rear_left;
    state->infrared[3] = sensors.infrared.rear_right;
    state->color[0] = sensors.color.ambient;
    state->color[1] = sensors.color.red;
    state->color[2] = sensors.color.green;
    state->color[3] = sensors.color.blue;
    state->navigation_col = navigation.col;
    state->navigation_row = navigation.row;
    state->navigation_request_id = navigation.request_id;
    state->navigation_cell_col = navigation.confirmed_cell_col;
    state->navigation_cell_row = navigation.confirmed_cell_row;
    state->navigation_heading_index = navigation.heading_index;
    state->navigation_heading_deg = navigation.desired_heading_deg;
    state->vision_heading_offset_deg = navigation.vision_heading_offset_deg;
    state->navigation_waypoint_col = navigation.waypoint_col;
    state->navigation_waypoint_row = navigation.waypoint_row;
    state->navigation_segment_count = navigation.route_segment_count;
    state->navigation_segment_index = navigation.route_segment_index;
    state->navigation_blind_crossings = navigation.crossings_without_vision;
    state->navigation_replans = navigation.replan_count;
    state->navigation_wait_reason = (uint8_t)navigation.wait_reason;
    state->navigation_cancel_reason = (uint8_t)navigation.cancel_reason;
    state->navigation_grid_calibrated = navigation.grid_calibrated;
    state->navigation_grid_pattern = navigation.infrared_pattern;
    state->navigation_grid_calibrated_mask = navigation.infrared_calibrated_mask;
    state->navigation_motor_left = navigation.motor_left;
    state->navigation_motor_right = navigation.motor_right;
    state->navigation_error = navigation.error;
    state->navigation_failure_reason = (uint8_t)navigation.failure_reason;
    state->drive_left = drive.left;
    state->drive_right = drive.right;
    state->competition_delivered_mask = competition_runtime_delivered_mask();
    state->competition_available = competition_runtime_available();
    uint32_t frame_age = UINT32_MAX;
    state->vision_recent = recent_vision(&frame_age);
    state->vision_frame_age_ms = frame_age > UINT16_MAX ? UINT16_MAX : (uint16_t)frame_age;
    state->boot_id = diagnostics_boot_id();
    state->reset_reason = (uint8_t)diagnostics_reset_reason();
    competition_assignment_status_t assignment = {};
    competition_runtime_get_assignment_status(&assignment);
    tinyml_policy_status_t model = {};
    tinyml_policy_get_status(&model);
    state->assignment_id = assignment.id;
    state->assignment_color = assignment.color;
    state->assignment_result = assignment.result;
    state->assignment_phase = assignment.phase;
    state->model_version = model.version;
    state->model_crc32 = model.crc32;
    send_packet(&packet);
}

static void accept_state(const peer_packet_t *packet)
{
    const peer_state_payload_t *state = &packet->payload.state;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_status.boot_id != 0 && state->boot_id != 0 &&
        s_status.boot_id != state->boot_id) {
        s_forwarded_log_sequence = 0;
        s_pending_log_sequence = 0;
        s_next_log_retry_ms = 0;
    }
    s_status.rover_id = packet->source_id;
    s_status.timestamp_ms = state->timestamp_ms;
    s_status.mode = state->mode;
    s_status.imu_valid = (state->flags & 1U) != 0;
    s_status.imu_calibrated = (state->flags & 2U) != 0;
    s_status.ultrasonic_valid = (state->flags & 4U) != 0;
    s_status.infrared_valid = (state->flags & 8U) != 0;
    s_status.color_valid = (state->flags & 16U) != 0;
    s_status.navigation_has_target = (state->flags & 32U) != 0;
    s_status.temperature_c = state->temperature_c;
    memcpy(s_status.quaternion, state->quaternion, sizeof(s_status.quaternion));
    s_status.distance_mm = state->distance_mm;
    memcpy(s_status.infrared, state->infrared, sizeof(s_status.infrared));
    memcpy(s_status.color, state->color, sizeof(s_status.color));
    s_status.rssi = state->rssi;
    s_status.navigation_phase = state->navigation_phase;
    s_status.navigation_col = state->navigation_col;
    s_status.navigation_row = state->navigation_row;
    s_status.navigation_request_id = state->navigation_request_id;
    s_status.navigation_cell_col = state->navigation_cell_col;
    s_status.navigation_cell_row = state->navigation_cell_row;
    s_status.navigation_heading_index = state->navigation_heading_index;
    s_status.navigation_heading_deg = state->navigation_heading_deg;
    s_status.vision_heading_calibrated = (state->flags & 64U) != 0 &&
        isfinite(state->vision_heading_offset_deg) &&
        fabsf(state->vision_heading_offset_deg) <= 180.0f;
    s_status.vision_heading_offset_deg = s_status.vision_heading_calibrated
        ? state->vision_heading_offset_deg : 0.0f;
    s_status.navigation_waypoint_col = state->navigation_waypoint_col;
    s_status.navigation_waypoint_row = state->navigation_waypoint_row;
    s_status.navigation_segment_count = state->navigation_segment_count;
    s_status.navigation_segment_index = state->navigation_segment_index;
    s_status.navigation_blind_crossings = state->navigation_blind_crossings;
    s_status.navigation_replans = state->navigation_replans;
    s_status.navigation_wait_reason = state->navigation_wait_reason;
    s_status.navigation_cancel_reason = state->navigation_cancel_reason;
    s_status.navigation_grid_calibrated = state->navigation_grid_calibrated != 0;
    s_status.navigation_grid_pattern = state->navigation_grid_pattern;
    s_status.navigation_grid_calibrated_mask = state->navigation_grid_calibrated_mask;
    s_status.navigation_motor_left = state->navigation_motor_left;
    s_status.navigation_motor_right = state->navigation_motor_right;
    s_status.navigation_error = state->navigation_error;
    s_status.navigation_failure_reason = state->navigation_failure_reason;
    s_status.drive_left = state->drive_left;
    s_status.drive_right = state->drive_right;
    s_status.competition_delivered_mask = state->competition_delivered_mask;
    s_status.competition_available = state->competition_available != 0;
    s_status.vision_recent = state->vision_recent != 0;
    s_status.vision_frame_age_ms = state->vision_frame_age_ms;
    s_status.boot_id = state->boot_id;
    s_status.reset_reason = state->reset_reason;
    s_status.assignment_id = state->assignment_id;
    s_status.assignment_color = state->assignment_color;
    s_status.assignment_result = state->assignment_result;
    s_status.assignment_phase = state->assignment_phase;
    s_status.model_available = (state->flags & 128U) != 0;
    s_status.model_version = state->model_version;
    s_status.model_crc32 = state->model_crc32;
    s_last_seen_ms = esp_timer_get_time() / 1000;
    s_status.connected = true;
    s_status.last_error = ESP_OK;
    xSemaphoreGive(s_lock);
}

static void handle_received(const received_packet_t *received)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool expected = s_status.configured &&
        memcmp(received->source_mac, s_status.peer_mac, sizeof(s_status.peer_mac)) == 0;
    xSemaphoreGive(s_lock);
    if (!expected) return;
    if (received->packet.type == PEER_MESSAGE_LINK_REQUEST ||
        received->packet.type == PEER_MESSAGE_IDENTITY_REQUEST) {
        peer_packet_t reply = {0};
        reply.type = received->packet.type == PEER_MESSAGE_LINK_REQUEST
            ? PEER_MESSAGE_LINK_REPLY : PEER_MESSAGE_IDENTITY_REPLY;
        reply.payload.verification.nonce = received->packet.payload.verification.nonce;
        reply.payload.verification.identity = s_own_id;
        send_packet(&reply);
        return;
    }
    if (received->packet.type == PEER_MESSAGE_LINK_REPLY ||
        received->packet.type == PEER_MESSAGE_IDENTITY_REPLY) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (received->packet.type == PEER_MESSAGE_LINK_REPLY && s_link_nonce != 0 &&
            received->packet.payload.verification.nonce == s_link_nonce)
            s_status.link_verified = true;
        if (received->packet.type == PEER_MESSAGE_IDENTITY_REPLY && s_identity_nonce != 0 &&
            received->packet.payload.verification.nonce == s_identity_nonce) {
            s_status.identity_received = true;
            s_status.verified_identity = received->packet.payload.verification.identity;
        }
        xSemaphoreGive(s_lock);
        return;
    }
    if (!valid_identity(received->packet.source_id) ||
        received->packet.source_id == s_own_id) return;
    if (received->packet.type == PEER_MESSAGE_LOG) {
        diagnostics_store_peer(&received->packet.payload.log);
        peer_packet_t reply = {.type = PEER_MESSAGE_LOG_ACK};
        reply.payload.log_ack.boot_id = received->packet.payload.log.boot_id;
        reply.payload.log_ack.sequence = received->packet.payload.log.sequence;
        send_packet(&reply);
        return;
    }
    if (received->packet.type == PEER_MESSAGE_LOG_ACK) {
        if (received->packet.payload.log_ack.boot_id == s_pending_log_boot_id &&
            received->packet.payload.log_ack.sequence == s_pending_log_sequence) {
            s_forwarded_log_sequence = s_pending_log_sequence;
            s_pending_log_sequence = 0;
        }
        return;
    }
    if (received->packet.type == PEER_MESSAGE_COMPETITION_ACK) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.mode_ack_nonce = received->packet.payload.mode.nonce;
        s_status.mode_ack_accepted = received->packet.payload.mode.accepted != 0;
        s_status.mode_ack_reason = received->packet.payload.mode.reason;
        xSemaphoreGive(s_lock);
        return;
    }
    if (received->packet.type == PEER_MESSAGE_TARGET_ACK) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.target_ack_nonce = received->packet.payload.target_ack.nonce;
        s_status.target_ack_request_id = received->packet.payload.target_ack.request_id;
        s_status.target_ack_error = received->packet.payload.target_ack.error;
        xSemaphoreGive(s_lock);
        return;
    }
    if (received->packet.type == PEER_MESSAGE_COMPETITION_ENTER) {
        const uint32_t nonce = received->packet.payload.mode.nonce;
        uint8_t reason = 0;
        bool accepted = nonce != 0 && s_last_mode_nonce == nonce &&
            app_mode_get() == APP_MODE_COMPETITION;
        if (!accepted) {
            if (app_mode_get() != APP_MODE_TEST) reason = 1;
            else if (!recent_vision(NULL)) reason = 2;
            else if (app_mode_enter_competition() == ESP_OK) {
                accepted = true;
                s_last_mode_nonce = nonce;
            } else reason = 1;
        }
        peer_packet_t reply = {.type = PEER_MESSAGE_COMPETITION_ACK};
        reply.payload.mode.nonce = nonce;
        reply.payload.mode.accepted = accepted;
        reply.payload.mode.reason = reason;
        send_packet(&reply);
        return;
    }
    if (received->packet.type == PEER_MESSAGE_ASSIGNMENT_ACK) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.assignment_ack_id = received->packet.payload.assignment_ack.id;
        s_status.assignment_ack_accepted = received->packet.payload.assignment_ack.accepted != 0;
        xSemaphoreGive(s_lock);
        return;
    }
    if (received->packet.type == PEER_MESSAGE_ASSIGNMENT) {
        const peer_assignment_t assignment = {
            .id = received->packet.payload.assignment.id,
            .color = received->packet.payload.assignment.color,
            .retry = 0,
        };
        vision_status_t vision = {0};
        vision_service_get_status(&vision);
        bool accepted = app_mode_get() == APP_MODE_COMPETITION &&
            (vision.phase == VISION_PHASE_READY || vision.phase == VISION_PHASE_RUNNING) &&
            vision.connected && vision.protocol_valid && vision.received_ms != 0 &&
            (uint64_t)(esp_timer_get_time() / 1000) - vision.last_valid_frame_ms <= 750 &&
            assignment.id != 0 && assignment.color < VISION_MAX_CUBES;
        if (accepted) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (!s_assignment_complete || s_assignment.id != assignment.id)
                s_assignment = assignment;
            else if (s_assignment.color != assignment.color) accepted = false;
            s_assignment_complete = accepted;
            xSemaphoreGive(s_lock);
        }
        peer_packet_t reply = {.type = PEER_MESSAGE_ASSIGNMENT_ACK};
        reply.payload.assignment_ack.id = assignment.id;
        reply.payload.assignment_ack.accepted = accepted;
        send_packet(&reply);
        return;
    }
    switch (received->packet.type) {
        case PEER_MESSAGE_STATE:
            accept_state(&received->packet);
            break;
        case PEER_MESSAGE_DRIVE:
            manual_control_service_set(received->packet.payload.drive.left,
                                       received->packet.payload.drive.right);
            break;
        case PEER_MESSAGE_TARGET:
        {
            const uint32_t nonce = received->packet.payload.target.nonce;
            if (nonce != s_last_target_nonce || nonce == 0) {
                s_last_target_nonce = nonce;
                s_last_target_error = navigation_service_submit(
                    received->packet.payload.target.col,
                    received->packet.payload.target.row, &s_last_target_request_id);
                if (s_last_target_error != ESP_OK) s_last_target_request_id = 0;
            }
            peer_packet_t reply = {.type = PEER_MESSAGE_TARGET_ACK};
            reply.payload.target_ack.nonce = nonce;
            reply.payload.target_ack.request_id = s_last_target_request_id;
            reply.payload.target_ack.error = s_last_target_error;
            send_packet(&reply);
            break;
        }
        default:
            break;
    }
}

static esp_err_t initialize_esp_now(void)
{
    app_storage_config_t config = {0};
    esp_err_t err = app_storage_get_config(&config);
    if (err != ESP_OK || !config.peer_configured) {
        return err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
    }
    err = esp_now_init();
    if (err != ESP_OK) return err;
    err = esp_now_register_recv_cb(receive_callback);
    const esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    esp_now_peer_info_t configured_peer = peer;
    memcpy(configured_peer.peer_addr, config.peer_mac, sizeof(config.peer_mac));
    if (err == ESP_OK) err = esp_now_add_peer(&configured_peer);
    if (err != ESP_OK) {
        esp_now_deinit();
        return err;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_own_id = config.who_am_i;
    s_status.configured = true;
    s_status.initialized = true;
    memcpy(s_status.peer_mac, config.peer_mac, sizeof(config.peer_mac));
    s_status.last_error = ESP_OK;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "ESP-NOW listo para Rover %u", (unsigned)s_own_id);
    return ESP_OK;
}

static void peer_task(void *argument)
{
    (void)argument;
    int64_t next_state_ms = 0;
    int64_t next_init_ms = 0;
    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        const bool initialized = s_status.initialized;
        xSemaphoreGive(s_lock);
        if (!initialized && now_ms >= next_init_ms) {
            next_init_ms = now_ms + 1000;
            internet_adapter_status_t wifi = {0};
            if (rover_service_get_wifi(&wifi) == ESP_OK && wifi.connected) {
                const esp_err_t err = initialize_esp_now();
                if (err != ESP_OK) {
                    xSemaphoreTake(s_lock, portMAX_DELAY);
                    s_status.last_error = err;
                    xSemaphoreGive(s_lock);
                }
            }
        }
        received_packet_t received;
        while (xQueueReceive(s_received, &received, 0) == pdTRUE) handle_received(&received);
        if (initialized && now_ms >= next_state_ms) {
            send_local_state();
            next_state_ms = now_ms + PEER_STATE_PERIOD_MS;
        }
        if (initialized && now_ms >= s_next_log_retry_ms) {
            diagnostic_entry_t entry;
            if (diagnostics_local_next(s_forwarded_log_sequence, &entry)) {
                peer_packet_t log_packet = {.type = PEER_MESSAGE_LOG};
                log_packet.payload.log = entry;
                s_pending_log_sequence = entry.sequence;
                s_pending_log_boot_id = entry.boot_id;
                send_packet(&log_packet);
                s_next_log_retry_ms = now_ms + 100;
            }
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_status.connected && now_ms - s_last_seen_ms > PEER_TIMEOUT_MS) s_status.connected = false;
        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t peer_comms_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_received = xQueueCreate(8, sizeof(received_packet_t));
    if (s_lock == NULL || s_received == NULL) return ESP_ERR_NO_MEM;
    app_storage_config_t config = {0};
    if (app_storage_get_config(&config) == ESP_OK) {
        s_own_id = config.who_am_i;
        s_status.configured = config.peer_configured && valid_identity(config.who_am_i);
        if (s_status.configured) memcpy(s_status.peer_mac, config.peer_mac, sizeof(config.peer_mac));
    }
    return xTaskCreate(peer_task, "peer_espnow", 4096, NULL, 5, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

void peer_comms_service_get_status(peer_comms_status_t *status)
{
    if (status == NULL) return;
    memset(status, 0, sizeof(*status));
    if (s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *status = s_status;
    const int64_t age = esp_timer_get_time() / 1000 - s_last_seen_ms;
    status->age_ms = s_last_seen_ms == 0 ? UINT32_MAX : (uint32_t)(age > UINT32_MAX ? UINT32_MAX : age);
    status->connected = status->connected && status->age_ms <= PEER_TIMEOUT_MS;
    xSemaphoreGive(s_lock);
}

esp_err_t peer_comms_service_send_drive(int16_t left, int16_t right)
{
    peer_comms_status_t status;
    peer_comms_service_get_status(&status);
    if (!status.connected) return ESP_ERR_INVALID_STATE;
    if (status.mode != APP_MODE_TEST) return ESP_ERR_INVALID_STATE;
    peer_packet_t packet = {.type = PEER_MESSAGE_DRIVE};
    packet.payload.drive.left = left;
    packet.payload.drive.right = right;
    return send_packet(&packet);
}

esp_err_t peer_comms_service_send_target(float col, float row, uint32_t *request_id)
{
    peer_comms_status_t status;
    peer_comms_service_get_status(&status);
    if (!status.connected) return ESP_ERR_INVALID_STATE;
    if (status.mode != APP_MODE_TEST) return ESP_ERR_INVALID_STATE;
    if (!isfinite(col) || !isfinite(row) || col < 0.0f || row < 0.0f) return ESP_ERR_INVALID_ARG;
    peer_packet_t packet = {.type = PEER_MESSAGE_TARGET};
    packet.payload.target.col = col;
    packet.payload.target.row = row;
    packet.payload.target.nonce = (uint32_t)esp_timer_get_time() | 1U;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.target_ack_nonce = 0;
    xSemaphoreGive(s_lock);
    for (int attempt = 0; attempt < 4; ++attempt) {
        const esp_err_t sent = send_packet(&packet);
        if (sent != ESP_OK) return sent;
        for (int wait = 0; wait < 10; ++wait) {
            vTaskDelay(pdMS_TO_TICKS(20));
            peer_comms_service_get_status(&status);
            if (status.target_ack_nonce == packet.payload.target.nonce) {
                if (request_id != NULL) *request_id = status.target_ack_request_id;
                return status.target_ack_error;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t peer_comms_service_request_competition(uint8_t *reason)
{
    peer_comms_status_t peer = {0};
    peer_comms_service_get_status(&peer);
    if (!peer.connected || peer.mode != APP_MODE_TEST) return ESP_ERR_INVALID_STATE;
    const uint32_t nonce = (uint32_t)esp_timer_get_time() | 1U;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.mode_ack_nonce = 0;
    s_status.mode_ack_accepted = false;
    s_status.mode_ack_reason = 0;
    xSemaphoreGive(s_lock);
    peer_packet_t request = {.type = PEER_MESSAGE_COMPETITION_ENTER};
    request.payload.mode.nonce = nonce;
    for (int attempt = 0; attempt < 6; ++attempt) {
        const esp_err_t sent = send_packet(&request);
        if (sent != ESP_OK) return sent;
        for (int wait = 0; wait < 10; ++wait) {
            vTaskDelay(pdMS_TO_TICKS(20));
            peer_comms_service_get_status(&peer);
            if (peer.mode_ack_nonce == nonce) {
                if (reason != NULL) *reason = peer.mode_ack_reason;
                return peer.mode_ack_accepted ? ESP_OK : ESP_ERR_INVALID_STATE;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t peer_comms_service_send_assignment(const peer_assignment_t *assignment)
{
    if (assignment == NULL || assignment->id == 0 ||
        assignment->color >= VISION_MAX_CUBES || assignment->retry > 1 ||
        app_mode_get() != APP_MODE_COMPETITION) return ESP_ERR_INVALID_ARG;
    peer_packet_t packet = {.type = PEER_MESSAGE_ASSIGNMENT};
    packet.payload.assignment.id = assignment->id;
    packet.payload.assignment.color = assignment->color;
    return send_packet(&packet);
}

bool peer_comms_service_get_assignment(peer_assignment_t *assignment)
{
    if (assignment == NULL || s_lock == NULL) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool complete = s_assignment_complete;
    if (complete) *assignment = s_assignment;
    xSemaphoreGive(s_lock);
    return complete;
}

void peer_comms_service_reset_verification(void)
{
    if (s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.link_verified = false;
    s_status.identity_received = false;
    s_status.verified_identity = 0;
    s_link_nonce = 0;
    s_identity_nonce = 0;
    memset(&s_assignment, 0, sizeof(s_assignment));
    s_assignment_complete = false;
    s_status.assignment_ack_id = 0;
    s_status.assignment_ack_accepted = false;
    s_status.vision_heading_offset_deg = 0.0f;
    s_status.vision_heading_calibrated = false;
    xSemaphoreGive(s_lock);
}

static esp_err_t send_verification(uint8_t type, uint32_t *nonce)
{
    if (s_lock == NULL) return ESP_ERR_INVALID_STATE;
    peer_packet_t packet = {.type = type};
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (*nonce == 0) {
        *nonce = (uint32_t)esp_timer_get_time() ^ ++s_sequence;
        if (*nonce == 0) *nonce = 1;
    }
    packet.payload.verification.nonce = *nonce;
    xSemaphoreGive(s_lock);
    return send_packet(&packet);
}

esp_err_t peer_comms_service_probe_link(void)
{
    return send_verification(PEER_MESSAGE_LINK_REQUEST, &s_link_nonce);
}

esp_err_t peer_comms_service_query_identity(void)
{
    return send_verification(PEER_MESSAGE_IDENTITY_REQUEST, &s_identity_nonce);
}

