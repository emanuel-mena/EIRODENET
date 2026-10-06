#include "competition_runtime.hpp"
#include "competition_strategy.hpp"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>

#include "app_storage.hpp"
#include "app_mode.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "motion_control.hpp"
#include "motor_adapter.hpp"
#include "navigation_geometry.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"

#define FRESH_MS 750U
#define CUBE_APPROACH_CELLS 8.3f
#define CUBE_DETOUR_LIMIT_CELLS COMPETITION_DETOUR_MIN_DISTANCE_CELLS
#define STALL_DISTANCE_CELLS 1.0f
#define STALL_TIME_MS 3000U
#define STALL_PAUSE_MS 5000U
#define PEER_BLOCK_PERSIST_MS 1000U

typedef enum {
    EXEC_WAIT = 0,
    EXEC_TO_CUBE = 1,
    EXEC_DETOUR_TURN = 2,
    EXEC_DETOUR_ADVANCE = 3,
    EXEC_ALIGN_DEPOT = 4,
    EXEC_TO_DEPOT = 5,
    EXEC_PUSH = 6,
    EXEC_DONE = 7,
    EXEC_HOLD = 8,
    EXEC_STALL_PAUSE = 9,
} exec_phase_t;

static const char *TAG = "competition_run";
static peer_mission_t s_local;
static peer_mission_t s_remote;
static exec_phase_t s_phase = EXEC_WAIT;
static uint8_t s_delivered_mask;
static uint8_t s_assigned_mask;
static uint8_t s_reserved_color = UINT8_MAX;
static uint8_t s_received_mission_color = UINT8_MAX;
static uint32_t s_received_mission_id;
static uint32_t s_next_id;
static uint32_t s_generation;
static uint32_t s_last_delivery_event_id;
static uint32_t s_delivery_mission_id;
static uint8_t s_delivery_color;
static uint64_t s_delivery_started_ms;
static uint64_t s_delivery_last_send_ms;
static uint64_t s_remote_sent_ms;
static uint8_t s_remote_fragment;
static bool s_remote_pending;
static bool s_remote_ready;
static bool s_reserve_assigned;
static bool s_have_detour_target;
static float s_detour_col;
static float s_detour_row;
static float s_detour_heading;
static uint64_t s_stall_since_ms;
static float s_stall_col;
static float s_stall_row;
static bool s_stall_anchor_valid;
static uint64_t s_hold_ms;
static uint64_t s_peer_block_since_ms;

static uint64_t now_ms(void) { return (uint64_t)(esp_timer_get_time() / 1000); }
static float distance(float ax, float ay, float bx, float by)
{
    return hypotf(ax - bx, ay - by);
}
static float wrap(float angle) { return motion_wrap_degrees(angle); }
static float heading_error(float desired, float current) { return wrap(desired - current); }
static float clamp_float(float value, float low, float high)
{
    return fmaxf(low, fminf(high, value));
}

static bool fresh(const vision_status_t *v)
{
    return v != NULL && v->connected && v->protocol_valid && v->pose_valid &&
           v->last_valid_frame_ms != 0 && now_ms() - v->last_valid_frame_ms <= FRESH_MS;
}

static bool cube_ready(const vision_status_t *v, uint8_t color)
{
    return v != NULL && color < VISION_MAX_CUBES && v->cube_valid[color] &&
           v->depot_valid[color] && v->cubes[color].age_ms <= FRESH_MS;
}

static bool delivered(const vision_status_t *v, uint8_t color)
{
    return cube_ready(v, color) && v->cube_in_depot[color];
}

static float current_heading(const vision_status_t *v)
{
    navigation_status_t navigation = {};
    navigation_service_get_status(&navigation);
    return navigation.pose_valid ? navigation.theta_deg : v->theta_deg;
}

typedef enum {
    PEER_BLOCK_NONE = 0,
    PEER_BLOCK_STOP,
    PEER_BLOCK_REVERSE,
} peer_block_action_t;

static peer_block_action_t peer_block_action(const vision_status_t *v, float step,
                                             competition_role_t role)
{
    peer_comms_status_t peer = {};
    peer_comms_service_get_status(&peer);
    if (!v->peer_valid || !peer.connected) {
        s_peer_block_since_ms = 0;
        return PEER_BLOCK_NONE;
    }
    const float radians = current_heading(v) * 0.01745329252f;
    const bool overlap = navigation_rovers_overlap(
        v->col + step * cosf(radians), v->row - step * sinf(radians),
        current_heading(v), v->peer_col, v->peer_row, v->peer_theta_deg);
    if (!overlap) {
        s_peer_block_since_ms = 0;
        return PEER_BLOCK_NONE;
    }

    if (role != COMPETITION_ROLE_SOLDIER) {
        // The commander yields the passage decision to the soldier.
        s_peer_block_since_ms = 0;
        return PEER_BLOCK_STOP;
    }

    // Let the soldier keep its assigned movement briefly while the commander
    // stops. If the overlap persists, the soldier yields by reversing at the
    // configured half-power command (700/1000 is the minimum usable PWM).
    const uint64_t now = now_ms();
    if (s_peer_block_since_ms == 0) s_peer_block_since_ms = now;
    return now - s_peer_block_since_ms >= PEER_BLOCK_PERSIST_MS
        ? PEER_BLOCK_REVERSE : PEER_BLOCK_NONE;
}

static void stop(void)
{
    motor_adapter_stop();
    navigation_service_cancel(NAVIGATION_CANCEL_MODE);
}

static uint8_t own_identity(void)
{
    app_storage_config_t config = {};
    return app_storage_get_config(&config) == ESP_OK ? config.who_am_i : 0;
}

static peer_mission_t mission_for(const vision_status_t *v, uint8_t color)
{
    peer_mission_t mission = {};
    mission.id = ++s_next_id;
    if (mission.id == 0) mission.id = ++s_next_id;
    mission.color = color;
    mission.point_count = 1;
    mission.points[0] = {v->cubes[color].col, v->cubes[color].row};
    return mission;
}

static void begin_local_mission(const peer_mission_t *mission)
{
    if (mission == NULL || mission->id == 0) return;
    s_local = *mission;
    s_assigned_mask |= (uint8_t)(1U << mission->color);
    s_phase = EXEC_WAIT;
    s_stall_anchor_valid = false;
    s_stall_since_ms = 0;
    s_have_detour_target = false;
}

static void begin_remote_mission(const peer_mission_t *mission)
{
    if (mission == NULL || mission->id == 0) return;
    s_remote = *mission;
    s_remote_pending = true;
    s_remote_ready = false;
    s_remote_fragment = 0;
    s_remote_sent_ms = 0;
    s_assigned_mask |= (uint8_t)(1U << mission->color);
}

static void assign_initial(const vision_status_t *v)
{
    if (s_local.id || s_remote.id || !fresh(v)) return;
    uint8_t available[VISION_MAX_CUBES];
    uint8_t count = 0;
    for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c)
        if (cube_ready(v, c) && !delivered(v, c)) available[count++] = c;
    if (count == 0) return;

    uint8_t nearest = available[0], farthest = available[0];
    float nearest_distance = distance(v->col, v->row, v->cubes[nearest].col, v->cubes[nearest].row);
    float farthest_distance = nearest_distance;
    for (uint8_t i = 1; i < count; ++i) {
        const uint8_t color = available[i];
        const float candidate = distance(v->col, v->row, v->cubes[color].col, v->cubes[color].row);
        if (candidate < nearest_distance ||
            (candidate == nearest_distance && color < nearest)) {
            nearest = color;
            nearest_distance = candidate;
        }
        if (candidate > farthest_distance ||
            (candidate == farthest_distance && color < farthest)) {
            farthest = color;
            farthest_distance = candidate;
        }
    }

    const peer_mission_t nearest_mission = mission_for(v, nearest);
    begin_local_mission(&nearest_mission);
    if (count >= 2 && farthest != nearest) {
        const peer_mission_t farthest_mission = mission_for(v, farthest);
        begin_remote_mission(&farthest_mission);
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (available[i] != nearest && available[i] != farthest) {
            s_reserved_color = available[i];
            break;
        }
    }
    ESP_LOGI(TAG, "Asignacion directa: comandante cubo=%u soldado cubo=%u reservado=%u",
             nearest, s_remote.id ? s_remote.color : UINT8_MAX, s_reserved_color);
}

static void send_remote(void)
{
    if (!s_remote_pending || s_remote_ready) return;
    peer_comms_status_t peer = {};
    peer_comms_service_get_status(&peer);
    if (!peer.connected || peer.mode != APP_MODE_COMPETITION) return;
    const uint8_t fragments = (s_remote.point_count + PEER_MISSION_FRAGMENT_POINTS - 1) /
                              PEER_MISSION_FRAGMENT_POINTS;
    if (peer.mission_ack_id == s_remote.id &&
        peer.mission_ack_fragment == s_remote_fragment && peer.mission_ack_accepted) {
        ++s_remote_fragment;
        s_remote_sent_ms = 0;
        if (s_remote_fragment >= fragments) {
            s_remote_ready = true;
            s_remote_pending = false;
            ESP_LOGI(TAG, "Mision %lu confirmada por soldado", (unsigned long)s_remote.id);
            return;
        }
    }
    if (s_remote_sent_ms == 0 || now_ms() - s_remote_sent_ms >= 250) {
        if (peer_comms_service_send_mission_fragment(&s_remote, s_remote_fragment) == ESP_OK)
            s_remote_sent_ms = now_ms();
    }
}

static bool turn_to(const vision_status_t *v, float target, int max_pwm)
{
    rover_imu_state_t imu = {};
    rover_service_get_imu(&imu);
    const float error = heading_error(target, current_heading(v));
    bool settled = false;
    const int pwm = motion_turn_pwm(error,
                                    imu.valid ? imu.sample.gyro_dps[2] : 0.0f,
                                    (uint32_t)(now_ms() / 10U), &settled);
    if (settled) {
        motor_adapter_stop();
        return true;
    }
    if (pwm == 0) {
        motor_adapter_stop();
        return false;
    }
    const int magnitude = std::min(abs(pwm), max_pwm);
    motor_adapter_set(error > 0 ? -magnitude : magnitude,
                      error > 0 ? magnitude : -magnitude);
    return false;
}

static void direct_drive(const vision_status_t *v, float target_col, float target_row,
                         int base_pwm)
{
    rover_imu_state_t imu = {};
    rover_service_get_imu(&imu);
    competition_status_t competition = {};
    competition_service_get_status(&competition);
    const float desired = atan2f(-(target_row - v->row), target_col - v->col) * 57.2957795f;
    const float error = heading_error(desired, current_heading(v));
    if (fabsf(error) > 28.0f) {
        turn_to(v, desired, 1000);
        return;
    }
    const float trim = -competition.motor_trim_pwm;
    const float correction = clamp_float(8.0f * error -
        (imu.valid ? 1.5f * imu.sample.gyro_dps[2] : 0.0f) + trim,
        -300.0f, 300.0f);
    int left = base_pwm;
    int right = base_pwm;
    const int amount = (int)lroundf(fabsf(correction));
    if (base_pwm <= 700) {
        // 700 is the minimum usable drive. Correct heading by adding power to
        // one side instead of dropping the other side below that floor.
        if (correction >= 0) right = 700 + amount;
        else left = 700 + amount;
    } else if (correction >= 0) {
        left -= amount;
    } else {
        right -= amount;
    }
    motor_adapter_set((int16_t)left, (int16_t)right);
}

static bool obstacle_requires_detour(const vision_status_t *v, uint8_t color,
                                     const rover_sensor_state_t *sensors)
{
    return sensors->ultrasonic_valid && sensors->distance_mm <= COMPETITION_OBSTACLE_MM &&
           distance(v->col, v->row, v->cubes[color].col, v->cubes[color].row) >
               CUBE_DETOUR_LIMIT_CELLS;
}

static void start_detour(const vision_status_t *v)
{
    const float heading = current_heading(v) - 60.0f;
    const float radians = heading * 0.01745329252f;
    s_detour_heading = wrap(heading);
    s_detour_col = v->col + COMPETITION_DETOUR_CELLS * cosf(radians);
    s_detour_row = v->row - COMPETITION_DETOUR_CELLS * sinf(radians);
    s_have_detour_target = navigation_pose_inside_mm(s_detour_col, s_detour_row,
                                                      s_detour_heading,
                                                      v->grid_cols, v->grid_rows,
                                                      v->cell_mm);
    if (s_have_detour_target) s_phase = EXEC_DETOUR_TURN;
    else s_phase = EXEC_HOLD;
    motor_adapter_stop();
}

static void update_stall(const vision_status_t *v)
{
    if (s_stall_since_ms == 0 || !s_stall_anchor_valid) {
        s_stall_since_ms = now_ms();
        s_stall_col = v->col;
        s_stall_row = v->row;
        s_stall_anchor_valid = true;
        return;
    }
    if (distance(v->col, v->row, s_stall_col, s_stall_row) > STALL_DISTANCE_CELLS) {
        s_stall_since_ms = now_ms();
        s_stall_col = v->col;
        s_stall_row = v->row;
        return;
    }
    if (now_ms() - s_stall_since_ms >= STALL_TIME_MS) {
        stop();
        s_hold_ms = now_ms();
        s_phase = EXEC_STALL_PAUSE;
        s_stall_since_ms = 0;
        ESP_LOGW(TAG, "Soldado detenido por falta de progreso durante 5 segundos");
    }
}

static void complete_delivery(const vision_status_t *v, competition_role_t role)
{
    const uint8_t color = s_local.color;
    stop();
    if (delivered(v, color)) s_delivered_mask |= (uint8_t)(1U << color);
    s_delivery_mission_id = s_local.id;
    s_delivery_color = color;
    s_delivery_started_ms = now_ms();
    s_delivery_last_send_ms = 0;
    peer_comms_service_send_delivery(s_local.id, color);
    ESP_LOGI(TAG, "Entrega confirmada: cubo=%u mision=%lu", color,
             (unsigned long)s_local.id);
    s_phase = EXEC_DONE;
    if (role == COMPETITION_ROLE_COMMANDER &&
        s_reserved_color < VISION_MAX_CUBES && !s_reserve_assigned &&
        s_delivered_mask != 0) {
        const peer_mission_t reserve = mission_for(v, s_reserved_color);
        s_reserve_assigned = true;
        if (own_identity() == APP_STORAGE_ROVER_10 || own_identity() == APP_STORAGE_ROVER_11) {
            // The commander is the only rover allowed to choose the winner;
            // its own delivery makes it the local winner.
            begin_local_mission(&reserve);
        }
    }
}

static void execute_local(const vision_status_t *v, competition_role_t role)
{
    if (!s_local.id || s_phase == EXEC_DONE) return;
    if (s_phase == EXEC_WAIT) {
        if (v->phase != VISION_PHASE_RUNNING) return;
        s_phase = EXEC_TO_CUBE;
    }
    const uint8_t color = s_local.color;
    if (!cube_ready(v, color)) {
        stop();
        s_phase = EXEC_HOLD;
        s_hold_ms = now_ms();
        return;
    }
    if (s_phase == EXEC_HOLD) {
        stop();
        if (now_ms() - s_hold_ms >= 1000) s_phase = EXEC_TO_CUBE;
        return;
    }
    if (s_phase == EXEC_TO_CUBE) {
        rover_sensor_state_t sensors = {};
        rover_service_get_sensors(&sensors);
        if (obstacle_requires_detour(v, color, &sensors)) {
            start_detour(v);
            return;
        }
        const peer_block_action_t peer_action = peer_block_action(v, 0.8f, role);
        if (peer_action == PEER_BLOCK_STOP) {
            stop();
            return;
        }
        if (peer_action == PEER_BLOCK_REVERSE) {
            motor_adapter_set(-COMPETITION_HALF_PWM, -COMPETITION_HALF_PWM);
            return;
        }
        if (distance(v->col, v->row, v->cubes[color].col, v->cubes[color].row) <=
            CUBE_APPROACH_CELLS) {
            motor_adapter_stop();
            s_phase = EXEC_ALIGN_DEPOT;
            s_stall_anchor_valid = false;
            return;
        }
        direct_drive(v, v->cubes[color].col, v->cubes[color].row, 1000);
        return;
    }
    if (s_phase == EXEC_DETOUR_TURN) {
        if (turn_to(v, s_detour_heading, 1000)) s_phase = EXEC_DETOUR_ADVANCE;
        return;
    }
    if (s_phase == EXEC_DETOUR_ADVANCE) {
        if (!s_have_detour_target) {
            s_phase = EXEC_HOLD;
            s_hold_ms = now_ms();
            return;
        }
        if (distance(v->col, v->row, s_detour_col, s_detour_row) <= 0.8f) {
            motor_adapter_stop();
            s_have_detour_target = false;
            s_phase = EXEC_TO_CUBE;
            return;
        }
        const peer_block_action_t peer_action = peer_block_action(v, 0.8f, role);
        if (peer_action == PEER_BLOCK_STOP) {
            stop();
            return;
        }
        if (peer_action == PEER_BLOCK_REVERSE) {
            motor_adapter_set(-COMPETITION_HALF_PWM, -COMPETITION_HALF_PWM);
            return;
        }
        direct_drive(v, s_detour_col, s_detour_row, 1000);
        return;
    }
    if (s_phase == EXEC_ALIGN_DEPOT) {
        const float target = atan2f(-(v->depot_row[color] - v->row),
                                    v->depot_col[color] - v->col) * 57.2957795f;
        if (turn_to(v, target, 700)) {
            s_phase = EXEC_TO_DEPOT;
            s_stall_since_ms = 0;
            s_stall_anchor_valid = false;
        }
        return;
    }
    if (s_phase == EXEC_TO_DEPOT) {
        if (distance(v->col, v->row, v->depot_col[color], v->depot_row[color]) <=
            COMPETITION_DEPOT_STOP_CELLS) {
            motor_adapter_stop();
            s_phase = EXEC_PUSH;
            return;
        }
        if (role == COMPETITION_ROLE_SOLDIER) update_stall(v);
        if (s_phase == EXEC_STALL_PAUSE) return;
        const peer_block_action_t peer_action = peer_block_action(v, 0.8f, role);
        if (peer_action == PEER_BLOCK_STOP) {
            stop();
            return;
        }
        if (peer_action == PEER_BLOCK_REVERSE) {
            motor_adapter_set(-COMPETITION_HALF_PWM, -COMPETITION_HALF_PWM);
            return;
        }
        direct_drive(v, v->depot_col[color], v->depot_row[color], 1000);
        return;
    }
    if (s_phase == EXEC_STALL_PAUSE) {
        stop();
        if (now_ms() - s_hold_ms >= STALL_PAUSE_MS) {
            s_stall_anchor_valid = false;
            s_stall_since_ms = 0;
            s_phase = EXEC_TO_DEPOT;
        }
        return;
    }
    if (s_phase == EXEC_PUSH) {
        if (delivered(v, color)) {
            complete_delivery(v, role);
            return;
        }
        if (role == COMPETITION_ROLE_SOLDIER) update_stall(v);
        if (s_phase == EXEC_STALL_PAUSE) return;
        direct_drive(v, v->depot_col[color], v->depot_row[color], COMPETITION_HALF_PWM);
    }
}

void competition_runtime_reset(void)
{
    stop();
    memset(&s_local, 0, sizeof(s_local));
    memset(&s_remote, 0, sizeof(s_remote));
    s_phase = EXEC_WAIT;
    s_delivered_mask = 0;
    s_assigned_mask = 0;
    s_reserved_color = UINT8_MAX;
    s_received_mission_color = UINT8_MAX;
    s_received_mission_id = 0;
    s_remote_pending = false;
    s_remote_ready = false;
    s_reserve_assigned = false;
    s_remote_sent_ms = 0;
    s_remote_fragment = 0;
    s_have_detour_target = false;
    s_stall_since_ms = 0;
    s_stall_anchor_valid = false;
    s_hold_ms = 0;
    s_peer_block_since_ms = 0;
    s_last_delivery_event_id = 0;
    s_delivery_mission_id = 0;
    s_delivery_color = UINT8_MAX;
    s_delivery_started_ms = 0;
    s_delivery_last_send_ms = 0;
}

uint8_t competition_runtime_delivered_mask(void) { return s_delivered_mask; }
bool competition_runtime_available(void)
{
    return s_phase == EXEC_DONE || (s_phase == EXEC_WAIT && !s_local.id);
}

void competition_runtime_get_status(competition_runtime_status_t *status)
{
    if (status == NULL) return;
    memset(status, 0, sizeof(*status));
    status->phase = (uint8_t)s_phase;
    status->target_color = s_local.id ? s_local.color : UINT8_MAX;
    status->detour_active = s_have_detour_target;
    status->detour_heading_deg = s_detour_heading;
    status->soldier_paused = s_phase == EXEC_STALL_PAUSE;
    status->delivered_mask = s_delivered_mask;
    if (!s_local.id) return;
    vision_status_t v = {};
    vision_service_get_status(&v);
    if (s_local.color >= VISION_MAX_CUBES) return;
    if (s_phase == EXEC_ALIGN_DEPOT || s_phase == EXEC_TO_DEPOT ||
        s_phase == EXEC_PUSH) {
        status->target_col = v.depot_col[s_local.color];
        status->target_row = v.depot_row[s_local.color];
    } else if (s_have_detour_target) {
        status->target_col = s_detour_col;
        status->target_row = s_detour_row;
    } else {
        status->target_col = v.cubes[s_local.color].col;
        status->target_row = v.cubes[s_local.color].row;
    }
    if (v.pose_valid)
        status->distance_remaining_cells = distance(v.col, v.row,
                                                    status->target_col,
                                                    status->target_row);
}

void competition_runtime_tick(competition_role_t role, uint32_t generation)
{
    if (generation != s_generation) {
        competition_runtime_reset();
        s_generation = generation;
        s_next_id = generation << 16;
    }
    vision_status_t v = {};
    vision_service_get_status(&v);
    peer_comms_status_t peer = {};
    peer_comms_service_get_status(&peer);
    if (app_mode_get() != APP_MODE_COMPETITION ||
        (v.phase != VISION_PHASE_READY && v.phase != VISION_PHASE_RUNNING) ||
        !fresh(&v) || !peer.connected || peer.mode != APP_MODE_COMPETITION) {
        if (s_phase != EXEC_WAIT && s_phase != EXEC_DONE) stop();
        return;
    }

    // Delivery notifications are idempotent and repeated briefly to survive
    // an ESP-NOW loss while the commander arbitrates the reserve cube.
    if (s_delivery_mission_id != 0 && now_ms() - s_delivery_started_ms <= 5000 &&
        (s_delivery_last_send_ms == 0 || now_ms() - s_delivery_last_send_ms >= 250)) {
        peer_comms_service_send_delivery(s_delivery_mission_id, s_delivery_color);
        s_delivery_last_send_ms = now_ms();
    }

    if (role == COMPETITION_ROLE_COMMANDER) {
        if (!s_assigned_mask) assign_initial(&v);
        send_remote();
        peer_delivery_event_t event = {};
        if (peer_comms_service_get_delivery_event(&event) &&
            event.valid && event.mission_id != s_last_delivery_event_id &&
            event.rover_id != own_identity() && s_reserved_color < VISION_MAX_CUBES &&
            !s_reserve_assigned) {
            if (event.mission_id == s_remote.id || event.color == s_remote.color) {
                s_last_delivery_event_id = event.mission_id;
                const peer_mission_t reserve = mission_for(&v, s_reserved_color);
                begin_remote_mission(&reserve);
                s_reserve_assigned = true;
            }
        }
    } else {
        peer_mission_t received = {};
        if (peer_comms_service_get_mission(&received) &&
            received.id != s_received_mission_id &&
            (s_phase == EXEC_WAIT || s_phase == EXEC_DONE)) {
            s_received_mission_id = received.id;
            s_received_mission_color = received.color;
            begin_local_mission(&received);
        }
    }

    execute_local(&v, role);
}
