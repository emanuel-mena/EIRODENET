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
    EXEC_DEPOT_WAIT_VISION = 10,
    EXEC_REVERSE_AFTER_DELIVERY = 11,
    EXEC_STAGE_CUBE = 12,
    EXEC_REPOSITION = 13,
    EXEC_ALIGN_REAR_TO_CENTER = 14,
    EXEC_REVERSE_TO_CENTER = 15,
    EXEC_PUSH_WAIT_VISION = 16,
    EXEC_VERIFY_DEPOT_ALIGNMENT = 17,
    EXEC_DEPOT_DETOUR_TURN = 18,
    EXEC_DEPOT_DETOUR_ADVANCE = 19,
    EXEC_DEPOT_BLOCKED_WAIT = 20,
    EXEC_DEPOT_ESCAPE_TURN = 21,
    EXEC_DEPOT_ESCAPE_ADVANCE = 22,
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
static bool s_cube_staged;
static float s_detour_col;
static float s_detour_row;
static float s_detour_heading;
static uint64_t s_stall_since_ms;
static float s_stall_col;
static float s_stall_row;
static bool s_stall_anchor_valid;
static uint64_t s_hold_ms;
static uint64_t s_peer_block_since_ms;
static uint32_t s_depot_frame_sequence;
static uint32_t s_depot_turn_frame_sequence;
static uint32_t s_push_step_frame_sequence;
static float s_push_step_anchor_col;
static float s_push_step_anchor_row;
static bool s_push_step_active;
static uint64_t s_cube_clearance_timestamp_ms;
static uint64_t s_cube_near_since_ms;
static uint64_t s_reverse_started_ms;
static uint8_t s_pending_reserve_color = UINT8_MAX;
static uint64_t s_commander_delay_started_ms;
static uint64_t s_pose_frame_timestamp_ms;
static uint64_t s_pose_capture_ms;
static bool s_cube_anchor_valid;
static float s_cube_anchor_col;
static float s_cube_anchor_row;
static uint64_t s_cube_anchor_frame_ms;
static int8_t s_reposition_direction;
static uint64_t s_reposition_last_drive_ms;
static uint32_t s_reposition_driven_ms;
static uint64_t s_center_reverse_last_drive_ms;
static uint32_t s_center_reverse_driven_ms;
static bool s_first_delivery_retreat;
static bool s_cube_straight_active;
static bool s_cube_held;
static float s_cube_straight_start_col;
static float s_cube_straight_start_row;
static float s_cube_straight_heading;

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

static vision_status_t controlled_pose(const vision_status_t *vision)
{
    vision_status_t pose = *vision;
    if (vision->frame_timestamp_ms != s_pose_frame_timestamp_ms) {
        s_pose_frame_timestamp_ms = vision->frame_timestamp_ms;
        s_pose_capture_ms = now_ms();
    }
    navigation_status_t navigation = {};
    navigation_service_get_status(&navigation);
    const float displacement = distance(vision->col, vision->row,
                                        navigation.pose_col, navigation.pose_row);
    const float capture_age_s = s_pose_capture_ms && now_ms() >= s_pose_capture_ms
        ? fminf(0.75f, (now_ms() - s_pose_capture_ms) / 1000.0f) : 0.0f;
    const float plausible = 1.5f + 12.0f * capture_age_s;
    if (navigation.pose_valid && isfinite(navigation.pose_col) &&
        isfinite(navigation.pose_row) && isfinite(navigation.theta_deg) &&
        isfinite(navigation.linear_speed_cells_s) &&
        isfinite(navigation.uncertainty_cells) &&
        navigation.uncertainty_cells <= 2.0f && displacement <= plausible) {
        pose.col = navigation.pose_col;
        pose.row = navigation.pose_row;
        pose.theta_deg = navigation.theta_deg;
    }
    return pose;
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

    // Both rovers stop when the predicted envelopes overlap. Reversing while
    // the peer is still moving is not a safe yield: the delayed peer pose can
    // make both vehicles drive into the same corridor and contact each other.
    if (s_peer_block_since_ms == 0) s_peer_block_since_ms = now_ms();
    return PEER_BLOCK_STOP;
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
    s_cube_staged = false;
    s_cube_anchor_valid = false;
    s_cube_anchor_frame_ms = 0;
    s_reposition_direction = 0;
    s_reposition_last_drive_ms = 0;
    s_reposition_driven_ms = 0;
    s_center_reverse_last_drive_ms = 0;
    s_center_reverse_driven_ms = 0;
    s_depot_frame_sequence = 0;
    s_depot_turn_frame_sequence = 0;
    s_push_step_frame_sequence = 0;
    s_push_step_anchor_col = 0.0f;
    s_push_step_anchor_row = 0.0f;
    s_push_step_active = false;
    s_cube_clearance_timestamp_ms = 0;
    s_cube_near_since_ms = 0;
    s_commander_delay_started_ms = 0;
    s_pose_frame_timestamp_ms = 0;
    s_pose_capture_ms = 0;
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


    uint8_t soldier_color = farthest;
    if (count >= 3 && v->peer_valid && v->peer_age_ms <= FRESH_MS) {
        bool path_blocked = false;
        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t color = available[i];
            if (color == farthest) continue;
            if (competition_assignment_line_hits_cube(
                    v->peer_col, v->peer_row,
                    v->cubes[farthest].col, v->cubes[farthest].row,
                    v->cubes[color].col, v->cubes[color].row)) {
                path_blocked = true;
                break;
            }
        }
        if (path_blocked) {
            for (uint8_t i = 0; i < count; ++i) {
                if (available[i] != nearest && available[i] != farthest) {
                    soldier_color = available[i];
                    break;
                }
            }
            ESP_LOGI(TAG, "Ruta del soldado bloqueada: cubo=%u reemplazado por cubo=%u",
                     farthest, soldier_color);
        }
    }

    const peer_mission_t nearest_mission = mission_for(v, nearest);
    begin_local_mission(&nearest_mission);
    if (count >= 2 && soldier_color != nearest) {
        const peer_mission_t soldier_mission = mission_for(v, soldier_color);
        begin_remote_mission(&soldier_mission);
    }
    for (uint8_t i = 0; i < count; ++i) {
        if (available[i] != nearest && available[i] != soldier_color) {
            s_reserved_color = available[i];
            break;
        }
    }
    ESP_LOGI(TAG, "Asignacion: comandante cubo=%u soldado cubo=%u reservado=%u",
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

static bool turn_to(const vision_status_t *v, float target, int max_pwm,
                    bool half_speed = false)
{
    rover_imu_state_t imu = {};
    rover_service_get_imu(&imu);
    const float error = heading_error(target, current_heading(v));
    bool settled = false;
    const uint32_t tick = (uint32_t)(now_ms() / 10U);
    const int pwm = motion_turn_pwm(error,
                                    imu.valid ? imu.sample.gyro_dps[2] : 0.0f,
                                    tick, &settled);
    if (settled) {
        motor_adapter_stop();
        return true;
    }
    if (pwm == 0) {
        motor_adapter_stop();
        return false;
    }
    if ((s_cube_held && tick % COMPETITION_CUBE_TURN_PERIOD_TICKS >=
                           COMPETITION_CUBE_TURN_PERIOD_TICKS *
                               COMPETITION_CUBE_TURN_DUTY_PERCENT / 100U) ||
        (!s_cube_held && half_speed && tick % 16U >= 8U)) {
        // The driver raises commands below 700 to 700. Pulse captured turns
        // at the configured duty cycle instead of requesting unusable lower PWM.
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
    const float current = current_heading(v);
    const float cell_mm = v->cell_mm > 0.0f ? v->cell_mm : NAV_DEFAULT_CELL_MM;
    // Targets supplied by the strategy are body-centre coordinates. Differential
    // drive commands, however, control the motor axle. Convert both the current
    // and target pose to the axle reference before calculating the bearing. This
    // prevents a heading correction from sweeping the 95 mm body and arms across
    // the target or the edge of the field.
    float current_axle_col, current_axle_row;
    navigation_axle_from_center(v->col, v->row, current, cell_mm,
                                &current_axle_col, &current_axle_row);
    const float target_heading = atan2f(-(target_row - v->row),
                                        target_col - v->col) * 57.2957795f;
    float target_axle_col, target_axle_row;
    navigation_axle_from_center(target_col, target_row, target_heading, cell_mm,
                                &target_axle_col, &target_axle_row);
    const float desired = atan2f(-(target_axle_row - current_axle_row),
                                 target_axle_col - current_axle_col) * 57.2957795f;
    const float error = heading_error(desired, current);
    if (fabsf(error) > 28.0f) {
        turn_to(v, desired, base_pwm);
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

static void drive_straight_to_cube(const vision_status_t *v, uint8_t color, int pwm)
{
    const float dx = v->cubes[color].col - v->col;
    const float dy = v->cubes[color].row - v->row;
    const float bearing = atan2f(-dy, dx) * 57.2957795f;
    if (s_cube_straight_active) {
        const float travelled = distance(v->col, v->row,
                                         s_cube_straight_start_col, s_cube_straight_start_row);
        if (travelled >= 3.0f ||
            fabsf(heading_error(s_cube_straight_heading, current_heading(v))) > 8.0f ||
            fabsf(heading_error(bearing, current_heading(v))) > 12.0f) {
            motor_adapter_stop();
            s_cube_straight_active = false;
            return;
        }
        motor_adapter_set(pwm, pwm);
        return;
    }
    if (!turn_to(v, bearing, pwm)) return;
    s_cube_straight_heading = bearing;
    s_cube_straight_start_col = v->col;
    s_cube_straight_start_row = v->row;
    s_cube_straight_active = true;
}

static bool cube_push_is_safe(const vision_status_t *v, uint8_t color)
{
    if (!v || color >= VISION_MAX_CUBES) return false;
    const float dx = v->cubes[color].col - v->col;
    const float dy = v->cubes[color].row - v->row;
    return competition_cube_push_safe(v->cubes[color].col, v->cubes[color].row,
                                      dx, dy, v->grid_cols, v->grid_rows);
}

static bool carried_pose_clear(const vision_status_t *v, uint8_t color,
                               float col, float row, float heading,
                               float *blocker_col = nullptr,
                               float *blocker_row = nullptr)
{
    if (!navigation_pose_inside_mm(col, row, heading, v->grid_cols,
                                   v->grid_rows, v->cell_mm)) return false;
    // Project the observed cube offset with the rover. It is carried in front
    // of the arms, so the arm envelope alone does not cover its swept path.
    const float current_rad = current_heading(v) * 0.01745329252f;
    const float dx = v->cubes[color].col - v->col;
    const float dy = v->cubes[color].row - v->row;
    const float forward = dx * cosf(current_rad) - dy * sinf(current_rad);
    const float lateral = dx * sinf(current_rad) + dy * cosf(current_rad);
    const float radians = heading * 0.01745329252f;
    const float cube_col = col + forward * cosf(radians) + lateral * sinf(radians);
    const float cube_row = row - forward * sinf(radians) + lateral * cosf(radians);
    const float cube_radius = v->cube_side * 0.7071067812f;
    const float margin_col = navigation_workspace_margin_cells(v->grid_cols, v->cell_mm);
    const float margin_row = navigation_workspace_margin_cells(v->grid_rows, v->cell_mm);
    if (cube_col < cube_radius - margin_col ||
        cube_col > v->grid_cols + margin_col - cube_radius ||
        cube_row < cube_radius - margin_row ||
        cube_row > v->grid_rows + margin_row - cube_radius) return false;
    for (uint8_t i = 0; i < v->obstacle_count; ++i)
        if (v->obstacles[i].age_ms <= FRESH_MS && (
            !navigation_rover_clear_of_square(col, row, heading,
                v->obstacles[i].col, v->obstacles[i].row,
                NAV_OBSTACLE_SIDE_CELLS + 2.0f * NAV_CLEARANCE_CELLS) ||
            distance(cube_col, cube_row, v->obstacles[i].col,
                     v->obstacles[i].row) <= cube_radius +
                NAV_OBSTACLE_SIDE_CELLS * 0.7071067812f + NAV_CLEARANCE_CELLS)) {
            if (blocker_col && blocker_row) {
                *blocker_col = v->obstacles[i].col;
                *blocker_row = v->obstacles[i].row;
            }
            return false;
        }
    for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i)
        if (i != color && v->cube_valid[i] && v->cubes[i].age_ms <= FRESH_MS && (
            !navigation_rover_clear_of_square(col, row, heading,
                v->cubes[i].col, v->cubes[i].row,
                v->cube_side + 2.0f * NAV_CLEARANCE_CELLS) ||
            distance(cube_col, cube_row, v->cubes[i].col,
                     v->cubes[i].row) <= 2.0f * cube_radius + NAV_CLEARANCE_CELLS)) {
            if (blocker_col && blocker_row) {
                *blocker_col = v->cubes[i].col;
                *blocker_row = v->cubes[i].row;
            }
            return false;
        }
    if (v->peer_valid && v->peer_age_ms <= FRESH_MS &&
        (navigation_rovers_overlap(col, row, heading,
                                   v->peer_col, v->peer_row,
                                   v->peer_theta_deg) ||
         distance(cube_col, cube_row, v->peer_col, v->peer_row) <=
             cube_radius + navigation_body_swept_radius() + NAV_CLEARANCE_CELLS)) {
        if (blocker_col && blocker_row) {
            *blocker_col = v->peer_col;
            *blocker_row = v->peer_row;
        }
        return false;
    }
    return true;
}

static bool carried_segment_clear(const vision_status_t *v, uint8_t color,
                                  float end_col, float end_row, float heading,
                                  float *blocker_col = nullptr,
                                  float *blocker_row = nullptr)
{
    const float length = distance(v->col, v->row, end_col, end_row);
    const int steps = (int)ceilf(length / 0.5f) + 1;
    for (int i = 0; i <= steps; ++i) {
        const float t = (float)i / steps;
        if (!carried_pose_clear(v, color,
                v->col + t * (end_col - v->col),
                v->row + t * (end_row - v->row), heading,
                blocker_col, blocker_row)) return false;
    }
    return true;
}

static bool carried_turn_clear(const vision_status_t *v, uint8_t color,
                               float target_heading)
{
    const float heading = current_heading(v);
    const float error = heading_error(target_heading, heading);
    float axle_col, axle_row;
    navigation_axle_from_center(v->col, v->row, heading, v->cell_mm,
                                &axle_col, &axle_row);
    const int steps = (int)ceilf(fabsf(error) / NAV_SWEEP_SAMPLE_DEG) + 1;
    for (int i = 0; i <= steps; ++i) {
        const float angle = heading + error * i / steps;
        float col, row;
        navigation_center_from_axle(axle_col, axle_row, angle, v->cell_mm,
                                    &col, &row);
        if (!carried_pose_clear(v, color, col, row, angle)) return false;
    }
    return true;
}

static bool depot_path_clear(const vision_status_t *v, uint8_t color,
                             float *blocker_col = nullptr,
                             float *blocker_row = nullptr)
{
    const float heading = atan2f(-(v->depot_row[color] - v->row),
                                 v->depot_col[color] - v->col) * 57.2957795f;
    const float length = distance(v->col, v->row, v->depot_col[color],
                                  v->depot_row[color]);
    const float step = fmaxf(0.0f, length - COMPETITION_DEPOT_STEP_START_CELLS);
    const float radians = heading * 0.01745329252f;
    return carried_segment_clear(v, color,
        v->col + step * cosf(radians), v->row - step * sinf(radians), heading,
        blocker_col, blocker_row);
}

static void plan_depot_detour(const vision_status_t *v, uint8_t color)
{
    float blocker_col = NAN, blocker_row = NAN;
    depot_path_clear(v, color, &blocker_col, &blocker_row);
    motor_adapter_stop();
    const float bearing = atan2f(-(v->depot_row[color] - v->row),
                                 v->depot_col[color] - v->col) * 57.2957795f;
    const float first = isfinite(blocker_col) && isfinite(blocker_row) ?
        heading_error(competition_depot_detour_heading(v->col, v->row,
            v->depot_col[color], v->depot_row[color], blocker_col, blocker_row), bearing) :
        COMPETITION_DEPOT_DETOUR_DEGREES;
    const float offsets[] = {first, -first, copysignf(90.0f, first),
                             -copysignf(90.0f, first)};
    for (float offset : offsets) {
        const float heading = wrap(bearing + offset);
        const float radians = heading * 0.01745329252f;
        const float end_col = v->col + COMPETITION_DEPOT_DETOUR_CELLS * cosf(radians);
        const float end_row = v->row - COMPETITION_DEPOT_DETOUR_CELLS * sinf(radians);
        if (!carried_turn_clear(v, color, heading) ||
            !carried_segment_clear(v, color, end_col, end_row, heading)) continue;
        s_detour_heading = heading;
        s_detour_col = end_col;
        s_detour_row = end_row;
        s_phase = EXEC_DEPOT_DETOUR_TURN;
        return;
    }
    // None of the checked side routes works. Move away from the blockage and
    // plan again from the new pose.
    s_detour_heading = wrap(current_heading(v) + 180.0f);
    const float radians = s_detour_heading * 0.01745329252f;
    s_detour_col = v->col + COMPETITION_DEPOT_DETOUR_CELLS * cosf(radians);
    s_detour_row = v->row - COMPETITION_DEPOT_DETOUR_CELLS * sinf(radians);
    s_phase = EXEC_DEPOT_ESCAPE_TURN;
}

static bool reposition_safe(const vision_status_t *v, uint8_t color,
                            int direction, uint32_t remaining_ms)
{
    const float heading = current_heading(v);
    const float radians = heading * 0.01745329252f;
    const float travel = COMPETITION_REPOSITION_SPEED_CELLS_S *
                         remaining_ms / 1000.0f + 1.0f;
    const float dc = direction * travel * cosf(radians);
    const float dr = -direction * travel * sinf(radians);
    if (!navigation_segment_inside_mm(v->col, v->row, v->col + dc, v->row + dr,
                                      heading, v->grid_cols, v->grid_rows,
                                      v->cell_mm)) return false;
    for (float delta : {-8.0f, 8.0f}) {
        if (!navigation_pose_inside_mm(v->col + dc, v->row + dr, heading + delta,
                                       v->grid_cols, v->grid_rows,
                                       v->cell_mm)) return false;
    }
    if (v->peer_valid && navigation_rovers_overlap(v->col + dc, v->row + dr,
                                                    heading, v->peer_col,
                                                    v->peer_row, v->peer_theta_deg))
        return false;
    if (direction > 0) {
        const float cube_dc = v->cubes[color].col - v->col;
        const float cube_dr = v->cubes[color].row - v->row;
        const float ahead = cube_dc * cosf(radians) - cube_dr * sinf(radians);
        const float lateral = cube_dc * sinf(radians) + cube_dr * cosf(radians);
        if (ahead >= 0.0f && ahead <= travel + 8.0f && fabsf(lateral) <= 4.0f &&
            !competition_cube_inside(v->cubes[color].col + dc,
                                     v->cubes[color].row + dr,
                                     v->grid_cols, v->grid_rows)) return false;
    }
    return true;
}

static void begin_reposition(const vision_status_t *v, uint8_t color)
{
    const float radians = current_heading(v) * 0.01745329252f;
    const float toward_cube = (v->cubes[color].col - v->col) * cosf(radians) -
                              (v->cubes[color].row - v->row) * sinf(radians);
    const int preferred = toward_cube >= 0.0f ? -1 : 1;
    const bool preferred_safe = reposition_safe(v, color, preferred,
                                                 COMPETITION_REPOSITION_MS);
    const bool alternate_safe = reposition_safe(v, color, -preferred,
                                                 COMPETITION_REPOSITION_MS);
    s_reposition_direction = preferred_safe ? preferred : alternate_safe ? -preferred : 0;
    s_reposition_last_drive_ms = 0;
    s_reposition_driven_ms = 0;
    s_cube_anchor_col = v->cubes[color].col;
    s_cube_anchor_row = v->cubes[color].row;
    s_cube_anchor_frame_ms = v->frame_timestamp_ms;
    s_cube_anchor_valid = true;
    s_cube_staged = false;
    s_cube_straight_active = false;
    s_have_detour_target = false;
    stop();
    if (s_reposition_direction) {
        s_phase = EXEC_REPOSITION;
        ESP_LOGI(TAG, "Cubo desplazado >2.5 celdas: recolocacion %s",
                 s_reposition_direction < 0 ? "atras" : "adelante");
    } else {
        s_phase = EXEC_HOLD;
        s_hold_ms = now_ms();
        ESP_LOGW(TAG, "Cubo desplazado: no hay recorrido seguro para recolocacion");
    }
}

static bool obstacle_requires_detour(float cube_distance,
                                     const rover_sensor_state_t *sensors)
{
    return cube_distance > COMPETITION_CUBE_NEAR_CELLS &&
           sensors->ultrasonic_valid &&
           sensors->distance_mm <= COMPETITION_CUBE_DETECT_MM;
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
    s_cube_held = false;
    peer_comms_status_t peer = {};
    peer_comms_service_get_status(&peer);
    bool other_cube_delivered = false;
    for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c)
        if (c != color && delivered(v, c)) other_cube_delivered = true;
    s_first_delivery_retreat = s_delivered_mask == 0 &&
        peer.competition_delivered_mask == 0 && !other_cube_delivered;
    if (delivered(v, color)) s_delivered_mask |= (uint8_t)(1U << color);
    s_delivery_mission_id = s_local.id;
    s_delivery_color = color;
    s_delivery_started_ms = now_ms();
    s_delivery_last_send_ms = 0;
    peer_comms_service_send_delivery(s_local.id, color);
    ESP_LOGI(TAG, "Entrega confirmada: cubo=%u mision=%lu", color,
             (unsigned long)s_local.id);
    s_phase = EXEC_REVERSE_AFTER_DELIVERY;
    s_reverse_started_ms = now_ms();
    if (role == COMPETITION_ROLE_COMMANDER &&
        s_reserved_color < VISION_MAX_CUBES && !s_reserve_assigned &&
        s_delivered_mask != 0) {
        s_reserve_assigned = true;
        if (own_identity() == APP_STORAGE_ROVER_10 || own_identity() == APP_STORAGE_ROVER_11) {
            // Keep the reserve mission pending until the mandatory reverse
            // maneuver has completed.
            s_pending_reserve_color = s_reserved_color;
        }
    }
}

static void retry_cube_capture(uint8_t color)
{
    motor_adapter_stop();
    s_cube_held = false;
    s_cube_staged = true;
    s_cube_straight_active = false;
    s_cube_clearance_timestamp_ms = 0;
    s_cube_near_since_ms = 0;
    s_stall_since_ms = 0;
    s_stall_anchor_valid = false;
    s_phase = EXEC_TO_CUBE;
    ESP_LOGW(TAG, "Verificacion de agarre previa al deposito fallo; reintentando cubo=%u",
             color);
}

static void activate_pending_reserve(const vision_status_t *v, competition_role_t role)
{
    if (role != COMPETITION_ROLE_COMMANDER || s_phase != EXEC_DONE ||
        s_pending_reserve_color >= VISION_MAX_CUBES || !fresh(v)) return;
    const peer_mission_t reserve = mission_for(v, s_pending_reserve_color);
    s_pending_reserve_color = UINT8_MAX;
    // The commander is the only rover allowed to choose the winner; its next
    // local mission starts only after the reverse maneuver has ended.
    begin_local_mission(&reserve);
}

static void execute_local(const vision_status_t *v, competition_role_t role)
{
    if (s_phase == EXEC_REVERSE_AFTER_DELIVERY) {
        if (now_ms() - s_reverse_started_ms < COMPETITION_DELIVERY_REVERSE_MS) {
            motor_adapter_set(-COMPETITION_HALF_PWM, -COMPETITION_HALF_PWM);
            return;
        }
        stop();
        s_phase = s_first_delivery_retreat ? EXEC_ALIGN_REAR_TO_CENTER : EXEC_DONE;
        s_center_reverse_last_drive_ms = 0;
        s_center_reverse_driven_ms = 0;
        return;
    }
    if (s_phase == EXEC_ALIGN_REAR_TO_CENTER) {
        const float center_col = v->grid_cols * 0.5f;
        const float center_row = v->grid_rows * 0.5f;
        const float desired = distance(v->col, v->row, center_col, center_row) > 0.1f
            ? atan2f(-(v->row - center_row), v->col - center_col) * 57.2957795f
            : current_heading(v);
        const float error = heading_error(desired, current_heading(v));
        if (!navigation_turn_inside_mm(v->col, v->row, current_heading(v), error,
                                       v->grid_cols, v->grid_rows, v->cell_mm)) {
            stop();
            ESP_LOGW(TAG, "Giro tras entrega cancelado: saldria del campo");
            s_phase = EXEC_DONE;
            return;
        }
        if (turn_to(v, desired, COMPETITION_HALF_PWM, true))
            s_phase = EXEC_REVERSE_TO_CENTER;
        return;
    }
    if (s_phase == EXEC_REVERSE_TO_CENTER) {
        const uint64_t tick_ms = now_ms();
        if (s_center_reverse_last_drive_ms && tick_ms > s_center_reverse_last_drive_ms &&
            tick_ms - s_center_reverse_last_drive_ms <= 100U)
            s_center_reverse_driven_ms +=
                (uint32_t)(tick_ms - s_center_reverse_last_drive_ms);
        s_center_reverse_last_drive_ms = 0;
        if (s_center_reverse_driven_ms >= COMPETITION_CENTER_REVERSE_MS) {
            stop();
            s_phase = EXEC_DONE;
            return;
        }
        if (!reposition_safe(v, s_local.color, -1,
                             COMPETITION_CENTER_REVERSE_MS - s_center_reverse_driven_ms)) {
            stop();
            ESP_LOGW(TAG, "Retirada hacia el centro cancelada: recorrido inseguro");
            s_phase = EXEC_DONE;
            return;
        }
        if (peer_block_action(v, -0.8f, role) != PEER_BLOCK_NONE) {
            stop();
            return;
        }
        motor_adapter_set(-COMPETITION_HALF_PWM, -COMPETITION_HALF_PWM);
        s_center_reverse_last_drive_ms = tick_ms;
        return;
    }
    if (!s_local.id || s_phase == EXEC_DONE) return;
    if (s_phase == EXEC_WAIT) {
        if (v->phase != VISION_PHASE_RUNNING) return;
        if (role == COMPETITION_ROLE_COMMANDER && s_commander_delay_started_ms == 0) {
            peer_comms_status_t peer = {};
            peer_comms_service_get_status(&peer);
            // Only the initial launch waits for the soldier. On the reserve
            // mission the soldier may already be done and stationary.
            if (s_delivered_mask == 0 && !peer.competition_moving) return;
            s_commander_delay_started_ms = now_ms();
        }
        if (role == COMPETITION_ROLE_COMMANDER &&
            now_ms() - s_commander_delay_started_ms < COMPETITION_COMMANDER_START_DELAY_MS) {
            motor_adapter_stop();
            return;
        }
        s_phase = EXEC_STAGE_CUBE;
    }
    const uint8_t color = s_local.color;
    if (!cube_ready(v, color)) {
        stop();
        s_cube_straight_active = false;
        s_phase = EXEC_HOLD;
        s_hold_ms = now_ms();
        return;
    }
    if (s_cube_held &&
        distance(v->col, v->row, v->cubes[color].col,
                 v->cubes[color].row) > COMPETITION_CUBE_NEAR_CELLS + 1.5f) {
        retry_cube_capture(color);
        return;
    }
    if (s_phase == EXEC_REPOSITION) {
        const uint64_t tick_ms = now_ms();
        if (s_reposition_last_drive_ms && tick_ms > s_reposition_last_drive_ms &&
            tick_ms - s_reposition_last_drive_ms <= 100U)
            s_reposition_driven_ms += (uint32_t)(tick_ms - s_reposition_last_drive_ms);
        s_reposition_last_drive_ms = 0;
        if (s_reposition_driven_ms >= COMPETITION_REPOSITION_MS) {
            stop();
            s_cube_anchor_col = v->cubes[color].col;
            s_cube_anchor_row = v->cubes[color].row;
            s_cube_anchor_frame_ms = v->frame_timestamp_ms;
            s_phase = EXEC_STAGE_CUBE;
            return;
        }
        if (!reposition_safe(v, color, s_reposition_direction,
                             COMPETITION_REPOSITION_MS - s_reposition_driven_ms)) {
            stop();
            s_cube_anchor_col = v->cubes[color].col;
            s_cube_anchor_row = v->cubes[color].row;
            s_cube_anchor_frame_ms = v->frame_timestamp_ms;
            s_phase = EXEC_HOLD;
            s_hold_ms = tick_ms;
            return;
        }
        if (peer_block_action(v, s_reposition_direction * 0.8f, role) != PEER_BLOCK_NONE) {
            stop();
            return;
        }
        const int16_t pwm = s_reposition_direction * COMPETITION_HALF_PWM;
        motor_adapter_set(pwm, pwm);
        s_reposition_last_drive_ms = tick_ms;
        return;
    }
    if (s_phase == EXEC_STAGE_CUBE || s_phase == EXEC_TO_CUBE) {
        if (!s_cube_anchor_valid) {
            s_cube_anchor_col = v->cubes[color].col;
            s_cube_anchor_row = v->cubes[color].row;
            s_cube_anchor_frame_ms = v->frame_timestamp_ms;
            s_cube_anchor_valid = true;
        } else if (v->frame_timestamp_ms != s_cube_anchor_frame_ms &&
                   distance(v->cubes[color].col, v->cubes[color].row,
                            s_cube_anchor_col, s_cube_anchor_row) >
                       COMPETITION_CUBE_REPOSITION_CELLS) {
            begin_reposition(v, color);
            return;
        }
    }
    if (s_phase == EXEC_HOLD) {
        stop();
        s_cube_straight_active = false;
        if (now_ms() - s_hold_ms >= 1000)
            s_phase = s_cube_held ? EXEC_ALIGN_DEPOT :
                s_cube_staged ? EXEC_TO_CUBE : EXEC_STAGE_CUBE;
        return;
    }
    if (s_phase == EXEC_STAGE_CUBE) {
        navigation_status_t navigation = {};
        navigation_service_get_status(&navigation);
        const float speed = navigation.pose_valid
            ? fabsf(navigation.linear_speed_cells_s) : 9.0f;
        const float remaining = distance(v->col, v->row,
                                         v->cubes[color].col, v->cubes[color].row);
        if (remaining <= fmaxf(4.7f, competition_stopping_distance(speed) + 0.7f)) {
            stop();
            s_cube_straight_active = false;
            s_cube_staged = true;
            s_phase = EXEC_TO_CUBE;
            return;
        }
        if (peer_block_action(v, 1.5f, role) != PEER_BLOCK_NONE) {
            stop();
            s_cube_straight_active = false;
            return;
        }
        drive_straight_to_cube(v, color,
                               remaining < 7.0f ? COMPETITION_HALF_PWM : COMPETITION_FULL_PWM);
        return;
    }
    if (s_phase == EXEC_TO_CUBE) {
        rover_sensor_state_t sensors = {};
        rover_service_get_sensors(&sensors);
        const float cube_distance = distance(v->col, v->row,
                                             v->cubes[color].col, v->cubes[color].row);
        const bool ultrasound_fresh = sensors.timestamp_ms != 0 &&
            now_ms() >= sensors.timestamp_ms &&
            now_ms() - sensors.timestamp_ms <= 500U;
        if (ultrasound_fresh && sensors.ultrasonic_valid &&
            sensors.distance_mm >= COMPETITION_CUBE_CLEARANCE_MM)
            s_cube_clearance_timestamp_ms = sensors.timestamp_ms;
        if (cube_distance <= COMPETITION_CUBE_NEAR_CELLS) {
            if (s_cube_near_since_ms == 0) s_cube_near_since_ms = now_ms();
        } else {
            s_cube_near_since_ms = 0;
        }
        const float cube_bearing = atan2f(-(v->cubes[color].row - v->row),
                                           v->cubes[color].col - v->col) * 57.2957795f;
        const bool cube_ahead = fabsf(heading_error(cube_bearing, current_heading(v))) <= 18.0f;
        if (competition_cube_acquired(cube_distance,
                                      s_cube_clearance_timestamp_ms,
                                      s_cube_near_since_ms,
                                      sensors.timestamp_ms,
                                      ultrasound_fresh,
                                      sensors.ultrasonic_valid,
                                      sensors.ultrasonic_error == ESP_ERR_TIMEOUT) && cube_ahead) {
            motor_adapter_stop();
            s_cube_straight_active = false;
            s_cube_held = true;
            s_phase = EXEC_ALIGN_DEPOT;
            s_stall_anchor_valid = false;
            ESP_LOGI(TAG, "Cubo recogido: color=%u distancia=%.2f confirmacion=%s",
                     color, cube_distance,
                     cube_distance < COMPETITION_CUBE_HELD_CELLS ? "vision" : "timeout");
            return;
        }
        // Once the rover is close enough to contact the cube, its forward
        // velocity becomes a push. Refuse a push whose predicted cube motion
        // would take the cube outside the playable mat.
        if (cube_distance <= COMPETITION_CUBE_NEAR_CELLS &&
            !cube_push_is_safe(v, color)) {
            motor_adapter_stop();
            s_cube_straight_active = false;
            s_hold_ms = now_ms();
            s_phase = EXEC_HOLD;
            ESP_LOGW(TAG, "Aproximacion cancelada: empujaria el cubo fuera del tablero");
            return;
        }
        if (ultrasound_fresh && obstacle_requires_detour(cube_distance, &sensors)) {
            start_detour(v);
            return;
        }
        const peer_block_action_t peer_action = peer_block_action(v, 0.8f, role);
        if (peer_action == PEER_BLOCK_STOP) {
            stop();
            s_cube_straight_active = false;
            return;
        }
        if (peer_action == PEER_BLOCK_REVERSE) {
            motor_adapter_set(-COMPETITION_HALF_PWM, -COMPETITION_HALF_PWM);
            return;
        }
        const int pwm = cube_distance <= COMPETITION_CUBE_NEAR_CELLS
            ? COMPETITION_HALF_PWM : COMPETITION_FULL_PWM;
        drive_straight_to_cube(v, color, pwm);
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
        if (!depot_path_clear(v, color)) {
            plan_depot_detour(v, color);
            return;
        }
        const float target = atan2f(-(v->depot_row[color] - v->row),
                                    v->depot_col[color] - v->col) * 57.2957795f;
        if (!carried_turn_clear(v, color, target)) {
            motor_adapter_stop();
            s_phase = EXEC_DEPOT_BLOCKED_WAIT;
            return;
        }
        if (turn_to(v, target, COMPETITION_HALF_PWM, true)) {
            s_depot_turn_frame_sequence = v->sequence;
            s_phase = EXEC_VERIFY_DEPOT_ALIGNMENT;
            s_stall_since_ms = 0;
            s_stall_anchor_valid = false;
        }
        return;
    }
    if (s_phase == EXEC_VERIFY_DEPOT_ALIGNMENT) {
        motor_adapter_stop();
        if (!competition_vision_frame_is_new(v->sequence, s_depot_turn_frame_sequence) ||
            !cube_ready(v, color)) return;
        if (!depot_path_clear(v, color)) {
            plan_depot_detour(v, color);
            return;
        }
        const bool line_hits_cube = competition_assignment_line_hits_cube(
            v->col, v->row, v->depot_col[color], v->depot_row[color],
            v->cubes[color].col, v->cubes[color].row);
        if (!line_hits_cube) {
            retry_cube_capture(color);
            return;
        }
        s_depot_frame_sequence = v->sequence;
        s_phase = EXEC_TO_DEPOT;
        return;
    }
    if (s_phase == EXEC_TO_DEPOT) {
        if (delivered(v, color)) {
            complete_delivery(v, role);
            return;
        }
        if (!depot_path_clear(v, color)) {
            plan_depot_detour(v, color);
            return;
        }
        const float cube_to_depot = distance(v->cubes[color].col,
                                             v->cubes[color].row,
                                             v->depot_col[color],
                                             v->depot_row[color]);
        if (cube_to_depot <= COMPETITION_DEPOT_STEP_START_CELLS) {
            motor_adapter_stop();
            s_push_step_active = false;
            s_phase = EXEC_PUSH;
            return;
        }
        const float depot_distance = distance(v->col, v->row,
                                              v->depot_col[color], v->depot_row[color]);
        if (!competition_depot_needs_correction(depot_distance)) {
            motor_adapter_stop();
            s_push_step_active = false;
            s_phase = EXEC_PUSH;
            return;
        }
        if (competition_vision_frame_is_new(v->sequence, s_depot_frame_sequence)) {
            motor_adapter_stop();
            s_phase = EXEC_DEPOT_WAIT_VISION;
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
    if (s_phase == EXEC_DEPOT_DETOUR_TURN) {
        if (!carried_turn_clear(v, color, s_detour_heading)) {
            motor_adapter_stop();
            s_phase = EXEC_DEPOT_BLOCKED_WAIT;
            return;
        }
        if (turn_to(v, s_detour_heading, COMPETITION_HALF_PWM, true))
            s_phase = EXEC_DEPOT_DETOUR_ADVANCE;
        return;
    }
    if (s_phase == EXEC_DEPOT_DETOUR_ADVANCE) {
        if (distance(v->col, v->row, s_detour_col, s_detour_row) <= 0.8f) {
            motor_adapter_stop();
            s_phase = EXEC_ALIGN_DEPOT;
            return;
        }
        if (!carried_segment_clear(v, color, s_detour_col, s_detour_row,
                                   s_detour_heading)) {
            motor_adapter_stop();
            s_phase = EXEC_DEPOT_BLOCKED_WAIT;
            return;
        }
        if (peer_block_action(v, 0.8f, role) != PEER_BLOCK_NONE) {
            motor_adapter_stop();
            return;
        }
        direct_drive(v, s_detour_col, s_detour_row, COMPETITION_HALF_PWM);
        return;
    }
    if (s_phase == EXEC_DEPOT_BLOCKED_WAIT) {
        motor_adapter_stop();
        if (depot_path_clear(v, color)) {
            s_phase = EXEC_ALIGN_DEPOT;
        } else {
            plan_depot_detour(v, color);
        }
        return;
    }
    if (s_phase == EXEC_DEPOT_ESCAPE_TURN) {
        if (turn_to(v, s_detour_heading, COMPETITION_HALF_PWM, true))
            s_phase = EXEC_DEPOT_ESCAPE_ADVANCE;
        return;
    }
    if (s_phase == EXEC_DEPOT_ESCAPE_ADVANCE) {
        if (distance(v->col, v->row, s_detour_col, s_detour_row) <= 0.8f) {
            motor_adapter_stop();
            s_phase = EXEC_ALIGN_DEPOT;
            return;
        }
        direct_drive(v, s_detour_col, s_detour_row, COMPETITION_HALF_PWM);
        return;
    }
    if (s_phase == EXEC_DEPOT_WAIT_VISION) {
        s_phase = EXEC_ALIGN_DEPOT;
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
    if (s_phase == EXEC_PUSH_WAIT_VISION) {
        motor_adapter_stop();
        if (!competition_vision_frame_is_new(v->sequence,
                                              s_push_step_frame_sequence)) return;
        if (delivered(v, color)) {
            complete_delivery(v, role);
            return;
        }
        s_push_step_active = false;
        s_phase = EXEC_PUSH;
        return;
    }
    if (s_phase == EXEC_PUSH) {
        if (delivered(v, color)) {
            complete_delivery(v, role);
            return;
        }
        const float cube_to_depot = distance(v->cubes[color].col,
                                             v->cubes[color].row,
                                             v->depot_col[color],
                                             v->depot_row[color]);
        if (cube_to_depot <= COMPETITION_DEPOT_STEP_START_CELLS) {
            if (!s_push_step_active) {
                s_push_step_anchor_col = v->cubes[color].col;
                s_push_step_anchor_row = v->cubes[color].row;
                s_push_step_frame_sequence = v->sequence;
                s_push_step_active = true;
            }
            if (distance(v->cubes[color].col, v->cubes[color].row,
                         s_push_step_anchor_col, s_push_step_anchor_row) >=
                COMPETITION_DEPOT_STEP_CELLS) {
                motor_adapter_stop();
                s_phase = EXEC_PUSH_WAIT_VISION;
                return;
            }
        } else {
            s_push_step_active = false;
        }
        if (role == COMPETITION_ROLE_SOLDIER) update_stall(v);
        if (s_phase == EXEC_STALL_PAUSE) return;
        if (cube_to_depot <= COMPETITION_DEPOT_STEP_START_CELLS) {
            const uint32_t pulse_period = 100U / COMPETITION_DEPOT_STEP_DUTY_PERCENT;
            if ((now_ms() / 10U) % pulse_period != 0U) {
                motor_adapter_stop();
                return;
            }
        }
        direct_drive(v, v->depot_col[color], v->depot_row[color],
                     COMPETITION_DEPOT_SLOW_PWM);
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
    s_cube_staged = false;
    s_cube_anchor_valid = false;
    s_cube_anchor_frame_ms = 0;
    s_reposition_direction = 0;
    s_reposition_last_drive_ms = 0;
    s_reposition_driven_ms = 0;
    s_center_reverse_last_drive_ms = 0;
    s_center_reverse_driven_ms = 0;
    s_first_delivery_retreat = false;
    s_cube_straight_active = false;
    s_cube_held = false;
    s_stall_since_ms = 0;
    s_stall_anchor_valid = false;
    s_hold_ms = 0;
    s_peer_block_since_ms = 0;
    s_depot_frame_sequence = 0;
    s_depot_turn_frame_sequence = 0;
    s_push_step_frame_sequence = 0;
    s_push_step_anchor_col = 0.0f;
    s_push_step_anchor_row = 0.0f;
    s_push_step_active = false;
    s_cube_clearance_timestamp_ms = 0;
    s_cube_near_since_ms = 0;
    s_reverse_started_ms = 0;
    s_pending_reserve_color = UINT8_MAX;
    s_commander_delay_started_ms = 0;
    s_pose_frame_timestamp_ms = 0;
    s_pose_capture_ms = 0;
    s_last_delivery_event_id = 0;
    s_delivery_mission_id = 0;
    s_delivery_color = UINT8_MAX;
    s_delivery_started_ms = 0;
    s_delivery_last_send_ms = 0;
}

uint8_t competition_runtime_delivered_mask(void) { return s_delivered_mask; }
bool competition_runtime_moving(void)
{
    return s_phase == EXEC_STAGE_CUBE || s_phase == EXEC_TO_CUBE ||
           s_phase == EXEC_REPOSITION || s_phase == EXEC_DETOUR_TURN ||
           s_phase == EXEC_DETOUR_ADVANCE || s_phase == EXEC_ALIGN_DEPOT ||
           s_phase == EXEC_VERIFY_DEPOT_ALIGNMENT || s_phase == EXEC_TO_DEPOT || s_phase == EXEC_PUSH ||
           s_phase == EXEC_REVERSE_AFTER_DELIVERY ||
           s_phase == EXEC_ALIGN_REAR_TO_CENTER || s_phase == EXEC_REVERSE_TO_CENTER ||
           s_phase == EXEC_DEPOT_DETOUR_TURN || s_phase == EXEC_DEPOT_DETOUR_ADVANCE ||
           s_phase == EXEC_DEPOT_ESCAPE_TURN || s_phase == EXEC_DEPOT_ESCAPE_ADVANCE;
}
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
        s_phase == EXEC_DEPOT_WAIT_VISION || s_phase == EXEC_PUSH ||
        s_phase == EXEC_PUSH_WAIT_VISION ||
        s_phase == EXEC_REVERSE_AFTER_DELIVERY) {
        status->target_col = v.depot_col[s_local.color];
        status->target_row = v.depot_row[s_local.color];
    } else if (s_phase == EXEC_ALIGN_REAR_TO_CENTER ||
               s_phase == EXEC_REVERSE_TO_CENTER) {
        status->target_col = v.grid_cols * 0.5f;
        status->target_row = v.grid_rows * 0.5f;
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
        (v.phase != VISION_PHASE_READY && v.phase != VISION_PHASE_RUNNING)) {
        if (s_phase != EXEC_WAIT && s_phase != EXEC_DONE) stop();
        return;
    }
    if (!fresh(&v) || !peer.connected || peer.mode != APP_MODE_COMPETITION) {
        s_reposition_last_drive_ms = 0;
        s_center_reverse_last_drive_ms = 0;
        if (s_phase == EXEC_REVERSE_AFTER_DELIVERY) execute_local(&v, role);
        else if (s_phase != EXEC_WAIT && s_phase != EXEC_DONE) stop();
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
        activate_pending_reserve(&v, role);
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

    const vision_status_t pose = controlled_pose(&v);
    execute_local(&pose, role);
}
