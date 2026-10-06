#include "competition_runtime.hpp"

#include <math.h>
#include <string.h>

#include "app_mode.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor_adapter.hpp"
#include "navigation_geometry.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "tinyml_policy.hpp"
#include "vision_service.hpp"

#define FRESH_MS 750U
#define POLICY_PERIOD_MS 50U
#define ASSIGNMENT_RETRY_MS 250U
#define PROGRESS_WINDOW_MS 8000U
#define RETREAT_WINDOW_MS 5000U
#define PROGRESS_CELLS 0.5f
#define RETREAT_CELLS 2.5f
#define CAPTURE_MM 30U
#define OBSTACLE_MM 150U
#define MAX_LINEAR_CELLS_S 7.5f
#define MAX_ANGULAR_DPS 180.0f
#define FULL_SPEED_MM_S 180.0f
#define WHEEL_TRACK_MM 80.0f
#define AXLE_BEHIND_CENTER_CELLS 1.375f
#define TRACK_GUARD_CELLS 0.75f

static const char *TAG = "competition_run";
static peer_assignment_t s_local, s_remote;
static tinyml_policy_phase_t s_phase;
static peer_assignment_result_t s_result;
static uint8_t s_delivered_mask, s_assigned_mask, s_blocked_mask, s_retry_mask;
static uint8_t s_capture_samples, s_delivery_samples, s_yield_count, s_reassignments;
static uint8_t s_failure_reason;
static uint32_t s_generation, s_next_id, s_received_assignment_id;
static uint64_t s_remote_sent_ms, s_last_policy_ms, s_last_frame_ms;
static uint64_t s_progress_ms;
static float s_progress_best, s_retreat_col, s_retreat_row;
static float s_linear_action, s_angular_action;
static int s_left, s_right;
static bool s_remote_pending;

static uint64_t now_ms(void) { return (uint64_t)(esp_timer_get_time() / 1000); }
static float distance(float ax, float ay, float bx, float by) { return hypotf(ax - bx, ay - by); }
static float wrap(float value)
{
    while (value > 180) value -= 360;
    while (value <= -180) value += 360;
    return value;
}

static bool fresh(const vision_status_t *v)
{
    return v->connected && v->protocol_valid && v->pose_valid && v->peer_valid &&
           v->received_ms && now_ms() - v->last_valid_frame_ms <= FRESH_MS;
}

static bool cube_ready(const vision_status_t *v, uint8_t color)
{
    return color < VISION_MAX_CUBES && v->cube_valid[color] && v->depot_valid[color] &&
           v->cubes[color].age_ms <= FRESH_MS;
}

static bool delivered(const vision_status_t *v, uint8_t color)
{
    return cube_ready(v, color) && v->cube_in_depot[color];
}

static bool opening(const vision_status_t *v, uint8_t color, float *forward_out = nullptr,
                    float *lateral_out = nullptr)
{
    const float a = v->theta_deg * 0.01745329252f;
    const float dx = v->cubes[color].col - v->col;
    const float dy = v->cubes[color].row - v->row;
    const float forward = dx * cosf(a) - dy * sinf(a);
    const float lateral = dx * sinf(a) + dy * cosf(a);
    if (forward_out) *forward_out = forward;
    if (lateral_out) *lateral_out = lateral;
    return forward >= 2.0f && forward <= 6.0f && fabsf(lateral) <= 1.5f;
}

static float assignment_cost(const vision_status_t *v, uint8_t color, bool peer)
{
    const float x = peer ? v->peer_col : v->col;
    const float y = peer ? v->peer_row : v->row;
    return distance(x, y, v->cubes[color].col, v->cubes[color].row) +
           distance(v->cubes[color].col, v->cubes[color].row,
                    v->depot_col[color], v->depot_row[color]);
}

static peer_assignment_t make_assignment(uint8_t color, uint8_t retry)
{
    peer_assignment_t assignment = {.id = ++s_next_id, .color = color, .retry = retry};
    if (!assignment.id) assignment.id = ++s_next_id;
    return assignment;
}

static void stop(void)
{
    s_left = s_right = 0;
    s_linear_action = s_angular_action = 0;
    motor_adapter_stop();
    navigation_service_cancel(NAVIGATION_CANCEL_MODE);
}

static void start_local(peer_assignment_t assignment)
{
    s_local = assignment;
    s_phase = TINYML_PHASE_TRANSIT;
    s_result = PEER_ASSIGNMENT_ACTIVE;
    s_capture_samples = s_delivery_samples = 0;
    s_failure_reason = NAVIGATION_FAILURE_NONE;
    s_progress_best = INFINITY;
    s_progress_ms = now_ms();
    s_linear_action = s_angular_action = 0;
    s_last_policy_ms = 0;
    s_last_frame_ms = 0;
    ESP_LOGI(TAG, "Asignacion %lu: cubo %u intento %u", (unsigned long)assignment.id,
             assignment.color, assignment.retry);
}

static void queue_remote(peer_assignment_t assignment)
{
    s_remote = assignment;
    s_remote_pending = true;
    s_remote_sent_ms = 0;
}

static void assign_initial(const vision_status_t *v)
{
    float best_max = INFINITY, best_sum = INFINITY;
    uint8_t own = UINT8_MAX, remote = UINT8_MAX;
    for (uint8_t a = 0; a < VISION_MAX_CUBES; ++a) {
        if (!cube_ready(v, a) || delivered(v, a)) continue;
        for (uint8_t b = 0; b < VISION_MAX_CUBES; ++b) {
            if (a == b || !cube_ready(v, b) || delivered(v, b)) continue;
            const float ca = assignment_cost(v, a, false);
            const float cb = assignment_cost(v, b, true);
            const float maximum = fmaxf(ca, cb), sum = ca + cb;
            if (maximum < best_max || (maximum == best_max && sum < best_sum)) {
                best_max = maximum; best_sum = sum; own = a; remote = b;
            }
        }
    }
    if (own == UINT8_MAX) {
        for (uint8_t color = 0; color < VISION_MAX_CUBES; ++color) {
            if (!cube_ready(v, color) || delivered(v, color)) continue;
            if (assignment_cost(v, color, false) <= assignment_cost(v, color, true)) own = color;
            else remote = color;
            break;
        }
    }
    if (own != UINT8_MAX) {
        start_local(make_assignment(own, 0));
        s_assigned_mask |= 1U << own;
    }
    if (remote != UINT8_MAX) {
        queue_remote(make_assignment(remote, 0));
        s_assigned_mask |= 1U << remote;
    }
}

static void send_remote(void)
{
    if (!s_remote_pending) return;
    peer_comms_status_t peer = {};
    peer_comms_service_get_status(&peer);
    if (peer.assignment_ack_id == s_remote.id && peer.assignment_ack_accepted) {
        s_remote_pending = false;
        ESP_LOGI(TAG, "Asignacion %lu confirmada", (unsigned long)s_remote.id);
        return;
    }
    if (!s_remote_sent_ms || now_ms() - s_remote_sent_ms >= ASSIGNMENT_RETRY_MS) {
        if (peer_comms_service_send_assignment(&s_remote) == ESP_OK) s_remote_sent_ms = now_ms();
    }
}

static bool peer_conflict(const vision_status_t *v, const peer_comms_status_t *peer)
{
    if (!v->peer_valid || peer->competition_available) return false;
    const float clearance = navigation_peer_clearance();
    if (distance(v->col, v->row, v->peer_col, v->peer_row) < clearance + 1.0f) return true;
    const float own_a = v->theta_deg * 0.01745329252f;
    const float peer_a = v->peer_theta_deg * 0.01745329252f;
    const float rx = v->peer_col - v->col, ry = v->peer_row - v->row;
    const float vx = 5.0f * (cosf(peer_a) - cosf(own_a));
    const float vy = -5.0f * (sinf(peer_a) - sinf(own_a));
    const float vv = vx * vx + vy * vy;
    const float t = vv > 0.001f ? fmaxf(0.0f, fminf(1.5f, -(rx * vx + ry * vy) / vv)) : 0;
    return hypotf(rx + vx * t, ry + vy * t) < clearance;
}

static void predict_pose(float col, float row, float heading, float linear, float angular,
                         float dt, float *next_col, float *next_row, float *next_heading)
{
    const float old_a = heading * 0.01745329252f;
    const float new_heading = heading + angular * dt;
    const float mid_a = (heading + angular * dt * 0.5f) * 0.01745329252f;
    const float axle_col = col - AXLE_BEHIND_CENTER_CELLS * cosf(old_a) +
                           linear * dt * cosf(mid_a);
    const float axle_row = row + AXLE_BEHIND_CENTER_CELLS * sinf(old_a) -
                           linear * dt * sinf(mid_a);
    const float new_a = new_heading * 0.01745329252f;
    *next_col = axle_col + AXLE_BEHIND_CENTER_CELLS * cosf(new_a);
    *next_row = axle_row - AXLE_BEHIND_CENTER_CELLS * sinf(new_a);
    *next_heading = wrap(new_heading);
}

static bool safe_command(const vision_status_t *v, const rover_sensor_state_t *sensors,
                         float linear, float angular, bool yielding)
{
    if (linear > 0 && sensors->ultrasonic_valid && sensors->distance_mm < OBSTACLE_MM) return false;
    float col = v->col, row = v->row, heading = v->theta_deg;
    const float initial_peer = distance(col, row, v->peer_col, v->peer_row);
    for (int sample = 0; sample < 5; ++sample) {
        predict_pose(col, row, heading, linear, angular, 0.02f, &col, &row, &heading);
        if (!navigation_pose_inside(col - TRACK_GUARD_CELLS, row - TRACK_GUARD_CELLS,
                                    heading, v->grid_cols - 2 * TRACK_GUARD_CELLS,
                                    v->grid_rows - 2 * TRACK_GUARD_CELLS)) return false;
        for (uint8_t i = 0; i < v->obstacle_count; ++i)
            if (v->obstacles[i].age_ms <= FRESH_MS &&
                distance(col, row, v->obstacles[i].col, v->obstacles[i].row) <
                    navigation_square_clearance(NAV_OBSTACLE_SIDE_CELLS)) return false;
        for (uint8_t color = 0; color < VISION_MAX_CUBES; ++color)
            if (color != s_local.color && v->cube_valid[color] &&
                distance(col, row, v->cubes[color].col, v->cubes[color].row) <
                    navigation_square_clearance(v->cube_side)) return false;
        if (v->peer_valid && navigation_rovers_overlap(col, row, heading, v->peer_col,
                v->peer_row, v->peer_theta_deg)) return false;
    }
    if (linear > 0 && (s_phase == TINYML_PHASE_CAPTURE || s_phase == TINYML_PHASE_PUSH) &&
        cube_ready(v, s_local.color)) {
        const float a = v->theta_deg * 0.01745329252f;
        const float cube_col = v->cubes[s_local.color].col + linear * 0.1f * cosf(a);
        const float cube_row = v->cubes[s_local.color].row - linear * 0.1f * sinf(a);
        const float radius = v->cube_side * 0.7071067812f + TRACK_GUARD_CELLS;
        if (cube_col < radius || cube_row < radius ||
            cube_col > v->grid_cols - radius || cube_row > v->grid_rows - radius) return false;
    }
    return !yielding || !v->peer_valid ||
           distance(col, row, v->peer_col, v->peer_row) >= initial_peer - 0.05f;
}

static int wheel_pwm(float speed_mm_s)
{
    if (fabsf(speed_mm_s) < 5.0f) return 0;
    const int magnitude = (int)lroundf(fminf(1.0f, fabsf(speed_mm_s) / FULL_SPEED_MM_S) * 1000.0f);
    const int pwm = magnitude < 700 ? 700 : magnitude;
    return speed_mm_s < 0 ? -pwm : pwm;
}

static void apply_velocity(float linear_cells_s, float angular_dps, float cell_mm)
{
    const float linear_mm_s = linear_cells_s * cell_mm;
    const float angular_rad_s = angular_dps * 0.01745329252f;
    const float left_speed = linear_mm_s - angular_rad_s * WHEEL_TRACK_MM * 0.5f;
    const float right_speed = linear_mm_s + angular_rad_s * WHEEL_TRACK_MM * 0.5f;
    s_left = wheel_pwm(left_speed);
    s_right = wheel_pwm(right_speed);
    motor_adapter_set(s_left, s_right);
}

static float progress_metric(const vision_status_t *v)
{
    const uint8_t color = s_local.color;
    if (s_phase == TINYML_PHASE_PUSH)
        return distance(v->cubes[color].col, v->cubes[color].row,
                        v->depot_col[color], v->depot_row[color]);
    if (s_phase == TINYML_PHASE_RETREAT)
        return -distance(v->col, v->row, s_retreat_col, s_retreat_row);
    return distance(v->col, v->row, v->cubes[color].col, v->cubes[color].row);
}

static void fail_local(void)
{
    stop();
    s_result = PEER_ASSIGNMENT_FAILED;
    ESP_LOGW(TAG, "Asignacion %lu sin progreso", (unsigned long)s_local.id);
}

static void update_progress(const vision_status_t *v)
{
    if (s_result != PEER_ASSIGNMENT_ACTIVE || s_phase == TINYML_PHASE_YIELD) return;
    const float metric = progress_metric(v);
    if (!isfinite(s_progress_best) || metric <= s_progress_best - PROGRESS_CELLS) {
        s_progress_best = metric;
        s_progress_ms = now_ms();
    }
    const uint64_t limit = s_phase == TINYML_PHASE_RETREAT ? RETREAT_WINDOW_MS : PROGRESS_WINDOW_MS;
    if (now_ms() - s_progress_ms >= limit) fail_local();
}

static void update_phase(const vision_status_t *v, const rover_sensor_state_t *sensors)
{
    const uint8_t color = s_local.color;
    if (delivered(v, color) && s_phase != TINYML_PHASE_RETREAT) {
        stop();
        if (v->frame_timestamp_ms != s_last_frame_ms) {
            s_last_frame_ms = v->frame_timestamp_ms;
            if (++s_delivery_samples >= 5) {
                s_phase = TINYML_PHASE_RETREAT;
                s_retreat_col = v->col; s_retreat_row = v->row;
                s_progress_best = INFINITY; s_progress_ms = now_ms();
            }
        }
        return;
    }
    if (s_phase == TINYML_PHASE_RETREAT) {
        if (distance(v->col, v->row, s_retreat_col, s_retreat_row) >= RETREAT_CELLS) {
            stop();
            s_delivered_mask |= 1U << color;
            s_result = PEER_ASSIGNMENT_DONE;
            ESP_LOGI(TAG, "Cubo %u entregado", color);
        }
        return;
    }
    float forward = 0, lateral = 0;
    const bool in_opening = opening(v, color, &forward, &lateral);
    if (s_phase == TINYML_PHASE_PUSH && (!in_opening || forward > 8.5f || fabsf(lateral) > 1.8f)) {
        s_phase = TINYML_PHASE_ALIGN; s_capture_samples = 0; s_progress_best = INFINITY;
        s_progress_ms = now_ms();
    } else if (s_phase == TINYML_PHASE_CAPTURE) {
        const bool contact = in_opening && ((sensors->ultrasonic_valid &&
            sensors->distance_mm <= CAPTURE_MM));
        if (contact && v->frame_timestamp_ms != s_last_frame_ms) {
            s_last_frame_ms = v->frame_timestamp_ms;
            if (++s_capture_samples >= 3) {
                s_phase = TINYML_PHASE_PUSH; s_progress_best = INFINITY; s_progress_ms = now_ms();
            }
        } else if (!contact) s_capture_samples = 0;
    } else if (in_opening && fabsf(lateral) <= 1.0f) {
        s_phase = TINYML_PHASE_CAPTURE;
    } else {
        const float range = distance(v->col, v->row, v->cubes[color].col, v->cubes[color].row);
        s_phase = range <= 10.0f ? TINYML_PHASE_ALIGN : TINYML_PHASE_TRANSIT;
    }
}

static void execute_policy(competition_role_t role, vision_status_t *v,
                           const peer_comms_status_t *peer, float linear_speed_cells_s)
{
    if (!s_local.id || s_result != PEER_ASSIGNMENT_ACTIVE || v->phase != VISION_PHASE_RUNNING) {
        stop(); return;
    }
    rover_sensor_state_t sensors = {};
    rover_imu_state_t imu = {};
    rover_service_get_sensors(&sensors);
    rover_service_get_imu(&imu);
    if (!cube_ready(v, s_local.color) || !imu.valid || !imu.calibration_valid ||
        !sensors.infrared_valid ||
        (!sensors.ultrasonic_valid && sensors.ultrasonic_error != ESP_ERR_TIMEOUT) ||
        !tinyml_policy_ready()) {
        s_failure_reason = !imu.valid ? NAVIGATION_FAILURE_IMU :
            !imu.calibration_valid ? NAVIGATION_FAILURE_IMU_CALIBRATION :
            !sensors.infrared_valid ? NAVIGATION_FAILURE_INFRARED :
            (!sensors.ultrasonic_valid && sensors.ultrasonic_error != ESP_ERR_TIMEOUT)
                ? NAVIGATION_FAILURE_ULTRASONIC :
            !tinyml_policy_ready() ? NAVIGATION_FAILURE_ROUTE : NAVIGATION_FAILURE_POSE;
        stop(); return;
    }
    s_failure_reason = NAVIGATION_FAILURE_NONE;
    const bool conflict = peer_conflict(v, peer);
    if (role == COMPETITION_ROLE_SOLDIER && conflict && s_phase != TINYML_PHASE_YIELD) {
        s_phase = TINYML_PHASE_YIELD; ++s_yield_count; stop();
        ESP_LOGI(TAG, "Cesion de paso %u", s_yield_count);
    } else if (s_phase == TINYML_PHASE_YIELD &&
               distance(v->col, v->row, v->peer_col, v->peer_row) > 12.5f) {
        s_phase = TINYML_PHASE_TRANSIT; s_progress_best = INFINITY; s_progress_ms = now_ms();
    }
    if (s_phase != TINYML_PHASE_YIELD) update_phase(v, &sensors);
    if (s_result != PEER_ASSIGNMENT_ACTIVE) return;
    update_progress(v);
    if (s_result != PEER_ASSIGNMENT_ACTIVE) return;
    if (!s_last_policy_ms || now_ms() - s_last_policy_ms >= POLICY_PERIOD_MS) {
        tinyml_policy_context_t context = {
            .commander = role == COMPETITION_ROLE_COMMANDER,
            .phase = s_phase,
            .color = s_local.color,
            .peer_active = !peer->competition_available,
            .conflict = conflict,
            .linear_speed_cells_s = linear_speed_cells_s,
            .previous_linear = s_linear_action,
            .previous_angular = s_angular_action,
        };
        float linear = 0, angular = 0;
        if (tinyml_policy_infer(&context, v, &sensors, &imu, &linear, &angular) != ESP_OK) {
            stop(); return;
        }
        s_linear_action = linear;
        s_angular_action = angular;
        s_last_policy_ms = now_ms();
    }
    float linear = s_linear_action * MAX_LINEAR_CELLS_S;
    const float angular = s_angular_action * MAX_ANGULAR_DPS;
    if (s_phase == TINYML_PHASE_RETREAT && linear > 0) linear = -linear;
    if (!safe_command(v, &sensors, linear, angular, s_phase == TINYML_PHASE_YIELD)) {
        stop(); return;
    }
    apply_velocity(linear, angular, v->cell_mm);
}

void competition_runtime_reset(void)
{
    stop();
    memset(&s_local, 0, sizeof(s_local));
    memset(&s_remote, 0, sizeof(s_remote));
    s_phase = TINYML_PHASE_TRANSIT;
    s_result = PEER_ASSIGNMENT_NONE;
    s_delivered_mask = s_assigned_mask = s_blocked_mask = s_retry_mask = 0;
    s_capture_samples = s_delivery_samples = s_yield_count = s_reassignments = 0;
    s_failure_reason = NAVIGATION_FAILURE_NONE;
    s_received_assignment_id = 0;
    s_remote_pending = false;
    s_remote_sent_ms = s_last_policy_ms = s_last_frame_ms = s_progress_ms = 0;
    s_progress_best = INFINITY;
}

uint8_t competition_runtime_delivered_mask(void) { return s_delivered_mask; }
bool competition_runtime_available(void)
{
    return !s_local.id || s_result == PEER_ASSIGNMENT_DONE || s_result == PEER_ASSIGNMENT_FAILED;
}

void competition_runtime_get_assignment_status(competition_assignment_status_t *status)
{
    if (!status) return;
    *status = {.id = s_local.id, .color = s_local.color, .result = (uint8_t)s_result,
               .phase = (uint8_t)s_phase, .yield_count = s_yield_count,
               .reassignment_count = s_reassignments, .blocked_mask = s_blocked_mask,
               .failure_reason = s_failure_reason};
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
    navigation_status_t navigation = {};
    navigation_service_get_status(&navigation);
    v.theta_deg = navigation.pose_valid ? navigation.theta_deg :
        wrap(v.theta_deg + navigation.vision_heading_offset_deg);
    if (peer.connected && peer.vision_heading_calibrated)
        v.peer_theta_deg = wrap(v.peer_theta_deg + peer.vision_heading_offset_deg);
    tinyml_policy_status_t model = {};
    tinyml_policy_get_status(&model);
    rover_sensor_state_t health = {};
    rover_service_get_sensors(&health);
    if (!health.infrared_valid) s_failure_reason = NAVIGATION_FAILURE_INFRARED;
    else if (!health.ultrasonic_valid && health.ultrasonic_error != ESP_ERR_TIMEOUT)
        s_failure_reason = NAVIGATION_FAILURE_ULTRASONIC;
    else if (!model.available) s_failure_reason = NAVIGATION_FAILURE_ROUTE;
    if (app_mode_get() != APP_MODE_COMPETITION || !fresh(&v) || !peer.connected ||
        peer.mode != APP_MODE_COMPETITION || !model.available ||
        !peer.model_available || peer.model_version != model.version ||
        peer.model_crc32 != model.crc32) {
        stop(); return;
    }
    if (role == COMPETITION_ROLE_COMMANDER) {
        if (!s_local.id && !s_remote.id) assign_initial(&v);
        send_remote();
        if (s_result == PEER_ASSIGNMENT_FAILED && s_local.retry == 0 &&
            !(s_retry_mask & (1U << s_local.color)) &&
            !s_remote_pending) {
            queue_remote(make_assignment(s_local.color, 1));
            s_retry_mask |= 1U << s_local.color;
            ++s_reassignments;
        } else if (s_result == PEER_ASSIGNMENT_FAILED && s_local.retry != 0) {
            s_blocked_mask |= 1U << s_local.color;
        }
        if (peer.assignment_result == PEER_ASSIGNMENT_FAILED && s_remote.id &&
            peer.assignment_id == s_remote.id && s_remote.retry == 0 &&
            !(s_retry_mask & (1U << s_remote.color)) &&
            competition_runtime_available()) {
            start_local(make_assignment(s_remote.color, 1));
            s_retry_mask |= 1U << s_remote.color;
            ++s_reassignments;
        } else if (peer.assignment_result == PEER_ASSIGNMENT_FAILED && s_remote.id &&
                   peer.assignment_id == s_remote.id && s_remote.retry != 0) {
            s_blocked_mask |= 1U << s_remote.color;
        }
        const bool own_free = competition_runtime_available();
        const bool peer_free = peer.competition_available && !s_remote_pending;
        if (own_free || peer_free) {
            for (uint8_t color = 0; color < VISION_MAX_CUBES; ++color) {
                if ((s_assigned_mask | s_blocked_mask) & (1U << color) || delivered(&v, color)) continue;
                peer_assignment_t next = make_assignment(color, 0);
                if (own_free && (!peer_free || assignment_cost(&v, color, false) <=
                                             assignment_cost(&v, color, true))) start_local(next);
                else queue_remote(next);
                s_assigned_mask |= 1U << color;
                break;
            }
        }
    } else {
        peer_assignment_t received = {};
        if (peer_comms_service_get_assignment(&received) &&
            received.id != s_received_assignment_id && competition_runtime_available()) {
            s_received_assignment_id = received.id;
            start_local(received);
        }
    }
    execute_policy(role, &v, &peer, navigation.linear_speed_cells_s);
}

#ifdef EIRO_HOST_SIM
int competition_sim_phase(void) { return (int)s_phase; }
int competition_sim_left(void) { return s_left; }
int competition_sim_right(void) { return s_right; }
#endif
