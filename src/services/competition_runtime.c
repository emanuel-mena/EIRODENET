#include "competition_runtime.h"

#include <math.h>
#include <string.h>

#include "app_mode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "grid_planner.h"
#include "motor_adapter.h"
#include "motion_control.h"
#include "navigation_service.h"
#include "peer_comms_service.h"
#include "rover_service.h"
#include "vision_service.h"

#define FRESH_MS 750U
#define ROVER_RADIUS 5.60f
#define CUBE_RADIUS_FACTOR 0.707107f
#define CLEARANCE 0.40f
#define STAGE_DISTANCE 6.50f
#define PRESTAGE_DISTANCE 9.50f
#define DOCK_MM 30U
#define PUSH_MM 40U
#define MAX_ROUTE 64U

typedef enum
{
    EXEC_WAIT,
    EXEC_ROUTE,
    EXEC_NAVIGATING,
    EXEC_ALIGN,
    EXEC_CAPTURE,
    EXEC_PUSH,
    EXEC_RETREAT,
    EXEC_DONE,
    EXEC_HOLD
} exec_phase_t;

static const char *TAG = "competition_run";
static uint8_t s_occupied[GRID_PLANNER_MAX_CELLS];
static grid_planner_route_t s_grid_route;
static peer_mission_t s_local, s_remote;
static exec_phase_t s_phase;
static uint8_t s_point_index, s_delivered_mask, s_assigned_mask;
static uint8_t s_capture_samples, s_delivery_samples;
static uint8_t s_align_stable_samples;
static uint32_t s_nav_request, s_next_id;
static uint8_t s_remote_fragment;
static uint64_t s_remote_sent_ms, s_motor_pulse_ms, s_sample_ms, s_frame_ms;
static uint64_t s_hold_ms;
static uint8_t s_retries;
static float s_retreat_col, s_retreat_row;
static bool s_remote_pending, s_remote_ready, s_third_assigned;
static uint32_t s_generation;

static uint64_t now_ms(void) { return (uint64_t)(esp_timer_get_time() / 1000); }
static float length(float x, float y) { return hypotf(x, y); }
static bool fresh(const vision_status_t *v)
{
    return v->connected && v->protocol_valid && v->pose_valid && v->peer_valid &&
           v->received_ms != 0 && now_ms() - v->last_valid_frame_ms <= FRESH_MS;
}

static bool cube_ready(const vision_status_t *v, uint8_t color)
{
    return color < VISION_MAX_CUBES && v->cube_valid[color] &&
           v->depot_valid[color] && v->cubes[color].age_ms <= FRESH_MS;
}

static bool delivered(const vision_status_t *v, uint8_t color)
{
    if (!cube_ready(v, color))
        return false;
    const float radius = v->cube_side * CUBE_RADIUS_FACTOR;
    const float x = v->depot_col[color], y = v->depot_row[color];
    const float top = y, bottom = v->grid_rows - y;
    const float left = x, right = v->grid_cols - x;
    const bool horizontal = fminf(top, bottom) < fminf(left, right);
    const float half_col = (horizontal ? v->depot_length : v->depot_depth) / 2.0f;
    const float half_row = (horizontal ? v->depot_depth : v->depot_length) / 2.0f;
    return fabsf(v->cubes[color].col - x) <= half_col - radius &&
           fabsf(v->cubes[color].row - y) <= half_row - radius;
}

static void mark_circle(uint8_t cols, uint8_t rows, float col, float row, float radius)
{
    const int c0 = (int)floorf(col - radius), c1 = (int)ceilf(col + radius);
    const int r0 = (int)floorf(row - radius), r1 = (int)ceilf(row + radius);
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
        {
            if (c >= 0 && c < cols && r >= 0 && r < rows &&
                length(c + 0.5f - col, r + 0.5f - row) <= radius)
                s_occupied[r * cols + c] = 1;
        }
}

static bool plan(const vision_status_t *v, uint8_t color, bool peer_start,
                 peer_mission_t *mission)
{
    if (!cube_ready(v, color) || v->grid_cols > GRID_PLANNER_MAX_DIM ||
        v->grid_rows > GRID_PLANNER_MAX_DIM || !v->grid_cols || !v->grid_rows)
        return false;
    const float cx = v->cubes[color].col, cy = v->cubes[color].row;
    const float dx = v->depot_col[color] - cx, dy = v->depot_row[color] - cy;
    const float distance = length(dx, dy);
    if (distance < 0.1f)
        return false;
    const float stage_col = cx - STAGE_DISTANCE * dx / distance;
    const float stage_row = cy - STAGE_DISTANCE * dy / distance;
    const float pre_col = cx - PRESTAGE_DISTANCE * dx / distance;
    const float pre_row = cy - PRESTAGE_DISTANCE * dy / distance;
    const bool has_prestage = pre_col >= 2 && pre_row >= 2 &&
                              pre_col < v->grid_cols - 2 && pre_row < v->grid_rows - 2;
    const float start_col = peer_start ? v->peer_col : v->col;
    const float start_row = peer_start ? v->peer_row : v->row;
    if (!has_prestage || stage_col < 0 || stage_row < 0 || stage_col >= v->grid_cols ||
        stage_row >= v->grid_rows)
        return false;
    const uint8_t cols = v->grid_cols, rows = v->grid_rows;
    memset(s_occupied, 0, (size_t)cols * rows);
    for (uint8_t r = 0; r < rows; ++r)
        for (uint8_t c = 0; c < cols; ++c)
            if (c < 2 || r < 2 || c >= cols - 2 || r >= rows - 2)
                s_occupied[r * cols + c] = 1;
    for (uint8_t i = 0; i < v->obstacle_count; ++i)
        if (v->obstacles[i].age_ms <= FRESH_MS)
            mark_circle(cols, rows, v->obstacles[i].col, v->obstacles[i].row,
                        ROVER_RADIUS + 5.0f + CLEARANCE);
    for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i)
        if (v->cube_valid[i] && v->cubes[i].age_ms <= FRESH_MS)
            mark_circle(cols, rows, v->cubes[i].col, v->cubes[i].row,
                        ROVER_RADIUS + v->cube_side * CUBE_RADIUS_FACTOR + CLEARANCE);
    if (length(v->col - v->peer_col, v->row - v->peer_row) >
        2 * ROVER_RADIUS + CLEARANCE)
        mark_circle(cols, rows, peer_start ? v->col : v->peer_col,
                    peer_start ? v->row : v->peer_row, 2 * ROVER_RADIUS + CLEARANCE);
    const grid_planner_cell_t start = {
        .col = (uint8_t)start_col,
        .row = (uint8_t)start_row,
    };
    const grid_planner_cell_t goal = {
        .col = (uint8_t)pre_col,
        .row = (uint8_t)pre_row,
    };
    s_occupied[start.row * cols + start.col] = 0;
    if (s_occupied[goal.row * cols + goal.col])
        return false;
    if (grid_planner_plan(cols, rows, s_occupied, start, goal, &s_grid_route) != ESP_OK)
        return false;
    memset(mission, 0, sizeof(*mission));
    mission->id = ++s_next_id;
    if (mission->id == 0)
        mission->id = ++s_next_id;
    mission->color = color;
    int last_dc = 0, last_dr = 0;
    for (uint16_t i = 1; i < s_grid_route.count; ++i)
    {
        const int dc = (int)s_grid_route.cells[i].col - s_grid_route.cells[i - 1].col;
        const int dr = (int)s_grid_route.cells[i].row - s_grid_route.cells[i - 1].row;
        if (i > 1 && (dc != last_dc || dr != last_dr))
        {
            if (mission->point_count >= MAX_ROUTE - 1)
                return false;
            const grid_planner_cell_t cell = s_grid_route.cells[i - 1];
            mission->points[mission->point_count++] = (peer_mission_point_t){
                cell.col + 0.5f, cell.row + 0.5f};
        }
        last_dc = dc;
        last_dr = dr;
    }
    if (mission->point_count >= MAX_ROUTE - 1)
        return false;
    mission->points[mission->point_count++] = (peer_mission_point_t){pre_col, pre_row};
    if (mission->point_count >= MAX_ROUTE)
        return false;
    mission->points[mission->point_count++] = (peer_mission_point_t){stage_col, stage_row};
    return true;
}

static float mission_length(const vision_status_t *v, const peer_mission_t *mission,
                            bool peer_start)
{
    float x = peer_start ? v->peer_col : v->col;
    float y = peer_start ? v->peer_row : v->row;
    float total = 0;
    for (uint8_t i = 0; i < mission->point_count; ++i)
    {
        total += length(mission->points[i].col - x, mission->points[i].row - y);
        x = mission->points[i].col;
        y = mission->points[i].row;
    }
    return total;
}

static void stop(void)
{
    motor_adapter_stop();
    navigation_service_cancel(NAVIGATION_CANCEL_MODE);
}

void competition_runtime_reset(void)
{
    stop();
    memset(&s_local, 0, sizeof(s_local));
    memset(&s_remote, 0, sizeof(s_remote));
    s_phase = EXEC_WAIT;
    s_point_index = s_delivered_mask = s_assigned_mask = 0;
    s_capture_samples = s_delivery_samples = 0;
    s_nav_request = 0;
    s_remote_fragment = 0;
    s_remote_pending = s_remote_ready = s_third_assigned = false;
    s_remote_sent_ms = s_motor_pulse_ms = s_sample_ms = s_frame_ms = 0;
    s_hold_ms = 0;
    s_retries = 0;
    s_retreat_col = s_retreat_row = 0;
}

uint8_t competition_runtime_delivered_mask(void) { return s_delivered_mask; }
bool competition_runtime_available(void)
{
    return s_phase == EXEC_DONE || (s_phase == EXEC_WAIT && !s_local.id);
}

static void assign_initial(const vision_status_t *v)
{
    float best = INFINITY;
    peer_mission_t own = {0}, other = {0};
    for (uint8_t a = 0; a < VISION_MAX_CUBES; ++a)
        for (uint8_t b = 0; b < VISION_MAX_CUBES; ++b)
        {
            if (a == b || !cube_ready(v, a) || !cube_ready(v, b) ||
                delivered(v, a) || delivered(v, b))
                continue;
            peer_mission_t ma, mb;
            if (!plan(v, a, false, &ma) || !plan(v, b, true, &mb))
                continue;
            const float cost = mission_length(v, &ma, false) +
                               mission_length(v, &mb, true) +
                               length(v->depot_col[a] - v->cubes[a].col,
                                      v->depot_row[a] - v->cubes[a].row) +
                               length(v->depot_col[b] - v->cubes[b].col,
                                      v->depot_row[b] - v->cubes[b].row);
            if (cost < best)
            {
                best = cost;
                own = ma;
                other = mb;
            }
        }
    if (!isfinite(best))
        return;
    s_local = own;
    s_remote = other;
    s_retries = 0;
    s_assigned_mask = (1U << own.color) | (1U << other.color);
    s_remote_pending = true;
    s_phase = EXEC_WAIT;
    ESP_LOGI(TAG, "READY: comandante cubo %u; soldado cubo %u",
             own.color, other.color);
}

static void send_remote(void)
{
    if (!s_remote_pending || s_remote_ready)
        return;
    peer_comms_status_t peer = {0};
    peer_comms_service_get_status(&peer);
    if (!peer.connected || peer.mode != APP_MODE_COMPETITION)
        return;
    if (peer.mission_ack_id == s_remote.id &&
        peer.mission_ack_fragment == s_remote_fragment && peer.mission_ack_accepted)
    {
        ++s_remote_fragment;
        s_remote_sent_ms = 0;
        if (s_remote_fragment >= (s_remote.point_count + PEER_MISSION_FRAGMENT_POINTS - 1) /
                                     PEER_MISSION_FRAGMENT_POINTS)
        {
            s_remote_ready = true;
            s_remote_pending = false;
            ESP_LOGI(TAG, "Mision %lu confirmada por soldado", (unsigned long)s_remote.id);
            return;
        }
    }
    if (s_remote_sent_ms == 0 || now_ms() - s_remote_sent_ms >= 250)
    {
        if (peer_comms_service_send_mission_fragment(&s_remote, s_remote_fragment) == ESP_OK)
            s_remote_sent_ms = now_ms();
    }
}

static bool cube_in_opening(const vision_status_t *v, uint8_t color)
{
    if (!cube_ready(v, color) || !v->pose_valid)
        return false;
    const float radians = v->theta_deg * 0.01745329252f;
    const float dx = v->cubes[color].col - v->col;
    const float dy = v->cubes[color].row - v->row;
    const float forward = dx * cosf(radians) - dy * sinf(radians);
    const float lateral = dx * sinf(radians) + dy * cosf(radians);
    return forward >= 2.0f && forward <= 5.5f && fabsf(lateral) <= 1.5f;
}

static void hold(void)
{
    stop();
    s_phase = EXEC_HOLD;
    s_hold_ms = now_ms();
    if (s_retries < UINT8_MAX)
        ++s_retries;
}

static bool cube_in_approach(const vision_status_t *v, uint8_t color)
{
    if (!cube_ready(v, color) || !v->pose_valid)
        return false;
    const float radians = v->theta_deg * 0.01745329252f;
    const float dx = v->cubes[color].col - v->col;
    const float dy = v->cubes[color].row - v->row;
    const float forward = dx * cosf(radians) - dy * sinf(radians);
    const float lateral = dx * sinf(radians) + dy * cosf(radians);
    return forward >= 2.0f && forward <= 7.0f && fabsf(lateral) <= 1.5f;
}

static bool next_step_clear(const vision_status_t *v, uint8_t carried_color,
                            float step)
{
    const float radians = v->theta_deg * 0.01745329252f;
    const float x = v->col + step * cosf(radians);
    const float y = v->row - step * sinf(radians);
    if (x < 2 || y < 2 || x > v->grid_cols - 2 || y > v->grid_rows - 2)
        return false;
    for (uint8_t i = 0; i < v->obstacle_count; ++i)
        if (v->obstacles[i].age_ms <= FRESH_MS &&
            length(x - v->obstacles[i].col, y - v->obstacles[i].row) <
                ROVER_RADIUS + 5.0f + CLEARANCE)
            return false;
    for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i)
        if (i != carried_color && v->cube_valid[i] &&
            length(x - v->cubes[i].col, y - v->cubes[i].row) <
                ROVER_RADIUS + v->cube_side * CUBE_RADIUS_FACTOR + CLEARANCE)
            return false;
    return true;
}

static float heading_error(float desired, float actual)
{
    float error = desired - actual;
    while (error > 180)
        error -= 360;
    while (error < -180)
        error += 360;
    return error;
}

static void execute_local(const vision_status_t *v)
{
    if (!s_local.id || s_phase == EXEC_DONE)
        return;
    if (s_phase == EXEC_WAIT)
    {
        if (v->phase != VISION_PHASE_RUNNING)
            return;
        s_phase = EXEC_ROUTE;
    }
    const uint8_t color = s_local.color;
    if (!cube_ready(v, color))
    {
        if (s_phase != EXEC_HOLD)
            hold();
        return;
    }
    if ((s_phase == EXEC_ROUTE || s_phase == EXEC_NAVIGATING ||
         s_phase == EXEC_ALIGN) &&
        s_local.point_count)
    {
        const float dx = v->depot_col[color] - v->cubes[color].col;
        const float dy = v->depot_row[color] - v->cubes[color].row;
        const float distance = length(dx, dy);
        const peer_mission_point_t target = s_local.points[s_local.point_count - 1];
        if (distance < 0.1f || length(target.col -
                                          (v->cubes[color].col - STAGE_DISTANCE * dx / distance),
                                      target.row -
                                          (v->cubes[color].row - STAGE_DISTANCE * dy / distance)) > 1.0f)
        {
            hold();
            return;
        }
    }
    if (s_phase == EXEC_HOLD)
    {
        if (s_retries > 5 || now_ms() - s_hold_ms < 1000)
            return;
        if (cube_in_opening(v, color) &&
            length(v->cubes[color].col - v->col,
                   v->cubes[color].row - v->row) < 8.2f)
            return;
        peer_mission_t retry = {0};
        if (plan(v, color, false, &retry))
        {
            s_local = retry;
            s_point_index = 0;
            s_phase = EXEC_ROUTE;
        }
        return;
    }
    if (s_phase == EXEC_ROUTE)
    {
        if (s_point_index >= s_local.point_count)
        {
            s_phase = EXEC_ALIGN;
            return;
        }
        peer_mission_point_t p = s_local.points[s_point_index];
        navigation_service_set_competition_cube(color,
                                                s_point_index + 1 == s_local.point_count);
        if (navigation_service_submit_competition(p.col, p.row, &s_nav_request) == ESP_OK)
            s_phase = EXEC_NAVIGATING;
        return;
    }
    if (s_phase == EXEC_NAVIGATING)
    {
        navigation_status_t nav = {0};
        navigation_service_get_status(&nav);
        if (nav.request_id != s_nav_request)
            return;
        if (nav.phase == NAVIGATION_ARRIVED)
        {
            ++s_point_index;
            s_phase = EXEC_ROUTE;
        }
        else if (nav.phase == NAVIGATION_BLOCKED || nav.phase == NAVIGATION_ERROR ||
                 nav.phase == NAVIGATION_CANCELLED)
        {
            hold();
        }
        return;
    }
    const float dx = v->depot_col[color] - v->cubes[color].col;
    const float dy = v->depot_row[color] - v->cubes[color].row;
    const float desired = atan2f(-dy, dx) * 57.2957795f;
    const float error = heading_error(desired, v->theta_deg);
    navigation_status_t navigation = {0};
    navigation_service_get_status(&navigation);
    rover_sensor_state_t sensors = {0};
    rover_imu_state_t imu = {0};
    rover_service_get_sensors(&sensors);
    rover_service_get_imu(&imu);
    if (!imu.valid || !imu.calibration_valid || !sensors.infrared_valid ||
        !next_step_clear(v, color, s_phase == EXEC_RETREAT ? -0.8f : 0.8f))
    {
        hold();
        return;
    }
    const bool opening = cube_in_opening(v, color);
    if (s_phase == EXEC_ALIGN)
    {
        const float align_error = navigation.pose_valid
            ? motion_wrap_degrees(desired - navigation.theta_deg) : error;
        if (fabsf(align_error) > 45.0f ||
            length(v->cubes[color].col - v->col,
                   v->cubes[color].row - v->row) < 8.2f)
        {
            hold();
            return;
        }
        bool settled = false;
        const int pwm = motion_turn_pwm(align_error, navigation.angular_speed_dps,
                                        (uint32_t)(now_ms() / 10U), &settled);
        if (!settled) {
            s_align_stable_samples = 0;
            motor_adapter_set(-pwm, pwm);
            return;
        }
        motor_adapter_stop();
        if (++s_align_stable_samples < 3) return;
        s_align_stable_samples = 0;
        s_phase = EXEC_CAPTURE;
        s_capture_samples = 0;
        return;
    }
    if (s_phase == EXEC_CAPTURE)
    {
        if (v->peer_valid && length(v->peer_col - v->col,
                                    v->peer_row - v->row) < 11.6f)
        {
            motor_adapter_stop();
            return;
        }
        if (!cube_in_approach(v, color) || fabsf(error) > 12.0f)
        {
            hold();
            return;
        }
        const bool new_sample = sensors.timestamp_ms != s_sample_ms;
        if (new_sample)
            s_sample_ms = sensors.timestamp_ms;
        if (new_sample && opening && sensors.ultrasonic_valid && sensors.distance_mm <= DOCK_MM)
        {
            if (++s_capture_samples >= 3)
            {
                motor_adapter_stop();
                s_phase = EXEC_PUSH;
                return;
            }
        }
        else if (new_sample && !sensors.ultrasonic_valid &&
                 sensors.ultrasonic_error == ESP_ERR_TIMEOUT && opening)
        {
            if (++s_capture_samples >= 3)
            {
                motor_adapter_stop();
                s_phase = EXEC_PUSH;
                return;
            }
        }
        else if (new_sample)
            s_capture_samples = 0;
        if (sensors.ultrasonic_valid && sensors.distance_mm <= DOCK_MM)
        {
            motor_adapter_stop();
            return;
        }
        if (!sensors.ultrasonic_valid && sensors.ultrasonic_error != ESP_ERR_TIMEOUT)
        {
            motor_adapter_stop();
            return;
        }
        if (v->frame_timestamp_ms != s_frame_ms)
        {
            s_frame_ms = v->frame_timestamp_ms;
            motor_adapter_set(700, 700);
            s_motor_pulse_ms = now_ms();
        }
        else if (now_ms() - s_motor_pulse_ms >= 40)
            motor_adapter_stop();
        return;
    }
    if (s_phase == EXEC_PUSH)
    {
        if (v->peer_valid && length(v->peer_col - v->col,
                                    v->peer_row - v->row) < 11.6f)
        {
            motor_adapter_stop();
            return;
        }
        if (delivered(v, color))
        {
            motor_adapter_stop();
            if (v->frame_timestamp_ms != s_frame_ms)
            {
                s_frame_ms = v->frame_timestamp_ms;
                if (++s_delivery_samples >= 5)
                {
                    s_retreat_col = v->col;
                    s_retreat_row = v->row;
                    s_phase = EXEC_RETREAT;
                }
            }
            return;
        }
        s_delivery_samples = 0;
        if (!opening || fabsf(error) > 12.0f ||
            (sensors.ultrasonic_valid && sensors.distance_mm > PUSH_MM) ||
            (!sensors.ultrasonic_valid && sensors.ultrasonic_error != ESP_ERR_TIMEOUT))
        {
            hold();
            return;
        }
        if (v->frame_timestamp_ms != s_frame_ms)
        {
            s_frame_ms = v->frame_timestamp_ms;
            int correction = (int)(error * 5.0f);
            if (correction > 60)
                correction = 60;
            if (correction < -60)
                correction = -60;
            motor_adapter_set(700 - correction, 700 + correction);
            s_motor_pulse_ms = now_ms();
        }
        else if (now_ms() - s_motor_pulse_ms >= 40)
            motor_adapter_stop();
        return;
    }
    if (s_phase == EXEC_RETREAT)
    {
        if (!delivered(v, color) || (v->peer_valid &&
                                     length(v->peer_col - v->col, v->peer_row - v->row) < 11.6f))
        {
            hold();
            return;
        }
        if (length(v->col - s_retreat_col, v->row - s_retreat_row) >= 2.5f)
        {
            motor_adapter_stop();
            s_delivered_mask |= 1U << color;
            s_phase = EXEC_DONE;
            ESP_LOGI(TAG, "Cubo %u entregado y rover retirado", color);
            return;
        }
        if (v->frame_timestamp_ms != s_frame_ms)
        {
            s_frame_ms = v->frame_timestamp_ms;
            motor_adapter_set(-700, -700);
            s_motor_pulse_ms = now_ms();
        }
        else if (now_ms() - s_motor_pulse_ms >= 40)
            motor_adapter_stop();
    }
}

void competition_runtime_tick(competition_role_t role, uint32_t generation)
{
    if (generation != s_generation)
    {
        competition_runtime_reset();
        s_generation = generation;
        s_next_id = generation << 16;
    }
    vision_status_t v = {0};
    vision_service_get_status(&v);
    navigation_status_t own_navigation = {0};
    navigation_service_get_status(&own_navigation);
    v.theta_deg = heading_error(v.theta_deg + own_navigation.vision_heading_offset_deg, 0.0f);
    peer_comms_status_t link = {0};
    peer_comms_service_get_status(&link);
    if (link.connected && link.mode == APP_MODE_COMPETITION &&
        link.vision_heading_calibrated)
        v.peer_theta_deg = heading_error(v.peer_theta_deg + link.vision_heading_offset_deg, 0.0f);
    if (app_mode_get() != APP_MODE_COMPETITION ||
        (v.phase != VISION_PHASE_READY && v.phase != VISION_PHASE_RUNNING) ||
        !fresh(&v))
    {
        if (s_phase != EXEC_WAIT && s_phase != EXEC_DONE)
            stop();
        if (v.phase == VISION_PHASE_FINISHED || v.phase == VISION_PHASE_IDLE)
            competition_runtime_reset();
        return;
    }
    if (!link.connected || link.mode != APP_MODE_COMPETITION)
    {
        if (v.phase == VISION_PHASE_RUNNING && s_local.id &&
            s_phase != EXEC_DONE && s_phase != EXEC_HOLD)
            hold();
        return;
    }
    for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c)
        if ((s_delivered_mask & (1U << c)) && cube_ready(&v, c) &&
            !delivered(&v, c))
            s_delivered_mask &= ~(1U << c);
    if (s_phase == EXEC_DONE && s_local.id &&
        !delivered(&v, s_local.color))
    {
        peer_mission_t repair = {0};
        if (plan(&v, s_local.color, false, &repair))
        {
            s_local = repair;
            s_point_index = s_retries = 0;
            s_phase = EXEC_ROUTE;
        }
    }
    if (role == COMPETITION_ROLE_COMMANDER)
    {
        if (!s_local.id && v.phase == VISION_PHASE_READY)
            assign_initial(&v);
        send_remote();
        if (!s_remote_ready && s_phase == EXEC_WAIT)
            return;
        peer_comms_status_t peer = {0};
        peer_comms_service_get_status(&peer);
        const bool own_free = s_phase == EXEC_DONE;
        const bool peer_free = peer.connected && peer.competition_available &&
                               peer.competition_delivered_mask != 0;
        if (!s_third_assigned &&
            (s_delivered_mask || peer.competition_delivered_mask) &&
            (own_free || peer_free))
        {
            for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c)
            {
                if ((s_assigned_mask & (1U << c)) || delivered(&v, c))
                    continue;
                peer_mission_t own_third = {0}, peer_third = {0};
                const bool own_path = own_free && plan(&v, c, false, &own_third);
                const bool peer_path = peer_free && plan(&v, c, true, &peer_third);
                if (peer_path && (!own_path ||
                                  mission_length(&v, &peer_third, true) <
                                      mission_length(&v, &own_third, false)))
                {
                    s_remote = peer_third;
                    s_remote_fragment = 0;
                    s_remote_pending = true;
                    s_remote_ready = false;
                    s_remote_sent_ms = 0;
                    s_assigned_mask |= 1U << c;
                    s_third_assigned = true;
                }
                else if (own_path)
                {
                    s_local = own_third;
                    s_point_index = 0;
                    s_retries = 0;
                    s_phase = EXEC_ROUTE;
                    s_assigned_mask |= 1U << c;
                    s_third_assigned = true;
                }
                break;
            }
        }
        if (s_third_assigned && s_phase == EXEC_DONE && !s_remote_pending)
        {
            for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c)
            {
                if (c == s_local.color || c == s_remote.color ||
                    delivered(&v, c))
                    continue;
                peer_mission_t repair = {0};
                if (plan(&v, c, false, &repair))
                {
                    s_local = repair;
                    s_point_index = s_retries = 0;
                    s_phase = EXEC_ROUTE;
                }
                break;
            }
        }
    }
    else
    {
        peer_mission_t received = {0};
        if (peer_comms_service_get_mission(&received) && received.id != s_local.id &&
            (s_phase == EXEC_WAIT || s_phase == EXEC_DONE))
        {
            s_local = received;
            s_point_index = 0;
            s_retries = 0;
            s_phase = EXEC_WAIT;
            s_capture_samples = s_delivery_samples = 0;
        }
    }
    execute_local(&v);
}
