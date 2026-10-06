#include "competition_runtime.hpp"

#include <math.h>
#include <string.h>

#include "app_mode.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "grid_planner.hpp"
#include "navigation_geometry.hpp"
#include "motor_adapter.hpp"
#include "motion_control.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"

#define FRESH_MS 750U
#define CUBE_RADIUS_FACTOR 0.707107f
// 166 mm leaves room for the front arm's swept radius and a 60 mm cube,
// including the navigation arrival tolerance (8 mm).
#define STAGE_DISTANCE 8.30f
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
static uint32_t s_received_mission_id;
static uint64_t s_assignment_ms;
static uint64_t s_peer_stall_ms;
static bool s_yielding;
static uint8_t s_yield_count;
static bool s_stage_push;
static bool s_stage_retreat;
static float s_push_goal_col, s_push_goal_row;
static float s_push_start_col, s_push_start_row;
static uint64_t s_no_plan_log_ms;
static uint8_t s_failed_stage_mask;
static bool s_approach_only;
static uint8_t s_intermediate_pushes;

static const char *mission_kind(bool staged, bool approach_only)
{
    return approach_only ? "aproximacion" : (staged ? "intermedia" : "directa");
}

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
    return v->cube_in_depot[color];
}

static bool mission_geometry_safe(const vision_status_t *v, const peer_mission_t *mission,
                                  bool peer_start, const char *kind)
{
    if (!v || !mission) return false;
    float col = peer_start ? v->peer_col : v->col;
    float row = peer_start ? v->peer_row : v->row;
    float heading = peer_start ? v->peer_theta_deg : v->theta_deg;
    for (uint8_t i = 0; i < mission->point_count; ++i) {
        const float next_col = mission->points[i].col;
        const float next_row = mission->points[i].row;
        const float dx = next_col - col, dy = next_row - row;
        if (length(dx, dy) < 0.05f) continue;
        const uint8_t direction = (uint8_t)fmodf(roundf(atan2f(-dy, dx) /
                                                         0.7853981634f) + 8.0f, 8.0f);
        const float forward_heading = 45.0f * direction;
        bool safe = navigation_turn_inside_mm(col, row, heading,
                                              forward_heading - heading,
                                              v->grid_cols, v->grid_rows, v->cell_mm) &&
            navigation_segment_inside_mm(col, row, next_col, next_row,
                                         forward_heading, v->grid_cols, v->grid_rows,
                                         v->cell_mm);
        if (!safe) {
            const float reverse_heading = forward_heading + 180.0f;
            safe = navigation_turn_inside_mm(col, row, heading,
                                             reverse_heading - heading,
                                             v->grid_cols, v->grid_rows, v->cell_mm) &&
                navigation_segment_inside_mm(col, row, next_col, next_row,
                                             reverse_heading, v->grid_cols, v->grid_rows,
                                             v->cell_mm);
        }
        if (!safe && i == 0) {
            const float escape_heading = roundf(heading / 45.0f) * 45.0f;
            const float radians = escape_heading * 0.01745329252f;
            for (uint8_t half_steps = 1; half_steps <= 8 && !safe; ++half_steps) {
                const float distance = 0.5f * half_steps;
                const float escape_col = col + distance * cosf(radians);
                const float escape_row = row - distance * sinf(radians);
                if (!navigation_segment_inside_mm(col, row, escape_col, escape_row,
                                                  escape_heading, v->grid_cols, v->grid_rows,
                                                  v->cell_mm)) continue;
                safe = navigation_turn_inside_mm(escape_col, escape_row, escape_heading,
                                                 forward_heading - escape_heading,
                                                 v->grid_cols, v->grid_rows, v->cell_mm) &&
                    navigation_segment_inside_mm(escape_col, escape_row, next_col, next_row,
                                                 forward_heading, v->grid_cols, v->grid_rows,
                                                 v->cell_mm);
            }
        }
        if (!safe) {
            ESP_LOGW(TAG, "MISSION_REJECT cube=%u type=%s reason=pivot_or_envelope point=%u from=%.2f,%.2f to=%.2f,%.2f heading=%.0f",
                     mission->color, kind, (unsigned)i, (double)col, (double)row,
                     (double)next_col, (double)next_row, (double)heading);
            return false;
        }
        col = next_col;
        row = next_row;
        heading = forward_heading;
    }
    return true;
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

static bool plan_to(const vision_status_t *v, uint8_t color, bool peer_start,
                    float goal_col, float goal_row, bool staged,
                    peer_mission_t *mission)
{
    if (!cube_ready(v, color) || v->grid_cols > GRID_PLANNER_MAX_DIM ||
        v->grid_rows > GRID_PLANNER_MAX_DIM || !v->grid_cols || !v->grid_rows)
        return false;
    const float cx = v->cubes[color].col, cy = v->cubes[color].row;
    const float dx = goal_col - cx, dy = goal_row - cy;
    const float distance = length(dx, dy);
    if (distance < 0.1f)
        return false;
    const float stage_col = cx - STAGE_DISTANCE * dx / distance;
    const float stage_row = cy - STAGE_DISTANCE * dy / distance;
    const float start_col = peer_start ? v->peer_col : v->col;
    const float start_row = peer_start ? v->peer_row : v->row;
    if (stage_col < 0 || stage_row < 0 || stage_col >= v->grid_cols ||
        stage_row >= v->grid_rows)
        return false;
    // Reserve the complete push corridor, not just the route to the cube.
    // A blocking cube must be delivered first instead of trapping the pusher.
    const float push_heading = atan2f(-dy, dx) * 57.2957795f;
    const float dock = NAV_BODY_HALF_LENGTH_CELLS + v->cube_side / 2;
    const float travel = fmaxf(0, distance + STAGE_DISTANCE - dock);
    const int samples = (int)ceilf(travel / 0.5f);
    for (int sample = 0; sample <= samples; ++sample) {
        const float t = samples ? travel * sample / samples : 0;
        const float x = stage_col + t * dx / distance, y = stage_row + t * dy / distance;
        if (!navigation_pose_inside_mm(x, y, push_heading, v->grid_cols, v->grid_rows,
                                       v->cell_mm)) {
            ESP_LOGW(TAG, "MISSION_REJECT cube=%u type=%s reason=push_envelope sample=%d",
                     color, staged ? "intermedia" : "directa", sample);
            return false;
        }
        // The simulator reports a cube outside when any rotated corner leaves
        // the board, so use the square's circumradius rather than its half
        // side. This keeps the guard independent of the cube yaw.
        const float cube_half = v->cube_side * CUBE_RADIUS_FACTOR;
        const float contact_distance = fmaxf(0.0f, STAGE_DISTANCE - dock);
        const float cube_progress = fminf(distance, fmaxf(0.0f, t - contact_distance));
        const float cube_col = cx + cube_progress * dx / distance;
        const float cube_row = cy + cube_progress * dy / distance;
        if (cube_col - cube_half < 0.0f || cube_row - cube_half < 0.0f ||
            cube_col + cube_half > v->grid_cols || cube_row + cube_half > v->grid_rows) {
            ESP_LOGW(TAG, "MISSION_REJECT cube=%u type=%s reason=cube_edge sample=%d",
                     color, staged ? "intermedia" : "directa", sample);
            return false;
        }
        for (uint8_t i = 0; i < v->obstacle_count; ++i)
            if (v->obstacles[i].age_ms <= FRESH_MS &&
                length(x - v->obstacles[i].col, y - v->obstacles[i].row) <
                    navigation_square_clearance(NAV_OBSTACLE_SIDE_CELLS)) {
                ESP_LOGW(TAG, "MISSION_REJECT cube=%u type=%s reason=obstacle_corridor",
                         color, staged ? "intermedia" : "directa");
                return false;
            }
        for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i)
            if (i != color && cube_ready(v, i) &&
                length(x - v->cubes[i].col, y - v->cubes[i].row) <
                    navigation_square_clearance(v->cube_side)) {
                ESP_LOGW(TAG, "MISSION_REJECT cube=%u type=%s reason=cube_corridor other=%u",
                         color, staged ? "intermedia" : "directa", i);
                return false;
            }
    }
    const uint8_t cols = v->grid_cols, rows = v->grid_rows;
    memset(s_occupied, 0, (size_t)cols * rows);
    for (uint8_t r = 0; r < rows; ++r)
        for (uint8_t c = 0; c < cols; ++c)
            if (c + 0.5f < NAV_BODY_TURN_RADIUS_CELLS || r + 0.5f < NAV_BODY_TURN_RADIUS_CELLS ||
                c + 0.5f > cols - NAV_BODY_TURN_RADIUS_CELLS || r + 0.5f > rows - NAV_BODY_TURN_RADIUS_CELLS)
                s_occupied[r * cols + c] = 1;
    for (uint8_t i = 0; i < v->obstacle_count; ++i)
        if (v->obstacles[i].age_ms <= FRESH_MS)
            mark_circle(cols, rows, v->obstacles[i].col, v->obstacles[i].row,
                        navigation_square_clearance(NAV_OBSTACLE_SIDE_CELLS));
    for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i)
        if (v->cube_valid[i] && v->cubes[i].age_ms <= FRESH_MS)
            mark_circle(cols, rows, v->cubes[i].col, v->cubes[i].row,
                        navigation_square_clearance(v->cube_side));
    const float peer_clearance = staged ? NAV_PEER_ROUTE_CLEARANCE_CELLS :
                                    navigation_peer_clearance();
    if (length(v->col - v->peer_col, v->row - v->peer_row) > peer_clearance)
        mark_circle(cols, rows, peer_start ? v->col : v->peer_col,
                    peer_start ? v->row : v->peer_row, peer_clearance);
    const grid_planner_cell_t start = {
        .col = (uint8_t)start_col,
        .row = (uint8_t)start_row,
    };
    const grid_planner_cell_t goal = {
        .col = (uint8_t)stage_col,
        .row = (uint8_t)stage_row,
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
    const float lattice_offset_col = start_col - floorf(start_col);
    const float lattice_offset_row = start_row - floorf(start_row);
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
                cell.col + lattice_offset_col, cell.row + lattice_offset_row};
        }
        last_dc = dc;
        last_dr = dr;
    }
    if (mission->point_count >= MAX_ROUTE - 1)
        return false;
    // Align outside the cube's turning clearance, then approach in a straight
    // line. Navigating closer first makes the alignment clearance impossible.
    if (staged) {
        const peer_mission_point_t previous = mission->point_count
            ? mission->points[mission->point_count - 1]
            : (peer_mission_point_t){start_col, start_row};
        const float leg_col = stage_col - previous.col;
        const float leg_row = stage_row - previous.row;
        if (length(leg_col, leg_row) > 0.1f) {
            const float heading = roundf(atan2f(-leg_row, leg_col) *
                                         57.2957795f / 45.0f) * 45.0f;
            const bool forward = navigation_pose_inside_mm(previous.col, previous.row,
                                                            heading, cols, rows, v->cell_mm) &&
                                 navigation_pose_inside_mm(stage_col, stage_row,
                                                            heading, cols, rows, v->cell_mm);
            const bool backward = navigation_pose_inside_mm(previous.col, previous.row,
                                                             heading + 180, cols, rows, v->cell_mm) &&
                                  navigation_pose_inside_mm(stage_col, stage_row,
                                                             heading + 180, cols, rows, v->cell_mm);
            if (!forward && !backward) return false;
        }
    }
    mission->points[mission->point_count++] = (peer_mission_point_t){stage_col, stage_row};
    return mission_geometry_safe(v, mission, peer_start, mission_kind(staged, false));
}

static bool plan(const vision_status_t *v, uint8_t color, bool peer_start,
                 peer_mission_t *mission)
{
    return plan_to(v, color, peer_start, v->depot_col[color],
                   v->depot_row[color], false, mission);
}

static float mission_length(const vision_status_t *v, const peer_mission_t *mission,
                            bool peer_start);

// When the direct push is unreachable, move a cube a short distance toward
// free space. Reobserve it before selecting the next action; never extrapolate
// several physical pushes from the initial pose.
static bool plan_stage(const vision_status_t *v, uint8_t color,
                       peer_mission_t *mission, float *goal_col, float *goal_row)
{
    if (!cube_ready(v, color)) return false;
    const float cx = v->cubes[color].col, cy = v->cubes[color].row;
    const float gx = v->depot_col[color] - cx;
    const float gy = v->depot_row[color] - cy;
    const float norm = length(gx, gy);
    if (norm < 0.1f) return false;
    float best = INFINITY;
    for (int i = 0; i < 32; ++i) {
        const float a = (float)i * 0.1963495408f;
        const float ux = cosf(a), uy = sinf(a);
        const float progress = (ux * gx + uy * gy) / norm;
        if (progress < 0.05f) continue;
        const float tx = cx + 5.0f * ux, ty = cy + 5.0f * uy;
        if (tx < v->cube_side / 2 || ty < v->cube_side / 2 ||
            tx > v->grid_cols - v->cube_side / 2 ||
            ty > v->grid_rows - v->cube_side / 2) continue;
        peer_mission_t candidate = {};
        if (!plan_to(v, color, false, tx, ty, true, &candidate)) continue;
        const float border = fmaxf(0.0f, 5.0f - tx) +
                             fmaxf(0.0f, 5.0f - ty) +
                             fmaxf(0.0f, tx - (v->grid_cols - 5.0f)) +
                             fmaxf(0.0f, ty - (v->grid_rows - 5.0f));
        const float cost = 0.2f * mission_length(v, &candidate, false) -
                           50.0f * progress + 8.0f * border;
        if (cost < best) {
            best = cost;
            *mission = candidate;
            *goal_col = tx;
            *goal_row = ty;
        }
    }
    return isfinite(best);
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
    s_received_mission_id = 0;
    s_assignment_ms = 0;
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
    s_peer_stall_ms = 0;
    s_yielding = false;
    s_yield_count = 0;
    s_stage_push = false;
    s_approach_only = false;
    s_intermediate_pushes = 0;
    s_stage_retreat = false;
    s_push_goal_col = s_push_goal_row = 0;
    s_push_start_col = s_push_start_row = 0;
    s_no_plan_log_ms = 0;
    s_failed_stage_mask = 0;
    s_retreat_col = s_retreat_row = 0;
}

uint8_t competition_runtime_delivered_mask(void) { return s_delivered_mask; }
bool competition_runtime_available(void)
{
    return s_phase == EXEC_DONE || (s_phase == EXEC_WAIT && !s_local.id);
}

static void assign_initial(const vision_status_t *v)
{
    if (s_assignment_ms && now_ms() - s_assignment_ms < 500) return;
    s_assignment_ms = now_ms();
    s_stage_push = false;
    s_approach_only = false;
    float best = INFINITY;
    peer_mission_t own = {0}, other = {0};
    for (uint8_t a = 0; a < VISION_MAX_CUBES; ++a)
        for (uint8_t b = 0; b < VISION_MAX_CUBES; ++b)
        {
            if (a == b || (s_failed_stage_mask & ((1U << a) | (1U << b))) ||
                !cube_ready(v, a) || !cube_ready(v, b) ||
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
    if (!isfinite(best)) {
        // One accessible cube is useful work even if a pair is unavailable.
        for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c) {
            if ((s_failed_stage_mask & (1U << c)) ||
                !cube_ready(v, c) || delivered(v, c)) continue;
            for (int remote = 0; remote < 2; ++remote) {
                peer_mission_t candidate = {};
                if (!plan(v, c, remote != 0, &candidate)) continue;
                const float cost = mission_length(v, &candidate, remote != 0);
                if (cost < best) {
                    best = cost;
                    own = {}; other = {};
                    if (remote) other = candidate; else own = candidate;
                }
            }
        }
        if (!isfinite(best)) {
            // A single intermediate push may create a valid final approach.
            for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c) {
                if ((s_failed_stage_mask & (1U << c)) ||
                    !cube_ready(v, c) || delivered(v, c)) continue;
                peer_mission_t candidate = {};
                float goal_col = 0, goal_row = 0;
                if (!plan_stage(v, c, &candidate, &goal_col, &goal_row)) continue;
                const float cost = mission_length(v, &candidate, false);
                if (cost < best) {
                    best = cost;
                    own = candidate;
                    other = {};
                    s_push_goal_col = goal_col;
                    s_push_goal_row = goal_row;
                    s_stage_push = true;
                }
            }
            if (!isfinite(best)) {
                // A bounded approach is still useful evidence for a difficult
                // distribution. It never relaxes the body/edge/peer checks.
                for (uint8_t c = 0; c < VISION_MAX_CUBES; ++c) {
                    if ((s_failed_stage_mask & (1U << c)) ||
                        !cube_ready(v, c) || delivered(v, c)) continue;
                    peer_mission_t candidate = {};
                    float goal_col = 0, goal_row = 0;
                    if (!plan_stage(v, c, &candidate, &goal_col, &goal_row)) continue;
                    const float cost = mission_length(v, &candidate, false);
                    if (cost < best) {
                        best = cost;
                        own = candidate;
                        other = {};
                        s_push_goal_col = goal_col;
                        s_push_goal_row = goal_row;
                        s_approach_only = true;
                    }
                }
                if (!isfinite(best)) {
                    if (!s_no_plan_log_ms || now_ms() - s_no_plan_log_ms >= 10000) {
                        s_no_plan_log_ms = now_ms();
                        ESP_LOGW(TAG, "MISSION_ABORT reason=no_safe_candidate pending_mask=0x%02x",
                                 (unsigned)(~s_delivered_mask & ((1U << VISION_MAX_CUBES) - 1)));
                    }
                    return;
                }
                ESP_LOGI(TAG, "MISSION_SELECT cube=%u type=aproximacion points=%u",
                         own.color, own.point_count);
            } else {
                ESP_LOGI(TAG, "MISSION_SELECT cube=%u type=intermedia points=%u",
                         own.color, own.point_count);
            }
        }
    }
    if (!s_stage_push) s_push_goal_col = s_push_goal_row = 0;
    s_local = own;
    s_remote = other;
    s_retries = 0;
    s_assigned_mask = (own.id ? 1U << own.color : 0) | (other.id ? 1U << other.color : 0);
    s_remote_pending = other.id != 0;
    s_remote_ready = !s_remote_pending;
    s_phase = EXEC_WAIT;
    if (other.id)
        ESP_LOGI(TAG, "READY: comandante cubo %u; soldado cubo %u",
                 own.color, other.color);
    else
        ESP_LOGI(TAG, "READY: comandante cubo %u tipo=%s; soldado sin mision",
                 own.color, mission_kind(s_stage_push, s_approach_only));
}

static void send_remote(const vision_status_t *v)
{
    if (!s_remote_pending || s_remote_ready)
        return;
    if (s_local.id && s_phase != EXEC_DONE && s_remote.point_count) {
        const peer_mission_point_t entry = s_remote.points[0];
        if (length(entry.col - v->col, entry.row - v->row) <=
            NAV_PEER_ROUTE_CLEARANCE_CELLS + 1.0f)
            return;
        const peer_mission_point_t target = s_remote.points[s_remote.point_count - 1];
        if (length(target.col - v->col, target.row - v->row) <=
            2.0f * navigation_peer_clearance())
            return;
    }
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
    return forward >= 2.0f && forward <= 6.0f && fabsf(lateral) <= 1.5f;
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
    return forward >= 2.0f && forward <= STAGE_DISTANCE + 1.0f && fabsf(lateral) <= 1.5f;
}

static bool next_step_clear(const vision_status_t *v, uint8_t carried_color,
                            float step)
{
    const float radians = v->theta_deg * 0.01745329252f;
    const float x = v->col + step * cosf(radians);
    const float y = v->row - step * sinf(radians);
    if (!navigation_segment_inside_mm(v->col, v->row, x, y, v->theta_deg,
                                      v->grid_cols, v->grid_rows, v->cell_mm))
        return false;
    for (uint8_t i = 0; i < v->obstacle_count; ++i)
        if (v->obstacles[i].age_ms <= FRESH_MS &&
            length(x - v->obstacles[i].col, y - v->obstacles[i].row) <
                navigation_square_clearance(NAV_OBSTACLE_SIDE_CELLS))
            return false;
    for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i)
        if (i != carried_color && v->cube_valid[i] &&
            length(x - v->cubes[i].col, y - v->cubes[i].row) <
                navigation_square_clearance(v->cube_side))
            return false;
    return true;
}

static bool cube_push_step_clear(const vision_status_t *v, uint8_t color,
                                 float step)
{
    if (color >= VISION_MAX_CUBES || !v->cube_valid[color]) return true;
    const float radians = v->theta_deg * 0.01745329252f;
    const float next_col = v->cubes[color].col + step * cosf(radians);
    const float next_row = v->cubes[color].row - step * sinf(radians);
    const float half = v->cube_side * CUBE_RADIUS_FACTOR;
    return next_col - half >= 0.0f && next_row - half >= 0.0f &&
           next_col + half <= v->grid_cols && next_row + half <= v->grid_rows;
}

static bool peer_blocks_step(const vision_status_t *v, float step)
{
    peer_comms_status_t peer = {};
    peer_comms_service_get_status(&peer);
    // Reserve manoeuvring room while the other rover is working. A parked
    // rover needs only its actual footprint, not a permanent circular roadblock.
    if (v->peer_valid && !peer.competition_available &&
        length(v->peer_col - v->col, v->peer_row - v->row) < 11.6f)
        return true;
    const float a = v->theta_deg * 0.01745329252f;
    return v->peer_valid && navigation_rovers_overlap(
        v->col + step * cosf(a), v->row - step * sinf(a), v->theta_deg,
        v->peer_col, v->peer_row, v->peer_theta_deg);
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
        const float dx = (s_stage_push ? s_push_goal_col : v->depot_col[color]) -
                         v->cubes[color].col;
        const float dy = (s_stage_push ? s_push_goal_row : v->depot_row[color]) -
                         v->cubes[color].row;
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
        // A cube can coast into the depot after a safety stop. The referee's
        // fresh verdict still completes the task; do not demand another push.
        if (delivered(v, color)) {
            s_retreat_col = v->col;
            s_retreat_row = v->row;
            s_phase = EXEC_RETREAT;
            return;
        }
        if (s_stage_push && cube_in_opening(v, color)) {
            s_retreat_col = v->col;
            s_retreat_row = v->row;
            s_stage_retreat = true;
            s_phase = EXEC_RETREAT;
            return;
        }
        if (s_retries >= 3) {
            if (s_retries == 3) {
                ESP_LOGW(TAG, "MISSION_ABORT cube=%u type=%s reason=attempt_limit attempts=%u",
                         color, mission_kind(s_stage_push, s_approach_only), (unsigned)s_retries);
                s_failed_stage_mask |= 1U << color;
                s_local = {};
                s_assigned_mask &= (uint8_t)~(1U << color);
                s_stage_push = false;
                s_approach_only = false;
                s_phase = EXEC_WAIT;
                s_assignment_ms = 0;
            }
            return;
        }
        if (now_ms() - s_hold_ms < 1000)
            return;
        if (cube_in_opening(v, color) &&
            length(v->cubes[color].col - v->col,
                   v->cubes[color].row - v->row) < 8.2f)
            return;
        peer_mission_t retry = {0};
        bool direct = plan(v, color, false, &retry);
        float goal_col = 0, goal_row = 0;
        bool staged = !direct && s_intermediate_pushes < 3 &&
                      plan_stage(v, color, &retry, &goal_col, &goal_row);
        if (direct || staged)
        {
            s_local = retry;
            s_stage_push = staged;
            s_approach_only = false;
            s_push_goal_col = goal_col;
            s_push_goal_row = goal_row;
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
        navigation_service_set_competition_cube(color, false);
        if (navigation_service_submit_competition(p.col, p.row, &s_nav_request) == ESP_OK)
            s_phase = EXEC_NAVIGATING;
        return;
    }
    if (s_phase == EXEC_NAVIGATING)
    {
        navigation_status_t nav = {};
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
    const float dx = (s_stage_push && s_phase >= EXEC_PUSH ? s_push_goal_col - s_push_start_col :
                      (s_stage_push ? s_push_goal_col : v->depot_col[color]) - v->cubes[color].col);
    const float dy = (s_stage_push && s_phase >= EXEC_PUSH ? s_push_goal_row - s_push_start_row :
                      (s_stage_push ? s_push_goal_row : v->depot_row[color]) - v->cubes[color].row);
    const float desired = atan2f(-dy, dx) * 57.2957795f;
    const float error = heading_error(desired, v->theta_deg);
    navigation_status_t navigation = {};
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
    if (s_phase == EXEC_PUSH && !cube_push_step_clear(v, color, 0.8f))
    {
        ESP_LOGW(TAG, "MISSION_REJECT cube=%u type=%s reason=cube_edge_runtime",
                 color, mission_kind(s_stage_push, s_approach_only));
        hold();
        return;
    }
    const bool opening = cube_in_opening(v, color);
    if (s_phase == EXEC_ALIGN)
    {
        float align_error = navigation.pose_valid
            ? motion_wrap_degrees(desired - navigation.theta_deg) : error;
        if (length(v->cubes[color].col - v->col,
                   v->cubes[color].row - v->row) <
            hypotf(NAV_FRONT_EXTENT_CELLS, NAV_ROVER_WIDTH_CELLS / 2) + v->cube_side * CUBE_RADIUS_FACTOR)
        {
            hold();
            return;
        }
        const float heading = navigation.pose_valid ? navigation.theta_deg : v->theta_deg;
        if (!navigation_turn_inside_mm(v->col, v->row, heading, align_error,
                                       v->grid_cols, v->grid_rows, v->cell_mm)) {
            align_error += align_error > 0 ? -360.0f : 360.0f;
            if (!navigation_turn_inside_mm(v->col, v->row, heading, align_error,
                                           v->grid_cols, v->grid_rows, v->cell_mm)) {
                hold();
                return;
            }
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
        if (s_approach_only) {
            ESP_LOGI(TAG, "MISSION_ATTEMPT cube=%u type=aproximacion result=observed_no_push",
                     color);
            s_failed_stage_mask |= 1U << color;
            s_local = {};
            s_assigned_mask &= (uint8_t)~(1U << color);
            s_approach_only = false;
            s_phase = EXEC_WAIT;
            s_assignment_ms = 0;
            return;
        }
        s_phase = EXEC_CAPTURE;
        s_capture_samples = 0;
        return;
    }
    if (s_phase == EXEC_CAPTURE)
    {
        if (peer_blocks_step(v, 0.8f))
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
                s_push_start_col = v->cubes[color].col;
                s_push_start_row = v->cubes[color].row;
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
                s_push_start_col = v->cubes[color].col;
                s_push_start_row = v->cubes[color].row;
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
        if (peer_blocks_step(v, 0.8f))
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
        if (s_stage_push) {
            const float sx = s_push_goal_col - s_push_start_col;
            const float sy = s_push_goal_row - s_push_start_row;
            const float progress = ((v->cubes[color].col - s_push_start_col) * sx +
                                    (v->cubes[color].row - s_push_start_row) * sy) /
                                   fmaxf(0.1f, length(sx, sy));
            if (progress >= 4.0f) {
                motor_adapter_stop();
                if (s_intermediate_pushes < UINT8_MAX) ++s_intermediate_pushes;
                s_retreat_col = v->col;
                s_retreat_row = v->row;
                s_stage_retreat = true;
                s_phase = EXEC_RETREAT;
                s_retries = 0;
                ESP_LOGI(TAG, "MISSION_ATTEMPT cube=%u type=intermedia push=%u result=stop_reobserve",
                         color, (unsigned)s_intermediate_pushes);
                return;
            }
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
        if ((!s_stage_retreat && !delivered(v, color)) || peer_blocks_step(v, -0.8f))
        {
            hold();
            return;
        }
        if (length(v->col - s_retreat_col, v->row - s_retreat_row) >= 2.5f)
        {
            motor_adapter_stop();
            if (s_stage_retreat) {
                s_stage_retreat = false;
                s_phase = EXEC_HOLD;
                s_hold_ms = now_ms();
                return;
            }
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
    navigation_status_t own_navigation = {};
    navigation_service_get_status(&own_navigation);
    v.theta_deg = own_navigation.pose_valid ? own_navigation.theta_deg :
        heading_error(v.theta_deg + own_navigation.vision_heading_offset_deg, 0.0f);
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
            s_stage_push = false;
            s_approach_only = false;
            s_point_index = s_retries = 0;
            s_phase = EXEC_ROUTE;
        }
    }
    if (role == COMPETITION_ROLE_COMMANDER)
    {
        if (!s_assigned_mask)
            assign_initial(&v);
        send_remote(&v);
        peer_comms_status_t peer = {0};
        peer_comms_service_get_status(&peer);
        const bool own_free = s_phase == EXEC_DONE || (s_phase == EXEC_WAIT && !s_local.id);
        const bool peer_free = peer.connected && peer.competition_available &&
                               (!s_remote.id || (peer.competition_delivered_mask & (1U << s_remote.color)));
        if (!s_remote_pending && own_free && peer_free && s_assigned_mask &&
            now_ms() - s_assignment_ms >= 500)
        {
            s_assignment_ms = now_ms();
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
                    s_stage_push = false;
                    s_approach_only = false;
                    s_point_index = 0;
                    s_retries = 0;
                    s_phase = EXEC_ROUTE;
                    s_assigned_mask |= 1U << c;
                    s_third_assigned = true;
                }
                else
                    continue;
                break;
            }
        }
        if (s_third_assigned && s_phase == EXEC_DONE && !s_remote_pending && peer_free)
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
                    s_stage_push = false;
                    s_approach_only = false;
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
        if (peer_comms_service_get_mission(&received) && received.id != s_received_mission_id &&
            (s_phase == EXEC_WAIT || s_phase == EXEC_DONE))
        {
            s_received_mission_id = received.id;
            s_local = received;
            s_stage_push = false;
            s_approach_only = false;
            s_point_index = 0;
            s_retries = 0;
            s_phase = EXEC_WAIT;
            s_capture_samples = s_delivery_samples = 0;
        }
    }
    // The collision guard in navigation stops both rovers when their next
    // segments point towards each other. The soldier yields to a point away
    // from the commander, then resumes the interrupted mission waypoint.
    if (s_yielding) {
        navigation_status_t nav = {};
        navigation_service_get_status(&nav);
        if (nav.request_id == s_nav_request && nav.phase == NAVIGATION_ARRIVED) {
            s_yielding = false;
            // The original waypoints were planned from the old side of the
            // crossing. Replan from the observed pose after giving way.
            s_phase = EXEC_HOLD;
            s_hold_ms = now_ms();
            s_retries = 0;
            s_peer_stall_ms = 0;
        } else if (nav.request_id == s_nav_request &&
                   (nav.phase == NAVIGATION_BLOCKED || nav.phase == NAVIGATION_ERROR ||
                    nav.phase == NAVIGATION_CANCELLED)) {
            s_yielding = false;
            ESP_LOGW(TAG, "Cesion de paso sin ruta segura");
            hold();
        }
        return;
    }
    if (role == COMPETITION_ROLE_SOLDIER && s_phase == EXEC_NAVIGATING &&
        v.peer_valid && !link.competition_available &&
        length(v.peer_col - v.col, v.peer_row - v.row) < 11.6f &&
        own_navigation.has_target && own_navigation.motor_left == 0 &&
        own_navigation.motor_right == 0) {
        if (!s_peer_stall_ms) s_peer_stall_ms = now_ms();
        if (now_ms() - s_peer_stall_ms >= 750 && s_yield_count >= 3) {
            ESP_LOGW(TAG, "Bloqueo entre rovers sin mas cesiones seguras");
            hold();
            return;
        }
        if (now_ms() - s_peer_stall_ms >= 750 && s_yield_count < 3) {
            const float dx = v.col - v.peer_col, dy = v.row - v.peer_row;
            const float distance = length(dx, dy);
            for (float step = 6.5f; step >= 3.0f; step -= 0.5f) {
                const float x = v.col + step * dx / distance;
                const float y = v.row + step * dy / distance;
                if (!navigation_pose_inside_mm(x, y, v.theta_deg, v.grid_cols, v.grid_rows,
                                               v.cell_mm))
                    continue;
                navigation_service_set_competition_cube(s_local.color, false);
                if (navigation_service_submit_competition(x, y, &s_nav_request) == ESP_OK) {
                    s_yielding = true;
                    ++s_yield_count;
                    ESP_LOGI(TAG, "Cesion de paso hacia %.2f, %.2f", (double)x, (double)y);
                    return;
                }
            }
            s_peer_stall_ms = now_ms();
        }
    } else s_peer_stall_ms = 0;
    execute_local(&v);
}

#ifdef EIRO_HOST_SIM
int competition_sim_phase(void) { return (int)s_phase; }
const peer_mission_t *competition_sim_mission(void) { return &s_local; }
#endif
