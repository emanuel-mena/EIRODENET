#include "navigation_service.hpp"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app_mode.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "grid_planner.hpp"
#include "navigation_geometry.hpp"
#include "motor_adapter.hpp"
#include "motion_control.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"

#define NAV_PERIOD_MS 10
#define GRID_MIN_ADC_SPAN 220
#define SENSOR_FRONT_MM 10.0f
#define SENSOR_LEFT_MM 20.0f
#define UNCERTAINTY_LIMIT_CELLS 25.0f
#define NAV_MAX_SEGMENTS 1024U
#define NAV_MAX_BLIND_CROSSINGS 2U
#define NAV_ROUTE_REPLAN_ATTEMPTS 8U
#define NAV_NO_PROGRESS_TIMEOUT_MS 3000U
#define NAV_TURN_TIMEOUT_MS 3000U
#define CELL_BOUNDARY_HYSTERESIS 0.12f
#define SENSOR_INVALID_SAMPLES 3U
#define TURN_PWM 700
#define DRIVE_PWM 850
#define SLOW_PWM 700
#define MAX_PWM 1000
#define ENTER_DRIVE_DEG 8.0f
#define RETURN_TURN_DEG 15.0f
#define SLOW_DISTANCE_CELLS 3.0f
#define ARRIVAL_CELLS 0.40f
#define ARRIVAL_SAMPLES 5
#define ARRIVAL_MAX_SPEED_CELLS_S 0.35f
#define SLOW_PULSE_CONTROL_TICKS 2U
#define SLOW_PULSE_INTERVAL_MS 150U
#define ALIGNMENT_SAMPLES 5
#define MISALIGNMENT_SAMPLES 3
#define IR_STABLE_SAMPLES 3
#define OBSTACLE_MM 150U
#define OBSTACLE_SAMPLES 3
#define VISION_FRESH_MS 750U

typedef struct {
    uint16_t low, high;
    bool high_level, calibrated, polarity_known, high_is_black;
    int8_t polarity_score;
} grid_sensor_t;

typedef struct {
    bool initialized;
    float col, row, theta_deg;
    float speed_cells_s, angular_speed_dps, gyro_bias_dps;
    float vision_heading_offset_deg, uncertainty_cells;
    bool vision_heading_calibrated;
    uint64_t vision_frame_timestamp_ms;
    uint64_t vision_ms;
    float vision_col, vision_row;
    grid_sensor_t sensor[4];
} estimator_t;

typedef struct {
    float start_col, start_row;
    float end_col, end_row;
    int8_t delta_col, delta_row;
    uint8_t heading_index;
    bool reverse;
} route_segment_t;

typedef struct {
    bool valid;
    uint8_t grid_cols, grid_rows;
    float cell_mm;
    uint16_t segment_count, segment_index;
    route_segment_t segments[NAV_MAX_SEGMENTS];
    grid_planner_route_t cells;
    uint16_t cell_index;
    uint64_t occupancy_frame_timestamp_ms;
} navigation_route_t;

static const char *TAG = "navigation";
static SemaphoreHandle_t s_lock;
static navigation_status_t s_status = {.phase = NAVIGATION_WAITING_FOR_POSE, .core_id = 1};
static estimator_t s_estimator;
static navigation_route_t s_route;
static uint8_t s_occupancy[GRID_PLANNER_MAX_CELLS];
static uint32_t s_next_request;
static bool s_competition_target;
static uint8_t s_competition_cube = UINT8_MAX;
static bool s_competition_contact;
static uint8_t s_arrival_samples, s_obstacle_samples;
static uint8_t s_alignment_samples, s_misalignment_samples;
static uint8_t s_saturation_samples;
static uint8_t s_slow_pulse_ticks;
static uint64_t s_slow_pulse_ms;
static uint8_t s_ultrasonic_invalid_samples;
static uint64_t s_ultrasonic_sample_ms;
static uint8_t s_stable_pattern, s_stable_pattern_samples;
static uint8_t s_confirmed_pattern;
static bool s_confirmed_cell_valid;
static motion_bias_window_t s_bias_window;
static uint32_t s_bias_generation = UINT32_MAX;
static bool s_bias_frozen;
static uint64_t s_last_imu_timestamp_ms;
static uint32_t s_motion_tick;
static float s_motor_trim;
static uint64_t s_heading_still_ms;
static uint64_t s_progress_ms;
static uint64_t s_progress_log_ms;
static float s_progress_col, s_progress_row;
static float s_progress_distance, s_progress_heading_error;
static uint16_t s_progress_segment;
static uint8_t s_route_forbidden[GRID_PLANNER_MAX_CELLS];
static navigation_failure_reason_t s_planning_failure_reason = NAVIGATION_FAILURE_NONE;

static float wrap_degrees(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

static float angle_error(float target, float current) { return wrap_degrees(target - current); }

static float blend_angle(float current, float target, float gain)
{
    return wrap_degrees(current + gain * angle_error(target, current));
}

static int sign_float(float value)
{
    return value > 0.0001f ? 1 : (value < -0.0001f ? -1 : 0);
}

static uint8_t direction_index(int delta_col, int delta_row)
{
    static const int8_t directions[8][2] = {
        {1, 0}, {1, -1}, {0, -1}, {-1, -1},
        {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    };
    for (uint8_t i = 0; i < 8; ++i)
        if (directions[i][0] == delta_col && directions[i][1] == delta_row) return i;
    return 0;
}

static float direction_heading(uint8_t index)
{
    return 45.0f * index;
}

static int coordinate_cell(float coordinate, uint8_t dimension)
{
    int cell = (int)floorf(coordinate);
    if (cell < 0) cell = 0;
    if (cell >= dimension) cell = dimension - 1;
    return cell;
}

static int stable_coordinate_cell(float coordinate, int confirmed, uint8_t dimension)
{
    const int candidate = coordinate_cell(coordinate, dimension);
    if (!s_confirmed_cell_valid || candidate == confirmed) return candidate;
    if (candidate == confirmed + 1 &&
        coordinate < (float)(confirmed + 1) + CELL_BOUNDARY_HYSTERESIS) return confirmed;
    if (candidate == confirmed - 1 &&
        coordinate > (float)confirmed - CELL_BOUNDARY_HYSTERESIS) return confirmed;
    return candidate;
}

static uint8_t classify_ir(const infrared_adapter_state_t *ir)
{
    const uint16_t raw[] = {ir->front_left, ir->front_right, ir->rear_left, ir->rear_right};
    uint8_t pattern = 0;
    for (size_t i = 0; i < 4; ++i) {
        grid_sensor_t *sensor = &s_estimator.sensor[i];
        if (sensor->low == 0 && sensor->high == 0) sensor->low = sensor->high = raw[i];
        if (raw[i] < sensor->low) sensor->low = raw[i];
        if (raw[i] > sensor->high) sensor->high = raw[i];
        const uint16_t span = sensor->high - sensor->low;
        sensor->calibrated = span >= GRID_MIN_ADC_SPAN;
        if (sensor->calibrated) {
            const uint16_t lower = sensor->low + (uint16_t)(span * 0.42f);
            const uint16_t upper = sensor->low + (uint16_t)(span * 0.58f);
            if (sensor->high_level && raw[i] < lower) sensor->high_level = false;
            else if (!sensor->high_level && raw[i] > upper) sensor->high_level = true;
        }
        if (sensor->high_level) pattern |= (uint8_t)(1U << i);
    }
    return pattern;
}

static void sensor_position(size_t index, float col, float row, float theta_deg,
                            float cell_mm, float *sensor_col, float *sensor_row)
{
    const float forward = index < 2 ? SENSOR_FRONT_MM : -SENSOR_FRONT_MM;
    const float left = (index == 0 || index == 2) ? SENSOR_LEFT_MM : -SENSOR_LEFT_MM;
    const float radians = theta_deg * (float)M_PI / 180.0f;
    *sensor_col = col + (forward * cosf(radians) - left * sinf(radians)) / cell_mm;
    *sensor_row = row + (-forward * sinf(radians) - left * cosf(radians)) / cell_mm;
}

static bool black_square(float col, float row)
{
    return (((int)floorf(col) + (int)floorf(row)) & 1) != 0;
}

static uint8_t expected_pattern(float col, float row, float theta, float cell_mm)
{
    uint8_t pattern = 0;
    for (size_t i = 0; i < 4; ++i) {
        float sensor_col, sensor_row;
        sensor_position(i, col, row, theta, cell_mm, &sensor_col, &sensor_row);
        const bool black = black_square(sensor_col, sensor_row);
        const bool high = s_estimator.sensor[i].high_is_black ? black : !black;
        if (high) pattern |= (uint8_t)(1U << i);
    }
    return pattern;
}

static void learn_polarity(uint8_t pattern, const vision_status_t *vision)
{
    if (!vision->pose_valid || vision->cell_mm <= 0) return;
    for (size_t i = 0; i < 4; ++i) {
        grid_sensor_t *sensor = &s_estimator.sensor[i];
        if (!sensor->calibrated) continue;
        float col, row;
        sensor_position(i, vision->col, vision->row,
                        vision->theta_deg + s_estimator.vision_heading_offset_deg,
                        vision->cell_mm, &col, &row);
        if (fabsf(col - roundf(col)) < 0.15f || fabsf(row - roundf(row)) < 0.15f) continue;
        const bool high = (pattern & (1U << i)) != 0;
        sensor->polarity_score += high == black_square(col, row) ? 1 : -1;
        if (sensor->polarity_score > 20) sensor->polarity_score = 20;
        if (sensor->polarity_score < -20) sensor->polarity_score = -20;
        if (abs(sensor->polarity_score) >= 5) {
            sensor->polarity_known = true;
            sensor->high_is_black = sensor->polarity_score > 0;
        }
    }
}

static uint8_t calibration_mask(void)
{
    uint8_t mask = 0;
    for (size_t i = 0; i < 4; ++i)
        if (s_estimator.sensor[i].calibrated && s_estimator.sensor[i].polarity_known)
            mask |= (uint8_t)(1U << i);
    return mask;
}

static bool apply_vision(const vision_status_t *vision, uint64_t now_ms)
{
    if (!vision->pose_valid ||
        (s_estimator.vision_ms != 0 &&
         vision->frame_timestamp_ms == s_estimator.vision_frame_timestamp_ms)) return false;
    if (!s_estimator.initialized) {
        s_estimator.initialized = true;
        s_estimator.col = vision->col;
        s_estimator.row = vision->row;
        s_estimator.theta_deg = wrap_degrees(
            vision->theta_deg + s_estimator.vision_heading_offset_deg);
        s_estimator.uncertainty_cells = 0.25f;
    } else {
        s_estimator.col += 0.25f * (vision->col - s_estimator.col);
        s_estimator.row += 0.25f * (vision->row - s_estimator.row);
        // Camera frames can arrive after a turn. Feeding their old heading
        // back into a calibrated gyro while moving causes overshoot. Reconcile
        // heading only after a stationary observation window in competition.
        if (app_mode_get() != APP_MODE_COMPETITION ||
            (s_heading_still_ms && now_ms - s_heading_still_ms >= 500))
            s_estimator.theta_deg = blend_angle(s_estimator.theta_deg,
                vision->theta_deg + s_estimator.vision_heading_offset_deg, 0.18f);
        s_estimator.uncertainty_cells = fmaxf(0.15f, s_estimator.uncertainty_cells * 0.65f);
        if (s_estimator.vision_ms && now_ms > s_estimator.vision_ms) {
            const float dt = (float)(now_ms - s_estimator.vision_ms) / 1000.0f;
            const float dx = vision->col - s_estimator.vision_col;
            const float dy = vision->row - s_estimator.vision_row;
            if (dt > 0.02f && dt < 2.0f) {
                float measured = hypotf(dx, dy) / dt;
                const float radians = s_estimator.theta_deg * (float)M_PI / 180.0f;
                if (dx * cosf(radians) - dy * sinf(radians) < 0) measured = -measured;
                if (fabsf(measured) < 30.0f)
                    s_estimator.speed_cells_s = 0.7f * s_estimator.speed_cells_s + 0.3f * measured;
            }
        }
    }
    s_estimator.vision_ms = now_ms;
    s_estimator.vision_col = vision->col;
    s_estimator.vision_row = vision->row;
    s_estimator.vision_frame_timestamp_ms = vision->frame_timestamp_ms;
    s_status.last_correction = NAVIGATION_CORRECTION_VISION;
    return true;
}

static bool update_estimator(const rover_imu_state_t *imu, const rover_sensor_state_t *sensors,
                             const vision_status_t *vision, float dt, uint64_t now_ms)
{
    const uint8_t observed = sensors->infrared_valid ? classify_ir(&sensors->infrared) : 0;
    if (observed == s_stable_pattern) {
        if (s_stable_pattern_samples < UINT8_MAX) ++s_stable_pattern_samples;
    } else {
        s_stable_pattern = observed;
        s_stable_pattern_samples = 1;
    }
    if (sensors->infrared_valid) learn_polarity(observed, vision);
    if (s_estimator.initialized && imu->valid &&
        imu->timestamp_ms != s_last_imu_timestamp_ms) {
        float imu_dt = s_last_imu_timestamp_ms
            ? (float)(imu->timestamp_ms - s_last_imu_timestamp_ms) / 1000.0f : dt;
        if (imu_dt <= 0.0f || imu_dt > 0.1f) imu_dt = dt;
        s_last_imu_timestamp_ms = imu->timestamp_ms;
        if (app_mode_get() != APP_MODE_COMPETITION && !s_status.motor_left &&
            !s_status.motor_right && fabsf(imu->sample.gyro_dps[2]) < 5.0f)
            s_estimator.gyro_bias_dps = 0.995f * s_estimator.gyro_bias_dps +
                                       0.005f * imu->sample.gyro_dps[2];
        s_estimator.angular_speed_dps = imu->sample.gyro_dps[2] - s_estimator.gyro_bias_dps;
        motion_integrate_axle_pose(&s_estimator.col, &s_estimator.row,
                                   &s_estimator.theta_deg,
                                   s_estimator.speed_cells_s,
                                   s_estimator.angular_speed_dps,
                                   navigation_axle_offset_cells(vision->cell_mm),
                                   imu_dt);
        s_estimator.speed_cells_s *= 0.998f;
        s_estimator.uncertainty_cells += vision->pose_valid ? 0.0002f : 0.0015f;
        s_status.last_correction = NAVIGATION_CORRECTION_IMU;
    }
    if (fabsf(s_estimator.angular_speed_dps) > 3.0f || fabsf(s_estimator.speed_cells_s) > 0.2f ||
        s_status.motor_left || s_status.motor_right)
        s_heading_still_ms = 0;
    else if (!s_heading_still_ms)
        s_heading_still_ms = now_ms;
    const bool new_vision_frame = apply_vision(vision, now_ms);
    s_status.grid_correction_active = false;
    s_status.infrared_pattern = observed;
    s_status.infrared_calibrated_mask = calibration_mask();
    s_status.grid_calibrated = s_status.infrared_calibrated_mask == 0x0f;
    return new_vision_frame;
}

static void mark_occupied(uint8_t cols, uint8_t rows, float center_col, float center_row,
                          float radius)
{
    for (int row = (int)floorf(center_row - radius); row <= (int)ceilf(center_row + radius); ++row) {
        if (row < 0 || row >= rows) continue;
        for (int col = (int)floorf(center_col - radius); col <= (int)ceilf(center_col + radius); ++col) {
            if (col < 0 || col >= cols) continue;
            if (hypotf(col + 0.5f - center_col, row + 0.5f - center_row) <= radius)
                s_occupancy[row * cols + col] = 1;
        }
    }
}

static esp_err_t build_occupancy(const vision_status_t *vision)
{
    if (vision->grid_cols == 0 || vision->grid_rows == 0 ||
        vision->grid_cols > GRID_PLANNER_MAX_DIM || vision->grid_rows > GRID_PLANNER_MAX_DIM)
        return ESP_ERR_INVALID_SIZE;
    const uint8_t cols = (uint8_t)vision->grid_cols;
    const uint8_t rows = (uint8_t)vision->grid_rows;
    memset(s_occupancy, 0, (size_t)cols * rows);
    if (s_competition_target) {
        for (uint8_t r = 0; r < rows; ++r)
            for (uint8_t c = 0; c < cols; ++c)
                if (c + 0.5f < NAV_BODY_TURN_RADIUS_CELLS || r + 0.5f < NAV_BODY_TURN_RADIUS_CELLS ||
                    c + 0.5f > cols - NAV_BODY_TURN_RADIUS_CELLS || r + 0.5f > rows - NAV_BODY_TURN_RADIUS_CELLS)
                    s_occupancy[r * cols + c] = 1;
    }
    for (uint8_t i = 0; i < vision->obstacle_count; ++i) {
        const vision_position_t *obstacle = &vision->obstacles[i];
        if (obstacle->age_ms > VISION_FRESH_MS) continue;
        mark_occupied(cols, rows, obstacle->col, obstacle->row,
                      navigation_square_clearance(NAV_OBSTACLE_SIDE_CELLS));
    }
    if (vision->peer_valid && vision->peer_age_ms <= VISION_FRESH_MS) {
        peer_comms_status_t peer = {};
        peer_comms_service_get_status(&peer);
        // A free peer only needs its physical footprint. Reserve the larger
        // route corridor while it is executing or its state is unavailable.
        mark_occupied(cols, rows, vision->peer_col, vision->peer_row,
                      peer.competition_available ? navigation_peer_clearance() :
                      NAV_PEER_ROUTE_CLEARANCE_CELLS);
    }
    if (s_competition_target) {
        for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i) {
            if ((i == s_competition_cube && s_competition_contact) ||
                !vision->cube_valid[i] ||
                vision->cubes[i].age_ms > VISION_FRESH_MS) continue;
            mark_occupied(cols, rows, vision->cubes[i].col, vision->cubes[i].row,
                          navigation_square_clearance(vision->cube_side));
        }
    }
    return ESP_OK;
}

static esp_err_t append_segment(float start_col, float start_row, float end_col, float end_row)
{
    const int delta_col = sign_float(end_col - start_col);
    const int delta_row = sign_float(end_row - start_row);
    if (delta_col == 0 && delta_row == 0) return ESP_OK;
    const uint8_t heading = direction_index(delta_col, delta_row);
    bool reverse = false;
    if (s_competition_target) {
        const float cols = s_route.grid_cols, rows = s_route.grid_rows;
        const float angle = direction_heading(heading);
        bool forward = navigation_pose_inside_mm(start_col, start_row, angle, cols, rows,
                                                 s_route.cell_mm) &&
            navigation_pose_inside_mm(end_col, end_row, angle, cols, rows, s_route.cell_mm);
        // Reverse before reaching the edge, while there is still room to turn
        // the asymmetric arms (including arrival and braking tolerances).
        const float turn_margin = 7.2f;
        if ((delta_col > 0 && end_col > cols - turn_margin) ||
            (delta_col < 0 && end_col < turn_margin) ||
            (delta_row > 0 && end_row > rows - turn_margin) ||
            (delta_row < 0 && end_row < turn_margin)) forward = false;
        const bool backward = navigation_pose_inside_mm(start_col, start_row, angle + 180, cols, rows,
                                                        s_route.cell_mm) &&
            navigation_pose_inside_mm(end_col, end_row, angle + 180, cols, rows, s_route.cell_mm);
        if (!forward && !backward) {
            if (hypotf(end_col - start_col, end_row - start_row) < 2.0f)
                return ESP_ERR_NOT_FOUND;
            const float mid_col = (start_col + end_col) / 2, mid_row = (start_row + end_row) / 2;
            esp_err_t err = append_segment(start_col, start_row, mid_col, mid_row);
            return err == ESP_OK ? append_segment(mid_col, mid_row, end_col, end_row) : err;
        }
        reverse = !forward;
    }
    if (s_route.segment_count > 0) {
        route_segment_t *previous = &s_route.segments[s_route.segment_count - 1];
        if (previous->delta_col == delta_col && previous->delta_row == delta_row && previous->reverse == reverse &&
            fabsf(previous->end_col - start_col) < 0.001f &&
            fabsf(previous->end_row - start_row) < 0.001f) {
            previous->end_col = end_col;
            previous->end_row = end_row;
            return ESP_OK;
        }
    }
    if (s_route.segment_count >= NAV_MAX_SEGMENTS) return ESP_ERR_INVALID_SIZE;
    s_route.segments[s_route.segment_count++] = (route_segment_t){
        .start_col = start_col, .start_row = start_row,
        .end_col = end_col, .end_row = end_row,
        .delta_col = (int8_t)delta_col, .delta_row = (int8_t)delta_row,
        .heading_index = heading,
        .reverse = reverse,
    };
    return ESP_OK;
}

static esp_err_t append_queen_move(float start_col, float start_row,
                                   float end_col, float end_row)
{
    const float dx = end_col - start_col;
    const float dy = end_row - start_row;
    const float diagonal = fminf(fabsf(dx), fabsf(dy));
    float cursor_col = start_col;
    float cursor_row = start_row;
    esp_err_t err = ESP_OK;
    if (diagonal > 0.001f) {
        cursor_col += copysignf(diagonal, dx);
        cursor_row += copysignf(diagonal, dy);
        err = append_segment(start_col, start_row, cursor_col, cursor_row);
    }
    if (err == ESP_OK) err = append_segment(cursor_col, cursor_row, end_col, end_row);
    return err;
}

static void publish_current_segment(void)
{
    s_status.route_segment_count = s_route.segment_count;
    s_status.route_segment_index = s_route.segment_index;
    if (!s_route.valid || s_route.segment_index >= s_route.segment_count) {
        s_status.heading_index = 0;
        s_status.desired_heading_deg = 0;
        s_status.waypoint_col = s_estimator.col;
        s_status.waypoint_row = s_estimator.row;
        return;
    }
    const route_segment_t *segment = &s_route.segments[s_route.segment_index];
    s_status.heading_index = segment->heading_index;
    s_status.desired_heading_deg = wrap_degrees(direction_heading(segment->heading_index) +
                                               (segment->reverse ? 180.0f : 0.0f));
    s_status.waypoint_col = segment->end_col;
    s_status.waypoint_row = segment->end_row;
}

static bool route_geometry_valid_locked(const vision_status_t *vision,
                                        uint16_t *bad_cell, bool *bad_pivot)
{
    if (!vision || !s_route.cells.count) return false;
    float previous_heading = s_estimator.theta_deg;
    for (uint16_t i = 0; i < s_route.segment_count; ++i) {
        const route_segment_t *segment = &s_route.segments[i];
        float heading = direction_heading(segment->heading_index);
        if (segment->reverse) heading = wrap_degrees(heading + 180.0f);
        const float turn_error = angle_error(heading, previous_heading);
        float turn_col = segment->start_col;
        float turn_row = segment->start_row;
        // The controller changes segment when it is within ARRIVAL_CELLS of
        // the waypoint. Validate the earliest plausible turn as well, with a
        // small model-error margin, so a delayed vision frame cannot move the
        // pivot into the border during that transition.
        if (i > 0) {
            const route_segment_t *previous = &s_route.segments[i - 1];
            const float previous_length = hypotf(previous->end_col - previous->start_col,
                                                 previous->end_row - previous->start_row);
            const float margin = fminf(previous_length, ARRIVAL_CELLS + 0.25f);
            if (previous_length > 0.001f) {
                turn_col -= margin * (previous->end_col - previous->start_col) / previous_length;
                turn_row -= margin * (previous->end_row - previous->start_row) / previous_length;
            }
        }
        if (!navigation_turn_inside_mm(turn_col, turn_row, previous_heading, turn_error,
                                       vision->grid_cols, vision->grid_rows,
                                       s_route.cell_mm)) {
            if (bad_cell) {
                const uint16_t cell_index = i + 1U < s_route.cells.count ? i + 1U : i;
                *bad_cell = s_route.cells.cells[cell_index].row * s_route.grid_cols +
                    s_route.cells.cells[cell_index].col;
            }
            if (bad_pivot) *bad_pivot = true;
            return false;
        }
        if (!navigation_segment_inside_mm(segment->start_col, segment->start_row,
                                          segment->end_col, segment->end_row, heading,
                                          vision->grid_cols, vision->grid_rows,
                                          s_route.cell_mm)) {
            if (bad_cell) {
                const uint16_t cell_index = i + 1U < s_route.cells.count ? i + 1U : i;
                *bad_cell = s_route.cells.cells[cell_index].row * s_route.grid_cols +
                    s_route.cells.cells[cell_index].col;
            }
            if (bad_pivot) *bad_pivot = false;
            return false;
        }
        previous_heading = heading;
    }
    return true;
}

static bool try_initial_escape_locked(const vision_status_t *vision,
                                      float target_heading, float *escape_col,
                                      float *escape_row)
{
    const float escape_heading = roundf(s_estimator.theta_deg / 45.0f) * 45.0f;
    const float radians = escape_heading * (float)M_PI / 180.0f;
    const float step_col = cosf(radians), step_row = -sinf(radians);
    const uint8_t cols = (uint8_t)vision->grid_cols;
    const uint8_t rows = (uint8_t)vision->grid_rows;
    for (uint8_t half_steps = 1; half_steps <= 8; ++half_steps) {
        const float distance = 0.5f * half_steps;
        const float candidate_col = s_estimator.col + distance * step_col;
        const float candidate_row = s_estimator.row + distance * step_row;
        const bool segment_safe = navigation_segment_inside_mm(
            s_estimator.col, s_estimator.row, candidate_col, candidate_row,
            escape_heading, cols, rows, s_route.cell_mm);
        bool turn_safe = navigation_turn_inside_mm(
            candidate_col, candidate_row, escape_heading,
            angle_error(target_heading, escape_heading), cols, rows, s_route.cell_mm);
        const float turn_margin = fminf(distance, ARRIVAL_CELLS + 0.25f);
        const float early_col = candidate_col - turn_margin * step_col;
        const float early_row = candidate_row - turn_margin * step_row;
        turn_safe = turn_safe && navigation_turn_inside_mm(
            early_col, early_row, escape_heading,
            angle_error(target_heading, escape_heading), cols, rows, s_route.cell_mm);
        if (!segment_safe || !turn_safe) continue;
        bool occupied = false;
        const int sample_count = (int)ceilf(distance / 0.25f);
        for (int sample = 1; sample <= sample_count; ++sample) {
            const float fraction = (float)sample / sample_count;
            const int cell_col = coordinate_cell(s_estimator.col +
                fraction * distance * step_col, cols);
            const int cell_row = coordinate_cell(s_estimator.row +
                fraction * distance * step_row, rows);
            if (s_occupancy[cell_row * cols + cell_col]) {
                occupied = true;
                break;
            }
        }
        if (occupied) continue;
        *escape_col = candidate_col;
        *escape_row = candidate_row;
        ESP_LOGI(TAG, "Escape inicial seguro %.1f celdas hacia %.0f grados", (double)distance,
                 (double)escape_heading);
        return true;
    }
    ESP_LOGW(TAG, "Escape inicial rechazado para rumbo %.0f desde %.2f,%.2f",
             (double)target_heading, (double)s_estimator.col, (double)s_estimator.row);
    return false;
}

static esp_err_t plan_route_locked(const vision_status_t *vision, bool replanning)
{
    const uint8_t cols = (uint8_t)vision->grid_cols;
    const uint8_t rows = (uint8_t)vision->grid_rows;
    const grid_planner_cell_t start = {
        .col = (uint8_t)(s_confirmed_cell_valid
            ? s_status.confirmed_cell_col : coordinate_cell(s_estimator.col, cols)),
        .row = (uint8_t)(s_confirmed_cell_valid
            ? s_status.confirmed_cell_row : coordinate_cell(s_estimator.row, rows)),
    };
    const grid_planner_cell_t goal = {
        .col = (uint8_t)coordinate_cell(s_status.col, cols),
        .row = (uint8_t)coordinate_cell(s_status.row, rows),
    };
    const float lattice_offset_col = s_estimator.col - start.col;
    const float lattice_offset_row = s_estimator.row - start.row;
    memset(s_route_forbidden, 0, sizeof(s_route_forbidden));
    s_planning_failure_reason = NAVIGATION_FAILURE_NO_ROUTE;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    bool planned = false;
    for (uint8_t attempt = 0; attempt < NAV_ROUTE_REPLAN_ATTEMPTS; ++attempt) {
        err = build_occupancy(vision);
        if (err != ESP_OK) return err;
        for (uint8_t r = 0; r < rows; ++r)
            for (uint8_t c = 0; c < cols; ++c)
                if (s_route_forbidden[r * cols + c]) s_occupancy[r * cols + c] = 1;
        s_occupancy[start.row * cols + start.col] = 0;
        memset(&s_route, 0, sizeof(s_route));
        s_route.grid_cols = cols;
        s_route.grid_rows = rows;
        s_route.cell_mm = navigation_cell_mm(vision->cell_mm);
        err = grid_planner_plan(cols, rows, s_occupancy, start, goal, &s_route.cells);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Ruta candidata %u rechazada: sin camino desde (%u,%u) hacia (%u,%u)",
                     (unsigned)(attempt + 1), start.col, start.row, goal.col, goal.row);
            break;
        }
        float cursor_col = s_estimator.col;
        float cursor_row = s_estimator.row;
        const float first_col = s_route.cells.count > 1
            ? s_route.cells.cells[1].col + lattice_offset_col : cursor_col;
        const float first_row = s_route.cells.count > 1
            ? s_route.cells.cells[1].row + lattice_offset_row : cursor_row;
        if (s_route.cells.count > 1) {
            const float first_heading = direction_heading(direction_index(
                sign_float(first_col - cursor_col), sign_float(first_row - cursor_row)));
            if (!navigation_turn_inside_mm(cursor_col, cursor_row, s_estimator.theta_deg,
                                           angle_error(first_heading, s_estimator.theta_deg),
                                           cols, rows, s_route.cell_mm)) {
                if (try_initial_escape_locked(vision, first_heading, &cursor_col, &cursor_row)) {
                    err = append_segment(s_estimator.col, s_estimator.row, cursor_col, cursor_row);
                } else {
                    const uint16_t blocked = s_route.cells.cells[1].row * cols +
                        s_route.cells.cells[1].col;
                    s_route_forbidden[blocked] = 1;
                    s_planning_failure_reason = NAVIGATION_FAILURE_PIVOT_SWEEP;
                    ESP_LOGW(TAG, "Ruta candidata %u rechazada: giro inicial peligroso; celda %u bloqueada",
                             (unsigned)(attempt + 1), (unsigned)blocked);
                    continue;
                }
            } else err = ESP_OK;
        }
        for (uint16_t i = 1; err == ESP_OK && i < s_route.cells.count; ++i) {
            const float next_col = s_route.cells.cells[i].col + lattice_offset_col;
            const float next_row = s_route.cells.cells[i].row + lattice_offset_row;
            err = append_segment(cursor_col, cursor_row, next_col, next_row);
            cursor_col = next_col;
            cursor_row = next_row;
        }
        if (err == ESP_OK && hypotf(s_status.col - cursor_col, s_status.row - cursor_row) > ARRIVAL_CELLS)
            err = append_queen_move(cursor_col, cursor_row, s_status.col, s_status.row);
        if (err == ESP_OK) {
            uint16_t bad_cell = 0;
            bool bad_pivot = false;
            if (route_geometry_valid_locked(vision, &bad_cell, &bad_pivot)) {
                planned = true;
                break;
            }
            if (bad_cell < (uint16_t)cols * rows &&
                !(bad_cell == (uint16_t)start.row * cols + start.col)) {
                s_route_forbidden[bad_cell] = 1;
                s_planning_failure_reason = bad_pivot ? NAVIGATION_FAILURE_PIVOT_SWEEP
                                                       : NAVIGATION_FAILURE_ROUTE;
                ESP_LOGW(TAG, "Ruta candidata %u rechazada: %s en celda %u; replanificando",
                         (unsigned)(attempt + 1), bad_pivot ? "barrido de pivote" : "envolvente",
                         (unsigned)bad_cell);
                continue;
            }
            s_planning_failure_reason = bad_pivot ? NAVIGATION_FAILURE_PIVOT_SWEEP
                                                   : NAVIGATION_FAILURE_ROUTE;
        } else {
            s_planning_failure_reason = err == ESP_ERR_NOT_FOUND
                ? NAVIGATION_FAILURE_PIVOT_SWEEP : NAVIGATION_FAILURE_ROUTE;
        }
        ESP_LOGW(TAG, "Ruta candidata %u rechazada: error %d",
                 (unsigned)(attempt + 1), (int)err);
        if (s_route.cells.count > 1) {
            const uint16_t blocked = s_route.cells.cells[1].row * cols + s_route.cells.cells[1].col;
            s_route_forbidden[blocked] = 1;
        }
    }
    if (!planned) {
        ESP_LOGW(TAG, "Sin maniobra segura tras %u intentos hacia %.2f, %.2f (motivo %u)",
                 (unsigned)NAV_ROUTE_REPLAN_ATTEMPTS, (double)s_status.col,
                 (double)s_status.row, (unsigned)s_planning_failure_reason);
        return err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
    }
    s_route.valid = true;
    s_route.occupancy_frame_timestamp_ms = vision->frame_timestamp_ms;
    s_status.confirmed_cell_col = start.col;
    s_status.confirmed_cell_row = start.row;
    s_confirmed_cell_valid = true;
    s_confirmed_pattern = s_stable_pattern;
    s_status.wait_reason = NAVIGATION_WAIT_NONE;
    s_alignment_samples = s_misalignment_samples = s_arrival_samples = 0;
    s_slow_pulse_ticks = 0;
    s_progress_ms = (uint64_t)(esp_timer_get_time() / 1000);
    s_progress_col = s_estimator.col;
    s_progress_row = s_estimator.row;
    s_progress_distance = s_route.segment_count
        ? hypotf(s_route.segments[0].end_col - s_estimator.col,
                 s_route.segments[0].end_row - s_estimator.row) : 0.0f;
    s_progress_heading_error = s_route.segment_count
        ? fabsf(angle_error(direction_heading(s_route.segments[0].heading_index),
                            s_estimator.theta_deg)) : 180.0f;
    s_progress_segment = s_route.segment_index;
    if (replanning) ++s_status.replan_count;
    publish_current_segment();
    return ESP_OK;
}

static bool remaining_route_blocked(const vision_status_t *vision)
{
    if (!s_route.valid || build_occupancy(vision) != ESP_OK) return false;
    const uint8_t cols = (uint8_t)vision->grid_cols;
    for (uint16_t i = s_route.cell_index + 1; i < s_route.cells.count; ++i) {
        const grid_planner_cell_t cell = s_route.cells.cells[i];
        if (s_occupancy[cell.row * cols + cell.col]) return true;
    }
    return false;
}

static void update_confirmed_cell(const vision_status_t *vision, bool new_vision_frame)
{
    if (new_vision_frame) {
        if (!vision->grid_cols || !vision->grid_rows) return;
        s_status.confirmed_cell_col = stable_coordinate_cell(
            vision->col, s_status.confirmed_cell_col, (uint8_t)vision->grid_cols);
        s_status.confirmed_cell_row = stable_coordinate_cell(
            vision->row, s_status.confirmed_cell_row, (uint8_t)vision->grid_rows);
        s_confirmed_cell_valid = true;
        s_status.crossings_without_vision = 0;
        s_confirmed_pattern = s_stable_pattern;
        if (!s_status.has_target || !s_route.valid) return;
        for (uint16_t i = s_route.cell_index; i < s_route.cells.count; ++i) {
            const grid_planner_cell_t cell = s_route.cells.cells[i];
            if (cell.col == s_status.confirmed_cell_col &&
                cell.row == s_status.confirmed_cell_row) {
                s_route.cell_index = i;
                break;
            }
        }
        return;
    }
    if (!s_status.has_target || !s_route.valid) return;
    const route_segment_t *segment = s_route.segment_index < s_route.segment_count
        ? &s_route.segments[s_route.segment_index] : NULL;
    if (segment == NULL || !s_status.grid_calibrated ||
        s_stable_pattern_samples < IR_STABLE_SAMPLES || s_stable_pattern == 0 ||
        s_stable_pattern == 0x0f || s_stable_pattern == s_confirmed_pattern) return;
    const int candidate_col = stable_coordinate_cell(
        s_estimator.col, s_status.confirmed_cell_col, (uint8_t)vision->grid_cols);
    const int candidate_row = stable_coordinate_cell(
        s_estimator.row, s_status.confirmed_cell_row, (uint8_t)vision->grid_rows);
    const int delta_col = candidate_col - s_status.confirmed_cell_col;
    const int delta_row = candidate_row - s_status.confirmed_cell_row;
    if (delta_col != segment->delta_col || delta_row != segment->delta_row) return;
    const uint8_t expected = expected_pattern(s_estimator.col, s_estimator.row,
        direction_heading(segment->heading_index) + (segment->reverse ? 180.0f : 0.0f), vision->cell_mm);
    if (expected != s_stable_pattern) return;
    if (s_status.crossings_without_vision >= NAV_MAX_BLIND_CROSSINGS) {
        s_status.phase = NAVIGATION_WAITING_FOR_VISION;
        s_status.wait_reason = NAVIGATION_WAIT_VISION_LIMIT;
        s_estimator.speed_cells_s = 0;
        return;
    }
    s_status.confirmed_cell_col = (int16_t)candidate_col;
    s_status.confirmed_cell_row = (int16_t)candidate_row;
    ++s_status.crossings_without_vision;
    s_confirmed_pattern = s_stable_pattern;
    if (s_route.cell_index + 1 < s_route.cells.count) {
        const grid_planner_cell_t next = s_route.cells.cells[s_route.cell_index + 1];
        if (next.col == candidate_col && next.row == candidate_row) ++s_route.cell_index;
    }
}

static void set_motors_locked(int left, int right)
{
    left = left > MAX_PWM ? MAX_PWM : (left < -MAX_PWM ? -MAX_PWM : left);
    right = right > MAX_PWM ? MAX_PWM : (right < -MAX_PWM ? -MAX_PWM : right);
    if (left > 0 && left < MOTOR_ADAPTER_MIN_COMMAND) left = MOTOR_ADAPTER_MIN_COMMAND;
    if (left < 0 && left > -MOTOR_ADAPTER_MIN_COMMAND) left = -MOTOR_ADAPTER_MIN_COMMAND;
    if (right > 0 && right < MOTOR_ADAPTER_MIN_COMMAND) right = MOTOR_ADAPTER_MIN_COMMAND;
    if (right < 0 && right > -MOTOR_ADAPTER_MIN_COMMAND) right = -MOTOR_ADAPTER_MIN_COMMAND;
    const esp_err_t err = motor_adapter_set((int16_t)left, (int16_t)right);
    if (err == ESP_OK) {
        s_status.motor_left = (int16_t)left;
        s_status.motor_right = (int16_t)right;
        if (left == 0 && right == 0)
            s_estimator.speed_cells_s = 0;
        else if ((left < 0 && right > 0) || (left > 0 && right < 0))
            s_estimator.speed_cells_s *= 0.5f;
    } else {
        motor_adapter_stop();
        s_status.motor_left = s_status.motor_right = 0;
        s_status.phase = NAVIGATION_ERROR;
        s_status.has_target = false;
        s_status.error = err;
        s_status.failure_reason = NAVIGATION_FAILURE_MOTOR;
    }
}

static void reset_progress_monitor_locked(uint64_t now_ms)
{
    s_progress_ms = now_ms;
    s_progress_col = s_estimator.col;
    s_progress_row = s_estimator.row;
    s_progress_distance = 0.0f;
    s_progress_heading_error = 180.0f;
    if (s_route.valid && s_route.segment_index < s_route.segment_count) {
        const route_segment_t *segment = &s_route.segments[s_route.segment_index];
        s_progress_distance = hypotf(segment->end_col - s_estimator.col,
                                     segment->end_row - s_estimator.row);
        float desired = direction_heading(segment->heading_index);
        if (segment->reverse) desired = wrap_degrees(desired + 180.0f);
        s_progress_heading_error = fabsf(angle_error(desired, s_estimator.theta_deg));
    }
    s_progress_segment = s_route.segment_index;
}

static void finish_locked(navigation_phase_t phase, esp_err_t error);

static bool check_progress_locked(uint64_t now_ms)
{
    if (!s_status.has_target || !s_route.valid ||
        (s_status.phase != NAVIGATION_TURNING && s_status.phase != NAVIGATION_DRIVING))
        return true;
    float distance = 0.0f;
    float heading_error_now = 180.0f;
    if (s_route.segment_index < s_route.segment_count) {
        const route_segment_t *segment = &s_route.segments[s_route.segment_index];
        distance = hypotf(segment->end_col - s_estimator.col,
                          segment->end_row - s_estimator.row);
        float desired = direction_heading(segment->heading_index);
        if (segment->reverse) desired = wrap_degrees(desired + 180.0f);
        heading_error_now = fabsf(angle_error(desired, s_estimator.theta_deg));
    }
    const float distance_progress = s_progress_distance - distance;
    const float heading_progress = s_progress_heading_error - heading_error_now;
    if (distance_progress >= 0.08f ||
        (s_status.phase == NAVIGATION_TURNING && heading_progress >= 3.0f) ||
        s_progress_segment != s_route.segment_index) {
        if (s_progress_log_ms == 0 || now_ms - s_progress_log_ms >= 500) {
            ESP_LOGI(TAG, "Progreso observado segmento=%u avance=%.2f error_rumbo=%.1f",
                     (unsigned)s_route.segment_index, (double)distance_progress,
                     (double)heading_progress);
            s_progress_log_ms = now_ms;
        }
        reset_progress_monitor_locked(now_ms);
        return true;
    }
    if (s_progress_ms == 0) reset_progress_monitor_locked(now_ms);
    const uint64_t timeout = s_status.phase == NAVIGATION_TURNING
        ? NAV_TURN_TIMEOUT_MS : NAV_NO_PROGRESS_TIMEOUT_MS;
    if (now_ms - s_progress_ms < timeout) return true;
    if (s_competition_target && s_status.replan_count < NAV_ROUTE_REPLAN_ATTEMPTS) {
        set_motors_locked(0, 0);
        s_route.valid = false;
        s_status.phase = NAVIGATION_REPLANNING;
        s_status.wait_reason = NAVIGATION_WAIT_PATH_OCCUPIED;
        ESP_LOGW(TAG, "Navegacion sin progreso: replanificando antes de bloquear "
                      "intento=%lu", (unsigned long)(s_status.replan_count + 1U));
        reset_progress_monitor_locked(now_ms);
        return false;
    }
    s_status.failure_reason = s_status.phase == NAVIGATION_TURNING
        ? NAVIGATION_FAILURE_TURN_TIMEOUT : NAVIGATION_FAILURE_NO_PROGRESS;
    ESP_LOGW(TAG, "Navegacion bloqueada: %s tras %u ms",
             s_status.phase == NAVIGATION_TURNING ? "timeout de giro" : "falta de progreso",
             (unsigned)timeout);
    finish_locked(NAVIGATION_BLOCKED, ESP_ERR_TIMEOUT);
    return false;
}

static void finish_locked(navigation_phase_t phase, esp_err_t error)
{
    motor_adapter_stop();
    s_status.motor_left = s_status.motor_right = 0;
    s_status.phase = phase;
    s_status.has_target = false;
    s_status.error = error;
    s_route.valid = false;
    memset(&s_route, 0, sizeof(s_route));
    s_status.crossings_without_vision = 0;
    s_status.wait_reason = NAVIGATION_WAIT_NONE;
    s_slow_pulse_ticks = 0;
    s_status.motor_correction_saturated = false;
    s_saturation_samples = 0;
    s_progress_ms = 0;
    s_progress_log_ms = 0;
    publish_current_segment();
}

static void wait_for_vision_locked(navigation_wait_reason_t reason)
{
    set_motors_locked(0, 0);
    s_estimator.speed_cells_s = 0;
    s_slow_pulse_ticks = 0;
    s_status.phase = NAVIGATION_WAITING_FOR_VISION;
    s_status.wait_reason = reason;
}

static void update_controller(const rover_imu_state_t *imu, const rover_sensor_state_t *sensors,
                              const vision_status_t *vision, bool new_vision_frame)
{
    if (!s_status.has_target) return;
    if ((s_competition_target && (app_mode_get() != APP_MODE_COMPETITION ||
         vision->phase != VISION_PHASE_RUNNING || !vision->connected ||
         !vision->protocol_valid || vision->received_ms == 0 ||
         (uint64_t)(esp_timer_get_time() / 1000) - vision->last_valid_frame_ms > VISION_FRESH_MS)) ||
        (!s_competition_target && app_mode_get() != APP_MODE_TEST)) {
        s_status.cancel_reason = NAVIGATION_CANCEL_MODE;
        finish_locked(NAVIGATION_CANCELLED, ESP_ERR_INVALID_STATE);
        return;
    }
    if (!s_estimator.initialized || !imu->valid || !imu->calibration_valid ||
        !sensors->infrared_valid ||
        s_estimator.uncertainty_cells > UNCERTAINTY_LIMIT_CELLS) {
        s_status.failure_reason = !s_estimator.initialized ? NAVIGATION_FAILURE_POSE :
            !imu->valid ? NAVIGATION_FAILURE_IMU :
            !imu->calibration_valid ? NAVIGATION_FAILURE_IMU_CALIBRATION :
            !sensors->infrared_valid ? NAVIGATION_FAILURE_INFRARED :
            NAVIGATION_FAILURE_UNCERTAINTY;
        finish_locked(NAVIGATION_ERROR, ESP_ERR_INVALID_RESPONSE);
        return;
    }
    const bool new_ultrasonic_sample = sensors->timestamp_ms != s_ultrasonic_sample_ms;
    // A missing echo is not a detected obstacle. Only competition can use
    // fresh, validated vision and its collision-checked route as a fallback.
    const bool vision_no_echo = s_competition_target &&
        sensors->ultrasonic_error == ESP_ERR_TIMEOUT && vision->pose_valid &&
        vision->protocol_valid && vision->connected &&
        (uint64_t)(esp_timer_get_time() / 1000) - vision->last_valid_frame_ms <= VISION_FRESH_MS;
    if (!sensors->ultrasonic_valid && !vision_no_echo) {
        set_motors_locked(0, 0);
        s_slow_pulse_ticks = 0;
        if (new_ultrasonic_sample) {
            s_ultrasonic_sample_ms = sensors->timestamp_ms;
            if (s_ultrasonic_invalid_samples < UINT8_MAX) ++s_ultrasonic_invalid_samples;
        }
        if (s_ultrasonic_invalid_samples >= SENSOR_INVALID_SAMPLES) {
            s_status.failure_reason = NAVIGATION_FAILURE_ULTRASONIC;
            finish_locked(NAVIGATION_ERROR, ESP_ERR_INVALID_RESPONSE);
        }
        return;
    }
    if (new_ultrasonic_sample) {
        s_ultrasonic_sample_ms = sensors->timestamp_ms;
        s_ultrasonic_invalid_samples = 0;
        if (sensors->ultrasonic_valid && sensors->distance_mm <= (s_competition_target ? 30U : OBSTACLE_MM)) {
            if (s_obstacle_samples < UINT8_MAX) ++s_obstacle_samples;
        } else {
            s_obstacle_samples = 0;
        }
    }
    if (s_status.phase == NAVIGATION_WAITING_FOR_VISION) {
        set_motors_locked(0, 0);
        if (!new_vision_frame || !vision->pose_valid) return;
        s_status.phase = NAVIGATION_REPLANNING;
        const esp_err_t plan_error = plan_route_locked(vision, true);
        if (plan_error != ESP_OK) {
            s_status.failure_reason = s_planning_failure_reason != NAVIGATION_FAILURE_NONE
                ? s_planning_failure_reason
                : (plan_error == ESP_ERR_NOT_FOUND ? NAVIGATION_FAILURE_NO_ROUTE : NAVIGATION_FAILURE_ROUTE);
            finish_locked(plan_error == ESP_ERR_NOT_FOUND ? NAVIGATION_BLOCKED : NAVIGATION_ERROR,
                          plan_error);
            return;
        }
        s_status.phase = NAVIGATION_TURNING;
    }
    if (!vision->pose_valid && !s_status.grid_calibrated) {
        wait_for_vision_locked(NAVIGATION_WAIT_GRID_UNCALIBRATED);
        return;
    }
    if (!s_route.valid) {
        s_status.phase = NAVIGATION_PLANNING;
        const esp_err_t plan_error = plan_route_locked(vision, false);
        if (plan_error != ESP_OK) {
            s_status.failure_reason = s_planning_failure_reason != NAVIGATION_FAILURE_NONE
                ? s_planning_failure_reason
                : (plan_error == ESP_ERR_NOT_FOUND ? NAVIGATION_FAILURE_NO_ROUTE : NAVIGATION_FAILURE_ROUTE);
            finish_locked(plan_error == ESP_ERR_NOT_FOUND ? NAVIGATION_BLOCKED : NAVIGATION_ERROR,
                          plan_error);
            return;
        }
        s_status.phase = NAVIGATION_TURNING;
    } else if (new_vision_frame && remaining_route_blocked(vision)) {
        set_motors_locked(0, 0);
        s_status.phase = NAVIGATION_REPLANNING;
        s_status.wait_reason = NAVIGATION_WAIT_PATH_OCCUPIED;
        const esp_err_t plan_error = plan_route_locked(vision, true);
        if (plan_error != ESP_OK) {
            s_status.failure_reason = s_planning_failure_reason != NAVIGATION_FAILURE_NONE
                ? s_planning_failure_reason
                : (plan_error == ESP_ERR_NOT_FOUND ? NAVIGATION_FAILURE_NO_ROUTE : NAVIGATION_FAILURE_ROUTE);
            finish_locked(plan_error == ESP_ERR_NOT_FOUND ? NAVIGATION_BLOCKED : NAVIGATION_ERROR,
                          plan_error);
            return;
        }
        s_status.phase = NAVIGATION_TURNING;
    }
    if (s_obstacle_samples >= OBSTACLE_SAMPLES) {
        s_status.failure_reason = NAVIGATION_FAILURE_ULTRASONIC_OBSTACLE;
        finish_locked(NAVIGATION_BLOCKED, ESP_ERR_INVALID_STATE);
        return;
    }
    peer_comms_status_t peer_state = {};
    if (s_competition_target) peer_comms_service_get_status(&peer_state);
    if (s_competition_target && vision->peer_valid &&
        !peer_state.competition_available &&
        hypotf(vision->peer_col - s_estimator.col,
               vision->peer_row - s_estimator.row) < 11.6f) {
        if (!s_route.valid || s_route.segment_index >= s_route.segment_count) {
            set_motors_locked(0, 0);
            return;
        }
        const route_segment_t *next = &s_route.segments[s_route.segment_index];
        const float away = (next->end_col - s_estimator.col) *
            (s_estimator.col - vision->peer_col) +
            (next->end_row - s_estimator.row) *
            (s_estimator.row - vision->peer_row);
        if (away <= 0) { set_motors_locked(0, 0); return; }
    }
    if (s_competition_target && vision->pose_valid &&
        !navigation_pose_inside_mm(vision->col, vision->row, vision->theta_deg,
                                   vision->grid_cols, vision->grid_rows,
                                   vision->cell_mm)) {
        s_status.failure_reason = NAVIGATION_FAILURE_PIVOT_SWEEP;
        ESP_LOGW(TAG, "Navegacion detenida: pose publica fuera de envolvente segura");
        finish_locked(NAVIGATION_BLOCKED, ESP_ERR_INVALID_STATE);
        return;
    }
    if (!check_progress_locked((uint64_t)(esp_timer_get_time() / 1000))) return;
    if (s_status.phase == NAVIGATION_WAITING_FOR_VISION) {
        set_motors_locked(0, 0);
        return;
    }
    if (s_route.segment_index >= s_route.segment_count) {
        const float target_distance = hypotf(s_status.col - s_estimator.col,
                                             s_status.row - s_estimator.row);
        const float observed_speed = fabsf(s_estimator.speed_cells_s);
        set_motors_locked(0, 0);
        s_slow_pulse_ticks = 0;
        if (!new_vision_frame) return;
        if (target_distance <= ARRIVAL_CELLS &&
            observed_speed <= ARRIVAL_MAX_SPEED_CELLS_S) {
            if (s_arrival_samples < UINT8_MAX) ++s_arrival_samples;
            if (s_arrival_samples >= ARRIVAL_SAMPLES)
                finish_locked(NAVIGATION_ARRIVED, ESP_OK);
        } else {
            s_arrival_samples = 0;
            if (target_distance > ARRIVAL_CELLS) {
                s_route.valid = false;
                s_status.phase = NAVIGATION_REPLANNING;
            }
        }
        return;
    }
    route_segment_t *segment = &s_route.segments[s_route.segment_index];
    const float dx = segment->end_col - s_estimator.col;
    const float dy = segment->end_row - s_estimator.row;
    const float distance = hypotf(dx, dy);
    const float segment_dx = segment->end_col - segment->start_col;
    const float segment_dy = segment->end_row - segment->start_row;
    const float segment_length = hypotf(segment_dx, segment_dy);
    const float traveled_dx = s_estimator.col - segment->start_col;
    const float traveled_dy = s_estimator.row - segment->start_row;
    const float along = segment_length > 0
        ? (traveled_dx * segment_dx + traveled_dy * segment_dy) / segment_length : 0;
    const float signed_cross = segment_length > 0
        ? (traveled_dx * segment_dy - traveled_dy * segment_dx) / segment_length : 0;
    const float cross = fabsf(signed_cross);
    s_status.cross_track_cells = signed_cross;
    const bool final_segment = s_route.segment_index + 1 >= s_route.segment_count;
    if ((final_segment && along > segment_length + ARRIVAL_CELLS) || cross > 0.75f) {
        set_motors_locked(0, 0);
        s_slow_pulse_ticks = 0;
        s_route.valid = false;
        s_status.phase = NAVIGATION_REPLANNING;
        return;
    }
    const bool waypoint_reached = distance <= ARRIVAL_CELLS ||
        (!final_segment && along >= segment_length && cross <= 0.5f);
    if (waypoint_reached) {
        if (final_segment) {
            const float observed_speed = fabsf(s_estimator.speed_cells_s);
            set_motors_locked(0, 0);
            s_slow_pulse_ticks = 0;
            if (!new_vision_frame) return;
            if (observed_speed > ARRIVAL_MAX_SPEED_CELLS_S) {
                s_arrival_samples = 0;
                return;
            }
            if (s_arrival_samples < UINT8_MAX) ++s_arrival_samples;
            if (s_arrival_samples >= ARRIVAL_SAMPLES)
                finish_locked(NAVIGATION_ARRIVED, ESP_OK);
            return;
        }
        ++s_route.segment_index;
        s_alignment_samples = s_misalignment_samples = s_arrival_samples = 0;
        s_slow_pulse_ticks = 0;
        s_status.phase = NAVIGATION_TURNING;
        reset_progress_monitor_locked((uint64_t)(esp_timer_get_time() / 1000));
        publish_current_segment();
        return;
    }
    s_arrival_samples = 0;
    float target_heading = direction_heading(segment->heading_index);
    const bool reverse = segment->reverse;
    if (reverse) target_heading = wrap_degrees(target_heading + 180.0f);
    float error = angle_error(target_heading, s_estimator.theta_deg);
    if (s_competition_target &&
        !navigation_turn_inside_mm(vision->pose_valid ? vision->col : s_estimator.col,
                                   vision->pose_valid ? vision->row : s_estimator.row,
                                   vision->pose_valid ? vision->theta_deg : s_estimator.theta_deg,
                                   vision->pose_valid ? angle_error(target_heading, vision->theta_deg) : error,
                                   vision->grid_cols, vision->grid_rows, vision->cell_mm)) {
        const float safety_error = vision->pose_valid
            ? angle_error(target_heading, vision->theta_deg) : error;
        const float alternate_error = safety_error + (safety_error > 0 ? -360.0f : 360.0f);
        if (!navigation_turn_inside_mm(vision->pose_valid ? vision->col : s_estimator.col,
                                       vision->pose_valid ? vision->row : s_estimator.row,
                                       vision->pose_valid ? vision->theta_deg : s_estimator.theta_deg,
                                       alternate_error, vision->grid_cols, vision->grid_rows,
                                       vision->cell_mm)) {
            s_status.failure_reason = NAVIGATION_FAILURE_PIVOT_SWEEP;
            finish_locked(NAVIGATION_BLOCKED, ESP_ERR_INVALID_STATE);
            return;
        }
        error = alternate_error;
    }
    if (s_status.phase == NAVIGATION_DRIVING) {
        if (fabsf(error) > RETURN_TURN_DEG) {
            if (++s_misalignment_samples >= MISALIGNMENT_SAMPLES) {
                s_status.phase = NAVIGATION_TURNING;
                s_alignment_samples = s_misalignment_samples = 0;
                s_slow_pulse_ticks = 0;
            }
        } else s_misalignment_samples = 0;
    } else if (fabsf(error) <= (s_competition_target ? 3.0f : ENTER_DRIVE_DEG) &&
               (!s_competition_target || fabsf(s_estimator.angular_speed_dps) <= 20.0f)) {
        if (++s_alignment_samples >= ALIGNMENT_SAMPLES) {
            s_status.phase = NAVIGATION_DRIVING;
            s_alignment_samples = s_misalignment_samples = 0;
        }
    } else s_alignment_samples = 0;
    if (s_status.phase == NAVIGATION_DRIVING) {
        const float speed = fabsf(s_estimator.speed_cells_s);
        const float brake_distance = s_competition_target
            ? fmaxf(SLOW_DISTANCE_CELLS, speed * speed / 24.0f + speed * 0.15f)
            : SLOW_DISTANCE_CELLS;
        const bool slow = distance <= brake_distance;
        int left, right;
        if (s_competition_target && !slow) {
            bool saturated;
            if (new_vision_frame && vision->pose_valid && fabsf(error) < 5.0f &&
                fabsf(signed_cross) < 0.25f && fabsf(s_estimator.angular_speed_dps) < 15.0f)
                s_motor_trim = fmaxf(-100.0f, fminf(100.0f,
                    s_motor_trim - 0.15f * s_estimator.angular_speed_dps));
            motion_drive_command(error, reverse ? -signed_cross : signed_cross, s_estimator.angular_speed_dps,
                                 s_motor_trim, &left, &right, &saturated);
            s_status.motor_correction_saturated = saturated;
            s_status.motor_trim_pwm = s_motor_trim;
            if (saturated) {
                if (s_saturation_samples < UINT8_MAX) ++s_saturation_samples;
                if (s_saturation_samples >= 10) {
                    set_motors_locked(0, 0);
                    s_route.valid = false;
                    s_status.phase = NAVIGATION_REPLANNING;
                    s_saturation_samples = 0;
                    return;
                }
            } else s_saturation_samples = 0;
        } else {
            s_saturation_samples = 0;
            const int base = slow ? SLOW_PWM : DRIVE_PWM;
            int correction = (int)lroundf(error * 8.0f);
            correction = correction > 150 ? 150 : (correction < -150 ? -150 : correction);
            left = base - correction;
            right = base + correction;
            s_status.motor_correction_saturated = false;
        }
        if (slow) {
            const uint64_t now = (uint64_t)(esp_timer_get_time() / 1000);
            if (new_vision_frame && s_slow_pulse_ticks == 0 && now - s_slow_pulse_ms >= SLOW_PULSE_INTERVAL_MS) {
                s_slow_pulse_ticks = SLOW_PULSE_CONTROL_TICKS;
                s_slow_pulse_ms = now;
            }
            if (s_slow_pulse_ticks == 0) {
                set_motors_locked(0, 0);
                return;
            }
            --s_slow_pulse_ticks;
        } else {
            s_slow_pulse_ticks = 0;
        }
        if (reverse) {
            const int forward_left = left;
            left = -right;
            right = -forward_left;
        }
        set_motors_locked(left, right);
    } else {
        s_status.phase = NAVIGATION_TURNING;
        s_slow_pulse_ticks = 0;
        if (s_competition_target) {
            bool settled;
            const int pwm = motion_turn_pwm(error, s_estimator.angular_speed_dps,
                                            ++s_motion_tick, &settled);
            set_motors_locked(-pwm, pwm);
        } else if (fabsf(error) <= ENTER_DRIVE_DEG) {
            set_motors_locked(0, 0);
        } else {
            set_motors_locked(error > 0 ? -TURN_PWM : TURN_PWM,
                              error > 0 ? TURN_PWM : -TURN_PWM);
        }
    }
}

static void publish_status(const vision_status_t *vision)
{
    s_status.pose_valid = s_estimator.initialized;
    s_status.pose_col = s_estimator.col;
    s_status.pose_row = s_estimator.row;
    s_status.theta_deg = s_estimator.theta_deg;
    s_status.linear_speed_cells_s = s_estimator.speed_cells_s;
    s_status.angular_speed_dps = s_estimator.angular_speed_dps;
    s_status.uncertainty_cells = s_estimator.uncertainty_cells;
    s_status.vision_heading_offset_deg = s_estimator.vision_heading_offset_deg;
    s_status.vision_heading_calibrated = s_estimator.vision_heading_calibrated;
    s_status.vision_configured = vision->configured;
    s_status.vision_connected = vision->connected;
    s_status.vision_fresh = vision->pose_valid;
    s_status.vision_age_ms = vision->age_ms;
    if (!s_status.has_target && s_status.phase == NAVIGATION_WAITING_FOR_POSE && s_estimator.initialized)
        s_status.phase = NAVIGATION_IDLE;
}

// Shared deterministic iteration; the ESP32 task owns scheduling.
static uint64_t s_tick_previous_ms;
static bool s_tick_control, s_tick_pending_vision;
void navigation_service_tick(void)
{
    uint64_t &previous_ms = s_tick_previous_ms;
    bool &control_tick = s_tick_control;
    bool &pending_vision_frame = s_tick_pending_vision;
    rover_imu_state_t imu = {0};
    rover_sensor_state_t sensors = {0};
    vision_status_t vision = {0};
    rover_service_get_imu(&imu);
    rover_service_get_sensors(&sensors);
    vision_service_get_status(&vision);
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    float dt = (float)(now_ms - previous_ms) / 1000.0f;
    if (dt <= 0 || dt > 0.1f) dt = 0.01f;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const app_mode_t mode = app_mode_get();
    const uint32_t generation = app_mode_generation();
    if (generation != s_bias_generation) {
        s_bias_generation = generation;
        s_bias_frozen = false;
        motion_bias_reset(&s_bias_window);
        s_status.competition_bias_valid = false;
        s_status.competition_bias_samples = 0;
        s_status.competition_gyro_bias_dps = 0;
        s_estimator.gyro_bias_dps = 0;
        s_motor_trim = 0;
        s_status.motor_trim_pwm = 0;
    }
    if (mode == APP_MODE_COMPETITION && !s_bias_frozen) {
        if (vision.phase == VISION_PHASE_RUNNING) {
            float bias = 0;
            s_status.competition_bias_valid = motion_bias_result(&s_bias_window, &bias);
            s_status.competition_gyro_bias_dps = s_status.competition_bias_valid ? bias : 0;
            s_status.competition_bias_samples = s_bias_window.count;
            if (s_status.competition_bias_valid) s_estimator.gyro_bias_dps = bias;
            s_bias_frozen = true;
            ESP_LOGI(TAG, "Sesgo Z en competencia: %s, %u muestras, %+.4f dps",
                     s_status.competition_bias_valid ? "valido" : "sin ajuste",
                     (unsigned)s_bias_window.count, (double)s_status.competition_gyro_bias_dps);
        } else if (!s_status.motor_left && !s_status.motor_right && imu.valid &&
                   imu.calibration_valid && vision.phase != VISION_PHASE_FINISHED) {
            motion_bias_add(&s_bias_window, imu.timestamp_ms, imu.sample.accel_g,
                            imu.sample.gyro_dps);
            s_status.competition_bias_samples = s_bias_window.count;
        } else if (s_status.motor_left || s_status.motor_right) {
            motion_bias_reset(&s_bias_window);
            s_status.competition_bias_samples = 0;
        }
    }
    const bool new_vision_frame = update_estimator(&imu, &sensors, &vision, dt, now_ms);
    pending_vision_frame |= new_vision_frame;
    publish_status(&vision);
    update_confirmed_cell(&vision, new_vision_frame);
    control_tick = !control_tick;
    if (control_tick || (s_competition_target && s_status.has_target)) {
        update_controller(&imu, &sensors, &vision, pending_vision_frame);
        pending_vision_frame = false;
    }
    xSemaphoreGive(s_lock);
    previous_ms = now_ms;
}

#ifndef EIRO_HOST_SIM
static void navigation_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    s_tick_previous_ms = (uint64_t)(esp_timer_get_time() / 1000);
    while (true) {
        navigation_service_tick();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(NAV_PERIOD_MS));
    }
}
#endif

esp_err_t navigation_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;
    #ifdef EIRO_HOST_SIM
    return ESP_OK;
    #else
    return xTaskCreatePinnedToCore(navigation_task, "navigation", 7168, NULL, 6,
                                   NULL, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
    #endif
}

static esp_err_t navigation_service_submit_internal(float col, float row,
                                                    uint32_t *request_id, bool competition)
{
    if (s_lock == NULL || app_mode_get() != (competition ? APP_MODE_COMPETITION : APP_MODE_TEST) ||
        !isfinite(col) || !isfinite(row))
        return ESP_ERR_INVALID_STATE;
    vision_status_t vision = {0};
    rover_imu_state_t imu = {0};
    rover_sensor_state_t sensors = {0};
    vision_service_get_status(&vision);
    rover_service_get_imu(&imu);
    rover_service_get_sensors(&sensors);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_estimator.initialized || !vision.connected || !vision.protocol_valid ||
        !vision.pose_valid || !vision.grid_cols || !vision.grid_rows || !imu.valid ||
        !imu.calibration_valid || !sensors.infrared_valid) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (col < 0 || row < 0 || col >= vision.grid_cols || row >= vision.grid_rows) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_status.has_target) {
        motor_adapter_stop();
        s_status.cancel_reason = NAVIGATION_CANCEL_REPLACED;
    }
    s_status.col = col;
    s_status.row = row;
    s_competition_target = competition;
    s_status.request_id = ++s_next_request;
    s_status.has_target = true;
    s_status.phase = NAVIGATION_PLANNING;
    s_status.error = ESP_OK;
    s_status.wait_reason = NAVIGATION_WAIT_NONE;
    s_status.failure_reason = NAVIGATION_FAILURE_NONE;
    s_status.crossings_without_vision = 0;
    s_status.replan_count = 0;
    s_status.route_segment_count = s_status.route_segment_index = 0;
    memset(&s_route, 0, sizeof(s_route));
    s_arrival_samples = s_obstacle_samples = s_ultrasonic_invalid_samples = 0;
    s_ultrasonic_sample_ms = sensors.timestamp_ms;
    s_alignment_samples = s_misalignment_samples = 0;
    s_saturation_samples = 0;
    s_slow_pulse_ticks = 0;
    s_progress_ms = 0;
    s_progress_segment = 0;
    s_planning_failure_reason = NAVIGATION_FAILURE_NONE;
    const uint32_t id = s_status.request_id;
    xSemaphoreGive(s_lock);
    if (request_id != NULL) *request_id = id;
    ESP_LOGI(TAG, "Objetivo #%lu (%.2f, %.2f)", (unsigned long)id, (double)col, (double)row);
    return ESP_OK;
}

esp_err_t navigation_service_submit(float col, float row, uint32_t *request_id)
{
    return navigation_service_submit_internal(col, row, request_id, false);
}

esp_err_t navigation_service_submit_competition(float col, float row, uint32_t *request_id)
{
    if (app_mode_get() != APP_MODE_COMPETITION) return ESP_ERR_INVALID_STATE;
    vision_status_t vision = {0};
    vision_service_get_status(&vision);
    if (vision.phase != VISION_PHASE_RUNNING || !vision.connected ||
        !vision.protocol_valid || !vision.pose_valid || vision.received_ms == 0 ||
        (uint64_t)(esp_timer_get_time() / 1000) - vision.last_valid_frame_ms > VISION_FRESH_MS)
        return ESP_ERR_INVALID_STATE;
    esp_err_t err = navigation_service_submit_internal(col, row, request_id, true);
    return err;
}

void navigation_service_set_competition_cube(uint8_t color, bool allow_contact)
{
    if (s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_competition_cube = color < VISION_MAX_CUBES ? color : UINT8_MAX;
    s_competition_contact = allow_contact;
    xSemaphoreGive(s_lock);
}

esp_err_t navigation_service_cancel(navigation_cancel_reason_t reason)
{
    if (s_lock == NULL) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    motor_adapter_stop();
    s_status.motor_left = s_status.motor_right = 0;
    s_status.has_target = false;
    s_status.phase = NAVIGATION_CANCELLED;
    s_status.cancel_reason = reason;
    s_status.error = ESP_OK;
    s_status.wait_reason = NAVIGATION_WAIT_NONE;
    s_status.failure_reason = NAVIGATION_FAILURE_NONE;
    s_route.valid = false;
    memset(&s_route, 0, sizeof(s_route));
    s_status.crossings_without_vision = 0;
    publish_current_segment();
    s_arrival_samples = s_obstacle_samples = s_ultrasonic_invalid_samples = 0;
    s_slow_pulse_ticks = 0;
    s_progress_ms = 0;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

void navigation_service_get_status(navigation_status_t *status)
{
    if (status == NULL || s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *status = s_status;
    xSemaphoreGive(s_lock);
}

void navigation_service_set_heading_calibration(float offset_deg, bool calibrated)
{
    if (s_lock == NULL) return;
    if (!calibrated || !isfinite(offset_deg)) {
        offset_deg = 0.0f;
        calibrated = false;
    }
    offset_deg = wrap_degrees(offset_deg);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const float change = wrap_degrees(offset_deg - s_estimator.vision_heading_offset_deg);
    s_estimator.vision_heading_offset_deg = offset_deg;
    s_estimator.vision_heading_calibrated = calibrated;
    if (s_estimator.initialized)
        s_estimator.theta_deg = wrap_degrees(s_estimator.theta_deg + change);
    if (fabsf(change) > 0.001f) {
        for (size_t i = 0; i < 4; ++i) {
            s_estimator.sensor[i].polarity_score = 0;
            s_estimator.sensor[i].polarity_known = false;
        }
        s_status.grid_calibrated = false;
    }
    s_status.theta_deg = s_estimator.theta_deg;
    s_status.vision_heading_offset_deg = offset_deg;
    s_status.vision_heading_calibrated = calibrated;
    xSemaphoreGive(s_lock);
}
