#include "navigation_service.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

#include "app_mode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_adapter.h"
#include "rover_service.h"
#include "vision_service.h"

#define NAV_PERIOD_MS 10
#define GRID_MIN_ADC_SPAN 220
#define SENSOR_FRONT_MM 10.0f
#define SENSOR_LEFT_MM 20.0f
#define GRID_SEARCH_CELLS 0.45f
#define GRID_SEARCH_STEP 0.05f
#define UNCERTAINTY_LIMIT_CELLS 25.0f
#define TURN_PWM 700
#define DRIVE_PWM 850
#define SLOW_PWM 700
#define MAX_PWM 1000
#define ENTER_DRIVE_DEG 8.0f
#define RETURN_TURN_DEG 25.0f
#define SLOW_DISTANCE_CELLS 3.0f
#define ARRIVAL_CELLS 0.75f
#define ARRIVAL_SAMPLES 5
#define OBSTACLE_MM 150U
#define OBSTACLE_SAMPLES 3

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
    uint32_t vision_sequence;
    uint64_t vision_ms, grid_ms;
    float vision_col, vision_row;
    uint8_t previous_pattern;
    grid_sensor_t sensor[4];
} estimator_t;

static const char *TAG = "navigation";
static SemaphoreHandle_t s_lock;
static navigation_status_t s_status = {.phase = NAVIGATION_WAITING_FOR_POSE, .core_id = 1};
static estimator_t s_estimator;
static uint32_t s_next_request;
static uint8_t s_arrival_samples, s_obstacle_samples;

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

static bool match_grid(uint8_t observed, float cell_mm, float *col, float *row, float *theta)
{
    if (calibration_mask() != 0x0f || observed == 0 || observed == 0x0f ||
        observed == s_estimator.previous_pattern) return false;
    float best = FLT_MAX, second = FLT_MAX;
    for (float dx = -GRID_SEARCH_CELLS; dx <= GRID_SEARCH_CELLS + 0.001f; dx += GRID_SEARCH_STEP) {
        for (float dy = -GRID_SEARCH_CELLS; dy <= GRID_SEARCH_CELLS + 0.001f; dy += GRID_SEARCH_STEP) {
            for (float da = -8.0f; da <= 8.001f; da += 4.0f) {
                if (expected_pattern(s_estimator.col + dx, s_estimator.row + dy,
                                     s_estimator.theta_deg + da, cell_mm) != observed) continue;
                const float cost = dx * dx + dy * dy + (da / 20.0f) * (da / 20.0f);
                if (cost < best) {
                    second = best;
                    best = cost;
                    *col = s_estimator.col + dx;
                    *row = s_estimator.row + dy;
                    *theta = wrap_degrees(s_estimator.theta_deg + da);
                } else if (cost < second) second = cost;
            }
        }
    }
    return best != FLT_MAX && second - best >= 0.0004f;
}

static void apply_vision(const vision_status_t *vision, uint64_t now_ms)
{
    if (!vision->pose_valid ||
        (s_estimator.vision_ms != 0 && vision->sequence == s_estimator.vision_sequence)) return;
    if (!s_estimator.initialized) {
        s_estimator.initialized = true;
        s_estimator.col = vision->col;
        s_estimator.row = vision->row;
        s_estimator.theta_deg = wrap_degrees(vision->theta_deg);
        s_estimator.uncertainty_cells = 0.25f;
    } else {
        s_estimator.col += 0.25f * (vision->col - s_estimator.col);
        s_estimator.row += 0.25f * (vision->row - s_estimator.row);
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
    s_estimator.vision_sequence = vision->sequence;
    s_status.last_correction = NAVIGATION_CORRECTION_VISION;
}

static void update_estimator(const rover_imu_state_t *imu, const rover_sensor_state_t *sensors,
                             const vision_status_t *vision, float dt, uint64_t now_ms)
{
    const uint8_t observed = sensors->infrared_valid ? classify_ir(&sensors->infrared) : 0;
    if (sensors->infrared_valid) learn_polarity(observed, vision);
    if (s_estimator.initialized && imu->valid) {
        if (!s_status.motor_left && !s_status.motor_right && fabsf(imu->sample.gyro_dps[2]) < 5.0f)
            s_estimator.gyro_bias_dps = 0.995f * s_estimator.gyro_bias_dps +
                                       0.005f * imu->sample.gyro_dps[2];
        s_estimator.angular_speed_dps = imu->sample.gyro_dps[2] - s_estimator.gyro_bias_dps;
        s_estimator.theta_deg = wrap_degrees(s_estimator.theta_deg + s_estimator.angular_speed_dps * dt);
        const float radians = s_estimator.theta_deg * (float)M_PI / 180.0f;
        s_estimator.col += s_estimator.speed_cells_s * cosf(radians) * dt;
        s_estimator.row -= s_estimator.speed_cells_s * sinf(radians) * dt;
        s_estimator.speed_cells_s *= 0.998f;
        s_estimator.uncertainty_cells += vision->pose_valid ? 0.0002f : 0.0015f;
        s_status.last_correction = NAVIGATION_CORRECTION_IMU;
    }
    apply_vision(vision, now_ms);
    s_status.grid_correction_active = false;
    if (s_estimator.initialized && sensors->infrared_valid && vision->cell_mm > 0) {
        float col = 0, row = 0, theta = 0;
        if (match_grid(observed, vision->cell_mm, &col, &row, &theta)) {
            const float distance = hypotf(col - s_estimator.col, row - s_estimator.row);
            s_estimator.col = col;
            s_estimator.row = row;
            s_estimator.theta_deg = theta;
            if (s_estimator.grid_ms && now_ms > s_estimator.grid_ms &&
                s_status.motor_left > 0 && s_status.motor_right > 0) {
                const float event_dt = (float)(now_ms - s_estimator.grid_ms) / 1000.0f;
                if (event_dt > 0.01f && event_dt < 2.0f && distance / event_dt < 30.0f)
                    s_estimator.speed_cells_s = 0.65f * s_estimator.speed_cells_s +
                                                0.35f * distance / event_dt;
            }
            s_estimator.grid_ms = now_ms;
            s_estimator.uncertainty_cells = fmaxf(0.2f, s_estimator.uncertainty_cells * 0.8f);
            if (vision->pose_valid) {
                const float sample = angle_error(theta, vision->theta_deg);
                s_estimator.vision_heading_offset_deg = blend_angle(
                    s_estimator.vision_heading_offset_deg, sample, 0.02f);
            }
            s_status.grid_correction_active = true;
            s_status.last_correction = NAVIGATION_CORRECTION_GRID;
        }
    }
    s_estimator.previous_pattern = observed;
    s_status.infrared_pattern = observed;
    s_status.infrared_calibrated_mask = calibration_mask();
    s_status.grid_calibrated = s_status.infrared_calibrated_mask == 0x0f;
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
        if ((left < 0 && right > 0) || (left > 0 && right < 0))
            s_estimator.speed_cells_s *= 0.5f;
    } else {
        motor_adapter_stop();
        s_status.motor_left = s_status.motor_right = 0;
        s_status.phase = NAVIGATION_ERROR;
        s_status.has_target = false;
        s_status.error = err;
    }
}

static void finish_locked(navigation_phase_t phase, esp_err_t error)
{
    motor_adapter_stop();
    s_status.motor_left = s_status.motor_right = 0;
    s_status.phase = phase;
    s_status.has_target = false;
    s_status.error = error;
}

static void update_controller(const rover_imu_state_t *imu, const rover_sensor_state_t *sensors)
{
    if (!s_status.has_target) return;
    if (app_mode_get() != APP_MODE_TEST) {
        s_status.cancel_reason = NAVIGATION_CANCEL_MODE;
        finish_locked(NAVIGATION_CANCELLED, ESP_ERR_INVALID_STATE);
        return;
    }
    if (!s_estimator.initialized || !imu->valid || !sensors->infrared_valid ||
        s_estimator.uncertainty_cells > UNCERTAINTY_LIMIT_CELLS) {
        finish_locked(NAVIGATION_ERROR, ESP_ERR_INVALID_RESPONSE);
        return;
    }
    if (sensors->ultrasonic_valid && sensors->distance_mm <= OBSTACLE_MM) {
        if (s_obstacle_samples < UINT8_MAX) ++s_obstacle_samples;
    } else s_obstacle_samples = 0;
    if (s_obstacle_samples >= OBSTACLE_SAMPLES) {
        finish_locked(NAVIGATION_BLOCKED, ESP_ERR_INVALID_STATE);
        return;
    }
    const float dx = s_status.col - s_estimator.col;
    const float dy = s_status.row - s_estimator.row;
    const float distance = hypotf(dx, dy);
    if (distance <= ARRIVAL_CELLS) {
        set_motors_locked(0, 0);
        if (++s_arrival_samples >= ARRIVAL_SAMPLES) finish_locked(NAVIGATION_ARRIVED, ESP_OK);
        return;
    }
    s_arrival_samples = 0;
    const float target_heading = atan2f(-dy, dx) * 180.0f / (float)M_PI;
    const float error = angle_error(target_heading, s_estimator.theta_deg);
    if (s_status.phase == NAVIGATION_DRIVING && fabsf(error) > RETURN_TURN_DEG)
        s_status.phase = NAVIGATION_TURNING;
    if (s_status.phase != NAVIGATION_DRIVING && fabsf(error) <= ENTER_DRIVE_DEG)
        s_status.phase = NAVIGATION_DRIVING;
    if (s_status.phase == NAVIGATION_DRIVING) {
        const int base = distance <= SLOW_DISTANCE_CELLS ? SLOW_PWM : DRIVE_PWM;
        int correction = (int)lroundf(error * 8.0f);
        correction = correction > 150 ? 150 : (correction < -150 ? -150 : correction);
        set_motors_locked(base - correction, base + correction);
    } else {
        s_status.phase = NAVIGATION_TURNING;
        set_motors_locked(error > 0 ? -TURN_PWM : TURN_PWM,
                          error > 0 ? TURN_PWM : -TURN_PWM);
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
    s_status.vision_configured = vision->configured;
    s_status.vision_connected = vision->connected;
    s_status.vision_fresh = vision->pose_valid;
    s_status.vision_age_ms = vision->age_ms;
    if (!s_status.has_target && s_status.phase == NAVIGATION_WAITING_FOR_POSE && s_estimator.initialized)
        s_status.phase = NAVIGATION_IDLE;
}

static void navigation_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    uint64_t previous_ms = (uint64_t)(esp_timer_get_time() / 1000);
    bool control_tick = false;
    while (true) {
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
        update_estimator(&imu, &sensors, &vision, dt, now_ms);
        publish_status(&vision);
        control_tick = !control_tick;
        if (control_tick) update_controller(&imu, &sensors);
        xSemaphoreGive(s_lock);
        previous_ms = now_ms;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(NAV_PERIOD_MS));
    }
}

esp_err_t navigation_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;
    return xTaskCreatePinnedToCore(navigation_task, "navigation", 7168, NULL, 6,
                                   NULL, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t navigation_service_submit(float col, float row, uint32_t *request_id)
{
    if (s_lock == NULL || app_mode_get() != APP_MODE_TEST || !isfinite(col) || !isfinite(row))
        return ESP_ERR_INVALID_STATE;
    vision_status_t vision = {0};
    vision_service_get_status(&vision);
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!s_estimator.initialized || !vision.grid_cols || !vision.grid_rows) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (col < 0 || row < 0 || col > vision.grid_cols || row > vision.grid_rows) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_status.has_target) motor_adapter_stop();
    s_status.col = col;
    s_status.row = row;
    s_status.request_id = ++s_next_request;
    s_status.has_target = true;
    s_status.phase = NAVIGATION_TURNING;
    s_status.error = ESP_OK;
    s_arrival_samples = s_obstacle_samples = 0;
    const uint32_t id = s_status.request_id;
    xSemaphoreGive(s_lock);
    if (request_id != NULL) *request_id = id;
    ESP_LOGI(TAG, "Objetivo #%lu (%.2f, %.2f)", (unsigned long)id, (double)col, (double)row);
    return ESP_OK;
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
    s_arrival_samples = s_obstacle_samples = 0;
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
