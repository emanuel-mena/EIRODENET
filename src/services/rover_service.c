#include "rover_service.h"

#include <math.h>
#include <string.h>

#include "color_sensor_adapter.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu_adapter.h"
#include "infrared_adapter.h"
#include "ultrasonic_adapter.h"

#define IMU_PERIOD_MS 10
#define ENVIRONMENT_PERIOD_MS 200
#define CALIBRATION_SAMPLES 200
#define CALIBRATION_SETTLING_SAMPLES 50
#define CAL_ACCEL_STDDEV_MAX_G 0.05f
#define CAL_GYRO_STDDEV_MAX_DPS 1.0f
#define CAL_DOMINANT_MIN_G 0.70f
#define CAL_TRANSVERSE_MAX_G 0.50f
#define CAL_GRAVITY_MIN_G 0.75f
#define CAL_GRAVITY_MAX_G 1.25f
#define DEG_TO_RAD 0.01745329251994329577f

static const char *TAG = "rover_service";
static SemaphoreHandle_t s_lock;
static rover_imu_state_t s_imu;
static rover_sensor_state_t s_sensors;
static rover_calibration_status_t s_cal_status;
static bool s_imu_ready;
static bool s_ultrasonic_ready;
static bool s_infrared_ready;
static bool s_color_ready;
static bool s_reconnecting;
static bool s_reconnect_pending;
static esp_err_t s_reconnect_error = ESP_ERR_INVALID_STATE;
static float s_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

typedef struct {
    double accel_sum[3];
    double accel_sq_sum[3];
    double gyro_sum[3];
    double gyro_sq_sum[3];
    float face_accel[6][3];
    float face_gyro[6][3];
} calibration_work_t;

static calibration_work_t s_cal_work;

static void orientation_reset(void)
{
    s_q[0] = 1.0f;
    s_q[1] = s_q[2] = s_q[3] = 0.0f;
}

static void orientation_update(const float accel[3], const float gyro_dps[3], float dt)
{
    float ax = accel[0], ay = accel[1], az = accel[2];
    const float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.1f || !isfinite(norm)) return;
    ax /= norm; ay /= norm; az /= norm;
    const float q0 = s_q[0], q1 = s_q[1], q2 = s_q[2], q3 = s_q[3];
    const float vx = 2.0f * (q1 * q3 - q0 * q2);
    const float vy = 2.0f * (q0 * q1 + q2 * q3);
    const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    const float kp = 2.5f;
    float gx = gyro_dps[0] * DEG_TO_RAD + kp * (ay * vz - az * vy);
    float gy = gyro_dps[1] * DEG_TO_RAD + kp * (az * vx - ax * vz);
    float gz = gyro_dps[2] * DEG_TO_RAD + kp * (ax * vy - ay * vx);
    s_q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    s_q[1] += 0.5f * ( q0 * gx + q2 * gz - q3 * gy) * dt;
    s_q[2] += 0.5f * ( q0 * gy - q1 * gz + q3 * gx) * dt;
    s_q[3] += 0.5f * ( q0 * gz + q1 * gy - q2 * gx) * dt;
    const float qnorm = sqrtf(s_q[0] * s_q[0] + s_q[1] * s_q[1] +
                              s_q[2] * s_q[2] + s_q[3] * s_q[3]);
    if (qnorm > 0.0f) for (size_t i = 0; i < 4; ++i) s_q[i] /= qnorm;
}

static void apply_calibration(lsm6ds3tr_c_sample_t *sample, const imu_calibration_t *cal)
{
    if (!cal->valid) return;
    for (size_t axis = 0; axis < 3; ++axis) {
        sample->accel_g[axis] =
            (sample->accel_g[axis] - cal->accel_offset_g[axis]) * cal->accel_scale[axis];
        sample->gyro_dps[axis] -= cal->gyro_bias_dps[axis];
    }
}

static void finish_face_locked(void)
{
    const int face = s_cal_status.active_face;
    uint8_t rejection = ROVER_CAL_REJECT_NONE;
    for (size_t axis = 0; axis < 3; ++axis) {
        s_cal_status.accel_mean[axis] =
            (float)(s_cal_work.accel_sum[axis] / CALIBRATION_SAMPLES);
        s_cal_status.gyro_mean[axis] =
            (float)(s_cal_work.gyro_sum[axis] / CALIBRATION_SAMPLES);
        const float accel_var = (float)(s_cal_work.accel_sq_sum[axis] / CALIBRATION_SAMPLES) -
                                s_cal_status.accel_mean[axis] * s_cal_status.accel_mean[axis];
        const float gyro_var = (float)(s_cal_work.gyro_sq_sum[axis] / CALIBRATION_SAMPLES) -
                               s_cal_status.gyro_mean[axis] * s_cal_status.gyro_mean[axis];
        s_cal_status.accel_stddev[axis] = sqrtf(fmaxf(accel_var, 0.0f));
        s_cal_status.gyro_stddev[axis] = sqrtf(fmaxf(gyro_var, 0.0f));
        if (s_cal_status.accel_stddev[axis] > CAL_ACCEL_STDDEV_MAX_G)
            rejection |= ROVER_CAL_REJECT_ACCEL_MOVEMENT;
        if (s_cal_status.gyro_stddev[axis] > CAL_GYRO_STDDEV_MAX_DPS)
            rejection |= ROVER_CAL_REJECT_GYRO_MOVEMENT;
    }
    const int dominant_axis = face / 2;
    const float sign = (face % 2 == 0) ? 1.0f : -1.0f;
    if (sign * s_cal_status.accel_mean[dominant_axis] < CAL_DOMINANT_MIN_G)
        rejection |= ROVER_CAL_REJECT_WRONG_FACE;
    float transverse_sq = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        if (axis != dominant_axis)
            transverse_sq += s_cal_status.accel_mean[axis] * s_cal_status.accel_mean[axis];
    }
    if (sqrtf(transverse_sq) > CAL_TRANSVERSE_MAX_G) rejection |= ROVER_CAL_REJECT_TILT;
    const float norm = sqrtf(
        s_cal_status.accel_mean[0] * s_cal_status.accel_mean[0] +
        s_cal_status.accel_mean[1] * s_cal_status.accel_mean[1] +
        s_cal_status.accel_mean[2] * s_cal_status.accel_mean[2]);
    if (norm < CAL_GRAVITY_MIN_G || norm > CAL_GRAVITY_MAX_G)
        rejection |= ROVER_CAL_REJECT_GRAVITY;
    s_cal_status.rejection_mask = rejection;
    if (rejection == ROVER_CAL_REJECT_NONE) {
        memcpy(s_cal_work.face_accel[face], s_cal_status.accel_mean,
               sizeof(s_cal_status.accel_mean));
        memcpy(s_cal_work.face_gyro[face], s_cal_status.gyro_mean,
               sizeof(s_cal_status.gyro_mean));
        s_cal_status.captured_mask |= (uint8_t)(1U << face);
        s_cal_status.phase = ROVER_CAL_FACE_ACCEPTED;
        s_cal_status.error = ESP_OK;
    } else {
        s_cal_status.phase = ROVER_CAL_ERROR;
        s_cal_status.error = ESP_ERR_INVALID_RESPONSE;
    }
    s_cal_status.active_face = -1;
    s_cal_status.revision++;
}

static void imu_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    int64_t previous_us = esp_timer_get_time();
    while (true) {
        lsm6ds3tr_c_sample_t raw = {0};
        const esp_err_t err = s_imu_ready ? imu_adapter_read_raw(&raw) : ESP_ERR_INVALID_STATE;
        const int64_t now_us = esp_timer_get_time();
        xSemaphoreTake(s_lock, portMAX_DELAY);
        imu_calibration_t cal = {
            .version = IMU_CALIBRATION_VERSION, .accel_scale = {1.0f, 1.0f, 1.0f}};
        imu_adapter_get_calibration(&cal);
        if (err == ESP_OK && s_cal_status.phase == ROVER_CAL_CAPTURING) {
            if (s_cal_status.settling_remaining > 0) {
                s_cal_status.settling_remaining--;
                if ((s_cal_status.settling_remaining % 10U) == 0) s_cal_status.revision++;
            } else {
                for (size_t axis = 0; axis < 3; ++axis) {
                    s_cal_work.accel_sum[axis] += raw.accel_g[axis];
                    s_cal_work.accel_sq_sum[axis] += raw.accel_g[axis] * raw.accel_g[axis];
                    s_cal_work.gyro_sum[axis] += raw.gyro_dps[axis];
                    s_cal_work.gyro_sq_sum[axis] += raw.gyro_dps[axis] * raw.gyro_dps[axis];
                }
                if (++s_cal_status.samples >= CALIBRATION_SAMPLES) finish_face_locked();
                else if ((s_cal_status.samples % 10U) == 0) s_cal_status.revision++;
            }
        }
        if (err == ESP_OK) {
            lsm6ds3tr_c_sample_t corrected = raw;
            apply_calibration(&corrected, &cal);
            float dt = (float)(now_us - previous_us) / 1000000.0f;
            if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
            orientation_update(corrected.accel_g, corrected.gyro_dps, dt);
            s_imu.sample = corrected;
            memcpy(s_imu.quaternion, s_q, sizeof(s_q));
            s_imu.calibration_valid = cal.valid;
            s_imu.valid = true;
            s_imu.error = ESP_OK;
            s_imu.timestamp_ms = (uint64_t)(now_us / 1000);
        } else {
            s_imu.valid = false;
            s_imu.error = err;
        }
        xSemaphoreGive(s_lock);
        previous_us = now_us;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

static void environment_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        rover_sensor_state_t next = {0};
        next.timestamp_ms = (uint64_t)(esp_timer_get_time() / 1000);
        next.ultrasonic_error = s_ultrasonic_ready
            ? ultrasonic_adapter_read_mm(&next.distance_mm) : ESP_ERR_INVALID_STATE;
        next.ultrasonic_valid = next.ultrasonic_error == ESP_OK;
        next.infrared_error = s_infrared_ready
            ? infrared_adapter_read(&next.infrared) : ESP_ERR_INVALID_STATE;
        next.infrared_valid = next.infrared_error == ESP_OK;
        next.color_error = s_color_ready
            ? color_sensor_adapter_read(&next.color) : ESP_ERR_INVALID_STATE;
        next.color_valid = next.color_error == ESP_OK;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_sensors = next;
        xSemaphoreGive(s_lock);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ENVIRONMENT_PERIOD_MS));
    }
}

static void reconnect_task(void *argument)
{
    (void)argument;
    while (true) {
        const esp_err_t err = internet_adapter_reconnect(15000);
        ESP_LOGI(TAG, "Reconexión Wi-Fi: %s", esp_err_to_name(err));
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_reconnect_error = err;
        if (s_reconnect_pending) {
            s_reconnect_pending = false;
            xSemaphoreGive(s_lock);
            continue;
        }
        s_reconnecting = false;
        xSemaphoreGive(s_lock);
        break;
    }
    vTaskDelete(NULL);
}

esp_err_t rover_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;
    s_cal_status.active_face = -1;
    imu_calibration_t cal = {
        .version = IMU_CALIBRATION_VERSION, .accel_scale = {1.0f, 1.0f, 1.0f}};
    if (app_storage_get_imu_calibration(&cal) != ESP_OK) {
        memset(&cal, 0, sizeof(cal));
        cal.version = IMU_CALIBRATION_VERSION;
        for (size_t axis = 0; axis < 3; ++axis) cal.accel_scale[axis] = 1.0f;
    }
    imu_adapter_set_calibration(&cal);
    s_imu_ready = imu_adapter_init() == ESP_OK;
    s_ultrasonic_ready = ultrasonic_adapter_init() == ESP_OK;
    s_infrared_ready = infrared_adapter_init() == ESP_OK;
    s_color_ready = color_sensor_adapter_init() == ESP_OK;
    esp_err_t err = internet_adapter_init();
    if (err == ESP_OK) {
        s_reconnecting = true;
        if (xTaskCreate(reconnect_task, "wifi_connect", 4096, NULL, 4, NULL) != pdPASS) {
            s_reconnecting = false;
            return ESP_ERR_NO_MEM;
        }
    }
    if (xTaskCreate(imu_task, "imu_100hz", 4096, NULL, 6, NULL) != pdPASS ||
        xTaskCreate(environment_task, "sensors_5hz", 4096, NULL, 3, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "IMU=%d ultrasónico=%d IR=%d color=%d", s_imu_ready,
             s_ultrasonic_ready, s_infrared_ready, s_color_ready);
    return ESP_OK;
}

void rover_service_get_imu(rover_imu_state_t *state)
{
    if (state == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY); *state = s_imu; xSemaphoreGive(s_lock);
}

void rover_service_get_sensors(rover_sensor_state_t *state)
{
    if (state == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY); *state = s_sensors; xSemaphoreGive(s_lock);
}

esp_err_t rover_service_get_wifi(internet_adapter_status_t *status)
{
    return internet_adapter_get_status(status);
}

void rover_service_get_network_activity(bool *reconnecting, esp_err_t *last_error)
{
    if (reconnecting == NULL || last_error == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *reconnecting = s_reconnecting;
    *last_error = s_reconnect_error;
    xSemaphoreGive(s_lock);
}

esp_err_t rover_service_set_config(const app_storage_config_t *config, bool *wifi_reconnecting)
{
    if (config == NULL || wifi_reconnecting == NULL) return ESP_ERR_INVALID_ARG;
    app_storage_config_t previous;
    esp_err_t err = app_storage_get_config(&previous);
    if (err != ESP_OK) return err;
    const bool changed = strcmp(previous.wifi_ssid, config->wifi_ssid) != 0 ||
                         strcmp(previous.wifi_password, config->wifi_password) != 0;
    err = app_storage_set_config(config);
    if (err != ESP_OK) return err;
    *wifi_reconnecting = false;
    if (changed) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (!s_reconnecting) {
            s_reconnecting = true;
            s_reconnect_error = ESP_ERR_INVALID_STATE;
            *wifi_reconnecting = xTaskCreate(reconnect_task, "wifi_reconnect", 4096,
                                              NULL, 4, NULL) == pdPASS;
            if (!*wifi_reconnecting) s_reconnecting = false;
        } else {
            s_reconnect_pending = true;
            *wifi_reconnecting = true;
        }
        xSemaphoreGive(s_lock);
    }
    return ESP_OK;
}

esp_err_t rover_service_calibration_start(void)
{
    if (!s_imu_ready) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(&s_cal_work, 0, sizeof(s_cal_work));
    s_cal_status.phase = ROVER_CAL_READY;
    s_cal_status.captured_mask = 0;
    s_cal_status.active_face = -1;
    s_cal_status.samples = 0;
    s_cal_status.settling_remaining = 0;
    s_cal_status.rejection_mask = ROVER_CAL_REJECT_NONE;
    s_cal_status.error = ESP_OK;
    s_cal_status.revision++;
    orientation_reset();
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t rover_service_calibration_capture(uint8_t face)
{
    if (face >= 6) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if ((s_cal_status.phase != ROVER_CAL_READY &&
         s_cal_status.phase != ROVER_CAL_FACE_ACCEPTED &&
         s_cal_status.phase != ROVER_CAL_ERROR) || (s_cal_status.captured_mask & (1U << face))) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        memset(s_cal_work.accel_sum, 0, sizeof(s_cal_work.accel_sum));
        memset(s_cal_work.accel_sq_sum, 0, sizeof(s_cal_work.accel_sq_sum));
        memset(s_cal_work.gyro_sum, 0, sizeof(s_cal_work.gyro_sum));
        memset(s_cal_work.gyro_sq_sum, 0, sizeof(s_cal_work.gyro_sq_sum));
        s_cal_status.phase = ROVER_CAL_CAPTURING;
        s_cal_status.active_face = (int8_t)face;
        s_cal_status.samples = 0;
        s_cal_status.settling_remaining = CALIBRATION_SETTLING_SAMPLES;
        s_cal_status.rejection_mask = ROVER_CAL_REJECT_NONE;
        memset(s_cal_status.accel_mean, 0, sizeof(s_cal_status.accel_mean));
        memset(s_cal_status.accel_stddev, 0, sizeof(s_cal_status.accel_stddev));
        memset(s_cal_status.gyro_mean, 0, sizeof(s_cal_status.gyro_mean));
        memset(s_cal_status.gyro_stddev, 0, sizeof(s_cal_status.gyro_stddev));
        s_cal_status.error = ESP_OK;
        s_cal_status.revision++;
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t rover_service_calibration_commit(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_cal_status.captured_mask != 0x3f || s_cal_status.phase == ROVER_CAL_CAPTURING) {
        xSemaphoreGive(s_lock);
        return ESP_ERR_INVALID_STATE;
    }
    imu_calibration_t cal = {.version = IMU_CALIBRATION_VERSION, .valid = true};
    for (size_t axis = 0; axis < 3; ++axis) {
        const float positive = s_cal_work.face_accel[axis * 2][axis];
        const float negative = s_cal_work.face_accel[axis * 2 + 1][axis];
        const float span = positive - negative;
        if (span < 1.6f) {
            s_cal_status.phase = ROVER_CAL_ERROR;
            s_cal_status.error = ESP_ERR_INVALID_RESPONSE;
            s_cal_status.rejection_mask = ROVER_CAL_REJECT_SPAN;
            s_cal_status.revision++;
            xSemaphoreGive(s_lock);
            return ESP_ERR_INVALID_RESPONSE;
        }
        cal.accel_offset_g[axis] = (positive + negative) * 0.5f;
        cal.accel_scale[axis] = 2.0f / span;
        for (size_t face = 0; face < 6; ++face) cal.gyro_bias_dps[axis] += s_cal_work.face_gyro[face][axis] / 6.0f;
    }
    esp_err_t err = app_storage_set_imu_calibration(&cal);
    if (err == ESP_OK) err = imu_adapter_set_calibration(&cal);
    s_cal_status.phase = err == ESP_OK ? ROVER_CAL_COMPLETE : ROVER_CAL_ERROR;
    s_cal_status.error = err;
    s_cal_status.revision++;
    if (err == ESP_OK) orientation_reset();
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t rover_service_calibration_cancel(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(&s_cal_work, 0, sizeof(s_cal_work));
    s_cal_status.phase = ROVER_CAL_IDLE;
    s_cal_status.captured_mask = 0;
    s_cal_status.active_face = -1;
    s_cal_status.samples = 0;
    s_cal_status.settling_remaining = 0;
    s_cal_status.rejection_mask = ROVER_CAL_REJECT_NONE;
    s_cal_status.error = ESP_OK;
    s_cal_status.revision++;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

void rover_service_get_calibration_status(rover_calibration_status_t *status)
{
    if (status == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY); *status = s_cal_status; xSemaphoreGive(s_lock);
}
