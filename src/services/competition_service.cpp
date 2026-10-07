#include "competition_service.hpp"
#include "competition_runtime.hpp"
#include "competition_strategy.hpp"

#include <math.h>

#include "app_mode.hpp"
#include "app_storage.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_adapter.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"

#define WIFI_WAIT_MS 15000
#define PEER_WAIT_MS 5000
#define PEER_RETRY_MS 200
#define PREFLIGHT_SETTLE_MS 300
#define PREFLIGHT_MAX_YAW_DEG 180.0f

static const char *TAG = "competition";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static competition_status_t s_status;
static competition_role_t s_role = COMPETITION_ROLE_NONE;
static uint64_t s_phase_ms;
static uint64_t s_last_imu_timestamp_ms;
static float s_yaw_integral_deg;
static float s_motor_trim_pwm;
static bool s_have_imu_sample;
static bool s_drive_calibration_running;
static TaskHandle_t s_drive_calibration_task;
static volatile bool s_drive_calibration_cancelled;

static uint64_t now_ms(void) { return (uint64_t)(esp_timer_get_time() / 1000); }

static bool active(uint32_t generation)
{
    return app_mode_get() == APP_MODE_COMPETITION &&
           app_mode_generation() == generation;
}

static float clamp_float(float value, float low, float high)
{
    return fmaxf(low, fminf(high, value));
}

static uint64_t mac_number(const uint8_t mac[6])
{
    uint64_t number = 0;
    for (size_t i = 0; i < 6; ++i) number = (number << 8) | mac[i];
    return number;
}

static void set_preflight_status(competition_preflight_phase_t phase,
                                 uint8_t failed_step, esp_err_t error)
{
    const bool ready = phase == COMPETITION_PREFLIGHT_COMPLETE && failed_step == 0;
    taskENTER_CRITICAL(&s_lock);
    s_status.step = phase == COMPETITION_PREFLIGHT_IDLE ? 0 : 6;
    s_status.failed_step = failed_step;
    s_status.ready = ready;
    s_status.role = ready ? s_role : COMPETITION_ROLE_NONE;
    s_status.error = error;
    s_status.preflight_phase = phase;
    s_status.motor_trim_pwm = s_motor_trim_pwm;
    taskEXIT_CRITICAL(&s_lock);
    app_mode_set_competition_indicator(failed_step, ready);
}

static void publish_verification(uint8_t step, uint8_t failed_step,
                                 competition_role_t role, esp_err_t error)
{
    taskENTER_CRITICAL(&s_lock);
    s_status.step = step;
    s_status.failed_step = failed_step;
    s_status.ready = false;
    s_status.role = role;
    s_status.error = error;
    s_status.preflight_phase = COMPETITION_PREFLIGHT_IDLE;
    s_status.motor_trim_pwm = s_motor_trim_pwm;
    taskEXIT_CRITICAL(&s_lock);
    app_mode_set_competition_indicator(failed_step, false);
    if (failed_step != 0) {
        ESP_LOGE(TAG, "Verificacion fallo en paso %u: %s", failed_step,
                 esp_err_to_name(error));
        motor_adapter_stop();
    } else if (step != 0) {
        ESP_LOGI(TAG, "Verificando paso %u", step);
    }
}

void competition_service_get_status(competition_status_t *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL(&s_lock);
    *status = s_status;
    status->drive_calibration_running = s_drive_calibration_running;
    taskEXIT_CRITICAL(&s_lock);
}

const char *competition_service_status(void)
{
    competition_status_t status = {};
    competition_service_get_status(&status);
    switch (status.failed_step) {
        case 1: return "error_wifi";
        case 2: return "error_server_data";
        case 3: return "error_espnow";
        case 4: return "error_identity";
        case 5: return "error_mac";
        case 6: return "error_preflight";
        default: break;
    }
    if (status.preflight_phase != COMPETITION_PREFLIGHT_IDLE && !status.ready)
        return "preflight";
    if (status.ready)
        return status.role == COMPETITION_ROLE_COMMANDER ? "ready_commander" : "ready_soldier";
    return status.step == 0 ? "idle" : "verifying";
}

static void reset_preflight(void)
{
    motor_adapter_stop();
    s_phase_ms = now_ms();
    s_last_imu_timestamp_ms = 0;
    s_yaw_integral_deg = 0;
    drive_calibration_t calibration = {};
    s_motor_trim_pwm = app_storage_get_drive_calibration(&calibration) == ESP_OK
        ? calibration.motor_trim_pwm : 0.0f;
    s_have_imu_sample = false;
}

static bool sensors_ready(rover_imu_state_t *imu, rover_sensor_state_t *sensors)
{
    rover_service_get_imu(imu);
    rover_service_get_sensors(sensors);
    if (!imu->valid || !imu->calibration_valid || !sensors->infrared_valid ||
        imu->timestamp_ms == 0 || sensors->timestamp_ms == 0) return false;
    for (float rate : imu->sample.gyro_dps)
        if (!isfinite(rate)) return false;
    const uint16_t infrared[4] = {sensors->infrared.front_left,
                                  sensors->infrared.front_right,
                                  sensors->infrared.rear_left,
                                  sensors->infrared.rear_right};
    for (uint16_t value : infrared)
        if (value > 4095U) return false;
    return true;
}

static void integrate_preflight_sample(const rover_imu_state_t *imu)
{
    if (imu == NULL || !imu->valid || !isfinite(imu->sample.gyro_dps[2])) return;
    if (s_last_imu_timestamp_ms != 0 && imu->timestamp_ms > s_last_imu_timestamp_ms) {
        const float dt = clamp_float((float)(imu->timestamp_ms - s_last_imu_timestamp_ms) / 1000.0f,
                                     0.001f, 0.1f);
        s_yaw_integral_deg += imu->sample.gyro_dps[2] * dt;
    }
    s_last_imu_timestamp_ms = imu->timestamp_ms;
    s_have_imu_sample = true;
}

static void fail_preflight(esp_err_t error)
{
    motor_adapter_stop();
    set_preflight_status(COMPETITION_PREFLIGHT_FAILED, 6, error);
}

static void preflight_tick(uint32_t generation)
{
    if (!active(generation)) return;
    const uint64_t now = now_ms();
    rover_imu_state_t imu = {};
    rover_sensor_state_t sensors = {};
    rover_service_get_imu(&imu);
    rover_service_get_sensors(&sensors);
    const bool valid = imu.valid && imu.calibration_valid && imu.timestamp_ms != 0 &&
                       sensors.infrared_valid && sensors.timestamp_ms != 0;
    bool finite_imu = true;
    for (float rate : imu.sample.gyro_dps) finite_imu = finite_imu && isfinite(rate);

    competition_status_t status = {};
    competition_service_get_status(&status);
    if (status.preflight_phase == COMPETITION_PREFLIGHT_FAILED || status.ready) return;
    if (!valid || !finite_imu) {
        motor_adapter_stop();
        if (status.preflight_phase == COMPETITION_PREFLIGHT_WAIT_SENSORS &&
            now - s_phase_ms > PEER_WAIT_MS)
            fail_preflight(ESP_ERR_INVALID_STATE);
        else if (status.preflight_phase != COMPETITION_PREFLIGHT_WAIT_SENSORS)
            fail_preflight(ESP_ERR_INVALID_RESPONSE);
        return;
    }

    switch (status.preflight_phase) {
        case COMPETITION_PREFLIGHT_WAIT_SENSORS:
            motor_adapter_stop();
            if (now - s_phase_ms < PREFLIGHT_SETTLE_MS) return;
            set_preflight_status(COMPETITION_PREFLIGHT_COMPLETE, 0, ESP_OK);
            return;
        default:
            return;
    }
}

static void verify(uint32_t generation)
{
    app_storage_config_t config = {};
    peer_comms_service_reset_verification();
    motor_adapter_stop();
    publish_verification(1, 0, COMPETITION_ROLE_NONE, ESP_OK);
    const int64_t wifi_deadline = (int64_t)now_ms() + WIFI_WAIT_MS;
    internet_adapter_status_t wifi = {};
    bool connected = false;
    if (rover_service_get_wifi(&wifi) != ESP_OK || !wifi.connected || wifi.ipv4_address == 0) {
        const esp_err_t reconnect_err = rover_service_request_wifi_reconnect();
        if (reconnect_err != ESP_OK) {
            publish_verification(1, 1, COMPETITION_ROLE_NONE, reconnect_err);
            return;
        }
    }
    while (active(generation) && (int64_t)now_ms() < wifi_deadline) {
        if (rover_service_get_wifi(&wifi) == ESP_OK && wifi.connected && wifi.ipv4_address != 0) {
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!active(generation)) return;
    if (!connected) {
        publish_verification(1, 1, COMPETITION_ROLE_NONE, ESP_ERR_TIMEOUT);
        return;
    }
    publish_verification(2, 0, COMPETITION_ROLE_NONE, ESP_OK);
    esp_err_t err = app_storage_get_config(&config);
    if (err != ESP_OK || !config.server_configured) {
        publish_verification(2, 2, COMPETITION_ROLE_NONE,
                             err != ESP_OK ? err : ESP_ERR_INVALID_STATE);
        return;
    }
    publish_verification(3, 0, COMPETITION_ROLE_NONE, ESP_OK);
    const int64_t link_deadline = (int64_t)now_ms() + PEER_WAIT_MS;
    peer_comms_status_t peer = {};
    while (active(generation) && (int64_t)now_ms() < link_deadline) {
        peer_comms_service_probe_link();
        peer_comms_service_get_status(&peer);
        if (peer.link_verified) break;
        vTaskDelay(pdMS_TO_TICKS(PEER_RETRY_MS));
    }
    peer_comms_service_get_status(&peer);
    if (!active(generation)) return;
    if (!peer.link_verified) {
        publish_verification(3, 3, COMPETITION_ROLE_NONE, ESP_ERR_TIMEOUT);
        return;
    }
    publish_verification(4, 0, COMPETITION_ROLE_NONE, ESP_OK);
    if (config.who_am_i != APP_STORAGE_ROVER_10 && config.who_am_i != APP_STORAGE_ROVER_11) {
        publish_verification(4, 4, COMPETITION_ROLE_NONE, ESP_ERR_INVALID_STATE);
        return;
    }
    const int64_t identity_deadline = (int64_t)now_ms() + PEER_WAIT_MS;
    while (active(generation) && (int64_t)now_ms() < identity_deadline) {
        peer_comms_service_query_identity();
        peer_comms_service_get_status(&peer);
        if (peer.identity_received) break;
        vTaskDelay(pdMS_TO_TICKS(PEER_RETRY_MS));
    }
    peer_comms_service_get_status(&peer);
    if (!active(generation)) return;
    if (!peer.identity_received ||
        (peer.verified_identity != APP_STORAGE_ROVER_10 &&
         peer.verified_identity != APP_STORAGE_ROVER_11) ||
        peer.verified_identity == config.who_am_i) {
        publish_verification(4, 4, COMPETITION_ROLE_NONE,
                             peer.identity_received ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT);
        return;
    }
    publish_verification(5, 0, COMPETITION_ROLE_NONE, ESP_OK);
    uint8_t own_mac[6] = {0};
    err = esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK || !config.peer_configured ||
        mac_number(own_mac) == mac_number(config.peer_mac)) {
        publish_verification(5, 5, COMPETITION_ROLE_NONE,
                             err != ESP_OK ? err : ESP_ERR_INVALID_ARG);
        return;
    }
    s_role = mac_number(own_mac) > mac_number(config.peer_mac)
        ? COMPETITION_ROLE_COMMANDER : COMPETITION_ROLE_SOLDIER;
    reset_preflight();
    set_preflight_status(COMPETITION_PREFLIGHT_WAIT_SENSORS, 0, ESP_OK);
}

static void competition_task(void *argument)
{
    (void)argument;
    uint32_t handled_generation = UINT32_MAX;
    while (true) {
        const app_mode_t mode = app_mode_get();
        const uint32_t generation = app_mode_generation();
        if (mode == APP_MODE_COMPETITION && generation != handled_generation) {
            handled_generation = generation;
            competition_runtime_reset();
            s_role = COMPETITION_ROLE_NONE;
            reset_preflight();
            publish_verification(0, 0, COMPETITION_ROLE_NONE, ESP_OK);
            verify(generation);
        } else if (mode == APP_MODE_TEST && generation != handled_generation) {
            handled_generation = generation;
            competition_runtime_reset();
            peer_comms_service_reset_verification();
            reset_preflight();
            publish_verification(0, 0, COMPETITION_ROLE_NONE, ESP_OK);
        } else if (mode == APP_MODE_COMPETITION) {
            competition_status_t status = {};
            competition_service_get_status(&status);
            if (status.preflight_phase != COMPETITION_PREFLIGHT_COMPLETE &&
                status.preflight_phase != COMPETITION_PREFLIGHT_FAILED &&
                status.preflight_phase != COMPETITION_PREFLIGHT_IDLE) {
                preflight_tick(generation);
            } else if (status.ready && !status.failed_step) {
                vision_status_t vision = {};
                vision_service_get_status(&vision);
                if (vision.phase == VISION_PHASE_READY || vision.phase == VISION_PHASE_RUNNING)
                    competition_runtime_tick(status.role, generation);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t competition_service_start(void)
{
    return xTaskCreate(competition_task, "competition", 12288, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

static void drive_calibration_task(void *argument)
{
    (void)argument;
    const uint32_t generation = app_mode_generation();
    motor_adapter_stop();
    vTaskDelay(pdMS_TO_TICKS(PREFLIGHT_SETTLE_MS));
    if (s_drive_calibration_cancelled || app_mode_get() != APP_MODE_TEST ||
        app_mode_generation() != generation) {
        motor_adapter_stop();
        s_drive_calibration_running = false;
        s_drive_calibration_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    rover_imu_state_t imu = {};
    rover_sensor_state_t sensors = {};
    rover_service_get_imu(&imu);
    rover_service_get_sensors(&sensors);
    const bool valid = app_mode_get() == APP_MODE_TEST && imu.valid &&
        imu.calibration_valid && sensors.infrared_valid && imu.timestamp_ms != 0 &&
        sensors.timestamp_ms != 0;
    const uint64_t start = now_ms();
    s_yaw_integral_deg = 0;
    s_last_imu_timestamp_ms = valid ? imu.timestamp_ms : 0;
    s_have_imu_sample = false;
    esp_err_t err = valid ? motor_adapter_set(COMPETITION_FULL_PWM, COMPETITION_FULL_PWM)
                          : ESP_ERR_INVALID_STATE;
    while (err == ESP_OK && !s_drive_calibration_cancelled &&
           app_mode_get() == APP_MODE_TEST && app_mode_generation() == generation &&
           now_ms() - start < 400U) {
        rover_service_get_imu(&imu);
        if (!imu.valid || !imu.calibration_valid || !isfinite(imu.sample.gyro_dps[2])) {
            err = ESP_ERR_INVALID_RESPONSE;
            break;
        }
        integrate_preflight_sample(&imu);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motor_adapter_stop();
    if (err == ESP_OK && !s_drive_calibration_cancelled &&
        app_mode_get() == APP_MODE_TEST &&
        app_mode_generation() == generation && s_have_imu_sample &&
        isfinite(s_yaw_integral_deg) && fabsf(s_yaw_integral_deg) <= PREFLIGHT_MAX_YAW_DEG) {
        drive_calibration_t calibration = {
            .version = DRIVE_CALIBRATION_VERSION,
            .valid = true,
            .motor_trim_pwm = competition_trim_from_yaw(s_yaw_integral_deg),
        };
        err = app_storage_set_drive_calibration(&calibration);
        if (err == ESP_OK) {
            s_motor_trim_pwm = calibration.motor_trim_pwm;
            taskENTER_CRITICAL(&s_lock);
            s_status.motor_trim_pwm = s_motor_trim_pwm;
            taskEXIT_CRITICAL(&s_lock);
        }
    } else if (err == ESP_OK) err = ESP_ERR_INVALID_RESPONSE;
    if (err != ESP_OK) ESP_LOGW(TAG, "Calibración de motores no guardada: %s",
                                esp_err_to_name(err));
    s_drive_calibration_running = false;
    s_drive_calibration_task = NULL;
    taskENTER_CRITICAL(&s_lock);
    s_status.drive_calibration_running = false;
    taskEXIT_CRITICAL(&s_lock);
    vTaskDelete(NULL);
}

esp_err_t competition_service_drive_calibration_start(void)
{
    if (app_mode_get() != APP_MODE_TEST || s_drive_calibration_running)
        return ESP_ERR_INVALID_STATE;
    navigation_service_cancel(NAVIGATION_CANCEL_MANUAL);
    s_drive_calibration_running = true;
    taskENTER_CRITICAL(&s_lock);
    s_status.drive_calibration_running = true;
    taskEXIT_CRITICAL(&s_lock);
    s_drive_calibration_cancelled = false;
    if (xTaskCreate(drive_calibration_task, "drive_cal", 3072, NULL, 4,
                    &s_drive_calibration_task) != pdPASS) {
        s_drive_calibration_running = false;
        taskENTER_CRITICAL(&s_lock);
        s_status.drive_calibration_running = false;
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t competition_service_drive_calibration_stop(void)
{
    if (!s_drive_calibration_running) return ESP_ERR_INVALID_STATE;
    s_drive_calibration_cancelled = true;
    motor_adapter_stop();
    return ESP_OK;
}

bool competition_service_drive_calibration_running(void)
{
    return s_drive_calibration_running;
}
