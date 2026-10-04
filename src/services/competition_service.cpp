#include "competition_service.hpp"
#include "competition_runtime.hpp"

#include "app_mode.hpp"
#include "app_storage.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heading_calibration.hpp"
#include "motor_adapter.hpp"
#include "navigation_service.hpp"
#include "peer_comms_service.hpp"
#include "rover_service.hpp"
#include "vision_service.hpp"

#define WIFI_WAIT_MS 15000
#define SERVER_DATA_WAIT_MS 5000
#define PEER_WAIT_MS 5000
#define PEER_RETRY_MS 200
#define HEADING_CALIBRATION_WAIT_MS 3000

static const char *TAG = "competition";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static competition_status_t s_status;

static void publish(uint8_t step, uint8_t failed_step, bool ready,
                    competition_role_t role, esp_err_t error)
{
    taskENTER_CRITICAL(&s_lock);
    s_status = (competition_status_t){
        .step = step, .failed_step = failed_step, .ready = ready,
        .role = role, .error = error,
    };
    taskEXIT_CRITICAL(&s_lock);
    app_mode_set_competition_indicator(failed_step, ready);
    if (failed_step != 0) {
        ESP_LOGE(TAG, "Verificacion fallo en paso %u: %s", failed_step, esp_err_to_name(error));
        motor_adapter_stop();
    } else if (ready) {
        ESP_LOGI(TAG, "Verificacion completa: %s", role == COMPETITION_ROLE_COMMANDER
                 ? "comandante" : "soldado");
    } else if (step != 0) {
        ESP_LOGI(TAG, "Verificando paso %u", step);
    }
}

void competition_service_get_status(competition_status_t *status)
{
    if (status == NULL) return;
    taskENTER_CRITICAL(&s_lock);
    *status = s_status;
    taskEXIT_CRITICAL(&s_lock);
}

const char *competition_service_status(void)
{
    competition_status_t status;
    competition_service_get_status(&status);
    if (status.failed_step == 1) return "error_wifi";
    if (status.failed_step == 2) return "error_server_data";
    if (status.failed_step == 3) return "error_espnow";
    if (status.failed_step == 4) return "error_identity";
    if (status.failed_step == 5) return "error_mac";
    if (status.ready) return status.role == COMPETITION_ROLE_COMMANDER
        ? "ready_commander" : "ready_soldier";
    return status.step == 0 ? "idle" : "verifying";
}

static bool active(uint32_t generation)
{
    return app_mode_get() == APP_MODE_COMPETITION &&
        app_mode_generation() == generation;
}

static uint64_t mac_number(const uint8_t mac[6])
{
    uint64_t number = 0;
    for (size_t i = 0; i < 6; ++i) number = (number << 8) | mac[i];
    return number;
}

static void verify(uint32_t generation)
{
    app_storage_config_t config = {0};
    peer_comms_service_reset_verification();
    motor_adapter_stop();
    publish(1, 0, false, COMPETITION_ROLE_NONE, ESP_OK);
    const int64_t wifi_deadline = esp_timer_get_time() / 1000 + WIFI_WAIT_MS;
    internet_adapter_status_t wifi = {0};
    bool connected = false;
    if (rover_service_get_wifi(&wifi) != ESP_OK ||
        !wifi.connected || wifi.ipv4_address == 0) {
        const esp_err_t reconnect_err = rover_service_request_wifi_reconnect();
        if (reconnect_err != ESP_OK) {
            publish(1, 1, false, COMPETITION_ROLE_NONE, reconnect_err);
            return;
        }
    }
    while (active(generation) && esp_timer_get_time() / 1000 < wifi_deadline) {
        if (rover_service_get_wifi(&wifi) == ESP_OK &&
            wifi.connected && wifi.ipv4_address != 0) {
            connected = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!active(generation)) return;
    if (!connected) { publish(1, 1, false, COMPETITION_ROLE_NONE, ESP_ERR_TIMEOUT); return; }

    publish(2, 0, false, COMPETITION_ROLE_NONE, ESP_OK);
    esp_err_t err = app_storage_get_config(&config);
    if (err != ESP_OK) {
        publish(2, 2, false, COMPETITION_ROLE_NONE, err);
        return;
    }
    const uint64_t server_start_ms = (uint64_t)(esp_timer_get_time() / 1000);
    const uint64_t server_deadline_ms = server_start_ms + SERVER_DATA_WAIT_MS;
    vision_status_t vision = {0};
    bool receiving = false;
    while (active(generation) && (uint64_t)(esp_timer_get_time() / 1000) < server_deadline_ms) {
        vision_service_get_status(&vision);
        if (vision.connected && vision.protocol_valid &&
            vision.last_valid_frame_ms > server_start_ms) {
            receiving = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!active(generation)) return;
    if (!receiving) {
        publish(2, 2, false, COMPETITION_ROLE_NONE,
                vision.last_error == ESP_ERR_INVALID_RESPONSE
                    ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT);
        return;
    }

    publish(3, 0, false, COMPETITION_ROLE_NONE, ESP_OK);
    const int64_t link_deadline = esp_timer_get_time() / 1000 + PEER_WAIT_MS;
    peer_comms_status_t peer = {0};
    while (active(generation) && esp_timer_get_time() / 1000 < link_deadline) {
        peer_comms_service_probe_link();
        peer_comms_service_get_status(&peer);
        if (peer.link_verified) break;
        vTaskDelay(pdMS_TO_TICKS(PEER_RETRY_MS));
    }
    if (!active(generation)) return;
    peer_comms_service_get_status(&peer);
    if (!peer.link_verified) { publish(3, 3, false, COMPETITION_ROLE_NONE, ESP_ERR_TIMEOUT); return; }

    publish(4, 0, false, COMPETITION_ROLE_NONE, ESP_OK);
    if (config.who_am_i != APP_STORAGE_ROVER_10 && config.who_am_i != APP_STORAGE_ROVER_11) {
        publish(4, 4, false, COMPETITION_ROLE_NONE, ESP_ERR_INVALID_STATE);
        return;
    }
    const int64_t identity_deadline = esp_timer_get_time() / 1000 + PEER_WAIT_MS;
    while (active(generation) && esp_timer_get_time() / 1000 < identity_deadline) {
        peer_comms_service_query_identity();
        peer_comms_service_get_status(&peer);
        if (peer.identity_received) break;
        vTaskDelay(pdMS_TO_TICKS(PEER_RETRY_MS));
    }
    if (!active(generation)) return;
    peer_comms_service_get_status(&peer);
    if (!peer.identity_received ||
        (peer.verified_identity != APP_STORAGE_ROVER_10 &&
         peer.verified_identity != APP_STORAGE_ROVER_11) ||
        peer.verified_identity == config.who_am_i) {
        publish(4, 4, false, COMPETITION_ROLE_NONE,
                peer.identity_received ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT);
        return;
    }

    publish(5, 0, false, COMPETITION_ROLE_NONE, ESP_OK);
    uint8_t own_mac[6] = {0};
    err = esp_read_mac(own_mac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK || !config.peer_configured ||
        mac_number(own_mac) == mac_number(config.peer_mac)) {
        publish(5, 5, false, COMPETITION_ROLE_NONE,
                err != ESP_OK ? err : ESP_ERR_INVALID_ARG);
        return;
    }
    if (!active(generation)) return;
    publish(5, 0, true, mac_number(own_mac) > mac_number(config.peer_mac)
            ? COMPETITION_ROLE_COMMANDER : COMPETITION_ROLE_SOLDIER, ESP_OK);
}

static void calibrate_heading(uint32_t generation)
{
    heading_calibration_sample_t samples[HEADING_CALIBRATION_SAMPLES] = {0};
    uint8_t count = 0;
    const int64_t deadline_ms = esp_timer_get_time() / 1000 + HEADING_CALIBRATION_WAIT_MS;
    motor_adapter_stop();
    while (active(generation) && esp_timer_get_time() / 1000 < deadline_ms) {
        vision_status_t vision = {0};
        vision_service_get_status(&vision);
        if (vision.phase != VISION_PHASE_READY) break;
        if (vision.connected && vision.protocol_valid && vision.pose_valid &&
            vision.age_ms <= HEADING_CALIBRATION_MAX_AGE_MS &&
            vision.frame_timestamp_ms != 0 &&
            (count == 0 || vision.frame_timestamp_ms > samples[count - 1].frame_timestamp_ms)) {
            samples[count++] = (heading_calibration_sample_t){
                .frame_timestamp_ms = vision.frame_timestamp_ms,
                .age_ms = vision.age_ms,
                .col = vision.col,
                .row = vision.row,
                .theta_deg = vision.theta_deg,
            };
            if (count == HEADING_CALIBRATION_SAMPLES) break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (!active(generation)) return;
    vision_status_t latest = {0};
    vision_service_get_status(&latest);
    float offset_deg = 0.0f;
    const bool calibrated = latest.phase == VISION_PHASE_READY &&
        count == HEADING_CALIBRATION_SAMPLES &&
        heading_calibration_calculate(samples, &offset_deg);
    navigation_service_set_heading_calibration(offset_deg, calibrated);
    app_mode_set_heading_uncalibrated(!calibrated);
    if (calibrated)
        ESP_LOGI(TAG, "Rumbo calibrado con 5 capturas: desfase %+.2f grados", (double)offset_deg);
    else
        ESP_LOGW(TAG, "Rumbo sin calibrar (%u/5 capturas); se usa theta del servidor",
                 (unsigned)count);
}

static void competition_task(void *argument)
{
    (void)argument;
    uint32_t handled_generation = UINT32_MAX;
    bool heading_attempted = false;
    while (true) {
        const app_mode_t mode = app_mode_get();
        const uint32_t generation = app_mode_generation();
        if (mode == APP_MODE_COMPETITION && generation != handled_generation) {
            handled_generation = generation;
            heading_attempted = false;
            navigation_service_set_heading_calibration(0.0f, false);
            app_mode_set_heading_uncalibrated(false);
            verify(generation);
        } else if (mode == APP_MODE_TEST && generation != handled_generation) {
            handled_generation = generation;
            heading_attempted = false;
            navigation_service_set_heading_calibration(0.0f, false);
            app_mode_set_heading_uncalibrated(false);
            competition_runtime_reset();
            peer_comms_service_reset_verification();
            publish(0, 0, false, COMPETITION_ROLE_NONE, ESP_OK);
        } else if (mode == APP_MODE_COMPETITION) {
            competition_status_t status = {0};
            competition_service_get_status(&status);
            if (status.ready && !status.failed_step) {
                vision_status_t vision = {0};
                vision_service_get_status(&vision);
                if (!heading_attempted && vision.phase == VISION_PHASE_READY) {
                    heading_attempted = true;
                    calibrate_heading(generation);
                } else if (!heading_attempted && vision.phase == VISION_PHASE_RUNNING) {
                    heading_attempted = true;
                    app_mode_set_heading_uncalibrated(true);
                    ESP_LOGW(TAG, "READY termino antes de calibrar; se usa theta del servidor");
                }
                if (heading_attempted && active(generation))
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
