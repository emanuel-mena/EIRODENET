#include "serial_protocol.hpp"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "app_storage.hpp"
#include "cJSON.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "motor_adapter.hpp"
#include "navigation_service.hpp"
#include "rover_service.hpp"

#define PROTOCOL_PREFIX "@EIRO "
#define PROTOCOL_VERSION 1
#define MAX_LINE_LENGTH 1024

static const char *TAG = "serial_protocol";
static bool s_streaming;
static uint32_t s_sequence;
static TaskHandle_t s_motor_test_task;

static void motor_test_task(void *argument)
{
    (void)argument;
    navigation_service_cancel(NAVIGATION_CANCEL_MANUAL);
    const esp_err_t err = motor_adapter_test_forward();
    if (err != ESP_OK) ESP_LOGE(TAG, "Prueba de motores: %s", esp_err_to_name(err));
    s_motor_test_task = NULL;
    vTaskDelete(NULL);
}

static void transmit_json(cJSON *message)
{
    char *json = cJSON_PrintUnformatted(message);
    if (json != NULL) {
        printf(PROTOCOL_PREFIX "%s\n", json);
        fflush(stdout);
        cJSON_free(json);
    }
    cJSON_Delete(message);
}

static cJSON *base_message(const char *type)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "v", PROTOCOL_VERSION);
    cJSON_AddStringToObject(root, "type", type);
    return root;
}

static void respond_error(double id, const char *code, esp_err_t error)
{
    cJSON *root = base_message("response");
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON *body = cJSON_AddObjectToObject(root, "error");
    cJSON_AddStringToObject(body, "code", code);
    cJSON_AddStringToObject(body, "message", esp_err_to_name(error));
    transmit_json(root);
}

static void respond_ok(double id, cJSON *data)
{
    cJSON *root = base_message("response");
    cJSON_AddNumberToObject(root, "id", id);
    cJSON_AddBoolToObject(root, "ok", true);
    if (data != NULL) cJSON_AddItemToObject(root, "data", data);
    transmit_json(root);
}

static bool has_only(const cJSON *object, const char *const *allowed, size_t count)
{
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, object) {
        bool found = false;
        for (size_t i = 0; i < count; ++i) found |= strcmp(item->string, allowed[i]) == 0;
        if (!found) return false;
    }
    return true;
}

static void add_vec3(cJSON *parent, const char *name, const float value[3])
{
    cJSON *array = cJSON_AddArrayToObject(parent, name);
    for (size_t i = 0; i < 3; ++i) cJSON_AddItemToArray(array, cJSON_CreateNumber(value[i]));
}

static bool parse_mac(const char *text, uint8_t mac[6])
{
    unsigned int bytes[6];
    char tail;
    if (text == NULL || sscanf(text, "%2x:%2x:%2x:%2x:%2x:%2x%c", &bytes[0], &bytes[1],
        &bytes[2], &bytes[3], &bytes[4], &bytes[5], &tail) != 6) return false;
    for (size_t i = 0; i < 6; ++i) mac[i] = (uint8_t)bytes[i];
    return true;
}

static void handle_config_get(double id)
{
    app_storage_config_t config;
    esp_err_t err = app_storage_get_config(&config);
    if (err != ESP_OK) { respond_error(id, "storage_error", err); return; }
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "who_am_i", config.who_am_i);
    cJSON_AddStringToObject(data, "local_site", config.local_site);
    cJSON_AddStringToObject(data, "wifi_ssid", config.wifi_ssid);
    cJSON_AddStringToObject(data, "wifi_password", config.wifi_password);
    cJSON_AddStringToObject(data, "server_ipv4", config.server_configured ? config.server_ipv4 : "");
    cJSON_AddNumberToObject(data, "server_port", config.server_configured ? config.server_port : 0);
    char mac[18] = "";
    if (config.peer_configured) snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
        config.peer_mac[0], config.peer_mac[1], config.peer_mac[2], config.peer_mac[3],
        config.peer_mac[4], config.peer_mac[5]);
    cJSON_AddStringToObject(data, "peer_mac", mac);
    respond_ok(id, data);
}

static void handle_config_set(double id, const cJSON *request)
{
    static const char *const root_allowed[] = {"v", "id", "cmd", "data"};
    static const char *const data_allowed[] = {
        "who_am_i", "local_site", "wifi_ssid", "wifi_password", "server_ipv4", "server_port", "peer_mac"};
    const cJSON *data = cJSON_GetObjectItemCaseSensitive(request, "data");
    if (!has_only(request, root_allowed, 4) || !cJSON_IsObject(data) ||
        !has_only(data, data_allowed, 7)) {
        respond_error(id, "invalid_fields", ESP_ERR_INVALID_ARG); return;
    }
    const cJSON *who_am_i = cJSON_GetObjectItemCaseSensitive(data, "who_am_i");
    const cJSON *local_site = cJSON_GetObjectItemCaseSensitive(data, "local_site");
    const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(data, "wifi_ssid");
    const cJSON *password = cJSON_GetObjectItemCaseSensitive(data, "wifi_password");
    const cJSON *server = cJSON_GetObjectItemCaseSensitive(data, "server_ipv4");
    const cJSON *port = cJSON_GetObjectItemCaseSensitive(data, "server_port");
    const cJSON *peer = cJSON_GetObjectItemCaseSensitive(data, "peer_mac");
    if (!cJSON_IsString(local_site) || !cJSON_IsString(ssid) || !cJSON_IsString(password) || !cJSON_IsString(server) ||
        !cJSON_IsNumber(port) || port->valuedouble != port->valueint || !cJSON_IsString(peer)) {
        respond_error(id, "invalid_config", ESP_ERR_INVALID_ARG); return;
    }
    if (who_am_i != NULL && (!cJSON_IsNumber(who_am_i) ||
        who_am_i->valuedouble != who_am_i->valueint ||
        (who_am_i->valueint != APP_STORAGE_ROVER_UNCONFIGURED &&
         who_am_i->valueint != APP_STORAGE_ROVER_10 &&
         who_am_i->valueint != APP_STORAGE_ROVER_11))) {
        respond_error(id, "invalid_config", ESP_ERR_INVALID_ARG); return;
    }
    app_storage_config_t config;
    esp_err_t err = app_storage_get_config(&config);
    if (err != ESP_OK) { respond_error(id, "storage_error", err); return; }
    const uint8_t stored_identity = config.who_am_i;
    memset(&config, 0, sizeof(config));
    config.who_am_i = who_am_i != NULL ? (uint8_t)who_am_i->valueint : stored_identity;
    if (strlen(local_site->valuestring) > APP_STORAGE_LOCAL_SITE_MAX_LENGTH) {
        respond_error(id, "invalid_config", ESP_ERR_INVALID_SIZE); return;
    }
    strcpy(config.local_site, local_site->valuestring);
    if (strlen(ssid->valuestring) > APP_STORAGE_WIFI_SSID_MAX_LENGTH ||
        strlen(password->valuestring) > APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH ||
        strlen(server->valuestring) > APP_STORAGE_IPV4_MAX_LENGTH) {
        respond_error(id, "invalid_config", ESP_ERR_INVALID_SIZE); return;
    }
    strcpy(config.wifi_ssid, ssid->valuestring);
    strcpy(config.wifi_password, password->valuestring);
    if (server->valuestring[0] != '\0') {
        config.server_configured = true;
        strcpy(config.server_ipv4, server->valuestring);
        config.server_port = (uint16_t)port->valueint;
        if (port->valueint < 1 || port->valueint > 65535) {
            respond_error(id, "invalid_config", ESP_ERR_INVALID_ARG); return;
        }
    } else if (port->valueint != 0) {
        respond_error(id, "invalid_config", ESP_ERR_INVALID_ARG); return;
    }
    if (peer->valuestring[0] != '\0') {
        config.peer_configured = parse_mac(peer->valuestring, config.peer_mac);
        if (!config.peer_configured) {
            respond_error(id, "invalid_config", ESP_ERR_INVALID_ARG); return;
        }
    }
    bool reconnecting = false;
    err = rover_service_set_config(&config, &reconnecting);
    if (err != ESP_OK) { respond_error(id, "invalid_config", err); return; }
    cJSON *result = cJSON_CreateObject();
    cJSON_AddBoolToObject(result, "wifi_reconnecting", reconnecting);
    respond_ok(id, result);
}

static void handle_request(cJSON *root)
{
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "v");
    const cJSON *id_item = cJSON_GetObjectItemCaseSensitive(root, "id");
    const cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    const double id = cJSON_IsNumber(id_item) ? id_item->valuedouble : 0;
    if (!cJSON_IsNumber(version) || version->valueint != PROTOCOL_VERSION ||
        !cJSON_IsNumber(id_item) || id != id_item->valueint || !cJSON_IsString(cmd)) {
        respond_error(id, "invalid_request", ESP_ERR_INVALID_ARG); return;
    }
    static const char *const simple_allowed[] = {"v", "id", "cmd"};
    if (strcmp(cmd->valuestring, "config.set") == 0) { handle_config_set(id, root); return; }
    if (strcmp(cmd->valuestring, "calibration.capture") == 0) {
        static const char *const allowed[] = {"v", "id", "cmd", "face"};
        const cJSON *face = cJSON_GetObjectItemCaseSensitive(root, "face");
        if (!has_only(root, allowed, 4) || !cJSON_IsNumber(face) || face->valueint < 0 || face->valueint > 5) {
            respond_error(id, "invalid_face", ESP_ERR_INVALID_ARG); return;
        }
        esp_err_t err = rover_service_calibration_capture((uint8_t)face->valueint);
        if (err == ESP_OK) respond_ok(id, NULL); else respond_error(id, "calibration_state", err);
        return;
    }
    if (!has_only(root, simple_allowed, 3)) {
        respond_error(id, "invalid_fields", ESP_ERR_INVALID_ARG); return;
    }
    if (strcmp(cmd->valuestring, "device.info") == 0) {
        rover_imu_state_t imu; rover_service_get_imu(&imu);
        uint8_t mac_bytes[6] = {0};
        const esp_err_t mac_err = esp_read_mac(mac_bytes, ESP_MAC_WIFI_STA);
        char rover_mac[18] = "";
        if (mac_err == ESP_OK) {
            snprintf(rover_mac, sizeof(rover_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                     mac_bytes[0], mac_bytes[1], mac_bytes[2], mac_bytes[3],
                     mac_bytes[4], mac_bytes[5]);
        }
        cJSON *data = cJSON_CreateObject();
        cJSON_AddStringToObject(data, "device", "EIRODENET");
        cJSON_AddNumberToObject(data, "protocol", PROTOCOL_VERSION);
        cJSON_AddBoolToObject(data, "imu_calibrated", imu.calibration_valid);
        cJSON_AddStringToObject(data, "rover_mac", rover_mac);
        respond_ok(id, data);
    } else if (strcmp(cmd->valuestring, "config.get") == 0) {
        handle_config_get(id);
    } else if (strcmp(cmd->valuestring, "motors.test_forward") == 0) {
        if (s_motor_test_task != NULL) {
            respond_error(id, "motor_busy", ESP_ERR_INVALID_STATE);
        } else if (xTaskCreate(motor_test_task, "motor_test", 2048, NULL, 4,
                               &s_motor_test_task) == pdPASS) {
            cJSON *data = cJSON_CreateObject();
            cJSON_AddNumberToObject(data, "duration_ms", MOTOR_ADAPTER_TEST_DURATION_MS);
            respond_ok(id, data);
        } else {
            respond_error(id, "motor_error", ESP_ERR_NO_MEM);
        }
    } else if (strcmp(cmd->valuestring, "stream.start") == 0) {
        s_streaming = true; respond_ok(id, NULL);
    } else if (strcmp(cmd->valuestring, "stream.stop") == 0) {
        s_streaming = false; respond_ok(id, NULL);
    } else {
        esp_err_t err = ESP_ERR_NOT_SUPPORTED;
        if (strcmp(cmd->valuestring, "calibration.start") == 0) err = rover_service_calibration_start();
        else if (strcmp(cmd->valuestring, "calibration.commit") == 0) err = rover_service_calibration_commit();
        else if (strcmp(cmd->valuestring, "calibration.cancel") == 0) err = rover_service_calibration_cancel();
        else { respond_error(id, "unknown_command", err); return; }
        if (err == ESP_OK) respond_ok(id, NULL); else respond_error(id, "calibration_state", err);
    }
}

static void emit_imu(void)
{
    rover_imu_state_t state; rover_service_get_imu(&state);
    cJSON *root = base_message("telemetry");
    cJSON_AddStringToObject(root, "topic", "imu");
    cJSON_AddNumberToObject(root, "seq", ++s_sequence);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)state.timestamp_ms);
    cJSON_AddBoolToObject(root, "valid", state.valid);
    cJSON_AddNumberToObject(root, "error", state.error);
    cJSON_AddBoolToObject(root, "calibrated", state.calibration_valid);
    if (state.valid) {
        cJSON_AddNumberToObject(root, "temperature_c", state.sample.temperature_c);
        add_vec3(root, "accel_g", state.sample.accel_g);
        add_vec3(root, "gyro_dps", state.sample.gyro_dps);
        cJSON *q = cJSON_AddArrayToObject(root, "quaternion");
        for (size_t i = 0; i < 4; ++i) cJSON_AddItemToArray(q, cJSON_CreateNumber(state.quaternion[i]));
    }
    transmit_json(root);
}

static void emit_sensors(void)
{
    rover_sensor_state_t state; rover_service_get_sensors(&state);
    cJSON *root = base_message("telemetry");
    cJSON_AddStringToObject(root, "topic", "sensors");
    cJSON_AddNumberToObject(root, "seq", ++s_sequence);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)state.timestamp_ms);
    cJSON *ultra = cJSON_AddObjectToObject(root, "ultrasonic");
    cJSON_AddBoolToObject(ultra, "valid", state.ultrasonic_valid);
    cJSON_AddNumberToObject(ultra, "error", state.ultrasonic_error);
    if (state.ultrasonic_valid) cJSON_AddNumberToObject(ultra, "distance_mm", state.distance_mm);
    cJSON *ir = cJSON_AddObjectToObject(root, "infrared");
    cJSON_AddBoolToObject(ir, "valid", state.infrared_valid);
    cJSON_AddNumberToObject(ir, "error", state.infrared_error);
    cJSON_AddNumberToObject(ir, "front_left", state.infrared.front_left);
    cJSON_AddNumberToObject(ir, "front_right", state.infrared.front_right);
    cJSON_AddNumberToObject(ir, "rear_left", state.infrared.rear_left);
    cJSON_AddNumberToObject(ir, "rear_right", state.infrared.rear_right);
    cJSON *color = cJSON_AddObjectToObject(root, "color");
    cJSON_AddBoolToObject(color, "valid", state.color_valid);
    cJSON_AddNumberToObject(color, "error", state.color_error);
    cJSON_AddNumberToObject(color, "ambient", state.color.ambient);
    cJSON_AddNumberToObject(color, "red", state.color.red);
    cJSON_AddNumberToObject(color, "green", state.color.green);
    cJSON_AddNumberToObject(color, "blue", state.color.blue);
    transmit_json(root);
}

static void emit_status(void)
{
    internet_adapter_status_t status = {0};
    esp_err_t err = rover_service_get_wifi(&status);
    bool reconnecting = false;
    esp_err_t reconnect_error = ESP_ERR_INVALID_STATE;
    rover_service_get_network_activity(&reconnecting, &reconnect_error);
    char ip[16] = "0.0.0.0";
    if (status.ipv4_address != 0) {
        ip4_addr_t address = {.addr = status.ipv4_address};
        ip4addr_ntoa_r(&address, ip, sizeof(ip));
    }
    cJSON *root = base_message("telemetry");
    cJSON_AddStringToObject(root, "topic", "status");
    cJSON_AddNumberToObject(root, "seq", ++s_sequence);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)(esp_timer_get_time() / 1000));
    cJSON_AddBoolToObject(root, "wifi_connected", status.connected);
    cJSON_AddBoolToObject(root, "wifi_reconnecting", reconnecting);
    cJSON_AddNumberToObject(root, "reconnect_error", reconnect_error);
    cJSON_AddNumberToObject(root, "wifi_error", err);
    cJSON_AddNumberToObject(root, "rssi", status.rssi);
    cJSON_AddStringToObject(root, "local_ipv4", ip);
    transmit_json(root);
}

static void emit_calibration_if_changed(uint32_t *last_revision)
{
    rover_calibration_status_t status; rover_service_get_calibration_status(&status);
    if (status.revision == *last_revision) return;
    *last_revision = status.revision;
    cJSON *root = base_message("event");
    cJSON_AddStringToObject(root, "topic", "calibration");
    cJSON_AddNumberToObject(root, "revision", status.revision);
    cJSON_AddNumberToObject(root, "phase", status.phase);
    cJSON_AddNumberToObject(root, "captured_mask", status.captured_mask);
    cJSON_AddNumberToObject(root, "active_face", status.active_face);
    cJSON_AddNumberToObject(root, "rejection_mask", status.rejection_mask);
    cJSON_AddNumberToObject(root, "settling_remaining", status.settling_remaining);
    cJSON_AddNumberToObject(root, "samples", status.samples);
    cJSON_AddNumberToObject(root, "error", status.error);
    add_vec3(root, "accel_mean", status.accel_mean);
    add_vec3(root, "accel_stddev", status.accel_stddev);
    add_vec3(root, "gyro_mean", status.gyro_mean);
    add_vec3(root, "gyro_stddev", status.gyro_stddev);
    transmit_json(root);
}

static void protocol_task(void *argument)
{
    (void)argument;
    char line[MAX_LINE_LENGTH + 1];
    size_t length = 0;
    bool overflow = false;
    uint32_t last_cal_revision = UINT32_MAX;
    int64_t next_imu = 0, next_sensors = 0, next_status = 0;
    while (true) {
        uint8_t bytes[128];
        const int count = uart_read_bytes(UART_NUM_0, bytes, sizeof(bytes), pdMS_TO_TICKS(10));
        for (int i = 0; i < count; ++i) {
            if (bytes[i] == '\n') {
                if (overflow) {
                    respond_error(0, "line_too_long", ESP_ERR_INVALID_SIZE);
                } else if (length >= strlen(PROTOCOL_PREFIX)) {
                    line[length] = '\0';
                    if (length > 0 && line[length - 1] == '\r') line[--length] = '\0';
                    if (strncmp(line, PROTOCOL_PREFIX, strlen(PROTOCOL_PREFIX)) == 0) {
                        cJSON *root = cJSON_ParseWithLength(line + strlen(PROTOCOL_PREFIX),
                                                            length - strlen(PROTOCOL_PREFIX));
                        if (root != NULL && cJSON_IsObject(root)) handle_request(root);
                        else respond_error(0, "invalid_json", ESP_ERR_INVALID_ARG);
                        cJSON_Delete(root);
                    }
                }
                length = 0; overflow = false;
            } else if (!overflow) {
                if (length < MAX_LINE_LENGTH) line[length++] = (char)bytes[i];
                else overflow = true;
            }
        }
        const int64_t now = esp_timer_get_time() / 1000;
        if (s_streaming && now >= next_imu) { emit_imu(); next_imu = now + 50; }
        if (s_streaming && now >= next_sensors) { emit_sensors(); next_sensors = now + 200; }
        if (s_streaming && now >= next_status) { emit_status(); next_status = now + 1000; }
        emit_calibration_if_changed(&last_cal_revision);
    }
}

esp_err_t serial_protocol_start(void)
{
    uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(UART_NUM_0, &config);
    if (err == ESP_OK) err = uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    if (xTaskCreate(protocol_task, "serial_protocol", 6144, NULL, 5, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "Protocolo @EIRO v1 listo en UART0");
    return ESP_OK;
}
