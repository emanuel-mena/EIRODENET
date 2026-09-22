#include "local_site_service.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "app_mode.h"
#include "app_storage.h"
#include "cJSON.h"
#include "competition_service.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp_vfs.h"
#include "lwip/ip4_addr.h"
#include "manual_control_service.h"
#include "mdns.h"
#include "navigation_service.h"
#include "peer_comms_service.h"
#include "rover_service.h"

#define STATIC_PARTITION_LABEL "static"
#define STATIC_BASE_PATH "/static"
#define FILE_BUFFER_SIZE 1024
#define API_BODY_MAX 256

static const char *TAG = "local_site";
static httpd_handle_t s_server;
static bool s_mounted;
static bool s_mdns_started;
static char s_hostname[64];

static void set_api_headers(httpd_req_t *request)
{
    httpd_resp_set_type(request, "application/json; charset=utf-8");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(request, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
}

static esp_err_t send_json(httpd_req_t *request, cJSON *root)
{
    set_api_headers(request);
    char *text = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (text == NULL) return httpd_resp_send_500(request);
    const esp_err_t err = httpd_resp_sendstr(request, text);
    cJSON_free(text);
    return err;
}

static esp_err_t send_api_error(httpd_req_t *request, const char *status,
                                const char *code, esp_err_t error)
{
    httpd_resp_set_status(request, status);
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", false);
    cJSON_AddStringToObject(root, "code", code);
    cJSON_AddStringToObject(root, "message", esp_err_to_name(error));
    return send_json(request, root);
}

static void add_vec3(cJSON *parent, const char *name, const float values[3])
{
    cJSON *array = cJSON_AddArrayToObject(parent, name);
    for (size_t index = 0; index < 3; ++index) {
        cJSON_AddItemToArray(array, cJSON_CreateNumber(values[index]));
    }
}

static const char *navigation_phase_name(navigation_phase_t phase)
{
    switch (phase) {
        case NAVIGATION_IDLE: return "idle";
        case NAVIGATION_WAITING_FOR_POSE: return "waiting_for_pose";
        case NAVIGATION_TURNING: return "turning";
        case NAVIGATION_DRIVING: return "driving";
        case NAVIGATION_ARRIVED: return "arrived";
        case NAVIGATION_BLOCKED: return "blocked";
        case NAVIGATION_CANCELLED: return "cancelled";
        case NAVIGATION_ERROR: return "error";
        default: return "unknown";
    }
}

static esp_err_t state_handler(httpd_req_t *request)
{
    rover_imu_state_t imu = {0};
    rover_sensor_state_t sensors = {0};
    internet_adapter_status_t wifi = {0};
    navigation_status_t navigation = {0};
    manual_control_status_t drive = {0};
    app_storage_config_t config = {0};
    rover_service_get_imu(&imu);
    rover_service_get_sensors(&sensors);
    rover_service_get_wifi(&wifi);
    navigation_service_get_status(&navigation);
    manual_control_service_get_status(&drive);
    app_storage_get_config(&config);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)(esp_timer_get_time() / 1000));
    cJSON_AddNumberToObject(root, "rover_id", config.who_am_i);
    cJSON_AddStringToObject(root, "hostname", s_hostname);
    cJSON_AddStringToObject(root, "mode", app_mode_name(app_mode_get()));
    cJSON_AddStringToObject(root, "competition", competition_service_status());

    cJSON *imu_json = cJSON_AddObjectToObject(root, "imu");
    cJSON_AddBoolToObject(imu_json, "valid", imu.valid);
    cJSON_AddBoolToObject(imu_json, "calibrated", imu.calibration_valid);
    cJSON_AddNumberToObject(imu_json, "error", imu.error);
    if (imu.valid) {
        cJSON_AddNumberToObject(imu_json, "temperature_c", imu.sample.temperature_c);
        add_vec3(imu_json, "accel_g", imu.sample.accel_g);
        add_vec3(imu_json, "gyro_dps", imu.sample.gyro_dps);
        cJSON *quaternion = cJSON_AddArrayToObject(imu_json, "quaternion");
        for (size_t index = 0; index < 4; ++index) {
            cJSON_AddItemToArray(quaternion, cJSON_CreateNumber(imu.quaternion[index]));
        }
    }

    cJSON *sensor_json = cJSON_AddObjectToObject(root, "sensors");
    cJSON *ultrasonic = cJSON_AddObjectToObject(sensor_json, "ultrasonic");
    cJSON_AddBoolToObject(ultrasonic, "valid", sensors.ultrasonic_valid);
    cJSON_AddNumberToObject(ultrasonic, "error", sensors.ultrasonic_error);
    cJSON_AddNumberToObject(ultrasonic, "distance_mm", sensors.distance_mm);
    cJSON *infrared = cJSON_AddObjectToObject(sensor_json, "infrared");
    cJSON_AddBoolToObject(infrared, "valid", sensors.infrared_valid);
    cJSON_AddNumberToObject(infrared, "error", sensors.infrared_error);
    cJSON_AddNumberToObject(infrared, "front_left", sensors.infrared.front_left);
    cJSON_AddNumberToObject(infrared, "front_right", sensors.infrared.front_right);
    cJSON_AddNumberToObject(infrared, "rear_left", sensors.infrared.rear_left);
    cJSON_AddNumberToObject(infrared, "rear_right", sensors.infrared.rear_right);
    cJSON *color = cJSON_AddObjectToObject(sensor_json, "color");
    cJSON_AddBoolToObject(color, "valid", sensors.color_valid);
    cJSON_AddNumberToObject(color, "error", sensors.color_error);
    cJSON_AddNumberToObject(color, "ambient", sensors.color.ambient);
    cJSON_AddNumberToObject(color, "red", sensors.color.red);
    cJSON_AddNumberToObject(color, "green", sensors.color.green);
    cJSON_AddNumberToObject(color, "blue", sensors.color.blue);

    cJSON *network = cJSON_AddObjectToObject(root, "network");
    cJSON_AddStringToObject(network, "transport", "http-local");
    cJSON_AddBoolToObject(network, "connected", wifi.connected);
    cJSON_AddNumberToObject(network, "rssi", wifi.rssi);
    char address_text[16] = "0.0.0.0";
    if (wifi.ipv4_address != 0) {
        const ip4_addr_t address = {.addr = wifi.ipv4_address};
        ip4addr_ntoa_r(&address, address_text, sizeof(address_text));
    }
    cJSON_AddStringToObject(network, "ipv4", address_text);

    cJSON *nav_json = cJSON_AddObjectToObject(root, "navigation");
    cJSON_AddNumberToObject(nav_json, "phase", navigation.phase);
    cJSON_AddStringToObject(nav_json, "phase_name", navigation_phase_name(navigation.phase));
    cJSON_AddStringToObject(nav_json, "implementation", "fused-grid-v1");
    cJSON_AddNumberToObject(nav_json, "core", navigation.core_id);
    cJSON_AddBoolToObject(nav_json, "has_target", navigation.has_target);
    cJSON_AddNumberToObject(nav_json, "col", navigation.col);
    cJSON_AddNumberToObject(nav_json, "row", navigation.row);
    cJSON_AddNumberToObject(nav_json, "request_id", navigation.request_id);
    cJSON_AddNumberToObject(nav_json, "error", navigation.error);
    cJSON_AddNumberToObject(nav_json, "cancel_reason", navigation.cancel_reason);
    cJSON *pose_json = cJSON_AddObjectToObject(nav_json, "pose");
    cJSON_AddBoolToObject(pose_json, "valid", navigation.pose_valid);
    cJSON_AddNumberToObject(pose_json, "col", navigation.pose_col);
    cJSON_AddNumberToObject(pose_json, "row", navigation.pose_row);
    cJSON_AddNumberToObject(pose_json, "theta_deg", navigation.theta_deg);
    cJSON_AddNumberToObject(pose_json, "speed_cells_s", navigation.linear_speed_cells_s);
    cJSON_AddNumberToObject(pose_json, "angular_speed_dps", navigation.angular_speed_dps);
    cJSON_AddNumberToObject(pose_json, "uncertainty_cells", navigation.uncertainty_cells);
    cJSON_AddNumberToObject(pose_json, "vision_heading_offset_deg",
                            navigation.vision_heading_offset_deg);
    cJSON *vision_json = cJSON_AddObjectToObject(nav_json, "vision");
    cJSON_AddBoolToObject(vision_json, "configured", navigation.vision_configured);
    cJSON_AddBoolToObject(vision_json, "connected", navigation.vision_connected);
    cJSON_AddBoolToObject(vision_json, "fresh", navigation.vision_fresh);
    cJSON_AddNumberToObject(vision_json, "age_ms", navigation.vision_age_ms);
    cJSON *grid_json = cJSON_AddObjectToObject(nav_json, "grid_encoder");
    cJSON_AddBoolToObject(grid_json, "calibrated", navigation.grid_calibrated);
    cJSON_AddBoolToObject(grid_json, "correction_active", navigation.grid_correction_active);
    cJSON_AddNumberToObject(grid_json, "pattern", navigation.infrared_pattern);
    cJSON_AddNumberToObject(grid_json, "calibrated_mask", navigation.infrared_calibrated_mask);
    cJSON_AddNumberToObject(grid_json, "last_correction", navigation.last_correction);
    cJSON *nav_motors = cJSON_AddObjectToObject(nav_json, "motors");
    cJSON_AddNumberToObject(nav_motors, "left", navigation.motor_left);
    cJSON_AddNumberToObject(nav_motors, "right", navigation.motor_right);
    cJSON *drive_json = cJSON_AddObjectToObject(root, "drive");
    cJSON_AddNumberToObject(drive_json, "left", drive.left);
    cJSON_AddNumberToObject(drive_json, "right", drive.right);
    cJSON_AddBoolToObject(drive_json, "watchdog_armed", drive.watchdog_armed);
    return send_json(request, root);
}

static esp_err_t peer_state_handler(httpd_req_t *request)
{
    peer_comms_status_t peer;
    peer_comms_service_get_status(&peer);
    if (!peer.connected) {
        return send_api_error(request, "503 Service Unavailable",
                              peer.configured ? "peer_offline" : "peer_not_configured",
                              peer.last_error == ESP_OK ? ESP_ERR_INVALID_STATE : peer.last_error);
    }
    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddNumberToObject(root, "timestamp_ms", peer.timestamp_ms);
    cJSON_AddNumberToObject(root, "rover_id", peer.rover_id);
    cJSON_AddStringToObject(root, "hostname", "");
    cJSON_AddStringToObject(root, "mode", app_mode_name((app_mode_t)peer.mode));
    cJSON_AddStringToObject(root, "competition", "stub");
    cJSON *imu = cJSON_AddObjectToObject(root, "imu");
    cJSON_AddBoolToObject(imu, "valid", peer.imu_valid);
    cJSON_AddBoolToObject(imu, "calibrated", peer.imu_calibrated);
    cJSON_AddNumberToObject(imu, "error", peer.imu_valid ? ESP_OK : ESP_ERR_INVALID_STATE);
    if (peer.imu_valid) {
        cJSON_AddNumberToObject(imu, "temperature_c", peer.temperature_c);
        cJSON *quaternion = cJSON_AddArrayToObject(imu, "quaternion");
        for (size_t index = 0; index < 4; ++index) {
            cJSON_AddItemToArray(quaternion, cJSON_CreateNumber(peer.quaternion[index]));
        }
    }
    cJSON *sensors = cJSON_AddObjectToObject(root, "sensors");
    cJSON *ultrasonic = cJSON_AddObjectToObject(sensors, "ultrasonic");
    cJSON_AddBoolToObject(ultrasonic, "valid", peer.ultrasonic_valid);
    cJSON_AddNumberToObject(ultrasonic, "distance_mm", peer.distance_mm);
    cJSON *infrared = cJSON_AddObjectToObject(sensors, "infrared");
    cJSON_AddBoolToObject(infrared, "valid", peer.infrared_valid);
    cJSON_AddNumberToObject(infrared, "front_left", peer.infrared[0]);
    cJSON_AddNumberToObject(infrared, "front_right", peer.infrared[1]);
    cJSON_AddNumberToObject(infrared, "rear_left", peer.infrared[2]);
    cJSON_AddNumberToObject(infrared, "rear_right", peer.infrared[3]);
    cJSON *color = cJSON_AddObjectToObject(sensors, "color");
    cJSON_AddBoolToObject(color, "valid", peer.color_valid);
    cJSON_AddNumberToObject(color, "ambient", peer.color[0]);
    cJSON_AddNumberToObject(color, "red", peer.color[1]);
    cJSON_AddNumberToObject(color, "green", peer.color[2]);
    cJSON_AddNumberToObject(color, "blue", peer.color[3]);
    cJSON *network = cJSON_AddObjectToObject(root, "network");
    cJSON_AddStringToObject(network, "transport", "esp-now");
    cJSON_AddBoolToObject(network, "connected", true);
    cJSON_AddNumberToObject(network, "rssi", peer.rssi);
    cJSON_AddStringToObject(network, "ipv4", "ESP-NOW");
    cJSON_AddNumberToObject(network, "age_ms", peer.age_ms);
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", peer.peer_mac[0],
             peer.peer_mac[1], peer.peer_mac[2], peer.peer_mac[3], peer.peer_mac[4], peer.peer_mac[5]);
    cJSON_AddStringToObject(network, "peer_mac", mac);
    cJSON *navigation = cJSON_AddObjectToObject(root, "navigation");
    cJSON_AddNumberToObject(navigation, "phase", peer.navigation_phase);
    cJSON_AddStringToObject(navigation, "implementation", "fused-grid-v1");
    cJSON_AddNumberToObject(navigation, "core", 1);
    cJSON_AddBoolToObject(navigation, "has_target", peer.navigation_has_target);
    cJSON_AddNumberToObject(navigation, "col", peer.navigation_col);
    cJSON_AddNumberToObject(navigation, "row", peer.navigation_row);
    cJSON_AddNumberToObject(navigation, "request_id", peer.navigation_request_id);
    cJSON *drive = cJSON_AddObjectToObject(root, "drive");
    cJSON_AddNumberToObject(drive, "left", peer.drive_left);
    cJSON_AddNumberToObject(drive, "right", peer.drive_right);
    cJSON_AddBoolToObject(drive, "watchdog_armed", peer.drive_left != 0 || peer.drive_right != 0);
    return send_json(request, root);
}

static cJSON *receive_json(httpd_req_t *request)
{
    if (request->content_len <= 0 || request->content_len > API_BODY_MAX) return NULL;
    char body[API_BODY_MAX + 1];
    size_t received = 0;
    while (received < request->content_len) {
        const int count = httpd_req_recv(request, body + received, request->content_len - received);
        if (count <= 0) return NULL;
        received += (size_t)count;
    }
    body[received] = '\0';
    return cJSON_ParseWithLength(body, received);
}

static esp_err_t drive_handler(httpd_req_t *request)
{
    cJSON *body = receive_json(request);
    const cJSON *left = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "left");
    const cJSON *right = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "right");
    if (!cJSON_IsNumber(left) || !cJSON_IsNumber(right) || left->valuedouble != left->valueint ||
        right->valuedouble != right->valueint || left->valueint < -1000 || left->valueint > 1000 ||
        right->valueint < -1000 || right->valueint > 1000) {
        cJSON_Delete(body);
        return send_api_error(request, "400 Bad Request", "invalid_drive", ESP_ERR_INVALID_ARG);
    }
    const esp_err_t err = manual_control_service_set((int16_t)left->valueint, (int16_t)right->valueint);
    cJSON_Delete(body);
    if (err != ESP_OK) return send_api_error(request,
        err == ESP_ERR_INVALID_STATE ? "409 Conflict" : "500 Internal Server Error",
        err == ESP_ERR_INVALID_STATE ? "test_mode_required" : "drive_error", err);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    cJSON_AddNumberToObject(response, "watchdog_ms", 500);
    return send_json(request, response);
}

static esp_err_t target_handler(httpd_req_t *request)
{
    if (app_mode_get() != APP_MODE_TEST) {
        return send_api_error(request, "409 Conflict", "test_mode_required", ESP_ERR_INVALID_STATE);
    }
    cJSON *body = receive_json(request);
    const cJSON *col = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "col");
    const cJSON *row = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "row");
    if (!cJSON_IsNumber(col) || !cJSON_IsNumber(row)) {
        cJSON_Delete(body);
        return send_api_error(request, "400 Bad Request", "invalid_target", ESP_ERR_INVALID_ARG);
    }
    uint32_t request_id = 0;
    const esp_err_t err = navigation_service_submit((float)col->valuedouble,
                                                     (float)row->valuedouble, &request_id);
    cJSON_Delete(body);
    if (err != ESP_OK) return send_api_error(request,
        err == ESP_ERR_INVALID_STATE ? "409 Conflict" : "400 Bad Request",
        err == ESP_ERR_INVALID_STATE ? "navigation_not_ready" : "invalid_target", err);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    cJSON_AddNumberToObject(response, "request_id", request_id);
    cJSON_AddStringToObject(response, "implementation", "fused-grid-v1");
    return send_json(request, response);
}

static esp_err_t cancel_navigation_handler(httpd_req_t *request)
{
    if (app_mode_get() != APP_MODE_TEST) {
        return send_api_error(request, "409 Conflict", "test_mode_required", ESP_ERR_INVALID_STATE);
    }
    const esp_err_t err = navigation_service_cancel(NAVIGATION_CANCEL_USER);
    if (err != ESP_OK) return send_api_error(request, "500 Internal Server Error",
                                              "navigation_error", err);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    return send_json(request, response);
}

static esp_err_t peer_drive_handler(httpd_req_t *request)
{
    cJSON *body = receive_json(request);
    const cJSON *left = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "left");
    const cJSON *right = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "right");
    if (!cJSON_IsNumber(left) || !cJSON_IsNumber(right) || left->valuedouble != left->valueint ||
        right->valuedouble != right->valueint || left->valueint < -1000 || left->valueint > 1000 ||
        right->valueint < -1000 || right->valueint > 1000) {
        cJSON_Delete(body);
        return send_api_error(request, "400 Bad Request", "invalid_drive", ESP_ERR_INVALID_ARG);
    }
    const esp_err_t err = peer_comms_service_send_drive((int16_t)left->valueint,
                                                        (int16_t)right->valueint);
    cJSON_Delete(body);
    if (err != ESP_OK) return send_api_error(request, "503 Service Unavailable", "peer_offline", err);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    cJSON_AddStringToObject(response, "transport", "esp-now");
    return send_json(request, response);
}

static esp_err_t peer_target_handler(httpd_req_t *request)
{
    cJSON *body = receive_json(request);
    const cJSON *col = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "col");
    const cJSON *row = body == NULL ? NULL : cJSON_GetObjectItemCaseSensitive(body, "row");
    if (!cJSON_IsNumber(col) || !cJSON_IsNumber(row)) {
        cJSON_Delete(body);
        return send_api_error(request, "400 Bad Request", "invalid_target", ESP_ERR_INVALID_ARG);
    }
    const esp_err_t err = peer_comms_service_send_target((float)col->valuedouble,
                                                         (float)row->valuedouble);
    cJSON_Delete(body);
    if (err != ESP_OK) return send_api_error(request, "503 Service Unavailable", "peer_offline", err);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "ok", true);
    cJSON_AddStringToObject(response, "transport", "esp-now");
    return send_json(request, response);
}

static esp_err_t options_handler(httpd_req_t *request)
{
    set_api_headers(request);
    return httpd_resp_send(request, NULL, 0);
}

static bool valid_hostname(const char *hostname)
{
    const size_t length = hostname == NULL ? 0 : strlen(hostname);
    if (length == 0 || length > 63 || !isalnum((unsigned char)hostname[0]) ||
        !isalnum((unsigned char)hostname[length - 1])) return false;
    for (size_t i = 0; i < length; ++i) {
        const unsigned char character = (unsigned char)hostname[i];
        if (!isalnum(character) && character != '-' && character != '_') return false;
    }
    return true;
}

static const char *content_type(const char *path)
{
    const char *extension = strrchr(path, '.');
    if (extension == NULL) return "application/octet-stream";
    if (strcmp(extension, ".html") == 0) return "text/html; charset=utf-8";
    if (strcmp(extension, ".css") == 0) return "text/css; charset=utf-8";
    if (strcmp(extension, ".js") == 0) return "text/javascript; charset=utf-8";
    if (strcmp(extension, ".json") == 0) return "application/json; charset=utf-8";
    if (strcmp(extension, ".svg") == 0) return "image/svg+xml";
    if (strcmp(extension, ".png") == 0) return "image/png";
    if (strcmp(extension, ".jpg") == 0 || strcmp(extension, ".jpeg") == 0) return "image/jpeg";
    if (strcmp(extension, ".ico") == 0) return "image/x-icon";
    if (strcmp(extension, ".woff2") == 0) return "font/woff2";
    return "application/octet-stream";
}

static esp_err_t send_file(httpd_req_t *request, const char *path)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) return ESP_ERR_NOT_FOUND;
    httpd_resp_set_type(request, content_type(path));
    httpd_resp_set_hdr(request, "Cache-Control",
                       strcmp(path, STATIC_BASE_PATH "/index.html") == 0
                           ? "no-cache" : "public, max-age=31536000, immutable");
    char buffer[FILE_BUFFER_SIZE];
    size_t read;
    esp_err_t err = ESP_OK;
    while ((read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        err = httpd_resp_send_chunk(request, buffer, read);
        if (err != ESP_OK) break;
    }
    fclose(file);
    if (err == ESP_OK) err = httpd_resp_send_chunk(request, NULL, 0);
    return err;
}

static esp_err_t static_handler(httpd_req_t *request)
{
    char path[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN];
    const char *query = strchr(request->uri, '?');
    const size_t uri_length = query == NULL ? strlen(request->uri) : (size_t)(query - request->uri);
    if (strstr(request->uri, "..") != NULL) {
        httpd_resp_send_err(request, HTTPD_400_BAD_REQUEST, "Ruta inválida");
        return ESP_FAIL;
    }
    if (uri_length == 0 || (uri_length == 1 && request->uri[0] == '/')) {
        snprintf(path, sizeof(path), STATIC_BASE_PATH "/index.html");
    } else {
        if (uri_length >= sizeof(path) - strlen(STATIC_BASE_PATH)) {
            httpd_resp_send_err(request, HTTPD_414_URI_TOO_LONG, "URI demasiado larga");
            return ESP_FAIL;
        }
        snprintf(path, sizeof(path), STATIC_BASE_PATH "%.*s", (int)uri_length, request->uri);
    }
    struct stat info;
    if (stat(path, &info) != 0 || S_ISDIR(info.st_mode)) {
        snprintf(path, sizeof(path), STATIC_BASE_PATH "/index.html");
        if (stat(path, &info) != 0) {
            httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "Sitio no instalado");
            return ESP_FAIL;
        }
    }
    const esp_err_t err = send_file(request, path);
    if (err == ESP_ERR_NOT_FOUND) httpd_resp_send_err(request, HTTPD_404_NOT_FOUND, "Archivo no encontrado");
    return err;
}

static void stop_service(void)
{
    if (s_server != NULL) { httpd_stop(s_server); s_server = NULL; }
    if (s_mdns_started) { mdns_free(); s_mdns_started = false; }
    if (s_mounted) { esp_vfs_spiffs_unregister(STATIC_PARTITION_LABEL); s_mounted = false; }
    s_hostname[0] = '\0';
}

esp_err_t local_site_service_set_hostname(const char *hostname)
{
    if (hostname == NULL) return ESP_ERR_INVALID_ARG;
    if (hostname[0] == '\0') {
        stop_service();
        ESP_LOGI(TAG, "Sitio local desactivado");
        return ESP_OK;
    }
    if (!valid_hostname(hostname)) return ESP_ERR_INVALID_ARG;
    if (s_server != NULL && strcmp(s_hostname, hostname) == 0) return ESP_OK;
    stop_service();
    const esp_vfs_spiffs_conf_t filesystem = {
        .base_path = STATIC_BASE_PATH, .partition_label = STATIC_PARTITION_LABEL,
        .max_files = 6, .format_if_mount_failed = false};
    esp_err_t err = esp_vfs_spiffs_register(&filesystem);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo montar SPIFFS: %s", esp_err_to_name(err));
        return err;
    }
    s_mounted = true;
    err = mdns_init();
    if (err == ESP_OK) { s_mdns_started = true; err = mdns_hostname_set(hostname); }
    if (err == ESP_OK) err = mdns_instance_name_set("EIRODENET local site");
    if (err == ESP_OK) err = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo anunciar %s.local: %s", hostname, esp_err_to_name(err));
        stop_service();
        return err;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 9;
    err = httpd_start(&s_server, &config);
    if (err == ESP_OK) {
        const httpd_uri_t state_route = {
            .uri = "/api/v1/state", .method = HTTP_GET, .handler = state_handler};
        const httpd_uri_t drive_route = {
            .uri = "/api/v1/drive", .method = HTTP_POST, .handler = drive_handler};
        const httpd_uri_t target_route = {
            .uri = "/api/v1/navigation/target", .method = HTTP_POST, .handler = target_handler};
        const httpd_uri_t cancel_route = {
            .uri = "/api/v1/navigation/cancel", .method = HTTP_POST,
            .handler = cancel_navigation_handler};
        const httpd_uri_t peer_state_route = {
            .uri = "/api/v1/peer/state", .method = HTTP_GET, .handler = peer_state_handler};
        const httpd_uri_t peer_drive_route = {
            .uri = "/api/v1/peer/drive", .method = HTTP_POST, .handler = peer_drive_handler};
        const httpd_uri_t peer_target_route = {
            .uri = "/api/v1/peer/navigation/target", .method = HTTP_POST,
            .handler = peer_target_handler};
        const httpd_uri_t options_route = {
            .uri = "/api/*", .method = HTTP_OPTIONS, .handler = options_handler};
        const httpd_uri_t static_route = {
            .uri = "/*", .method = HTTP_GET, .handler = static_handler, .user_ctx = NULL};
        err = httpd_register_uri_handler(s_server, &state_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &drive_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &target_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &cancel_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &peer_state_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &peer_drive_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &peer_target_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &options_route);
        if (err == ESP_OK) err = httpd_register_uri_handler(s_server, &static_route);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar HTTP: %s", esp_err_to_name(err));
        stop_service();
        return err;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info(STATIC_PARTITION_LABEL, &total, &used);
    strcpy(s_hostname, hostname);
    ESP_LOGI(TAG, "Sitio disponible en http://%s.local (%u/%u bytes)",
             s_hostname, (unsigned)used, (unsigned)total);
    return ESP_OK;
}

const char *local_site_service_hostname(void)
{
    return s_hostname;
}
