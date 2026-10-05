#include "vision_service.hpp"
#include "vision_contract.hpp"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app_storage.hpp"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define VISION_MAX_LINE_BYTES 16384
#define VISION_MAX_POSE_AGE_MS 750U
#define VISION_RECONNECT_MS 1000U

static const char *TAG = "vision_client";
static SemaphoreHandle_t s_lock;
static vision_status_t s_status = {.last_error = ESP_ERR_INVALID_STATE};
static volatile uint32_t s_reload_generation;

static int cube_color(const char *color)
{
    if (strcmp(color, "green") == 0) return VISION_CUBE_GREEN;
    if (strcmp(color, "blue") == 0) return VISION_CUBE_BLUE;
    if (strcmp(color, "red") == 0) return VISION_CUBE_RED;
    return -1;
}

static void publish_connection(bool configured, bool connected, esp_err_t error)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.configured = configured;
    s_status.connected = connected;
    s_status.last_error = error;
    if (!connected) s_status.protocol_valid = false;
    xSemaphoreGive(s_lock);
}

static void parse_line(const char *line, size_t length, uint8_t own_id)
{
    cJSON *root = cJSON_ParseWithLength(line, length);
    const cJSON *rover = NULL;
    uint16_t cols = 0, rows = 0;
    float cell_mm = 0;
    if (root == NULL || !vision_contract_validate(root, &rover, own_id, &cols, &rows, &cell_mm)) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.protocol_valid = false;
        s_status.pose_valid = false;
        s_status.last_error = ESP_ERR_INVALID_RESPONSE;
        xSemaphoreGive(s_lock);
        cJSON_Delete(root);
        return;
    }
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    const cJSON *seq = cJSON_GetObjectItemCaseSensitive(root, "seq");
    const cJSON *timestamp = cJSON_GetObjectItemCaseSensitive(root, "ts_ms");
    const uint64_t frame_timestamp_ms = (uint64_t)timestamp->valuedouble;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool new_frame = s_status.received_ms == 0 ||
                           s_status.frame_timestamp_ms != frame_timestamp_ms;
    s_status.protocol_valid = true;
    s_status.last_valid_frame_ms = now_ms;
    s_status.sequence = (uint32_t)seq->valuedouble;
    s_status.grid_cols = cols;
    s_status.grid_rows = rows;
    s_status.cell_mm = cell_mm;
    const char *phase = cJSON_GetObjectItemCaseSensitive(root, "phase")->valuestring;
    s_status.phase = strcmp(phase, "READY") == 0 ? VISION_PHASE_READY :
        strcmp(phase, "RUNNING") == 0 ? VISION_PHASE_RUNNING :
        strcmp(phase, "FINISHED") == 0 ? VISION_PHASE_FINISHED : VISION_PHASE_IDLE;
    s_status.rover_id = own_id;
    s_status.last_error = ESP_OK;
    if (new_frame) {
        s_status.frame_timestamp_ms = frame_timestamp_ms;
        s_status.received_ms = now_ms;
        s_status.pose_valid = false;
        s_status.peer_valid = false;
        s_status.obstacle_count = 0;
        memset(s_status.cube_valid, 0, sizeof(s_status.cube_valid));
        memset(s_status.cube_in_depot, 0, sizeof(s_status.cube_in_depot));
        memset(s_status.depot_valid, 0, sizeof(s_status.depot_valid));
        s_status.depot_length = (float)cJSON_GetObjectItemCaseSensitive(
            cJSON_GetObjectItemCaseSensitive(root, "depot_size"), "length")->valuedouble;
        s_status.depot_depth = (float)cJSON_GetObjectItemCaseSensitive(
            cJSON_GetObjectItemCaseSensitive(root, "depot_size"), "depth")->valuedouble;
        s_status.cube_side = (float)cJSON_GetObjectItemCaseSensitive(root, "cube_side")->valuedouble;
        const cJSON *item = NULL;
        const cJSON *cubes = cJSON_GetObjectItemCaseSensitive(root, "cubes");
        cJSON_ArrayForEach(item, cubes) {
            const int color = cube_color(cJSON_GetObjectItemCaseSensitive(item, "color")->valuestring);
            if (color < 0) continue;
            s_status.cube_valid[color] = true;
            s_status.cube_in_depot[color] = cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(item, "in_depot"));
            s_status.cubes[color] = (vision_position_t){
                .col = (float)cJSON_GetObjectItemCaseSensitive(item, "col")->valuedouble,
                .row = (float)cJSON_GetObjectItemCaseSensitive(item, "row")->valuedouble,
                .age_ms = (uint32_t)cJSON_GetObjectItemCaseSensitive(item, "age_ms")->valuedouble,
            };
        }
        const cJSON *depots = cJSON_GetObjectItemCaseSensitive(root, "depots");
        cJSON_ArrayForEach(item, depots) {
            const int color = cube_color(cJSON_GetObjectItemCaseSensitive(item, "color")->valuestring);
            if (color < 0) continue;
            s_status.depot_valid[color] = true;
            s_status.depot_col[color] = (float)cJSON_GetObjectItemCaseSensitive(item, "col")->valuedouble;
            s_status.depot_row[color] = (float)cJSON_GetObjectItemCaseSensitive(item, "row")->valuedouble;
        }
        if (rover != NULL) {
            const uint32_t age =
                (uint32_t)cJSON_GetObjectItemCaseSensitive(rover, "age_ms")->valuedouble;
            s_status.age_ms = age;
            if (age <= VISION_MAX_POSE_AGE_MS) {
                s_status.col = (float)cJSON_GetObjectItemCaseSensitive(rover, "col")->valuedouble;
                s_status.row = (float)cJSON_GetObjectItemCaseSensitive(rover, "row")->valuedouble;
                s_status.theta_deg =
                    (float)cJSON_GetObjectItemCaseSensitive(rover, "theta")->valuedouble;
                s_status.pose_valid = true;
            }
        }
        const cJSON *rovers = cJSON_GetObjectItemCaseSensitive(root, "rovers");
        cJSON_ArrayForEach(item, rovers) {
            const uint8_t id = (uint8_t)cJSON_GetObjectItemCaseSensitive(item, "id")->valueint;
            if (id == own_id) continue;
            s_status.peer_id = id;
            s_status.peer_col = (float)cJSON_GetObjectItemCaseSensitive(item, "col")->valuedouble;
            s_status.peer_row = (float)cJSON_GetObjectItemCaseSensitive(item, "row")->valuedouble;
            s_status.peer_theta_deg =
                (float)cJSON_GetObjectItemCaseSensitive(item, "theta")->valuedouble;
            s_status.peer_age_ms =
                (uint32_t)cJSON_GetObjectItemCaseSensitive(item, "age_ms")->valuedouble;
            s_status.peer_valid = s_status.peer_age_ms <= VISION_MAX_POSE_AGE_MS;
        }
        const cJSON *obstacles = cJSON_GetObjectItemCaseSensitive(root, "obstacles");
        cJSON_ArrayForEach(item, obstacles) {
            vision_position_t *position = &s_status.obstacles[s_status.obstacle_count++];
            position->col = (float)cJSON_GetObjectItemCaseSensitive(item, "col")->valuedouble;
            position->row = (float)cJSON_GetObjectItemCaseSensitive(item, "row")->valuedouble;
            position->age_ms =
                (uint32_t)cJSON_GetObjectItemCaseSensitive(item, "age_ms")->valuedouble;
        }
    }
    xSemaphoreGive(s_lock);
    cJSON_Delete(root);
}

static int connect_server(const app_storage_config_t *config)
{
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (fd < 0) return -1;
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 500000};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(config->server_port),
        .sin_addr = {.s_addr = inet_addr(config->server_ipv4)},
    };
    if (connect(fd, (struct sockaddr *)&address, sizeof(address)) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static void vision_task(void *argument)
{
    (void)argument;
    char *line = static_cast<char *>(malloc(VISION_MAX_LINE_BYTES + 1));
    if (line == NULL) {
        publish_connection(false, false, ESP_ERR_NO_MEM);
        vTaskDelete(NULL);
        return;
    }
    while (true) {
        app_storage_config_t config = {0};
        esp_err_t config_error = app_storage_get_config(&config);
        if (config_error != ESP_OK || !config.server_configured ||
            (config.who_am_i != APP_STORAGE_ROVER_10 && config.who_am_i != APP_STORAGE_ROVER_11)) {
            publish_connection(false, false, config_error == ESP_OK ? ESP_ERR_NOT_FOUND : config_error);
            vTaskDelay(pdMS_TO_TICKS(VISION_RECONNECT_MS));
            continue;
        }
        const uint32_t generation = s_reload_generation;
        publish_connection(true, false, ESP_ERR_NOT_FINISHED);
        const int fd = connect_server(&config);
        if (fd < 0) {
            publish_connection(true, false, ESP_ERR_TIMEOUT);
            vTaskDelay(pdMS_TO_TICKS(VISION_RECONNECT_MS));
            continue;
        }
        ESP_LOGI(TAG, "Conectado a %s:%u para rover %u", config.server_ipv4,
                 (unsigned)config.server_port, (unsigned)config.who_am_i);
        publish_connection(true, true, ESP_OK);
        size_t used = 0;
        while (generation == s_reload_generation) {
            const int received = recv(fd, line + used, VISION_MAX_LINE_BYTES - used, 0);
            if (received == 0) break;
            if (received < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                break;
            }
            used += (size_t)received;
            size_t start = 0;
            for (size_t index = 0; index < used; ++index) {
                if (line[index] != '\n') continue;
                size_t length = index - start;
                if (length > 0 && line[start + length - 1] == '\r') --length;
                if (length > 0) parse_line(line + start, length, config.who_am_i);
                start = index + 1;
            }
            if (start > 0) {
                memmove(line, line + start, used - start);
                used -= start;
            }
            if (used == VISION_MAX_LINE_BYTES) {
                publish_connection(true, false, ESP_ERR_INVALID_SIZE);
                break;
            }
        }
        shutdown(fd, SHUT_RDWR);
        close(fd);
        publish_connection(true, false, ESP_ERR_TIMEOUT);
        vTaskDelay(pdMS_TO_TICKS(VISION_RECONNECT_MS));
    }
}

esp_err_t vision_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;
    return xTaskCreate(vision_task, "vision_tcp", 8192, NULL, 4, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

void vision_service_reload(void)
{
    ++s_reload_generation;
}

void vision_service_get_status(vision_status_t *status)
{
    if (status == NULL || s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *status = s_status;
    if (status->received_ms > 0) {
        const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
        const uint64_t elapsed = now_ms > status->received_ms ? now_ms - status->received_ms : 0;
        if (elapsed > UINT32_MAX - status->age_ms) status->age_ms = UINT32_MAX;
        else status->age_ms += (uint32_t)elapsed;
        status->pose_valid = status->pose_valid && status->age_ms <= VISION_MAX_POSE_AGE_MS;
        if (elapsed > UINT32_MAX - status->peer_age_ms) status->peer_age_ms = UINT32_MAX;
        else status->peer_age_ms += (uint32_t)elapsed;
        status->peer_valid = status->peer_valid &&
                             status->peer_age_ms <= VISION_MAX_POSE_AGE_MS;
        for (uint8_t i = 0; i < status->obstacle_count; ++i) {
            if (elapsed > UINT32_MAX - status->obstacles[i].age_ms)
                status->obstacles[i].age_ms = UINT32_MAX;
            else status->obstacles[i].age_ms += (uint32_t)elapsed;
        }
        for (uint8_t i = 0; i < VISION_MAX_CUBES; ++i) {
            if (elapsed > UINT32_MAX - status->cubes[i].age_ms)
                status->cubes[i].age_ms = UINT32_MAX;
            else status->cubes[i].age_ms += (uint32_t)elapsed;
            status->cube_valid[i] = status->cube_valid[i] &&
                status->cubes[i].age_ms <= VISION_MAX_POSE_AGE_MS;
        }
    }
    xSemaphoreGive(s_lock);
}
