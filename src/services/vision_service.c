#include "vision_service.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "app_storage.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define VISION_PROTOCOL_VERSION 2
#define VISION_MAX_LINE_BYTES 16384
#define VISION_MAX_POSE_AGE_MS 500U
#define VISION_RECONNECT_MS 1000U

static const char *TAG = "vision_client";
static SemaphoreHandle_t s_lock;
static vision_status_t s_status = {.last_error = ESP_ERR_INVALID_STATE};
static volatile uint32_t s_reload_generation;

static bool finite_number(const cJSON *item)
{
    return cJSON_IsNumber(item) && isfinite(item->valuedouble);
}

static bool integer_number(const cJSON *item)
{
    return finite_number(item) && item->valuedouble == floor(item->valuedouble);
}

static bool exact_fields(const cJSON *object, const char *const *names, size_t count)
{
    if (!cJSON_IsObject(object)) return false;
    size_t seen = 0;
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, object) {
        bool known = false;
        for (size_t i = 0; i < count; ++i) known |= strcmp(item->string, names[i]) == 0;
        if (!known) return false;
        ++seen;
    }
    if (seen != count) return false;
    for (size_t i = 0; i < count; ++i) {
        if (cJSON_GetObjectItemCaseSensitive(object, names[i]) == NULL) return false;
    }
    return true;
}

static bool valid_position_object(const cJSON *object, const char *const *fields,
                                  size_t field_count, bool with_age)
{
    if (!exact_fields(object, fields, field_count)) return false;
    const cJSON *col = cJSON_GetObjectItemCaseSensitive(object, "col");
    const cJSON *row = cJSON_GetObjectItemCaseSensitive(object, "row");
    if (!finite_number(col) || !finite_number(row)) return false;
    if (with_age) {
        const cJSON *age = cJSON_GetObjectItemCaseSensitive(object, "age_ms");
        if (!integer_number(age) || age->valuedouble < 0) return false;
    }
    return true;
}

static bool valid_contract(const cJSON *root, const cJSON **own_rover,
                           uint8_t own_id, uint16_t *cols, uint16_t *rows, float *cell_mm)
{
    static const char *const root_fields[] = {"v", "seq", "ts_ms", "phase", "clock", "grid",
        "rovers", "cubes", "obstacles", "start", "depots", "depot_size", "cube_side"};
    static const char *const grid_fields[] = {"cols", "rows", "cell_mm"};
    static const char *const clock_fields[] = {"elapsed_ms", "remaining_ms", "total_ms"};
    static const char *const rover_fields[] = {"id", "col", "row", "theta", "age_ms"};
    static const char *const cube_fields[] = {"color", "col", "row", "age_ms"};
    static const char *const obstacle_fields[] = {"col", "row", "age_ms"};
    static const char *const start_fields[] = {"col", "row"};
    static const char *const depot_fields[] = {"color", "col", "row"};
    static const char *const depot_size_fields[] = {"length", "depth"};
    if (!exact_fields(root, root_fields, sizeof(root_fields) / sizeof(root_fields[0]))) return false;
    const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "v");
    const cJSON *seq = cJSON_GetObjectItemCaseSensitive(root, "seq");
    const cJSON *timestamp = cJSON_GetObjectItemCaseSensitive(root, "ts_ms");
    const cJSON *phase = cJSON_GetObjectItemCaseSensitive(root, "phase");
    if (!integer_number(version) || version->valueint != VISION_PROTOCOL_VERSION ||
        !integer_number(seq) || seq->valuedouble < 0 || !integer_number(timestamp) ||
        timestamp->valuedouble < 0 || !cJSON_IsString(phase) ||
        (strcmp(phase->valuestring, "IDLE") && strcmp(phase->valuestring, "READY") &&
         strcmp(phase->valuestring, "RUNNING") && strcmp(phase->valuestring, "FINISHED"))) return false;

    const cJSON *grid = cJSON_GetObjectItemCaseSensitive(root, "grid");
    if (!exact_fields(grid, grid_fields, 3)) return false;
    const cJSON *grid_cols = cJSON_GetObjectItemCaseSensitive(grid, "cols");
    const cJSON *grid_rows = cJSON_GetObjectItemCaseSensitive(grid, "rows");
    const cJSON *grid_cell = cJSON_GetObjectItemCaseSensitive(grid, "cell_mm");
    if (!integer_number(grid_cols) || !integer_number(grid_rows) || !finite_number(grid_cell) ||
        grid_cols->valueint <= 0 || grid_rows->valueint <= 0 || grid_cols->valueint > UINT16_MAX ||
        grid_rows->valueint > UINT16_MAX || grid_cell->valuedouble <= 0) return false;

    const cJSON *clock = cJSON_GetObjectItemCaseSensitive(root, "clock");
    if (!exact_fields(clock, clock_fields, 3)) return false;
    const cJSON *elapsed = cJSON_GetObjectItemCaseSensitive(clock, "elapsed_ms");
    const cJSON *remaining = cJSON_GetObjectItemCaseSensitive(clock, "remaining_ms");
    const cJSON *total = cJSON_GetObjectItemCaseSensitive(clock, "total_ms");
    if (!integer_number(elapsed) || !integer_number(remaining) || !integer_number(total) ||
        elapsed->valuedouble < 0 || remaining->valuedouble < 0 || total->valuedouble < 0 ||
        elapsed->valuedouble + remaining->valuedouble != total->valuedouble) return false;

    const cJSON *start = cJSON_GetObjectItemCaseSensitive(root, "start");
    if (!valid_position_object(start, start_fields, 2, false)) return false;
    const cJSON *depot_size = cJSON_GetObjectItemCaseSensitive(root, "depot_size");
    if (!exact_fields(depot_size, depot_size_fields, 2) ||
        !finite_number(cJSON_GetObjectItemCaseSensitive(depot_size, "length")) ||
        !finite_number(cJSON_GetObjectItemCaseSensitive(depot_size, "depth")) ||
        cJSON_GetObjectItemCaseSensitive(depot_size, "length")->valuedouble <= 0 ||
        cJSON_GetObjectItemCaseSensitive(depot_size, "depth")->valuedouble <= 0) return false;
    const cJSON *cube_side = cJSON_GetObjectItemCaseSensitive(root, "cube_side");
    if (!finite_number(cube_side) || cube_side->valuedouble <= 0) return false;

    const cJSON *rovers = cJSON_GetObjectItemCaseSensitive(root, "rovers");
    const cJSON *cubes = cJSON_GetObjectItemCaseSensitive(root, "cubes");
    const cJSON *obstacles = cJSON_GetObjectItemCaseSensitive(root, "obstacles");
    const cJSON *depots = cJSON_GetObjectItemCaseSensitive(root, "depots");
    if (!cJSON_IsArray(rovers) || !cJSON_IsArray(cubes) || !cJSON_IsArray(obstacles) ||
        !cJSON_IsArray(depots)) return false;
    *own_rover = NULL;
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, rovers) {
        if (!valid_position_object(item, rover_fields, 5, true)) return false;
        const cJSON *id = cJSON_GetObjectItemCaseSensitive(item, "id");
        const cJSON *theta = cJSON_GetObjectItemCaseSensitive(item, "theta");
        if (!integer_number(id) || id->valuedouble < 0 || !finite_number(theta) ||
            theta->valuedouble < 0 || theta->valuedouble > 360) return false;
        const cJSON *other = rovers->child;
        while (other != item) {
            if (cJSON_GetObjectItemCaseSensitive(other, "id")->valuedouble == id->valuedouble)
                return false;
            other = other->next;
        }
        if (id->valuedouble == own_id) *own_rover = item;
    }
    bool depot_colors[3] = {false, false, false};
    cJSON_ArrayForEach(item, cubes) {
        if (!valid_position_object(item, cube_fields, 4, true)) return false;
        const cJSON *color = cJSON_GetObjectItemCaseSensitive(item, "color");
        if (!cJSON_IsString(color) ||
            (strcmp(color->valuestring, "green") && strcmp(color->valuestring, "blue") &&
             strcmp(color->valuestring, "red"))) return false;
        const cJSON *other = cubes->child;
        while (other != item) {
            if (strcmp(cJSON_GetObjectItemCaseSensitive(other, "color")->valuestring,
                       color->valuestring) == 0) return false;
            other = other->next;
        }
    }
    cJSON_ArrayForEach(item, obstacles) {
        if (!valid_position_object(item, obstacle_fields, 3, true)) return false;
    }
    cJSON_ArrayForEach(item, depots) {
        if (!valid_position_object(item, depot_fields, 3, false)) return false;
        const cJSON *color = cJSON_GetObjectItemCaseSensitive(item, "color");
        if (!cJSON_IsString(color)) return false;
        int color_index = strcmp(color->valuestring, "green") == 0 ? 0 :
                          strcmp(color->valuestring, "blue") == 0 ? 1 :
                          strcmp(color->valuestring, "red") == 0 ? 2 : -1;
        if (color_index < 0 || depot_colors[color_index]) return false;
        depot_colors[color_index] = true;
    }
    cJSON_ArrayForEach(item, cubes) {
        const char *color = cJSON_GetObjectItemCaseSensitive(item, "color")->valuestring;
        const int color_index = strcmp(color, "green") == 0 ? 0 :
                                strcmp(color, "blue") == 0 ? 1 : 2;
        if (!depot_colors[color_index]) return false;
    }
    *cols = (uint16_t)grid_cols->valueint;
    *rows = (uint16_t)grid_rows->valueint;
    *cell_mm = (float)grid_cell->valuedouble;
    return true;
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
    if (root == NULL || !valid_contract(root, &rover, own_id, &cols, &rows, &cell_mm)) {
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
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.protocol_valid = true;
    s_status.sequence = (uint32_t)seq->valuedouble;
    s_status.received_ms = now_ms;
    s_status.grid_cols = cols;
    s_status.grid_rows = rows;
    s_status.cell_mm = cell_mm;
    s_status.rover_id = own_id;
    s_status.pose_valid = false;
    s_status.last_error = ESP_OK;
    if (rover != NULL) {
        const uint32_t age = (uint32_t)cJSON_GetObjectItemCaseSensitive(rover, "age_ms")->valuedouble;
        s_status.age_ms = age;
        if (age <= VISION_MAX_POSE_AGE_MS) {
            s_status.col = (float)cJSON_GetObjectItemCaseSensitive(rover, "col")->valuedouble;
            s_status.row = (float)cJSON_GetObjectItemCaseSensitive(rover, "row")->valuedouble;
            s_status.theta_deg = (float)cJSON_GetObjectItemCaseSensitive(rover, "theta")->valuedouble;
            s_status.pose_valid = true;
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
        .sin_addr.s_addr = inet_addr(config->server_ipv4),
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
    char *line = malloc(VISION_MAX_LINE_BYTES + 1);
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
    if (status->pose_valid && status->received_ms > 0) {
        const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
        const uint64_t elapsed = now_ms > status->received_ms ? now_ms - status->received_ms : 0;
        status->age_ms = elapsed > UINT32_MAX - status->age_ms
            ? UINT32_MAX : status->age_ms + (uint32_t)elapsed;
        status->pose_valid = status->age_ms <= VISION_MAX_POSE_AGE_MS;
    }
    xSemaphoreGive(s_lock);
}
