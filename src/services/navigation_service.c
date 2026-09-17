#include "navigation_service.h"

#include <math.h>

#include "app_mode.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef struct {
    float col;
    float row;
    uint32_t request_id;
} navigation_target_t;

static const char *TAG = "navigation";
static QueueHandle_t s_targets;
static SemaphoreHandle_t s_lock;
static navigation_status_t s_status = {.core_id = 1};
static uint32_t s_next_request;

static void navigation_task(void *argument)
{
    (void)argument;
    navigation_target_t target;
    while (xQueueReceive(s_targets, &target, portMAX_DELAY) == pdTRUE) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_status.phase = NAVIGATION_STUB;
        s_status.has_target = true;
        s_status.col = target.col;
        s_status.row = target.row;
        s_status.request_id = target.request_id;
        xSemaphoreGive(s_lock);
        ESP_LOGW(TAG, "Stub recibio objetivo #%lu (col=%.2f, row=%.2f) en core %d",
                 (unsigned long)target.request_id, (double)target.col, (double)target.row,
                 xPortGetCoreID());
    }
    vTaskDelete(NULL);
}

esp_err_t navigation_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_targets = xQueueCreate(1, sizeof(navigation_target_t));
    if (s_lock == NULL || s_targets == NULL) return ESP_ERR_NO_MEM;
    return xTaskCreatePinnedToCore(navigation_task, "navigation_stub", 3072, NULL, 5,
                                   NULL, 1) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t navigation_service_submit(float col, float row, uint32_t *request_id)
{
    if (s_lock == NULL || s_targets == NULL) return ESP_ERR_INVALID_STATE;
    if (app_mode_get() != APP_MODE_TEST) return ESP_ERR_INVALID_STATE;
    if (!isfinite(col) || !isfinite(row) || col < 0.0f || row < 0.0f ||
        col > 1000.0f || row > 1000.0f) return ESP_ERR_INVALID_ARG;
    navigation_target_t target = {.col = col, .row = row, .request_id = ++s_next_request};
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.phase = NAVIGATION_TARGET_RECEIVED;
    s_status.has_target = true;
    s_status.col = col;
    s_status.row = row;
    s_status.request_id = target.request_id;
    xSemaphoreGive(s_lock);
    if (xQueueOverwrite(s_targets, &target) != pdTRUE) return ESP_FAIL;
    if (request_id != NULL) *request_id = target.request_id;
    return ESP_OK;
}

void navigation_service_get_status(navigation_status_t *status)
{
    if (status == NULL || s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *status = s_status;
    xSemaphoreGive(s_lock);
}
