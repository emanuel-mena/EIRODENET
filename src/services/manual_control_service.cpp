#include "manual_control_service.hpp"

#include "app_mode.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_adapter.hpp"
#include "navigation_service.hpp"

#define DRIVE_WATCHDOG_MS 500

static SemaphoreHandle_t s_lock;
static manual_control_status_t s_status;
static int64_t s_deadline_ms;

static int16_t normalize_command(int16_t command)
{
    if (command > 0 && command < MOTOR_ADAPTER_MIN_COMMAND) return MOTOR_ADAPTER_MIN_COMMAND;
    if (command < 0 && command > -MOTOR_ADAPTER_MIN_COMMAND) return -MOTOR_ADAPTER_MIN_COMMAND;
    return command;
}

static void watchdog_task(void *argument)
{
    (void)argument;
    while (true) {
        bool stop = false;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_status.watchdog_armed &&
            (app_mode_get() != APP_MODE_TEST || esp_timer_get_time() / 1000 >= s_deadline_ms)) {
            s_status.left = 0;
            s_status.right = 0;
            s_status.watchdog_armed = false;
            stop = true;
        }
        xSemaphoreGive(s_lock);
        if (stop) motor_adapter_stop();
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

esp_err_t manual_control_service_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;
    return xTaskCreate(watchdog_task, "drive_watchdog", 2048, NULL, 6, NULL) == pdPASS
        ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t manual_control_service_set(int16_t left, int16_t right)
{
    if (s_lock == NULL) return ESP_ERR_INVALID_STATE;
    if (app_mode_get() != APP_MODE_TEST) return ESP_ERR_INVALID_STATE;
    left = normalize_command(left);
    right = normalize_command(right);
    esp_err_t err = navigation_service_cancel(NAVIGATION_CANCEL_MANUAL);
    if (err == ESP_OK) err = motor_adapter_set(left, right);
    if (err != ESP_OK) return err;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_status.left = left;
    s_status.right = right;
    s_status.watchdog_armed = left != 0 || right != 0;
    s_deadline_ms = esp_timer_get_time() / 1000 + DRIVE_WATCHDOG_MS;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

void manual_control_service_get_status(manual_control_status_t *status)
{
    if (status == NULL || s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *status = s_status;
    xSemaphoreGive(s_lock);
}
