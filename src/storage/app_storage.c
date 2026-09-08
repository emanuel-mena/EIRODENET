#include "app_storage.h"

#include <limits.h>

#include "nvs.h"
#include "nvs_flash.h"

#define APP_NVS_NAMESPACE "app"
#define BOOT_COUNT_KEY "boot_count"

esp_err_t app_storage_init(uint32_t *boot_count)
{
    if (boot_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            return err;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    nvs_handle_t handle;
    err = nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t value = 0;
    err = nvs_get_u32(handle, BOOT_COUNT_KEY, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        value = 0;
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        value = value == UINT32_MAX ? 1 : value + 1;
        err = nvs_set_u32(handle, BOOT_COUNT_KEY, value);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    if (err == ESP_OK) {
        *boot_count = value;
    }
    return err;
}
