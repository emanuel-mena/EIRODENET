#include "app_storage.h"

#include <limits.h>
#include <stdbool.h>
#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"

#define APP_NVS_NAMESPACE "app"
#define BOOT_COUNT_KEY "boot_count"
#define WIFI_SSID_KEY "wifi_ssid"
#define WIFI_PASSWORD_KEY "wifi_pass"

static bool s_initialized;

static esp_err_t open_storage(nvs_open_mode_t mode, nvs_handle_t *handle)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return nvs_open(APP_NVS_NAMESPACE, mode, handle);
}

esp_err_t app_storage_init(uint32_t *boot_count)
{
    if (boot_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        return err;
    }
    s_initialized = true;

    nvs_handle_t handle;
    err = open_storage(NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    uint32_t value = 0;
    err = nvs_get_u32(handle, BOOT_COUNT_KEY, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
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

esp_err_t app_storage_get_string(const char *key, char *value, size_t value_size)
{
    if (key == NULL || value == NULL || value_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = open_storage(NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }
    size_t required_size = value_size;
    err = nvs_get_str(handle, key, value, &required_size);
    nvs_close(handle);
    return err;
}

esp_err_t app_storage_set_string(const char *key, const char *value)
{
    if (key == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = open_storage(NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t app_storage_get_wifi_credentials(app_storage_wifi_credentials_t *credentials)
{
    if (credentials == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(credentials, 0, sizeof(*credentials));
    esp_err_t err = app_storage_get_string(
        WIFI_SSID_KEY, credentials->ssid, sizeof(credentials->ssid));
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err != ESP_OK) {
        return err;
    }
    err = app_storage_get_string(
        WIFI_PASSWORD_KEY, credentials->password, sizeof(credentials->password));
    return err == ESP_ERR_NVS_NOT_FOUND ? ESP_ERR_NOT_FOUND : err;
}

esp_err_t app_storage_set_wifi_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || strlen(ssid) > APP_STORAGE_WIFI_SSID_MAX_LENGTH ||
        strlen(password) > APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = app_storage_set_string(WIFI_SSID_KEY, ssid);
    return err == ESP_OK ? app_storage_set_string(WIFI_PASSWORD_KEY, password) : err;
}
