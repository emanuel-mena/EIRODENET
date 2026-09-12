#include "app_storage.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "lwip/ip4_addr.h"
#include "nvs.h"
#include "nvs_flash.h"

#define APP_NVS_NAMESPACE "app"
#define BOOT_COUNT_KEY "boot_count"
#define WIFI_SSID_KEY "wifi_ssid"
#define WIFI_PASSWORD_KEY "wifi_pass"
#define SERVER_IP_KEY "server_ip"
#define SERVER_PORT_KEY "server_port"
#define PEER_MAC_KEY "peer_mac"
#define IMU_CAL_KEY "imu_cal"

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

static esp_err_t get_optional_string(nvs_handle_t handle, const char *key, char *value, size_t size)
{
    size_t required = size;
    esp_err_t err = nvs_get_str(handle, key, value, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        value[0] = '\0';
        return ESP_OK;
    }
    return err;
}

static bool valid_peer_mac(const uint8_t mac[6])
{
    bool any = false;
    bool all_ff = true;
    for (size_t i = 0; i < 6; ++i) {
        any |= mac[i] != 0;
        all_ff &= mac[i] == 0xff;
    }
    return any && !all_ff && (mac[0] & 1U) == 0;
}

esp_err_t app_storage_get_config(app_storage_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    memset(config, 0, sizeof(*config));
    nvs_handle_t handle;
    esp_err_t err = open_storage(NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    err = get_optional_string(handle, WIFI_SSID_KEY, config->wifi_ssid, sizeof(config->wifi_ssid));
    if (err == ESP_OK) err = get_optional_string(handle, WIFI_PASSWORD_KEY, config->wifi_password,
                                                  sizeof(config->wifi_password));
    if (err == ESP_OK) {
        err = get_optional_string(handle, SERVER_IP_KEY, config->server_ipv4,
                                  sizeof(config->server_ipv4));
    }
    if (err == ESP_OK && config->server_ipv4[0] != '\0') {
        err = nvs_get_u16(handle, SERVER_PORT_KEY, &config->server_port);
        config->server_configured = err == ESP_OK;
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_ERR_INVALID_CRC;
        if (err == ESP_OK && config->server_configured) {
            ip4_addr_t address;
            if (config->server_port == 0 || !ip4addr_aton(config->server_ipv4, &address)) {
                err = ESP_ERR_INVALID_CRC;
            }
        }
    } else if (err == ESP_OK) {
        uint16_t orphan_port = 0;
        esp_err_t port_err = nvs_get_u16(handle, SERVER_PORT_KEY, &orphan_port);
        if (port_err == ESP_OK) err = ESP_ERR_INVALID_CRC;
        else if (port_err != ESP_ERR_NVS_NOT_FOUND) err = port_err;
    }
    if (err == ESP_OK) {
        size_t mac_size = sizeof(config->peer_mac);
        esp_err_t mac_err = nvs_get_blob(handle, PEER_MAC_KEY, config->peer_mac, &mac_size);
        if (mac_err == ESP_OK && mac_size == sizeof(config->peer_mac)) {
            if (valid_peer_mac(config->peer_mac)) config->peer_configured = true;
            else err = ESP_ERR_INVALID_CRC;
        } else if (mac_err != ESP_ERR_NVS_NOT_FOUND && mac_err != ESP_OK) {
            err = mac_err;
        }
    }
    nvs_close(handle);
    return err;
}

esp_err_t app_storage_set_config(const app_storage_config_t *config)
{
    ip4_addr_t address;
    if (config == NULL || strlen(config->wifi_ssid) > APP_STORAGE_WIFI_SSID_MAX_LENGTH ||
        strlen(config->wifi_password) > APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH ||
        (config->server_configured &&
         (config->server_port == 0 || !ip4addr_aton(config->server_ipv4, &address))) ||
        (config->peer_configured && !valid_peer_mac(config->peer_mac))) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = open_storage(NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_str(handle, WIFI_SSID_KEY, config->wifi_ssid);
    if (err == ESP_OK) err = nvs_set_str(handle, WIFI_PASSWORD_KEY, config->wifi_password);
    if (err == ESP_OK && config->server_configured) {
        err = nvs_set_str(handle, SERVER_IP_KEY, config->server_ipv4);
        if (err == ESP_OK) err = nvs_set_u16(handle, SERVER_PORT_KEY, config->server_port);
    } else if (err == ESP_OK) {
        esp_err_t erase_err = nvs_erase_key(handle, SERVER_IP_KEY);
        if (erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) err = erase_err;
        erase_err = nvs_erase_key(handle, SERVER_PORT_KEY);
        if (err == ESP_OK && erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) err = erase_err;
    }
    if (err == ESP_OK && config->peer_configured) {
        err = nvs_set_blob(handle, PEER_MAC_KEY, config->peer_mac, sizeof(config->peer_mac));
    } else if (err == ESP_OK) {
        esp_err_t erase_err = nvs_erase_key(handle, PEER_MAC_KEY);
        if (erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) err = erase_err;
    }
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static bool valid_calibration(const imu_calibration_t *calibration)
{
    if (calibration == NULL || calibration->version != IMU_CALIBRATION_VERSION) return false;
    for (size_t axis = 0; axis < 3; ++axis) {
        if (!isfinite(calibration->accel_offset_g[axis]) ||
            !isfinite(calibration->accel_scale[axis]) ||
            !isfinite(calibration->gyro_bias_dps[axis]) ||
            calibration->accel_scale[axis] < 0.1f || calibration->accel_scale[axis] > 10.0f) {
            return false;
        }
    }
    return true;
}

esp_err_t app_storage_get_imu_calibration(imu_calibration_t *calibration)
{
    if (calibration == NULL) return ESP_ERR_INVALID_ARG;
    memset(calibration, 0, sizeof(*calibration));
    nvs_handle_t handle;
    esp_err_t err = open_storage(NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    size_t size = sizeof(*calibration);
    err = nvs_get_blob(handle, IMU_CAL_KEY, calibration, &size);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) return ESP_ERR_NOT_FOUND;
    if (err != ESP_OK) return err;
    return size == sizeof(*calibration) && valid_calibration(calibration) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t app_storage_set_imu_calibration(const imu_calibration_t *calibration)
{
    if (!valid_calibration(calibration)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = open_storage(NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, IMU_CAL_KEY, calibration, sizeof(*calibration));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t app_storage_set_wifi_credentials(const char *ssid, const char *password)
{
    if (ssid == NULL || password == NULL || strlen(ssid) > APP_STORAGE_WIFI_SSID_MAX_LENGTH ||
        strlen(password) > APP_STORAGE_WIFI_PASSWORD_MAX_LENGTH) {
        return ESP_ERR_INVALID_ARG;
    }
    app_storage_config_t config;
    esp_err_t err = app_storage_get_config(&config);
    if (err != ESP_OK) return err;
    strcpy(config.wifi_ssid, ssid);
    strcpy(config.wifi_password, password);
    return app_storage_set_config(&config);
}
