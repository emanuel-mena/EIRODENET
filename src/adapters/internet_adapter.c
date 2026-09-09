#include "internet_adapter.h"

#include <string.h>
#include "app_storage.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAILED_BIT BIT1
#define WIFI_MAX_RETRIES 5

static EventGroupHandle_t s_events;
static esp_netif_t *s_station;
static int s_retries;
static bool s_started;
static uint32_t s_ipv4_address;

static void wifi_event_handler(void *argument, esp_event_base_t base, int32_t id, void *data)
{
    (void)argument;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retries++ < WIFI_MAX_RETRIES) {
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_events, WIFI_FAILED_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = data;
        s_ipv4_address = event->ip_info.ip.addr;
        s_retries = 0;
        xEventGroupSetBits(s_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t internet_adapter_init(void)
{
    if (s_started) return ESP_OK;
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    s_station = esp_netif_create_default_wifi_sta();
    if (s_station == NULL) return ESP_ERR_NO_MEM;
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&config);
    if (err != ESP_OK) return err;
    s_events = xEventGroupCreate();
    if (s_events == NULL) return ESP_ERR_NO_MEM;
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    if (err == ESP_OK) err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    if (err == ESP_OK) err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) s_started = true;
    return err;
}

esp_err_t internet_adapter_connect(uint32_t timeout_ms)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    app_storage_wifi_credentials_t credentials;
    esp_err_t err = app_storage_get_wifi_credentials(&credentials);
    if (err != ESP_OK || credentials.ssid[0] == '\0') return err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
    wifi_config_t config = { 0 };
    memcpy(config.sta.ssid, credentials.ssid, strlen(credentials.ssid));
    memcpy(config.sta.password, credentials.password, strlen(credentials.password));
    config.sta.threshold.authmode = credentials.password[0] == '\0' ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    xEventGroupClearBits(s_events, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT);
    s_retries = 0;
    err = esp_wifi_set_config(WIFI_IF_STA, &config);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) return err;
    const EventBits_t bits = xEventGroupWaitBits(s_events, WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    if (bits & WIFI_CONNECTED_BIT) return ESP_OK;
    return (bits & WIFI_FAILED_BIT) ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

esp_err_t internet_adapter_get_status(internet_adapter_status_t *status)
{
    if (status == NULL) return ESP_ERR_INVALID_ARG;
    memset(status, 0, sizeof(*status));
    if (!s_started) return ESP_ERR_INVALID_STATE;
    wifi_ap_record_t access_point;
    esp_err_t err = esp_wifi_sta_get_ap_info(&access_point);
    if (err == ESP_OK) {
        status->connected = true;
        status->rssi = access_point.rssi;
        status->ipv4_address = s_ipv4_address;
    } else if (err == ESP_ERR_WIFI_NOT_CONNECT) {
        return ESP_OK;
    }
    return err;
}

esp_err_t internet_adapter_disconnect(void)
{
    return s_started ? esp_wifi_disconnect() : ESP_ERR_INVALID_STATE;
}
