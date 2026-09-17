#include "local_site_service.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "mdns.h"

#define STATIC_PARTITION_LABEL "static"
#define STATIC_BASE_PATH "/static"
#define FILE_BUFFER_SIZE 1024

static const char *TAG = "local_site";
static httpd_handle_t s_server;
static bool s_mounted;
static bool s_mdns_started;
static char s_hostname[64];

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
    config.max_uri_handlers = 4;
    err = httpd_start(&s_server, &config);
    if (err == ESP_OK) {
        const httpd_uri_t route = {
            .uri = "/*", .method = HTTP_GET, .handler = static_handler, .user_ctx = NULL};
        err = httpd_register_uri_handler(s_server, &route);
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
