file(
    GLOB_RECURSE app_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "../include"
    PRIV_REQUIRES led_strip mdns nvs_flash spiffs esp_http_server esp_partition esp_driver_i2c esp_driver_gpio
        esp_driver_ledc esp_timer esp_adc esp_wifi esp_event esp_netif
        esp_driver_uart json lwip
)
