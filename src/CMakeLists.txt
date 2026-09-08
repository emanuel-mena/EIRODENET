file(
    GLOB_RECURSE app_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "../include"
    PRIV_REQUIRES led_strip nvs_flash esp_partition
)
