#pragma once
#include <stdio.h>
#define ESP_LOGI(tag, fmt, ...) fprintf(stderr, "%s: " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW ESP_LOGI
#define ESP_LOGE ESP_LOGI
