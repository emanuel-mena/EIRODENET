#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef enum {
    APP_MODE_TEST = 0,
    APP_MODE_COMPETITION = 1,
} app_mode_t;

esp_err_t app_mode_start(uint8_t rover_identity);
app_mode_t app_mode_get(void);
const char *app_mode_name(app_mode_t mode);
