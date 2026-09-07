#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t app_storage_init(uint32_t *boot_count);
