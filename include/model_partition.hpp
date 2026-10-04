#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_partition.h"

#define MODEL_PARTITION_LABEL "model"
#define MODEL_PARTITION_SUBTYPE ((esp_partition_subtype_t)0x40)
#define MODEL_HEADER_MAGIC UINT32_C(0x4D524945)
#define MODEL_HEADER_VERSION UINT16_C(1)

typedef struct {
    const esp_partition_t *partition;
    uint32_t data_offset;
    uint32_t model_version;
    uint32_t model_size;
    uint32_t crc32;
} model_partition_info_t;

esp_err_t model_partition_validate(model_partition_info_t *info);
