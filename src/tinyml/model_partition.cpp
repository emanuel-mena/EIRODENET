#include "model_partition.hpp"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define MODEL_CRC_CHUNK_SIZE 1024
#define TFLITE_PREFIX_SIZE 8

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t header_version;
    uint16_t header_size;
    uint32_t model_version;
    uint32_t model_size;
    uint32_t model_crc32;
    uint32_t reserved[3];
} model_partition_header_t;

static_assert(sizeof(model_partition_header_t) == 32, "Unexpected model header size");

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    while (length-- > 0) {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1) ^ (UINT32_C(0xEDB88320) & mask);
        }
    }
    return crc;
}

esp_err_t model_partition_validate(model_partition_info_t *info)
{
    if (info == NULL) return ESP_ERR_INVALID_ARG;
    memset(info, 0, sizeof(*info));

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, MODEL_PARTITION_SUBTYPE, MODEL_PARTITION_LABEL);
    if (partition == NULL) return ESP_ERR_NOT_FOUND;

    model_partition_header_t header;
    esp_err_t err = esp_partition_read(partition, 0, &header, sizeof(header));
    if (err != ESP_OK) return err;
    if (header.magic == UINT32_MAX) return ESP_ERR_NOT_FOUND;
    if (header.magic != MODEL_HEADER_MAGIC ||
        header.header_version != MODEL_HEADER_VERSION ||
        header.header_size != sizeof(header)) return ESP_ERR_INVALID_RESPONSE;
    if (header.model_size < TFLITE_PREFIX_SIZE ||
        header.model_size > partition->size - header.header_size) return ESP_ERR_INVALID_SIZE;

    uint8_t prefix[TFLITE_PREFIX_SIZE];
    err = esp_partition_read(partition, header.header_size, prefix, sizeof(prefix));
    if (err != ESP_OK) return err;
    if (memcmp(&prefix[4], "TFL3", 4) != 0) return ESP_ERR_INVALID_RESPONSE;

    uint8_t buffer[MODEL_CRC_CHUNK_SIZE];
    uint32_t crc = UINT32_MAX;
    uint32_t offset = header.header_size;
    uint32_t remaining = header.model_size;
    while (remaining > 0) {
        const size_t chunk = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
        err = esp_partition_read(partition, offset, buffer, chunk);
        if (err != ESP_OK) return err;
        crc = crc32_update(crc, buffer, chunk);
        offset += chunk;
        remaining -= chunk;
    }
    crc ^= UINT32_MAX;
    if (crc != header.model_crc32) return ESP_ERR_INVALID_CRC;

    info->partition = partition;
    info->data_offset = header.header_size;
    info->model_version = header.model_version;
    info->model_size = header.model_size;
    info->crc32 = header.model_crc32;
    return ESP_OK;
}

