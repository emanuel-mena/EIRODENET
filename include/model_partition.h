#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_partition.h"

/** @file model_partition.h
 * @brief Localización y validación de modelos TensorFlow Lite en flash.
 */

/** @brief Etiqueta esperada de la partición del modelo. */
#define MODEL_PARTITION_LABEL "model"
/** @brief Subtipo privado registrado para el modelo. */
#define MODEL_PARTITION_SUBTYPE ((esp_partition_subtype_t)0x40)
/** @brief Magic EIRM de la imagen empaquetada. */
#define MODEL_HEADER_MAGIC UINT32_C(0x4D524945)
/** @brief Versión compatible de la cabecera EIRM. */
#define MODEL_HEADER_VERSION UINT16_C(1)

/** @brief Metadatos de un modelo validado y listo para consumo. */
typedef struct {
    const esp_partition_t *partition; /**< Partición encontrada por nombre y subtipo. */
    uint32_t data_offset;             /**< Inicio del FlatBuffer dentro de la partición. */
    uint32_t model_version;           /**< Versión funcional asignada al empaquetar. */
    uint32_t model_size;              /**< Longitud del FlatBuffer en bytes. */
    uint32_t crc32;                   /**< CRC32 esperado y verificado. */
} model_partition_info_t;

/**
 * @brief Valida cabecera EIRM, firma TFL3, tamaño y CRC32 del modelo.
 * @param[out] info Metadatos; se inicializan a cero incluso si la validación falla.
 * @return ESP_OK o un error que identifica ausencia, formato, tamaño, lectura o CRC.
 */
esp_err_t model_partition_validate(model_partition_info_t *info);
