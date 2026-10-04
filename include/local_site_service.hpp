#pragma once

#include <stdbool.h>
#include "esp_err.h"

/** @brief Sirve SPIFFS bajo hostname.local; una cadena vacía detiene el servicio. */
esp_err_t local_site_service_set_hostname(const char *hostname);

/** @brief Devuelve el hostname activo o una cadena vacía. */
const char *local_site_service_hostname(void);
