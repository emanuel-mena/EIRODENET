#pragma once

#include "esp_err.h"

/** @brief Inicia el receptor de comandos y el publicador de telemetría sobre UART0. */
esp_err_t serial_protocol_start(void);

