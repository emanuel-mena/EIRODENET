#pragma once

/* CRCibernetica IdeaBoard pin map.
 * Keep board-specific wiring here so application code remains portable.
 * All external GPIO logic is 3.3 V; do not apply 5 V to ESP32 GPIOs.
 */
#define IDEABOARD_I2C_SDA 21
#define IDEABOARD_I2C_SCL 22

#define IDEABOARD_M1_A 12
#define IDEABOARD_M1_B 14
#define IDEABOARD_M2_A 13
#define IDEABOARD_M2_B 15

#define IDEABOARD_RGB_LED 2

#define IDEABOARD_DAC_1 26
#define IDEABOARD_DAC_2 25
