#pragma once

/** @file board_pins.h
 * @brief Fuente única del mapa de pines y periféricos de EIRODENET.
 * @warning Los GPIO del ESP32 toleran como máximo lógica de 3.3 V.
 */

/** @name Sensor ultrasónico HY-SRF05 */
/**@{*/
#define BOARD_ULTRASONIC_ECHO_GPIO 33
#define BOARD_ULTRASONIC_TRIGGER_GPIO 4
/**@}*/

/** @name Sensor de color */
/**@{*/
#define BOARD_COLOR_NEOPIXEL_GPIO 26
#define BOARD_COLOR_PHOTORESISTOR_GPIO 32
/**@}*/

/** @name Arreglo TCRT5000 */
/**@{*/
#define BOARD_IR_FRONT_LEFT_GPIO 35
#define BOARD_IR_FRONT_RIGHT_GPIO 34
#define BOARD_IR_REAR_LEFT_GPIO 39
#define BOARD_IR_REAR_RIGHT_GPIO 36
/**@}*/

/** @name IMU LSM6DS3TR-C */
/**@{*/
#define BOARD_IMU_SDA_GPIO 21
#define BOARD_IMU_SCL_GPIO 22
#define BOARD_IMU_I2C_ADDRESS 0x6B
#define BOARD_IMU_I2C_CLOCK_HZ 100000
/**@}*/

/** @name Motores conectados mediante puentes H */
/**@{*/
#define BOARD_MOTOR_1_A_GPIO 12
#define BOARD_MOTOR_1_B_GPIO 14
#define BOARD_MOTOR_2_A_GPIO 13
#define BOARD_MOTOR_2_B_GPIO 15
/**@}*/

/** @name Selección de módulos de hardware */
/**@{*/
#define BOARD_ENABLE_IMU 1
#define BOARD_ENABLE_COLOR_SENSOR 1
/**@}*/

/** @name Interfaz de modos de la IdeaBoard */
/**@{*/
#define BOARD_BOOT_BUTTON_GPIO 0
#define BOARD_MODE_NEOPIXEL_GPIO 2
/**@}*/

#if BOARD_ENABLE_IMU && BOARD_ENABLE_COLOR_SENSOR && \
    (BOARD_IMU_SDA_GPIO == BOARD_COLOR_NEOPIXEL_GPIO)
#error "GPIO26 no puede controlar simultaneamente el NeoPixel y el bus I2C"
#endif
