# EIRODENET - CRCibernetica IdeaBoard

Configuración base para una CRCibernetica IdeaBoard con ESP32-WROOM-32E.

## Preparación y carga

1. Instalar PlatformIO y conectar la placa por USB-C.
2. Seleccionar el puerto serie CH340G si PlatformIO no lo detecta automáticamente.
3. Ejecutar `pio run -t upload` y luego `pio device monitor`.

La placa usa lógica de 3.3 V y alimentación externa de 5-9 V. La configuración conserva el perfil `esp32dev` de PlatformIO porque la IdeaBoard usa un ESP32 clásico, y fija explícitamente la flash de 8 MB anunciada por CRCibernética.

## Pines integrados

El mapa está centralizado en [`include/board_pins.h`](include/board_pins.h): I2C SDA/SCL en GPIO21/GPIO22 y los dos puentes H en GPIO12/14 y GPIO13/15. GPIO4, GPIO33 y GPIO25/26 quedan definidos como ejemplos de servo, entrada analógica y DAC.

El firmware inicializa el runtime ESP-IDF y emite un diagnóstico por serie a 115200 baudios; no mueve motores ni activa cargas al arrancar.
