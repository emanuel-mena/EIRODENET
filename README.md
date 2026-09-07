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

## Memoria persistente y TinyML

La tabla `partitions.csv` reserva 64 KB de NVS para preferencias, 5.875 MB para
la aplicación y 2 MB para un modelo TinyML independiente. En cada
arranque se incrementa `app/boot_count` en NVS y se valida la cabecera y el CRC32
del modelo. Un modelo ausente o dañado no impide que arranque la aplicación.

Para preparar y grabar un modelo por separado, desde PowerShell:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py build modelo.tflite .pio/build/model.bin --version 1
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py flash .pio/build/model.bin --port COM3
```

La carga normal con `pio run -t upload` sólo actualiza la aplicación y conserva
el contenido de `model`. Sustituye `COM3` si la placa aparece en otro puerto.
