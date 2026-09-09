# EIRODENET

**EspressIdea Rover Delivery Network** es el firmware ESP-IDF del rover construido
para el [Vision Rover Challenge](https://github.com/Universidad-Cenfotec/Vision-Rover-Challenge).
La aplicación integra movimiento diferencial, percepción local, conectividad Wi-Fi,
preferencias persistentes y una partición independiente para modelos TinyML.

El proyecto utiliza PlatformIO sobre una CRCibernética IdeaBoard con ESP32-WROOM-32E
y flash de 8 MB. Su punto de entrada es `src/main.c`; el hardware se consume mediante
adapters pequeños para que la lógica de navegación no dependa directamente de los
drivers de ESP-IDF.

## Estado del hardware probado

El firmware compila, se carga y funciona en el rover conectado por COM3. La última
prueba confirmó conexión Wi-Fi, lectura estable del LSM6DS3TR-C, los cuatro TCRT5000
y distancias ultrasónicas cercanas a 0.55 m. Motor 1 y Motor 2 respondieron en ambos
sentidos durante una prueba controlada al 30 %. La prueba automática fue retirada:
la versión normal siempre arranca con ambos motores detenidos.

La fotoresistencia actualmente devuelve cero en las cuatro lecturas del adapter de
color. Se debe revisar su alimentación y el divisor resistivo conectado a GPIO32.

## Mapa de conexiones

`include/board_pins.h` es la fuente única de verdad del cableado. Cualquier cambio
físico debe reflejarse allí, sin introducir números GPIO en la lógica de aplicación.

| Componente | Señal | GPIO |
|---|---|---:|
| HY-SRF05 | Trigger | 4 |
| HY-SRF05 | Echo | 33 |
| Sensor de color | NeoPixel | 26 |
| Sensor de color | Fotoresistencia/ADC1 | 32 |
| TCRT5000 SEN1 | Delantero izquierdo | 36 |
| TCRT5000 SEN2 | Delantero derecho | 39 |
| TCRT5000 SEN3 | Trasero izquierdo | 34 |
| TCRT5000 SEN4 | Trasero derecho | 35 |
| LSM6DS3TR-C | SDA | 21 |
| LSM6DS3TR-C | SCL | 22 |
| LSM6DS3TR-C | Dirección I2C | `0x6B` |
| Motor 1 | Puente H, entrada A/B | 12 / 14 |
| Motor 2 | Puente H, entrada A/B | 13 / 15 |

Todos los GPIO del ESP32 trabajan con lógica de 3.3 V. La salida Echo de un
HY-SRF05 alimentado a 5 V debe contar con adaptación de nivel si la tarjeta no la
incorpora. GPIO34, GPIO35, GPIO36 y GPIO39 son sólo de entrada y no incluyen
resistencias pull-up/pull-down internas.

## Módulos

| Módulo | API pública | Responsabilidad |
|---|---|---|
| Storage | `app_storage.h` | Inicialización NVS, preferencias y credenciales Wi-Fi |
| Internet | `internet_adapter.h` | Estación Wi-Fi, reintentos, estado, RSSI e IPv4 |
| Ultrasónico | `ultrasonic_adapter.h` | Disparo del HY-SRF05 y distancia en milímetros |
| Infrarrojos | `infrared_adapter.h` | Lectura posicional conjunta de SEN1 a SEN4 |
| Color | `color_sensor_adapter.h` | Iluminación RGB y cuatro muestras ADC reflectivas |
| IMU adapter | `imu_adapter.h` | Interfaz singleton configurada desde el mapa de pines |
| Driver IMU | `lsm6ds3tr_c.h` | Registros I2C, identificación y conversión física |
| Motores | `motor_adapter.h` | PWM independiente, sentido y parada segura |
| TinyML | `model_partition.h` | Localización y validación de la imagen del modelo |

Todas las APIs públicas incluyen documentación JavaDoc/Doxygen con parámetros,
valores de retorno, unidades y precondiciones.

## Control de motores

`motor_adapter_set(motor1, motor2)` recibe dos comandos independientes entre
`-1000` y `1000`. El signo selecciona el sentido, la magnitud controla el ciclo PWM
y cero deja el motor en rueda libre. Antes de invertir un motor, el adapter lleva
ambas entradas del puente H a cero para evitar conducción cruzada.

```c
ESP_ERROR_CHECK(motor_adapter_init());
ESP_ERROR_CHECK(motor_adapter_set(350, 0));  // Motor 1 al 35 %, Motor 2 detenido.
ESP_ERROR_CHECK(motor_adapter_stop());
```

## Credenciales y storage

El archivo `.env` de la raíz debe contener:

```dotenv
WIFI_SSD=nombre_de_red
WIFI_PASS=contraseña
```

También se acepta `WIFI_SSID`. Antes de compilar, `tools/generate_secrets.py`
genera `include/generated_secrets.h`; ambos archivos están ignorados por Git. En el
primer arranque las credenciales se guardan en NVS. Los arranques posteriores leen
NVS, permitiendo reemplazarlas en ejecución mediante
`app_storage_set_wifi_credentials()` sin modificar `main.c`.

## Preparación, compilación y carga

Se requiere PlatformIO Core y una conexión USB con el controlador CH340 disponible.
Desde PowerShell:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -t upload --upload-port COM3
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port COM3 --baud 115200
```

La carga normal actualiza bootloader, tabla de particiones y aplicación, pero no
escribe la partición `model`. Sustituya COM3 cuando el sistema asigne otro puerto.

## Diagnóstico de arranque

`app_main()` inicializa los motores en cero antes que el resto del hardware, abre
NVS, valida el modelo, inicializa sensores, conecta Wi-Fi y publica lecturas cada dos
segundos. Un sensor o modelo ausente no impide el funcionamiento de los demás
módulos. El monitor serie utiliza 115200 baudios.

En reposo, el acelerómetro debe medir aproximadamente 1 g sobre el eje alineado con
la gravedad. El giroscopio puede presentar un pequeño offset estacionario; debe
calibrarse antes de usarlo para navegación inercial acumulativa.

## Modelos TinyML

`partitions.csv` reserva una partición `model` de tipo `data`, subtipo `0x40`, offset
`0x600000` y tamaño máximo de 2 MiB. El firmware la localiza por nombre y subtipo,
sin acoplarse al offset fijo.

Sólo se admiten FlatBuffers TensorFlow Lite con identificador `TFL3`. Nunca se debe
grabar directamente un `.tflite`; primero se crea una imagen EIRM con versión,
longitud y CRC32:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py build modelo.tflite .pio/build/model.bin --version 1
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py flash .pio/build/model.bin --port COM3
```

El comando `flash` vuelve a validar cabecera, firma, límites y CRC antes de escribir.
Después del reinicio, el firmware informa versión, longitud y CRC; ante ausencia o
corrupción continúa operando con la inferencia deshabilitada.

## Estructura del repositorio

```text
include/                 APIs documentadas y mapa de pines
src/adapters/            Adaptación de sensores, red y actuadores
src/lsm6ds3tr_c/         Driver I2C del IMU
src/storage/             Preferencias persistentes NVS
src/tinyml/              Validación de la partición de modelo
tools/                   Generación de secretos y empaquetado/flasheo TinyML
partitions.csv           Distribución de la flash de 8 MB
platformio.ini           Entorno de compilación y carga
```

## Seguridad operativa

Mantenga el rover suspendido cuando pruebe motores, empiece con una magnitud baja y
termine siempre con `motor_adapter_stop()`. No alimente GPIO del ESP32 con 5 V. Los
pines GPIO12 y GPIO15 participan en el arranque del ESP32, por lo que el puente H no
debe forzar niveles incompatibles durante reset.
