# EIRODENET

**EspressIdea Rover Delivery Network** es el firmware ESP-IDF del rover construido
para el [Vision Rover Challenge](https://github.com/Universidad-Cenfotec/Vision-Rover-Challenge).
La aplicación integra movimiento diferencial, percepción local, conectividad Wi-Fi,
preferencias persistentes y telemetría serial para configuración y diagnóstico.

El proyecto utiliza PlatformIO sobre una CRCibernética IdeaBoard con ESP32-WROOM-32E
y flash de 8 MB. Su punto de entrada es `src/main.cpp`; el hardware se consume mediante
adapters pequeños para que la lógica de navegación no dependa directamente de los
drivers de ESP-IDF.

## Estado del hardware probado

El firmware compila, se carga y funciona en el rover conectado por USB. La última
prueba confirmó conexión Wi-Fi, lectura estable del LSM6DS3TR-C, los cuatro TCRT5000
y distancias ultrasónicas cercanas a 0.55 m. Motor 1 y Motor 2 respondieron en ambos
sentidos durante una prueba controlada al 30 %. La prueba automática fue retirada:
la versión normal siempre arranca con ambos motores detenidos.

La fotoresistencia actualmente devuelve cero en las cuatro lecturas del adapter de
color. Se debe revisar su alimentación y el divisor resistivo conectado a GPIO32.

## Mapa de conexiones

`include/board_pins.hpp` es la fuente única de verdad del cableado. Cualquier cambio
físico debe reflejarse allí, sin introducir números GPIO en la lógica de aplicación.

| Componente | Señal | GPIO |
|---|---|---:|
| HY-SRF05 | Trigger | 4 |
| HY-SRF05 | Echo | 33 |
| Sensor de color | NeoPixel | 26 |
| Sensor de color | Fotoresistencia/ADC1 | 32 |
| TCRT5000 SEN1 | Delantero izquierdo / ADC1_CH7 | 35 |
| TCRT5000 SEN2 | Delantero derecho / ADC1_CH6 | 34 |
| TCRT5000 SEN3 | Trasero izquierdo / ADC1_CH3 | 39 |
| TCRT5000 SEN4 | Trasero derecho / ADC1_CH0 | 36 |
| LSM6DS3TR-C | SDA | 21 |
| LSM6DS3TR-C | SCL | 22 |
| LSM6DS3TR-C | Dirección I2C | `0x6B` |
| Motor 1 | Puente H, entrada A/B | 12 / 14 |
| Motor 2 | Puente H, entrada A/B | 13 / 15 |
| Botón de modo | BOOT, activo bajo | 0 |
| Indicador de modo | NeoPixel integrado | 2 |

Todos los GPIO del ESP32 trabajan con lógica de 3.3 V. La salida Echo de un
HY-SRF05 alimentado a 5 V debe contar con adaptación de nivel si la tarjeta no la
incorpora. GPIO34, GPIO35, GPIO36 y GPIO39 son sólo de entrada y no incluyen
resistencias pull-up/pull-down internas.

## Módulos

| Módulo | API pública | Responsabilidad |
|---|---|---|
| Storage | `app_storage.hpp` | Inicialización NVS, preferencias y credenciales Wi-Fi |
| Internet | `internet_adapter.hpp` | Estación Wi-Fi, reintentos, estado, RSSI e IPv4 |
| Ultrasónico | `ultrasonic_adapter.hpp` | Disparo del HY-SRF05 y distancia en milímetros |
| Infrarrojos | `infrared_adapter.hpp` | Lectura ADC de 12 bits conjunta de SEN1 a SEN4 |
| Color | `color_sensor_adapter.hpp` | Iluminación RGB y cuatro muestras ADC reflectivas |
| IMU adapter | `imu_adapter.hpp` | Interfaz singleton configurada desde el mapa de pines |
| Driver IMU | `lsm6ds3tr_c.hpp` | Registros I2C, identificación y conversión física |
| Motores | `motor_adapter.hpp` | PWM independiente, sentido y parada segura |
| Servicios | `rover_service.hpp` | Muestreo concurrente, calibración y fusión de orientación |
| Serial | `serial_protocol.hpp` | Configuración y telemetría NDJSON sobre UART0 |
| Modos | `app_mode.hpp` | Cambio con BOOT e indicador NeoPixel de prueba/competencia |
| Navegación | `navigation_service.hpp` | Fusión visión/IMU/cuadrícula y control punto a punto en núcleo 1 |
| Control manual | `manual_control_service.hpp` | Comandos seriales con parada de seguridad a 500 ms |
| Competencia | `competition_service.hpp` | BOOT local, preflight, verificación y estrategia autónoma |
| Comunicación par | `peer_comms_service.hpp` | Telemetría y comandos entre rovers mediante ESP-NOW |

Todas las APIs públicas incluyen documentación JavaDoc/Doxygen con parámetros,
valores de retorno, unidades y precondiciones.

## Control de motores

`motor_adapter_set(motor1, motor2)` recibe dos comandos independientes entre
`-1000` y `1000`. El signo selecciona el sentido, la magnitud controla el ciclo PWM
y cero deja el motor en rueda libre. Antes de invertir un motor, el adapter lleva
ambas entradas del puente H a cero para evitar conducción cruzada.
Todo comando no nulo inferior a `700` se eleva automáticamente a `700`, porque el
rover necesita al menos 70 % de PWM para vencer la fricción estática.

```c
ESP_ERROR_CHECK(motor_adapter_init());
ESP_ERROR_CHECK(motor_adapter_set(700, 0));  // Motor 1 al 70 %, Motor 2 detenido.
ESP_ERROR_CHECK(motor_adapter_stop());
```

Para comprobar el cableado, `motor_adapter_test_forward()` mueve ambos motores al
70 % hacia delante durante un segundo y los detiene automáticamente. Mantenga el
rover suspendido y confirme visualmente que ambas ruedas giren en el sentido correcto.

## Credenciales y storage

Las credenciales Wi-Fi se configuran mediante la aplicación de escritorio o el
protocolo serial y se guardan en NVS. El firmware no incorpora credenciales desde
`.env` durante la compilación.

## Preparación, compilación y carga

Se requiere PlatformIO Core y una conexión USB con el controlador CH340 disponible.
Desde PowerShell:

```powershell
$env:EIRO_PORT = "PUERTO_SERIAL"
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -t upload --upload-port $env:EIRO_PORT
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device monitor --port $env:EIRO_PORT --baud 115200
```

Una carga normal con el objetivo `upload` escribe únicamente el firmware y la
partición de aplicación. La configuración y el diagnóstico se realizan por UART.

## Diagnóstico de arranque

`app_main()` inicializa los motores en cero antes que el resto del hardware, abre
NVS e inicia los servicios de sensores, Wi-Fi, visión, ESP-NOW y protocolo serial.
El IMU se adquiere a 100 Hz; la telemetría solicitada por la GUI se publica a 20 Hz,
5 Hz y 1 Hz según el tema. Un sensor ausente no impide el funcionamiento
de los demás módulos. El monitor serie utiliza 115200 baudios.

En reposo, el acelerómetro debe medir aproximadamente 1 g sobre el eje alineado con
la gravedad. El giroscopio puede presentar un pequeño offset estacionario; debe
calibrarse antes de usarlo para navegación inercial acumulativa.

## Aplicación de configuración y diagnóstico

La aplicación de escritorio permite editar SSID, contraseña, IPv4/puerto del
servidor y MAC del rover compañero. También muestra valores y gráficas de 60
segundos para todos los sensores, el estado Wi-Fi y el modelo tridimensional
`tools/assets/Mini Rover.glb` orientado con el IMU.

En Windows, instale las dependencias y ejecute la interfaz desde la raíz:

```powershell
python -m venv tools\.venv
.\tools\.venv\Scripts\Activate.ps1
python -m pip install -r tools\requirements-gui.txt
python tools\rover_gui\main.py
```

En Linux:

```bash
python3 -m venv tools/.venv
source tools/.venv/bin/activate
python -m pip install -r tools/requirements-gui.txt
python tools/rover_gui/main.py
```

Seleccione el puerto de la IdeaBoard y pulse **Conectar**. La contraseña se
transmite por el enlace USB porque puede leerse de vuelta, pero siempre se muestra
enmascarada inicialmente; el botón **Mostrar/Ocultar** permite cambiar su
visibilidad. La pantalla también muestra como dato de solo lectura la MAC Wi-Fi STA
del propio rover. La preferencia `WHO_AM_I` identifica persistentemente la placa
como Rover 10 o Rover 11. Al cambiar las credenciales la placa confirma NVS y
reconecta la
estación Wi-Fi sin reiniciar el resto del firmware.

### Calibración IMU de seis caras

Abra la pestaña de calibración, pulse **Iniciar** y coloque sucesivamente el eje
indicado (`+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`) apuntando hacia arriba. Mantenga el
rover inmóvil durante los dos segundos de cada captura. Las seis caras deben ser
aceptadas antes de guardar; una captura rechazada no elimina las anteriores y
cancelar conserva la calibración persistente previa.

Al seleccionar una cara, el panel derecho anima el modelo 3D desde su posición
normal hasta el apoyo requerido. La tríada superpuesta identifica X en rojo, Y en
verde y Z en azul; la flecha amarilla permanece fija para indicar la vertical.

El visor permite fijar la orientación actual como cero. El LSM6DS3TR-C no contiene
magnetómetro, por lo que roll y pitch se corrigen con gravedad, pero el yaw puede
derivar lentamente.

Todas las lecturas y caras usan el marco del rover, no el marco impreso del chip.
Debido al montaje del IMU, la conversión aplicada es `X=-Xchip`, `Y=-Ychip` y
`Z=Zchip`. La versión de calibración se incrementa cuando cambia esta conversión,
por lo que parámetros antiguos incompatibles se descartan de forma segura.

La misma calibración puede ejecutarse sin interfaz gráfica. La CLI informa la
media y desviación de cada eje, la cara realmente detectada, el ángulo de
desalineación y las causas concretas de cualquier rechazo:

```text
python tools/rover_cli.py probe --seconds 4
python tools/rover_cli.py calibrate
python tools/rover_cli.py test-motors
```

La CLI detecta automáticamente un único adaptador CH340. Si hay varios puertos,
use `--port PUERTO_SERIAL`. En Linux el usuario debe pertenecer al grupo propietario
del dispositivo serie, habitualmente `dialout` o `uucp`.

Antes de cada captura el firmware descarta 0.5 segundos para permitir que el rover
termine de asentarse. La cara solicitada debe ser dominante y quedar dentro de
aproximadamente 30 grados de la vertical; se recomienda usar soportes o cuñas para
que las ruedas y la carrocería no determinen una inclinación incorrecta.

## Protocolo serial

UART0 opera a 115200 baudios. Cada trama de aplicación es una línea JSON precedida
por `@EIRO `; de este modo los consumidores pueden ignorar los logs ESP-IDF. La
versión actual es `1`, el límite es 1024 bytes y las peticiones contienen `id`,
`cmd` y, cuando corresponde, `data` o `face`.

Los comandos son `device.info`, `config.get`, `config.set`, `motors.test_forward`, `stream.start`,
`stream.stop`, `calibration.start`, `calibration.capture`, `calibration.commit` y
`calibration.cancel`. La telemetría se divide en los temas `imu` (20 Hz), `sensors`
(5 Hz) y `status` (1 Hz). Las respuestas repiten el identificador y contienen
`ok`, `data` o un objeto `error`; los cambios de calibración llegan como eventos.

## Cliente del sistema de visión

`tools/vision_client.py` consume la telemetría TCP/NDJSON oficial del challenge.
Si no encuentra un proceso escuchando, inicia el sistema desde el clon configurado
con su ventana de cámara y abre otro visor que dibuja exactamente las coordenadas
recibidas. Una franja verde confirma que el mensaje satisface el contrato del clon;
una roja permite diagnosticar datos antiguos, pero advierte que no deben usarse.

Configure la ruta local en `.env` (use `.env.example` como referencia):

```dotenv
VISION_CHALLENGE_REPO=C:\ruta\al\Vision-Rover-Challenge
VISION_HOST=127.0.0.1
VISION_PORT=2026
VISION_CAMERA_INDEX=0
VISION_CAMERA_PROFILE_INDEX=1
```

Los índices empiezan en cero, igual que los menús del sistema de visión. El índice
de perfil se resuelve contra los archivos de calibración ordenados y se entrega al
servidor como el nombre requerido por `--camara`.

Instale y abra el visor desde la raíz de EIRODENET:

```powershell
python -m venv tools\.venv
.\tools\.venv\Scripts\Activate.ps1
python -m pip install -r tools/requirements-gui.txt
python tools/vision_client.py
```

Para agentes y pruebas automatizadas, `--count` escribe únicamente NDJSON válido
en stdout y los diagnósticos en stderr. Sale con código `2` si recibió un mensaje
que no cumple el contrato y `3` ante un problema de conexión o configuración:

```powershell
python tools/vision_client.py --count 1
python tools/vision_client.py --count 20 > .pio\vision-snapshot.ndjson
```

El servidor iniciado por la herramienta se cierra al salir; `--keep-server` lo
conserva. Un servidor que ya existía nunca se termina. Para pruebas sin cámara se
puede agregar `--synthetic`; `--no-start` exige que el servidor ya esté activo.

## Estrategia autónoma de competencia

El firmware siempre inicia en modo prueba. Sólo una pulsación local de BOOT cambia a
competencia; una orden ESP-NOW remota nunca puede activar ese cambio. Cada transición
detiene los motores. Antes de declarar READY, el rover valida sensores sin mover los
motores y carga el trim diferencial guardado en NVS. La calibración de drift se inicia
únicamente desde la GUI en modo prueba; el ensayo mueve ambos motores durante 0,4 s y
guarda el ajuste sólo si la medición del IMU es válida. La longitud de celda se obtiene
del flujo de visión (`cell_mm`) y se usa directamente en la navegación. La consigna de
media potencia es 700/1000.

La competencia espera READY y sólo mueve motores en RUNNING. El comandante asigna
el cubo más cercano a su posición y el soldado el más lejano, salvo que la línea
del soldado a ese cubo, con ancho de 5 celdas, intersecte otro cubo modelado como
un círculo de 3 celdas de diámetro; en ese caso el soldado recibe el tercer cubo y
el cubo desplazado queda reservado. El comandante comienza a moverse 3 segundos
después de que el estado ESP-NOW confirme que el soldado comenzó a moverse. El
primer rover que confirma una entrega publica un evento ESP-NOW con
identificador, color y tiempo; el comandante arbitra por tiempo y luego por ID y
reasigna el cubo restante. Las notificaciones se repiten de forma idempotente.

La navegación de competencia gira con la IMU hacia el cubo y avanza a PWM 1000
si está a más de 6 celdas. Si en esa zona el ultrasónico detecta algo a 6 cm o
menos, gira 60 grados a la derecha, avanza 7 celdas y recalcula el rumbo. A 6
celdas o menos se acerca a PWM 700. Confirma la recogida si la distancia entre
centros es menor que 3,5 celdas en visión, o si el ultrasónico da timeout en la
zona próxima después de una lectura previa de al menos 7 cm. El
primer giro hacia el acopio usa impulsos de PWM 700 con la mitad del tiempo activo.
Para el acople, el rover se detiene después de cada nueva pose de visión,
recalcula el rumbo al centro del acopio y vuelve a avanzar mientras está a más de
2.5 celdas. A 2.5 celdas o menos empuja hacia el centro con la consigna mínima móvil
de 700/1000 PWM hasta que el contrato v3 confirme `cube.in_depot`. Ante un
acopio confirmado, ambos motores retroceden durante un segundo antes de que el rover
quede disponible o active su siguiente objetivo. Ante un solapamiento previsto entre
rovers, el comandante se detiene y el soldado conserva su movimiento durante un
segundo; si el solapamiento persiste, el soldado retrocede con consigna 700 hasta
liberar la envolvente. Si el soldado no progresa más de una celda durante tres
segundos, se detiene cinco segundos y reanuda. La planificación de cuadrícula
permanece disponible para pruebas, pero no participa en competencia.

La configuración y el diagnóstico se conservan por UART a 115200 baudios mediante
	ools/rover_gui y 	ools/rover_cli.py. La telemetría serial expone la fase de
preflight, trim, velocidades IR/IMU, objetivo directo, distancia restante, desvío
ultrasónico, estado de entrega y pausa del soldado.

## Estructura del repositorio

```text
include/                 APIs documentadas y mapa de pines
src/adapters/            Adaptación de sensores, red y actuadores
src/lsm6ds3tr_c/         Driver I2C del IMU
src/storage/             Preferencias persistentes NVS
src/services/            Lógica concurrente de sensores, visión y competencia
tools/                   GUI, CLI y herramientas de visión
partitions.csv           Distribución de la flash de 8 MB
platformio.ini           Entorno de compilación y carga
```

## Seguridad operativa

Mantenga el rover suspendido cuando pruebe motores, empiece con una magnitud baja y
termine siempre con `motor_adapter_stop()`. No alimente GPIO del ESP32 con 5 V. Los
pines GPIO12 y GPIO15 participan en el arranque del ESP32, por lo que el puente H no
debe forzar niveles incompatibles durante reset.
