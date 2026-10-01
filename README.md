# EIRODENET

**EspressIdea Rover Delivery Network** es el firmware ESP-IDF del rover construido
para el [Vision Rover Challenge](https://github.com/Universidad-Cenfotec/Vision-Rover-Challenge).
La aplicación integra movimiento diferencial, percepción local, conectividad Wi-Fi,
preferencias persistentes y un sitio web estático servido desde la propia placa.

El proyecto utiliza PlatformIO sobre una CRCibernética IdeaBoard con ESP32-WROOM-32E
y flash de 8 MB. Su punto de entrada es `src/main.c`; el hardware se consume mediante
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

`include/board_pins.h` es la fuente única de verdad del cableado. Cualquier cambio
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
| Storage | `app_storage.h` | Inicialización NVS, preferencias y credenciales Wi-Fi |
| Internet | `internet_adapter.h` | Estación Wi-Fi, reintentos, estado, RSSI e IPv4 |
| Ultrasónico | `ultrasonic_adapter.h` | Disparo del HY-SRF05 y distancia en milímetros |
| Infrarrojos | `infrared_adapter.h` | Lectura ADC de 12 bits conjunta de SEN1 a SEN4 |
| Color | `color_sensor_adapter.h` | Iluminación RGB y cuatro muestras ADC reflectivas |
| IMU adapter | `imu_adapter.h` | Interfaz singleton configurada desde el mapa de pines |
| Driver IMU | `lsm6ds3tr_c.h` | Registros I2C, identificación y conversión física |
| Motores | `motor_adapter.h` | PWM independiente, sentido y parada segura |
| Sitio local | `local_site_service.h` | Montaje SPIFFS, HTTP y anuncio mDNS |
| Servicios | `rover_service.h` | Muestreo concurrente, calibración y fusión de orientación |
| Serial | `serial_protocol.h` | Configuración y telemetría NDJSON sobre UART0 |
| Modos | `app_mode.h` | Cambio con BOOT e indicador NeoPixel de prueba/competencia |
| Navegación | `navigation_service.h` | Fusión visión/IMU/cuadrícula y control punto a punto en núcleo 1 |
| Control manual | `manual_control_service.h` | Comandos web con parada de seguridad a 500 ms |
| Competencia | `competition_service.h` | Verificación de red, identidad y función antes de la estrategia autónoma |
| Comunicación par | `peer_comms_service.h` | Telemetría y comandos entre rovers mediante ESP-NOW |

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

Una carga normal con el objetivo `upload` ejecuta el build de Vite, genera la imagen
SPIFFS y escribe firmware y sitio web en la misma operación. Node.js y npm deben
estar disponibles en el equipo de desarrollo.

## Diagnóstico de arranque

`app_main()` inicializa los motores en cero antes que el resto del hardware, abre
NVS e inicia los servicios de sensores, Wi-Fi, sitio local y protocolo serial.
El IMU se adquiere a 100 Hz; la telemetría solicitada por la GUI se publica a 20 Hz,
5 Hz y 1 Hz según el tema. Un sensor ausente no impide el funcionamiento
de los demás módulos. El monitor serie utiliza 115200 baudios.

En reposo, el acelerómetro debe medir aproximadamente 1 g sobre el eje alineado con
la gravedad. El giroscopio puede presentar un pequeño offset estacionario; debe
calibrarse antes de usarlo para navegación inercial acumulativa.

## Aplicación de configuración y diagnóstico

La aplicación de escritorio permite editar SSID, contraseña, IPv4/puerto del
servidor, MAC del rover compañero y la preferencia `LOCAL_SITE`. También muestra valores y gráficas de 60
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

`LOCAL_SITE` guarda el hostname mDNS propio de cada rover. Por ejemplo, los valores
`rover-10` y `rover-11` publican `http://rover-10.local` y
`http://rover-11.local` sin conflictos. Un valor vacío desactiva el sitio. Cuando
está configurado, la placa monta `static`, inicia HTTP en el puerto 80 y anuncia el
nombre guardado. El equipo cliente debe estar en la misma red y soportar mDNS.

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

## Sitio web local

`partitions.csv` reserva la partición SPIFFS `static` en `0x400000`, con 2 MiB, y
conserva la partición TinyML `model` (`data/0x40`) en `0x600000`, con 2 MiB. El
proyecto Vite vanilla está en `web/` y genera sus archivos optimizados en `data/`.
`tools/build_web.py` se ejecuta como script previo de PlatformIO al cargar firmware,
compila la página, crea `.pio/build/esp32dev/spiffs.bin` y la agrega a la misma
operación de escritura. Para desarrollo aislado de la interfaz:

```powershell
cd web
npm install
npm run dev
```

La página sólo se conecta por HTTP al rover que la sirve, sin asumir nombres como
`rover-10.local` o `rover-11.local`. Ese rover expone su propio estado y actúa como
puente hacia el compañero mediante ESP-NOW y la MAC guardada en `peer_mac`. Ambos
rovers publican un paquete binario de estado ESP-NOW v6 cada 200 ms, incluida la
ruta discreta; tras 1500 ms sin paquetes,
el panel del compañero se marca desconectado y bloquea telemetría y comandos. Por
eso se puede abrir, por ejemplo, `http://cecilio.local` y operar ambos paneles sin
que el navegador conozca la URL o IP del segundo rover.

Al entrar en modo competencia, cada rover comprueba Wi-Fi con IPv4, espera hasta
cinco segundos una trama válida del servidor TCP configurado, confirma el enlace ESP-NOW en cinco segundos y consulta
`WHO_AM_I` del compañero en otros cinco segundos. Las identidades deben ser Rover
10 y Rover 11. Cada placa compara localmente su MAC STA con la MAC configurada del
compañero como enteros de 48 bits: la mayor es comandante y la menor soldado. Hasta
terminar la verificación los motores permanecen detenidos. Un fallo queda fijado
hasta salir y volver a entrar en competencia; el NeoPixel integrado muestra tantos
destellos rojos como el paso fallido (1 a 5), de 200 ms encendido y 200 ms apagado,
con un segundo de pausa entre grupos. En éxito conserva ámbar para Rover 10 y
violeta para Rover 11. El estado HTTP expone `competition_check` con paso, error,
resultado y función.

Durante la primera fase `READY` posterior a esa verificación, cada rover permanece
detenido hasta tres segundos para reunir cinco poses propias frescas con capturas
distintas. Si está quieto y mirando desde la salida del lado izquierdo hacia el
interior, calcula una sola vez el desfase entre el rumbo visual medio y 0 grados.
Lo aplica únicamente a la orientación, lo comunica al compañero en el estado
ESP-NOW y lo muestra en `navigation.pose` como `vision_heading_offset_deg` y
`vision_heading_calibrated`. Si faltan muestras, varían demasiado o termina
`READY`, continúa con el rumbo del servidor y `vision_heading_calibrated=false`.
En ese caso, el NeoPixel parpadea a 1 Hz en el color de identidad del rover
(ámbar para Rover 10, violeta para Rover 11); una calibración válida lo deja fijo.
Los destellos rojos siguen reservados para fallos de verificación.

| Destellos rojos por grupo | Paso fallido | Qué significa y qué revisar |
|---:|---|---|
| 1 | Wi-Fi | El rover no obtuvo conexión con dirección IPv4 en 15 segundos. Revise las credenciales, el punto de acceso y la asignación de IP. |
| 2 | Datos del servidor | No llegó una trama válida del servidor TCP en 5 segundos. Revise `server_ipv4`, `server_port`, la conexión de red y el contrato v2. No se requiere que el servidor responda a ping. |
| 3 | Enlace ESP-NOW | No llegó la respuesta del compañero en 5 segundos. Revise su alimentación, la MAC `peer_mac`, el canal Wi-Fi y que ambos tengan el protocolo ESP-NOW v6. |
| 4 | `WHO_AM_I` | No llegó la identidad en 5 segundos, es inválida o coincide con la propia. Configure uno como Rover 10 y el otro como Rover 11. |
| 5 | Comparación de MAC | No se pudo leer la MAC STA propia, falta la MAC del compañero o ambas son iguales. Revise `peer_mac` en la configuración. |

Los grupos se repiten hasta salir del modo competencia. La luz permanece apagada
durante las comprobaciones y vuelve al azul pulsante al entrar en modo prueba.

En modo prueba la web muestra IMU, sensores, red, la pose fusionada y una flecha 3D
de Three.js. Permite control diferencial directo y acepta objetivos decimales `(col, row)`
para navegación sobre una ruta A* de ocho direcciones. Los comandos destinados al compañero atraviesan ESP-NOW y se validan
otra vez en el rover receptor. En modo competencia los controles manuales quedan
deshabilitados.

La navegación exige primero una pose v2 fresca del servidor configurado, una IMU
calibrada y una lectura válida de infrarrojos. Si falta cualquiera de estas
precondiciones, la API rechaza el objetivo sin activar los motores. Un objetivo puede
quedar aceptado durante un timeout ultrasónico, pero los motores permanecen detenidos
hasta recuperar una lectura válida. Después
predice a 100 Hz con el giroscopio y usa los cuatro TCRT5000 para confirmar cruces sobre la
cuadrícula cuyo `cell_mm` publica visión; aprende automáticamente los dos niveles de cada sensor y su
polaridad, pero un patrón infrarrojo nunca sustituye directamente la pose continua. Las
retransmisiones con el mismo `ts_ms` no se vuelven a fusionar como si
fueran capturas nuevas y la pose caduca después de 750 ms sin una captura nueva.
Patrones ambiguos no corrigen la pose. El planificador evita obstáculos visuales y al
otro rover, mantiene rumbos múltiplos de 45 grados y no corta esquinas bloqueadas. Si
cae la visión continúa como máximo dos cruces confirmados con la cuadrícula calibrada;
después se detiene y replantea al recuperar una captura. La celda visual usa 0.12
celdas de histéresis en cada frontera y la ruta conserva el desplazamiento fraccional
inicial, por lo que no ordena ir al centro antes de comenzar A*. Dentro de tres
celdas del waypoint el avance se aplica en pulsos de 40 ms sincronizados con capturas
nuevas, y la llegada exige cinco capturas frescas dentro de 0.4 celdas con velocidad
menor o igual a 0.35 celdas/s. Un obstáculo
ultrasónico a 150 mm, un fallo de IMU/IR, tres lecturas ultrasónicas inválidas
consecutivas, un cambio de modo o un mando manual detienen y cancelan el movimiento;
una lectura ultrasónica inválida aislada sólo lo pausa. Se puede detener explícitamente
con `POST /api/v1/navigation/cancel`.

El firmware siempre inicia en modo prueba. Una pulsación de BOOT alterna entre
prueba y competencia y detiene los motores. GPIO2 pulsa en azul durante prueba; en
competencia queda amarillo para Rover 10 y morado para Rover 11. Una identidad sin
configurar se señala en rojo. En competencia, el comandante espera `READY`, toma
poses, cubos y depósitos frescos y asigna un cubo a cada rover. Envía al soldado
una ruta fragmentada por ESP-NOW y espera confirmación de cada fragmento. Los
motores permanecen detenidos hasta `RUNNING`. Cada rover navega hacia un punto
detrás de su cubo, se alinea y lo introduce entre los brazos con pulsos lentos.
El acoplo exige tres lecturas ultrasónicas de 30 mm o menos, o tres timeouts
acompañados de una posición visual fresca del cubo dentro de la abertura.
Después empuja hacia el centro del depósito de su color, confirma durante cinco
capturas nuevas que el cubo completo quedó dentro y retrocede antes de girar.
Al completarse la primera
entrega, el comandante asigna el tercer cubo al rover libre con la ruta de
aproximación más corta. La navegación de prueba sigue usando sus controles
manuales, que permanecen deshabilitados en competencia.

## Estructura del repositorio

```text
include/                 APIs documentadas y mapa de pines
src/adapters/            Adaptación de sensores, red y actuadores
src/lsm6ds3tr_c/         Driver I2C del IMU
src/storage/             Preferencias persistentes NVS
src/services/            Lógica concurrente y servidor del sitio local
web/                     Proyecto Vite vanilla del sitio embebido
tools/                   GUI, CLI y automatización del build web
partitions.csv           Distribución de la flash de 8 MB
platformio.ini           Entorno de compilación y carga
```

## Seguridad operativa

Mantenga el rover suspendido cuando pruebe motores, empiece con una magnitud baja y
termine siempre con `motor_adapter_stop()`. No alimente GPIO del ESP32 con 5 V. Los
pines GPIO12 y GPIO15 participan en el arranque del ESP32, por lo que el puente H no
debe forzar niveles incompatibles durante reset.
