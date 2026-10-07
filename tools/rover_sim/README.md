# Simulador de EIRODENET

La simulación recupera la física y la GUI de `dbaf934` y ejecuta el `competition_runtime.cpp` actual en dos procesos de host. El mundo Pymunk modela el chasis, los brazos, los cubos y los obstáculos. Cada proceso recibe visión v3 validada, IMU, infrarrojos, ultrasonido y mensajes del compañero. La GUI muestra el objetivo de la fase actual y los contactos.

## Preparación

En Windows, desde la raíz del proyecto:

```powershell
python -m venv tools\.venv
.\tools\.venv\Scripts\Activate.ps1
python -m pip install -r tools/requirements-sim.txt
python tools/rover_sim.py
```

En Linux use `python3 -m venv tools/.venv`, active `tools/.venv/bin/activate` e instale el mismo archivo. Se requiere `g++` con C++20 y cJSON del ESP-IDF que instala PlatformIO. Configure `CXX` y `PLATFORMIO_CORE_DIR` si sus rutas difieren. `VISION_CHALLENGE_REPO` debe apuntar al clon con `vision/config_vision.json` y `contrato/schema.py`.

```powershell
python tools/rover_sim.py --headless --scenario delivery --seconds 30 --output .pio/sim/delivery
python tools/rover_sim.py --replay .pio/sim/delivery/trace.ndjson --output .pio/sim/replay
python -m pytest test/test_rover_sim.py -q
```

Puede usar `--seed` y `--difficulty` para posiciones reproducibles, `--config` para repetir un escenario guardado y `--no-vision-entropy` para desactivar las fluctuaciones medidas. Los escenarios son `delivery`, `crossing`, `obstacles`, `blocked`, `edge`, `vision-loss`, `peer-loss` y `delays`. La ventana comienza pausada y permite iniciar, pausar, avanzar un paso de 10 ms, reiniciar y cambiar posiciones.

Cada ejecución guarda `scenario.json`, `manifest.json`, `trace.ndjson`, `report.json` y registros de ambos procesos C++. El reporte distingue las entregas declaradas por el firmware de los cubos físicamente dentro del depósito. La reproducción compara las salidas del controlador con las entradas guardadas.

## Alcance

El host compila la estrategia de competencia, el control de giro y el validador de visión del firmware actual. Emula las llamadas a motores, NVS, radio, sensores y reloj. Supone que la verificación y la calibración previas terminaron correctamente. El estado angular del servicio de navegación se aproxima con la última orientación de visión; no se ejecuta su estimador ni su tarea. Tampoco se modelan la cámara real, el servidor TCP, la radio física ni los efectos eléctricos. Los parámetros de tracción y ruido son estimaciones configurables. Una entrega simulada no sustituye las pruebas con los rovers físicos.

Los estados numéricos de competencia corresponden a `exec_phase_t` en `src/services/competition_runtime.cpp`: espera, hacia cubo, giro y avance de desvío, alineación y trayecto al depósito, empuje, terminado, espera segura, pausa por atasco, espera de visión y retirada. El objetivo que se ve en la GUI proviene de `competition_runtime_get_status()`.

## Diagnóstico de la estrategia actual

En una ejecución de 60 s del escenario `delivery` con parámetros predeterminados, el controlador no entregó cubos y el reporte registró salidas de pista y contactos entre rovers. La reproducción de sus 6000 pasos coincidió exactamente. La condición de captura del firmware usa menos de 3,5 celdas (70 mm) entre centros como una confirmación visual, mientras la geometría simulada coloca el primer contacto frontal con un cubo de 60 mm alrededor de 77,5 mm desde el centro del cuerpo. La otra confirmación requiere un timeout ultrasónico después de una lectura despejada. En una captura frontal el rayo puede seguir viendo el cubo, así que esa transición merece revisión y validación física antes de esperar entregas completas.
