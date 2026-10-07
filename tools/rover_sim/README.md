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

La aproximación actual apunta directamente al cubo asignado, avanza con ambos motores a la misma potencia durante tramos de hasta tres celdas y comprueba de nuevo el rumbo entre tramos. Frena al entrar en la envolvente de contacto antes de la fase final de captura. En una ejecución de 60 s con seed 1 y dificultad 0,2, dos cubos terminaron físicamente en sus depósitos y fueron declarados por el firmware; el tercero quedó pendiente. La simulación no demuestra que los brazos retengan el cubo durante un giro real, por lo que siguen pendientes las pruebas físicas de captura y entrega.
