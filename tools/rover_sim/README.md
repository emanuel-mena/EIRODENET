# Simulador de EIRODENET

Ejecuta navegaciÃ³n, A*, control de movimiento y estrategia de competencia **del cÃ³digo C++ real**, en dos procesos separados. Python/Pymunk representa el mundo fÃ­sico y PySide6 muestra la pista. No controla placas, cÃ¡mara, puertos serie ni el servidor de visiÃ³n.

## PreparaciÃ³n y uso

Desde la raÃ­z, en PowerShell:

```powershell
python -m venv tools\.venv
.\tools\.venv\Scripts\Activate.ps1
python -m pip install -r tools/requirements-sim.txt
python tools/rover_sim.py
```

Se requiere `g++` con C++20, disponible en PATH (por ejemplo MSYS2 UCRT64), y el paquete ESP-IDF instalado por PlatformIO. `CXX` permite seleccionar el compilador. El build reutiliza cJSON del paquete ESP-IDF y genera `.pio/sim/rover_host-<hash>.exe`. En Linux, usar `python3 -m venv tools/.venv`, `source tools/.venv/bin/activate` y g++ del sistema.

En este equipo tambiÃ©n quedÃ³ preparado `.pio/sim-venv/Scripts/python.exe` con las dependencias fijadas. Puede ejecutar directamente:

```powershell
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py
```

La ventana empieza pausada. Iniciar, Pausar, Un paso y Reiniciar controlan la ejecuciÃ³n. Cada reinicio conserva su registro en otra carpeta. El ritmo de pared depende de la PC; el reloj fÃ­sico siempre avanza exactamente 10 ms por paso.

```powershell
python tools/rover_sim.py --headless --scenario delivery --seconds 15 --output .pio/sim/delivery
python tools/rover_sim.py --headless --scenario vision-loss --seconds 15 --output .pio/sim/vision-loss
python tools/rover_sim.py --replay .pio/sim/delivery/trace.ndjson --output .pio/sim/replay
python tools/rover_sim.py --config .pio/sim/delivery/scenario.json --headless --seconds 15 --output .pio/sim/repeat
python -m pytest test/test_rover_sim.py -q
```

Escenarios: `delivery`, `crossing`, `obstacles`, `blocked`, `edge`, `vision-loss`, `peer-loss`, `delays`. Los escenarios de fallos cortan entradas durante el movimiento entre 3,2 y 5,5 segundos. Los parÃ¡metros fÃ­sicos, retardos y ruido se pueden cambiar en una copia del JSON de escenario y cargar con `--config`.

Cada ejecuciÃ³n guarda `scenario.json`, `manifest.json` con hashes del cÃ³digo/controlador/contrato, `trace.ndjson` con entradas y resultados de cada paso, los registros C++ de ambos rovers y `report.json` con transiciones y discrepancias de entrega. `--replay` vuelve a alimentar las mismas entradas a procesos nuevos y compara las salidas exactamente. Usar carpetas distintas en ejecuciones headless para conservar resultados anteriores.

## QuÃ© se ejecuta y quÃ© se supone

`navigation_service_tick()` es la misma iteraciÃ³n invocada por FreeRTOS en ESP32 y por el reloj del host. El host compila `navigation_service.cpp`, `competition_runtime.cpp`, `grid_planner.cpp` y `motion_control.cpp`. No usa la rÃ©plica Python `navigation_model.py` para conducir.

La simulaciÃ³n comienza con la verificaciÃ³n de red/identidad y calibraciÃ³n angular completadas; no ejecuta el arranque de `competition_service.cpp`, drivers, NVS, radio, adquisiciÃ³n Ã³ptica ni tareas concurrentes del ESP32. Es una prueba de la lÃ³gica de control, no una emulaciÃ³n completa del ESP32. Se ordena navegaciÃ³n antes de competencia en cada paso. Las carreras entre tareas quedan fuera de esta prueba.

El adaptador de visiÃ³n toma tramas validadas con `contrato/schema.py` del clon indicado por `VISION_CHALLENGE_REPO` en `.env`. El transporte del host es NDJSON sobre pipes, con lectura por lÃ­neas, nÃºmero de paso y tiempo lÃ­mite. Una trama rechazada detiene la simulaciÃ³n antes de entregarla al controlador. No se prueba aquÃ­ el parser TCP de producciÃ³n. El adaptador de compaÃ±eros reproduce fragmentos, confirmaciones y caducidad del enlace de 1500 ms, no las ondas de radio.

El sistema de coordenadas de visiÃ³n usa columnas hacia la derecha, filas hacia abajo y Ã¡ngulos antihorarios desde la derecha. La fÃ­sica usa X hacia la derecha e Y hacia arriba. La pose se refiere al centro del cuerpo de 95 mm; se supone que ese punto coincide con la referencia ArUco del rover.

El chasis tiene una envolvente rectangular de 100 Ã— 95 mm; no se modelan recortes de montaje ni ruedas que sobresalgan. Los motores están 20 mm delante del borde trasero, por lo que el eje queda 27,5 mm delante del centro del cuerpo. La velocidad diferencial se aplica en ese eje y el centro describe el arco correspondiente al girar. Los brazos son dos polÃ­gonos de 3 Ã— 55 mm, dentro del ancho total, con hueco de 94 mm. Son cuerpos solidarios al chasis; el cubo es otro cuerpo libre. El PDF suministrado determina la pista de 1000 mm, damero de 20 mm y cuatro marcadores, transcritos a celdas. El piso es plano, sin paredes artificiales: salir de la pista se registra como fallo. La vista suaviza los colores del damero para hacer visibles las rutas.

Masa, fricciÃ³n, respuesta al PWM y ancho entre ruedas son estimaciones configurables. La tracciÃ³n estÃ¡ limitada y los cubos tienen rozamiento con el piso. Hay al menos diez subpasos por ciclo, incrementados con la velocidad para limitar el desplazamiento de las puntas a aproximadamente 0,3 mm. No hay adhesiÃ³n, pinza activa, vuelcos ni flexiÃ³n. La IMU proporciona giro fÃ­sico; los infrarrojos leen el patrÃ³n bajo sus posiciones; el ultrasÃ³nico usa un rayo frontal desde el borde del cuerpo y devuelve timeout cuando no hay eco. Su cono real, zonas ciegas y errores de materiales requieren mediciones.

Los tests de contacto aplican comandos directos solo para validar la fÃ­sica de captura, giro, empuje y retirada. Las ejecuciones normales y headless siempre toman sus comandos del firmware.

## InterpretaciÃ³n del diagnÃ³stico

Las fases de competencia son 0 espera, 1 ruta, 2 navegaciÃ³n, 3 alineaciÃ³n, 4 captura, 5 empuje, 6 retirada, 7 terminado y 8 pausa segura. Los motivos numÃ©ricos de navegaciÃ³n corresponden a `navigation_failure_reason_t` en `include/navigation_service.hpp`.

`report.json` distingue la entrega declarada mediante la mÃ¡scara del firmware de la entrega fÃ­sica, que exige las cuatro esquinas del cubo dentro del depÃ³sito. Una diferencia momentÃ¡nea puede corresponder al tiempo de confirmaciÃ³n o retirada; el registro permite determinar cuÃ¡nto dura. La simulaciÃ³n conserva los fallos de estrategia: una ejecuciÃ³n que se detiene puede ser un resultado vÃ¡lido del diagnÃ³stico.

## Coordenadas oficiales de las zonas

Los escenarios nuevos cargan `vision/config_vision.json` del clon indicado por
`VISION_CHALLENGE_REPO` en `.env`. La cancha efectiva tiene 43 × 43 celdas de 20 mm;
el origen está en el centro del marcador superior izquierdo, a (70, 70) mm del
borde del tablero físico. La vista mantiene el tablero de 1000 × 1000 mm y marca
el límite efectivo de 860 × 860 mm. Rutas, objetos y tramas usan la misma conversión.

| Zona | Centro en celdas (col, row) | Centro físico en mm (x, fila) |
|---|---|---|
| Verde, arriba | (21.5, 3.75) | (500, 145) |
| Roja, derecha | (39.25, 21.5) | (855, 500) |
| Azul, abajo | (21.5, 39.25) | (500, 855) |
| Salida, izquierda | (3.75, 21.5) | (145, 500) |

Los JSON de ejecuciones anteriores conservan su geometría para reproducirlas;
para usar las zonas corregidas, iniciar un escenario nuevo sin `--config` antiguo.

Los dos rovers comienzan a la izquierda, en (145, 350) y (145, 650) mm,
mirando a la derecha (0 grados), separados 300 mm entre centros.


## Posiciones por seed de Vision Rover Challenge

En `docs/index.html` del repositorio indicado por `VISION_CHALLENGE_REPO`, introduce
una seed (entero de 0 a 4294967295) y la dificultad (0 a 1), y pulsa **Aplicar seed y
dificultad**. Usa esos mismos valores en el simulador:

```powershell
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py --seed 42 --difficulty 0.50
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py --seed 42 --difficulty 0.50 --headless --seconds 120 --output .pio/sim/seed-42-d050
```

La ventana también permite introducir **Seed** y **Dificultad** y pulsar **Aplicar
posiciones**; crea otra ejecución pausada con su propio registro. **Reiniciar**
conserva las posiciones. Sin `--seed` se mantienen los escenarios de regresión.
La misma seed y dificultad reproducen los mismos cubos con Mulberry32; cambia la
seed para obtener otra distribución. Las coordenadas de la página se giran 90°
en sentido horario para que su salida inferior coincida con la salida izquierda
del simulador, conservando el color de cada cubo y las zonas oficiales del firmware.
La seed reproduce la distribución original, antes de mover cubos manualmente.
`scenario.json` conserva posiciones, seed, dificultad y coordenadas de la página;
`--config` reproduce ese archivo y no se combina con `--seed` ni `--replay`.
Una distribución aleatoria puede bloquear la estrategia; no garantiza tres entregas.

## Estrategia actual

El firmware y el host aceptan exclusivamente el contrato v3 y comparten el
validador C++. Cada cubo requiere `in_depot` booleano; la estrategia confirma la
entrega con ese veredicto del árbitro únicamente mientras la detección es fresca.
El simulador usa `geometria_depot()` y `cubo_en_depot()` del clon de visión para
publicarlo, con `conteo_acopio.tolerancia_mm` guardada como `referee_tolerance_mm`
en el escenario. La comprobación física de las cuatro esquinas sigue siendo
independiente: puede discrepar del criterio conservador del árbitro y su tolerancia.
Los registros contienen el veredicto en las entradas de visión y la entrega física
en el estado del mundo. Las trazas v2 requieren el controlador anterior para su
reproducción exacta; los escenarios JSON conservados pueden ejecutarse con v3.

Consulte [ESTRATEGIA.md](ESTRATEGIA.md) para los cambios del firmware, resultados de entrega completa y limitaciones. El ultrasónico simulado actualiza sus muestras cada 200 ms.
