# Política TinyML cooperativa

La autonomía usa una sola política para ambos roles. El comandante asigna sólo
`{assignment_id, color}` mediante el protocolo v9; cada rover crea su observación
local y decide velocidad lineal y angular. Las fases, la seguridad, la cesión del
soldado, los temporizadores, la confirmación de entregas y la reasignación son
lógica C++ determinista.

El contrato del modelo es `int8 [1,68] -> int8 [1,2]`, con arquitectura
`Dense(64, ReLU) -> Dense(64, ReLU) -> Dense(2, tanh)`. Las salidas representan
velocidad lineal y angular normalizadas. El firmware reserva una arena fija de
32 KiB y bloquea la autonomía si el modelo falta, es inválido o no coincide en
versión y CRC con el otro rover.

La flash de 8 MiB reserva `factory` en `0x20000` con tamaño `0x4E0000`, `static`
en `0x500000` con tamaño `0x100000`, y `model` como `data/0x40` en `0x600000`
con tamaño `0x200000`. Una carga normal de firmware conserva `model`.

## Construcción y carga

Sólo se flashea una imagen EIRM validada:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py build modelo.tflite .pio/build/model.bin --version 1
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py flash .pio/build/model.bin --port COM5
```

La cabecera EIRM tiene 16 bytes: magic, versión `uint32`, longitud `uint32` y
CRC32 `uint32`, todos los enteros little-endian. El contenido debe ser un
FlatBuffer con identificador `TFL3`.

## Simulación y evaluación

TensorFlow permanece fuera del entorno básico:

```powershell
& .pio/sim-venv/Scripts/python.exe -m pip install -r tools/requirements-tinyml.txt
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py --headless --scenario delivery --seconds 120 --model modelo.tflite --domain-randomization --randomization-seed 1009 --output .pio/sim/modelo-1009-delivery
```

Cada inferencia registra observación, entrada y salida cuantizadas y acción
aplicada en `trace.ndjson`. Los reportes incluyen tiempos, máscaras física y
declarada, contactos, salidas, movimiento simultáneo, cesiones, bloqueos,
reasignaciones, recuperación después de fallos y SHA-256 del modelo.

Los conjuntos disjuntos de semillas están en
`tools/rover_sim/tinyml_splits.json`. La interfaz de evaluación ejecuta los cinco
escenarios obligatorios y ordena candidatos elegibles por tiempo medio y
percentil 95:

```powershell
& .pio/sim-venv/Scripts/python.exe tools/evaluate_tinyml.py modelo-1.tflite modelo-2.tflite --split classification --output .pio/sim/clasificacion
```

## Laboratorio gráfico de entrenamiento

La aplicación de escritorio entrena en un proceso separado, actualiza en vivo el
top 10, permite ejecutar visualmente cualquier candidato y construye/flashea su
imagen EIRM sin aceptar un `.tflite` directo. Prepare un entorno con todas sus
dependencias y ábrala desde la raíz:

```powershell
python -m venv tools\.venv
& .\tools\.venv\Scripts\python.exe -m pip install -r tools\requirements-tinyml-gui.txt
& .\tools\.venv\Scripts\python.exe tools\tinyml_gui\main.py
```

El entrenamiento parte de un profesor geométrico y evoluciona sus pesos. Los
episodios consumen, en orden, las seeds del conjunto `training` de
`tinyml_splits.json`; todos los candidatos de un episodio reciben exactamente la
misma colocación y dominio. Clasificación y calificación usan sus conjuntos
separados y nunca reutilizan las seeds de entrenamiento. Cada candidato se
convierte inmediatamente al contrato int8 fijo y se ejecuta en la
física parametrizada; el top 10 se persiste atómicamente en `leaderboard.json`.
Las carpetas `runs/<candidato>/episode-*` contienen manifiesto, traza, reporte y
logs C++ para auditar su puntaje. La detención solicitada desde la GUI se atiende
durante el episodio y conserva los candidatos ya clasificados.

La visualización siempre crea una ejecución nueva bajo `visualizations/`. El
botón de flasheo ejecuta primero `model_tool.py build`, guarda la imagen validada
en `.pio/build` y sólo entonces llama a `model_tool.py flash` para el puerto
elegido. Tras flashear hay que reiniciar la placa y confirmar por monitor serie a
115200 baudios que versión, longitud y CRC coinciden en ambos rovers.

La entropía sembrada coloca físicamente los tres cubos con el generador oficial
`mulberry32-v1` y después muestrea motores, masas, velocidad, tracción, respuesta,
fricción, desaceleración del cubo, retardos y ruido de sensores con la misma seed.
También se conservan los residuos temporales de visión. Cada manifiesto registra
la seed, dificultad, posiciones realizadas y algoritmos. Los resultados sólo se
comparan cuando los hashes del manifiesto corresponden al mismo código.

Las regresiones que exigen movimiento usan el candidato indicado en
`TINYML_TEST_MODEL`; sin esa variable se omiten de forma explícita, mientras las
pruebas de seguridad verifican que la autonomía permanezca detenida:

```powershell
$env:TINYML_TEST_MODEL = (Resolve-Path .\modelo.tflite)
& .pio/sim-venv/Scripts/python.exe -m pytest test/test_rover_sim.py -q
```
