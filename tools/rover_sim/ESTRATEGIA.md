# Estrategia del firmware y validación

Validación del 5 de octubre de 2026. El simulador compila los mismos servicios C++ de navegación, competencia, planificación y movimiento que el ESP32. Las entregas se verifican con las cuatro esquinas del cubo dentro de su depósito y con la máscara de entrega del firmware después de la retirada.

## Cambios de comportamiento

La navegación de competencia puede continuar sin eco ultrasónico únicamente con visión válida y reciente y una ruta comprobada. Un error de hardware distinto de timeout mantiene la parada. La pérdida de visión o del compañero sigue deteniendo el movimiento y permite replantear al recuperar los datos.

El punto de aproximación queda a 166 mm del centro del cubo. La alineación comprueba el radio de giro de los brazos y el espacio del cubo antes de comenzar la captura. La navegación conserva el cubo como obstáculo hasta terminar la aproximación. El frenado angular ahora calcula correctamente la velocidad hacia el objetivo tanto para giros positivos como negativos.

La asignación comprueba también el corredor de empuje hasta el depósito. Si un cubo bloquea otro, se busca primero una entrega accesible; no es obligatorio encontrar dos misiones simultáneas. El comandante despacha una misión activa a la vez, para evitar que dos rutas o empujes se bloqueen mutuamente. El soldado recuerda el identificador de la última misión recibida por separado de sus replanteamientos locales, evitando repetir una orden ya completada.

Las maniobras junto al borde consideran el cuerpo de 95 × 100 mm y los brazos que sobresalen 55 mm hacia delante. Los tramos pueden recorrerse marcha atrás y el cambio de orientación se prepara antes de llegar al borde. La comprobación entre rovers utiliza sus envolventes orientadas cuando el compañero está disponible; un compañero activo conserva espacio adicional de maniobra.

La orientación durante el movimiento procede del estimador con giroscopio. La corrección angular de visión se aplica tras una ventana de reposo, para que una imagen retrasada no deshaga un giro reciente. La aproximación lenta separa sus pulsos al menos 150 ms. El simulador toma muestras ultrasónicas cada 200 ms, como la tarea de sensores del firmware, en lugar de contar una muestra nueva cada 10 ms.

## Resultados reproducibles

Escenarios oficiales, semilla 1, física predeterminada y paso de 10 ms. Los tiempos se cuentan desde el inicio del simulador e incluyen los tres segundos de READY. En cada caso se entregaron los tres cubos, ambos rovers terminaron, no hubo salidas de pista ni contacto entre rovers u obstáculos durante la ejecución, y permanecieron detenidos durante un segundo adicional.

| Escenario | Tiempo hasta entrega y retirada completas |
|---|---:|
| delivery | 60,35 s |
| crossing | 104,80 s |
| vision-loss | 55,05 s |
| peer-loss | 55,05 s |
| delays, 250 ms | 76,70 s |

Los registros de esta validación están en `.pio/sim/strategy-validation/`; `summary.json` enlaza los informes individuales. Cada ejecución conserva su escenario, manifiesto con hashes, entradas, salidas, posiciones físicas y registros del controlador. El test de reproducción exacta vuelve a ejecutar 800 pasos con las mismas entradas.

Validación ejecutada: `pio run` correcto para ESP32 con flash de 8 MB; 27 tests del simulador; 30 tests de GUI y modelo de navegación; ejecutable C++ de control de movimiento y geometría. Las regresiones incluyen frenado simétrico, margen de giro, ritmo de muestreo, error ultrasónico de hardware, parada ante pérdida de entradas, entregas físicas y rechazo de misiones repetidas.

```powershell
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py --headless --scenario delivery --seconds 120 --output .pio/sim/strategy-delivery
& .pio/sim-venv/Scripts/python.exe -m pytest test/test_rover_sim.py -q
```

Cada revisión genera un ejecutable `rover_host-<hash>.exe`, permitiendo compilar mientras otra revisión sigue abierta en Windows. Para ver una estrategia nueva hay que iniciar una ejecución nueva del simulador; un proceso ya abierto conserva su controlador.

## Alcance y pendientes

Los escenarios `obstacles`, `blocked` y `edge` conservan pruebas de contrato y ejecución finita, pero no se consideran pruebas de tres entregas completas. Algunas posiciones de obstáculos de esos escenarios producen contacto inicial con los brazos; otros casos no ofrecen un corredor recto de empuje. La estrategia todavía no planifica empujes con varios cambios de dirección ni reubicaciones temporales de cubos.

Esta validación usa parámetros físicos estimados y no sustituye la prueba con los rovers reales, ruido de cámara, tracción medida y tareas concurrentes. El firmware compilado está listo para esa validación física; esta tarea no incluye una carga a las placas.
