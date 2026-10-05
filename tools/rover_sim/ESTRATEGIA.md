# Estrategia del firmware y validación

Validación del 5 de octubre de 2026. El simulador compila los mismos servicios C++ de navegación, competencia, planificación y movimiento que el ESP32. Las entregas se verifican con las cuatro esquinas del cubo dentro de su depósito y con la máscara de entrega del firmware después de la retirada.

## Cambios de comportamiento

La navegación de competencia puede continuar sin eco ultrasónico únicamente con visión válida y reciente y una ruta comprobada. Un error de hardware distinto de timeout mantiene la parada. La pérdida de visión o del compañero sigue deteniendo el movimiento y permite replantear al recuperar los datos.

El punto de aproximación queda a 166 mm del centro del cubo. La alineación comprueba el radio de giro de los brazos y el espacio del cubo antes de comenzar la captura; la ventana frontal admite hasta 6 celdas para tolerar el margen de llegada de navegación junto al umbral ultrasónico. La navegación conserva el cubo como obstáculo hasta terminar la aproximación. El frenado angular ahora calcula correctamente la velocidad hacia el objetivo tanto para giros positivos como negativos.

La asignación comprueba también el corredor de empuje hasta el depósito. Si un cubo bloquea otro, se busca primero una entrega accesible; no es obligatorio encontrar dos misiones simultáneas. Para evitar que las rutas se bloqueen al salir, el comandante espera hasta alejarse del punto de aproximación remoto; luego despacha la segunda misión mientras continúa con la suya. Así los rovers pueden navegar y trabajar en paralelo cuando hay espacio. El soldado recuerda el identificador de la última misión recibida por separado de sus replanteamientos locales, evitando repetir una orden ya completada.

Las maniobras junto al borde consideran el cuerpo de 95 × 100 mm y los brazos que sobresalen 55 mm hacia delante. Los tramos pueden recorrerse marcha atrás y el cambio de orientación se prepara antes de llegar al borde. La comprobación entre rovers utiliza sus envolventes orientadas cuando el compañero está disponible; un compañero activo conserva espacio adicional de maniobra.

La orientación durante el movimiento procede del estimador con giroscopio. La corrección angular de visión se aplica tras una ventana de reposo, para que una imagen retrasada no deshaga un giro reciente. La aproximación lenta separa sus pulsos al menos 150 ms. El simulador toma muestras ultrasónicas cada 200 ms, como la tarea de sensores del firmware, en lugar de contar una muestra nueva cada 10 ms.

## Resultados reproducibles

Escenarios oficiales, semilla 1, física predeterminada y paso de 10 ms. Los tiempos se cuentan desde el inicio del simulador e incluyen los tres segundos de READY. En cada caso se entregaron los tres cubos, ambos rovers terminaron, no hubo salidas de pista ni contacto entre rovers u obstáculos durante la ejecución, y permanecieron detenidos durante un segundo adicional.

| Escenario | Tiempo hasta entrega y retirada completas |
|---|---:|
| delivery | 46,45 s |
| crossing | 104,80 s |
| vision-loss | 55,05 s |
| peer-loss | 55,05 s |
| delays, 250 ms | 76,70 s |

Los registros de esta validación están en `.pio/sim/strategy-validation/`; `summary.json` enlaza los informes individuales. Cada ejecución conserva su escenario, manifiesto con hashes, entradas, salidas, posiciones físicas y registros del controlador. El test de reproducción exacta vuelve a ejecutar 800 pasos con las mismas entradas.

En la ejecución `delivery` de esta revisión, ambas misiones estuvieron activas a la vez entre 11,46 y 21,44 s, con comandos de motor simultáneos durante 1,85 s. Las tres entregas físicas y la retirada terminaron a los 46,45 s; no hubo contactos ni salidas de pista. `crossing` terminó sus tres entregas a los 104,80 s, también sin contactos ni salidas. Los informes completos están en `.pio/sim/tandem-after-20261005-e/` y `.pio/sim/tandem-crossing-20261005/`.

Validación ejecutada: `pio run` correcto para ESP32 con flash de 8 MB; 27 tests del simulador. Las regresiones incluyen frenado simétrico, margen de giro, ritmo de muestreo, error ultrasónico de hardware, parada ante pérdida de entradas, entregas físicas y rechazo de misiones repetidas.

```powershell
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py
& .pio/sim-venv/Scripts/python.exe tools/rover_sim.py --headless --scenario delivery --seconds 120 --output .pio/sim/strategy-delivery
& .pio/sim-venv/Scripts/python.exe -m pytest test/test_rover_sim.py -q
```

Cada revisión genera un ejecutable `rover_host-<hash>.exe`, permitiendo compilar mientras otra revisión sigue abierta en Windows. Para ver una estrategia nueva hay que iniciar una ejecución nueva del simulador; un proceso ya abierto conserva su controlador.

## Alcance y pendientes

Los escenarios `obstacles`, `blocked` y `edge` conservan pruebas de contrato y ejecución finita, pero no se consideran pruebas de tres entregas completas. Algunas posiciones de obstáculos de esos escenarios producen contacto inicial con los brazos; otros casos no ofrecen un corredor recto de empuje. La estrategia todavía no planifica empujes con varios cambios de dirección ni reubicaciones temporales de cubos.

Esta validación usa parámetros físicos estimados y no sustituye la prueba con los rovers reales, ruido de cámara, tracción medida y tareas concurrentes. El firmware compilado está listo para esa validación física; esta tarea no incluye una carga a las placas.


## Migración al contrato v3

El firmware y el host comparten `vision_contract_validate()` y aceptan únicamente v3. Cada cubo requiere `in_depot` booleano; la estrategia usa ese veredicto con detecciones frescas, sin recalcularlo a partir de coordenadas. El simulador publica el resultado de `geometria_depot()` y `cubo_en_depot()` del contrato con la tolerancia oficial de 2,5 mm guardada en el escenario. La entrega física sigue comprobándose con las cuatro esquinas del cubo.

Validación del 5 de octubre de 2026 con los servicios C++ actuales. Los cinco escenarios completaron tres entregas físicas, máscara combinada 7 y ambos rovers terminados y detenidos durante un segundo adicional, sin choques entre rovers u obstáculos ni salidas de pista en toda la traza.

| Escenario | Entrega y retirada completas con v3 |
|---|---:|
| delivery | 46,60 s |
| crossing | 115,25 s |
| vision-loss | 47,95 s |
| peer-loss | 46,30 s |
| delays | 67,00 s |

Los registros se conservan en `.pio/sim/v3-validation-20261005-b/`, con `summary.json`, escenarios, manifiestos, trazas, informes y logs C++. Sus hashes se compararon contra el código probado. Pasaron las 46 pruebas de simulación y cliente de visión, incluidas 7 regresiones de v3: rechazo de v2, veredicto ausente o no booleano, tolerancia del árbitro y obediencia al veredicto aunque las coordenadas indiquen lo contrario. La reproducción exacta cubre 800 pasos.

La ejecución de la seed 42 y dificultad 0,50 con v3 se conserva en `.pio/sim/v3-seed42-20261005-a/`: a 120 s no se asignaron misiones ni hubo entregas. La compatibilidad del contrato no garantiza que la estrategia resuelva todas las distribuciones aleatorias. La validación física sigue pendiente.

Compilación `pio run` correcta para ESP32 con flash de 8 MB. El binario incluye el validador compartido de v3.

## Recuperación de bloqueos y distribuciones aleatorias

La asignación retrasa la misión remota si su primer punto cruza la zona de maniobra del comandante. Si dos rovers activos quedan detenidos frente a frente, el soldado cede hacia una posición observada y replantea desde allí. Un rover aparcado conserva una zona de seguridad más amplia en el planificador de navegación. La estrategia comprueba la orientación del último tramo antes de aceptar un empuje intermedio, vuelve a observar el cubo tras empujarlo y puede posponer un cubo que acumula seis intentos fallidos. Los registros indican falta de ruta, falta de espacio para orientar el chasis o ausencia de una maniobra segura.

Con la seed 42 y dificultad 0,2, las tres entregas físicas, la máscara 7 y ambas retiradas terminaron a los 43,25 s; no hubo contactos entre rovers ni salidas de pista en 120 s. Se conservan escenario, manifiesto, traza, informe y registros en `.pio/sim/strategy-final-seed42-d020-20261005/`.

Con la seed 1 y dificultad 1, el cubo azul se acercó 498 mm a su depósito mediante empujes intermedios, pero ninguno de los tres quedó entregado a 120 s. El cubo verde cercano al borde sigue sin una aproximación con espacio suficiente para orientar el chasis; también bloquea el corredor final del azul. El controlador termina detenido y registra el límite, sin contactos entre rovers ni salidas de pista. La traza está en `.pio/sim/strategy-final-seed1-d100-20261005/`. Por tanto, esta revisión mejora la recuperación pero no garantiza tres entregas para todas las seeds. Hacen falta nuevas maniobras de contacto lateral y una comprobación de alcanzabilidad física antes de prometer esa garantía. No hay validación con placas reales.
