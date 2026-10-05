# Diagnóstico de la primera simulación

Este documento conserva el diagnóstico previo a las correcciones. El comportamiento actualizado y sus pruebas se documentan en [ESTRATEGIA.md](ESTRATEGIA.md).

Ejecución del 5 de octubre de 2026. Ocho escenarios de 15 segundos, paso de 10 ms, semilla 1 y parámetros físicos estimados predeterminados. Los registros completos están en `.pio/sim/acceptance/`; se pueden regenerar con los comandos de la guía. No se modificó la estrategia para hacer pasar estos escenarios.

## Bloqueo entre aproximación y alineación

En `delivery`, ambos rovers reciben misión, avanzan y llegan a sus puntos de aproximación. A los 6,11 segundos entran en alineación y a los 6,12 segundos pasan a `EXEC_HOLD`. El centro de cada rover queda aproximadamente a 126,75 mm del centro de su cubo; ninguno se entrega.

En `competition_runtime.cpp`, `STAGE_DISTANCE` coloca el último punto a 6,5 celdas (130 mm) del cubo. La rama `EXEC_ALIGN` ordena `hold()` cuando la distancia al cubo es menor que 8,2 celdas (164 mm), incluso con orientación correcta. El punto de aproximación normal cae dentro de esa condición. El replanteamiento desde esa posición tampoco consigue salir del bloqueo en este escenario. Esto es un conflicto entre umbrales del código actual, no una prueba de insuficiencia de fuerza del motor.

Evidencia: `delivery/report.json`, eventos de 6110 y 6120 ms, y `delivery/trace.ndjson`, que contiene posiciones, entradas y salidas exactas. La reproducción de los 1500 pasos dio las mismas salidas de ambos controladores.

## Obstáculos sin asignación de misión

En `obstacles` y `blocked`, ambos rovers permanecen en espera durante los 15 segundos y no hay entregas. El planificador no encuentra la pareja de misiones requerida por `assign_initial()`. Esa función retorna sin asignar cuando no existe coste finito; la espera no publica un motivo de navegación específico. El resultado sugiere mejorar el diagnóstico de asignación en un trabajo posterior, sin asumir que exista una ruta físicamente viable para estos escenarios.

## Ausencia de eco ultrasónico y recuperación

En `crossing`, aparecen pausas a partir de 3030 ms con motivo 8 (`NAVIGATION_FAILURE_ULTRASONIC`). Al no haber un objeto interceptado por el rayo frontal, el sensor simulado produce timeout; el controlador detiene navegación después de tres muestras inválidas. `edge` y `delays` también terminan en pausa. El resultado depende del modelo ultrasónico de un solo rayo: debe contrastarse con el cono y los ecos reales antes de atribuirlo a un defecto físico del rover.

En `vision-loss`, la entrada se interrumpe de 3200 a 5500 ms; los motores se detienen al caducar la visión. En `peer-loss`, la interrupción ocurre en el mismo intervalo y el enlace caduca 1500 ms después del último paquete. En ambos casos vuelven a llegar entradas y hay intentos de replanteamiento; no se completa una entrega dentro de los 15 segundos. La recuperación de datos no implica por sí sola recuperar la misión.

## Brazos y limitaciones

Las pruebas físicas independientes comprueban empuje centrado y descentrado, contacto lateral con brazos, giro, retirada sin arrastrar artificialmente el cubo y colisiones con obstáculos y otro rover. Las barras miden 3 × 55 mm, con 94 mm entre caras internas. El cubo de 60 mm dispone de 34 mm de holgura lateral total.

El firmware aproxima el rover mediante una envolvente de 100 × 150 mm centrada en la pose. La geometría física tiene cuerpo de 100 × 95 mm y añade 55 mm solo al frente; ambos modelos comparten longitud total pero no distribución alrededor del centro. Los márgenes de giro y contacto deben evaluarse con esa diferencia.

La física independiente demuestra que los cuerpos pueden empujar por contacto; **la estrategia actual no llegó a ejecutar una entrega completa en estos ocho escenarios**. Este simulador valida lógica de control y geometría aproximada, con arranque/calibración supuestos; no reemplaza pruebas de tracción, sensores ni temporización concurrente en placas.
