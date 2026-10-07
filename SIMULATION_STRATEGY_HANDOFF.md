# Contexto de estrategia de simulación

## Propósito

Este documento conserva los hallazgos de la estrategia de entrega para que otro modelo pueda continuar el trabajo sin perder el diagnóstico actual.

## Estado actual

La simulación compila el controlador C++ de competencia y ejecuta dos rovers contra un mundo físico Pymunk. El rover puede salir del área de juego sin invalidar la ejecución. Un cubo que sale del área queda perdido y la ejecución no puede considerarse exitosa.

Se aplicaron cambios parciales en `include/competition_strategy.hpp` y `src/services/competition_runtime.cpp`:

- `direct_drive()` convierte objetivos del centro del cuerpo a la referencia del eje motor desplazado 27,5 mm.
- La captura física se adelantó de 3,5 a 4,2 celdas porque el primer contacto de los brazos ocurre antes de que el centro del cubo alcance 3,5 celdas.
- La aproximación al cubo usa un waypoint a 1,2 celdas antes del centro del cubo.
- Se agregó una comprobación que rechaza un empuje cuya trayectoria prevista llevaría el cubo fuera del tablero.
- El desvío y la aproximación se validan parcialmente con la geometría de navegación existente.
- La coordinación entre rovers se hizo conservadora: ante solapamiento previsto, ambos deben detenerse en vez de que uno retroceda mientras el otro continúa.

Estos cambios no constituyen todavía una solución terminada.

## Comandos de verificación

Usar el entorno virtual del proyecto:

```powershell
.\tools\.venv\Scripts\python.exe -m pytest test/test_rover_sim.py -q
.\tools\.venv\Scripts\python.exe tools/rover_sim.py --headless --scenario delivery --seconds 60 --output .pio/sim/delivery
```

`pio run` no pudo ejecutarse durante el diagnóstico porque `pio` no estaba disponible en el `PATH` de la sesión. Resolver esa disponibilidad antes de declarar validado el firmware.

## Métricas de éxito

La ejecución solo es exitosa cuando `report.json` cumple simultáneamente:

```text
final.delivered = [true, true, true]
physical_delivery_mask = 7
declared_delivery_mask = 7
physical_delivery_mask == declared_delivery_mask
final.outside no contiene cubos
metrics.rover_contacts = 0
metrics.obstacle_contacts = 0
metrics.completion_ms != null
eligible = true
```

Las salidas del rover pueden aparecer en `final.outside` sin invalidar por sí mismas la ejecución. La salida de un cubo sí invalida la ejecución.

## Resultados observados

### Línea base

Con la estrategia anterior, `delivery` durante 60 segundos produjo:

- Entregas físicas: `0/3`.
- `outside_frames`: `3618`.
- Contactos entre rovers: `53`.
- Contactos con obstáculos: `0`.
- `completion_ms`: `null`.
- Rovers y cubos terminaron fuera de la trayectoria prevista.

El problema principal era que la conducción de competencia calculaba el rumbo desde el centro del cuerpo, aunque el eje motor está a `-27,5 mm` respecto al centro. Los giros y correcciones barrían una envolvente distinta de la usada por la planificación.

### Después de usar el eje motor en `direct_drive()`

La simulación produjo:

- Entregas físicas: `0/3`.
- Contactos entre rovers: `0`.
- `outside_frames`: `4209`.
- Un cubo terminó fuera del tablero.

Esto eliminó los contactos entre rovers en esa semilla, pero el rover continuó empujando un cubo antes de que la estrategia confirmara la captura.

### Después de adelantar la captura

La simulación siguió produciendo `0/3` entregas. El umbral por sí solo no resolvió el problema: la captura virtual no garantiza que el cubo ya esté físicamente retenido ni que pueda transportarse sin ser expulsado.

### Después del waypoint y protección de empuje

La última corrida de `delivery` produjo:

- Entregas físicas: `0/3`.
- `outside_frames`: `28`.
- Ningún cubo terminó fuera del tablero.
- Contactos entre rovers: `609`.
- Contactos con obstáculos: `0`.
- `completion_ms`: `null`.
- Fases finales: rover 10 en `EXEC_DEPOT_WAIT_VISION` y rover 11 en `EXEC_DETOUR_ADVANCE`.

La protección de pérdida de cubos funciona para esa corrida, pero la coordinación conservadora genera bloqueo y contactos persistentes; detener ambos rovers ante solapamiento no constituye una estrategia de resolución de conflictos.

Las pruebas unitarias del simulador pasan actualmente:

```text
42 passed
```

Esto valida contratos, geometría aislada, reproducción y casos de sensores, pero no demuestra que la estrategia complete la misión.

## Por qué no se cumplen las métricas

### No hay entregas físicas

La estrategia cambia de fase por confirmaciones parciales y no mantiene una separación suficientemente estricta entre:

- cubo detectado;
- cubo contactado;
- cubo capturado;
- cubo transportado;
- cubo dentro del depósito;
- entrega comunicada.

Llegar a `EXEC_PUSH` o recibir una confirmación sensorial no prueba que el cubo esté físicamente en el depósito.

### Riesgo de perder cubos

El rover se aproxima y empuja con una geometría que todavía no modela completamente la trayectoria del cubo durante el contacto. La validación nueva comprueba una dirección de empuje aproximada, pero no verifica todo el barrido temporal del cubo, el tamaño del depósito, los brazos y el cambio de orientación durante la maniobra.

### Contactos entre rovers

`peer_block_action()` decide usando la pose retrasada del compañero y una comprobación local de solapamiento. Si ambos detectan conflicto casi simultáneamente, ambos pueden frenar después de que ya exista velocidad y contacto físico. El modelo necesita prioridad de paso, reserva de corredor o un protocolo de negociación estable; frenar ambos no basta.

### Desvíos insuficientes

El desvío heredado usa una dirección fija y no selecciona el lado libre según el cubo, los bordes, el compañero y el depósito. Debe convertirse en una ruta de tres puntos con validación de todos los segmentos y giros.

### Recuperación insuficiente

La recuperación detecta falta de progreso, pero no conserva de forma explícita el último waypoint seguro ni verifica que retroceder y girar no empuje el cubo. Debe limitar intentos, retroceder a una pose segura y abandonar la maniobra si la siguiente acción puede perder el cubo.

## Trabajo pendiente recomendado

1. Implementar una máquina de estados de captura con confirmación física y timeout claramente separados.
2. Calcular el barrido del cubo durante aproximación, giro, retroceso y empuje; rechazar toda maniobra que cruce el margen del tablero.
3. Calcular una aproximación interior al cubo y un punto de salida que no apunte hacia el borde.
4. Reemplazar el desvío fijo por salida segura, waypoint lateral y reaproximación.
5. Implementar prioridad de paso determinista entre rovers basada en identidad, misión y corredor reservado.
6. Hacer que la recuperación use el último waypoint seguro y tenga límite de tiempo e intentos.
7. Añadir pruebas específicas para cubos cerca de bordes, empuje hacia fuera, conflicto de corredores, recuperación y diferencia entre entrega declarada y física.
8. Repetir `delivery` con la misma semilla y conservar `scenario.json`, `manifest.json`, `trace.ndjson` y `report.json`.

## Instrucción para el siguiente modelo

No borres este archivo mientras `eligible` sea `false`, falte alguna entrega, exista un cubo fuera del área, haya contactos entre rovers u obstáculos, o `completion_ms` sea `null`. Cuando una implementación posterior cumpla todas las métricas de éxito en una simulación reproducible y las pruebas pasen, elimina este archivo porque su función de contexto ya habrá sido cumplida.

## Revisión del 6 de octubre de 2026

Se repitió la corrida base con los cambios parciales que ya estaban en el árbol de trabajo. El resultado fue `0/3` entregas, `28` cuadros con objetos fuera, `609` contactos entre rovers, ningún contacto con obstáculos y `completion_ms: null`. El comandante terminó en `EXEC_DEPOT_WAIT_VISION` y el soldado en `EXEC_DETOUR_ADVANCE`. `pytest test/test_rover_sim.py -q` dio `42 passed`. `pio` sigue sin estar disponible en el `PATH` ni en `tools/.venv/Scripts`.

El `trace.ndjson` muestra que el comandante empujó el cubo verde desde (250, 450) hasta aproximadamente (526, 675) antes de pasar a `EXEC_ALIGN_DEPOT`; al girar junto a él, lo desplazó de vuelta hasta (292, 524). La transición de captura ocurrió con el cubo a 5,30 celdas y confirmación por timeout ultrasónico. Esto no demuestra que el cubo estuviera retenido. El soldado empujó el cubo rojo desde (500, 550) hasta aproximadamente (815, 574) y acabó detenido en un desvío. A partir de unos 14 segundos, ambos rovers quedaron esencialmente inmóviles y en contacto persistente.

Se probaron y retiraron dos variantes de aproximación por un punto detrás del cubo orientado hacia el depósito. La primera acabó con ambos rovers fuera del área; la segunda produjo `102880` contactos entre rovers, `399` cuadros fuera y ninguna entrega. La visión puede repetir la misma captura durante unos 600 ms: a velocidad máxima el rover recorre alrededor de 108 mm entre capturas. Una tolerancia de llegada de 0,8 a 4,5 celdas y un punto recalculado desde un cubo que ya se mueve no bastan para detenerlo antes del contacto. La solución necesita estimar el avance entre capturas y reservar una distancia de frenado real. Las variantes experimentales no quedaron en el código.

Se probó también quitar la espera al cambiar la secuencia de visión en `EXEC_TO_DEPOT`. No produjo avance en esta corrida, porque el bloqueo entre rovers detuvo la conducción; se retiró el cambio. La estrategia parcial original permanece en los dos archivos C++ modificados. No hay evidencia para eliminar este handoff.

## Avance posterior: predicción entre capturas

La estrategia ahora usa la pose estimada del servicio de navegación cuando es finita, suficientemente precisa y compatible con el último fotograma. El host de simulación integra los comandos de motor y el giróscopo entre capturas distintas; el controlador puede calcular el rumbo y la distancia de frenado sin confundir las publicaciones repetidas cada 50 ms con una nueva imagen de cámara. Antes de acercarse al cubo, busca un punto detrás de él respecto al depósito y frena según la velocidad estimada. La aceptación de captura exige además que el cubo esté delante del rover, evitando la falsa transición verde observada con timeout ultrasónico.

La prueba nueva reproduce publicaciones de una misma captura y verifica que el soldado se detenga a al menos 175 mm del cubo rojo sin moverlo durante la etapa de preparación. El resultado de esta versión es `43 passed` en `test/test_rover_sim.py`, prueba nativa de `test/test_competition_strategy.cpp` aprobada y `pio run` exitoso con el ejecutable de PlatformIO en `C:\Users\ecraf\.platformio\penv\Scripts\pio.exe`.

La corrida `delivery` de 60 segundos en `.pio/sim/delivery` produjo entrega física y declarada sólo del cubo rojo: ambas máscaras valen `4`; `rover_contacts = 0`, `obstacle_contacts = 0`, `outside_frames = 270`, `completion_ms = null` y `eligible = false`. El cubo verde sigue sin llegar al depósito; su primer giro para alcanzar el punto de preparación barre el cubo desde la pose inicial. El soldado entrega el rojo, pero su siguiente punto detrás del cubo azul queda fuera del margen seguro y entra en espera. Una variante que acortaba ese punto permitió avanzar, pero golpeó el cubo rojo ya entregado y lo sacó del tablero; se retiró. Faltan una ruta inicial que valide el giro contra cubos, un recorrido hacia el azul que evite cubos entregados y la comprobación de toda la envolvente durante transporte y recuperación. Mantener este handoff hasta satisfacer todas las métricas.

## Recolocación tras desplazamiento del cubo, 6 de octubre de 2026

Durante `EXEC_STAGE_CUBE` y `EXEC_TO_CUBE`, cada nueva captura compara la posición del cubo objetivo con el ancla de la tentativa actual. Si su desplazamiento acumulado supera 2,5 celdas, el rover evalúa la envolvente del cuerpo y los brazos para un movimiento recto de 1,3 segundos a PWM 700. Prefiere alejarse del cubo; si esa dirección saldría del campo, usa la contraria sólo cuando el barrido del rover y el empuje previsto del cubo permanecen dentro. Si ninguna dirección es segura, se detiene. La duración acumula tiempo de conducción efectiva y después reinicia la preparación de captura con la posición actual del cubo.

Las pruebas cubren el umbral estricto de 2,5 celdas, 130 comandos de 10 ms en retroceso y el caso de avance porque retroceder saldría del campo; la GUI reconoce la fase nueva. `test/test_rover_sim.py` terminó con `46 passed` y `pio run` compiló correctamente. La corrida reproducible de `delivery` durante 60 segundos quedó en `.pio/sim/delivery`: máscaras física y declarada `5` (verde y rojo), `rover_contacts = 0`, `obstacle_contacts = 0`, `outside_frames = 19`, ningún objeto fuera al final, `completion_ms = null` y `eligible = false`. El cubo azul sigue pendiente; conservar este archivo.

## Retirada hacia el centro después de entregar

Tras el retroceso existente de 1 segundo, cada rover orienta su parte trasera hacia el centro del tablero y retrocede otros 1,3 segundos. La maniobra verifica el barrido del giro y el segmento de marcha atrás; se detiene si ya no son seguros. La reserva permanece pendiente hasta terminar esta segunda retirada. La prueba de integración comprueba el orden de fases, la orientación, 130 comandos de PWM -700/-700 y que la distancia al centro disminuya. La GUI reconoce ambas fases nuevas. `test/test_rover_sim.py` registró `47 passed`, `pio run` compiló y una prueba específica de GUI pasó. En `delivery` de 60 segundos, ambos rovers completaron 130 comandos de esta segunda retirada: el comandante redujo su distancia al centro aproximadamente de 286 a 134 mm y el soldado de 196 a 45 mm. Las máscaras física y declarada siguen en `5`, sin contactos y con `outside_frames = 19`; el cubo azul sigue sin entrega y `eligible = false`.

## Captura del último cubo y giro hacia el acoplo

La retirada adicional de 1,3 s quedó limitada al rover que hace la primera entrega. Se retiró la ruta alternativa de dos puntos creada para el azul: cuando el punto de preparación estándar no cabe en el campo durante la segunda misión del rover, pasa a apuntar y acercarse directamente al cubo. Tras confirmar esa captura, compara la distancia desde el rover a los otros dos cubos. Si el más cercano queda a su derecha, gira a la izquierda para apuntar al depósito; en el caso contrario gira a la derecha. El sentido se conserva durante ese giro. Si no hay dos posiciones recientes, espera sin girar.

La prueba unitaria nativa verifica ambos sentidos y la selección del más cercano. `pio run` compiló, y `test/test_rover_sim.py` dio `47 passed`. La corrida completa de 60 segundos en `.pio/sim/delivery` no pasó: máscara física `1` (solo verde), máscara declarada `5`, cubo rojo fuera del campo, `outside_frames = 4309`, `rover_contacts = 0`, `obstacle_contacts = 0`, `completion_ms = null` y `eligible = false`. El rover intentó acercarse al azul, pero no llegó a confirmar su captura; por tanto, esta corrida no ejercitó el nuevo giro. No se agregó otra ruta. Conservar el handoff.
