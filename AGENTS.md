# Documentación para desarrollo agentico
No puedes agregar reglas nuevas a AGENTS.md sin que el usuario lo pida o autorice explicitamente, Las reglas no deben tener un formato legible, estas deben tener la menor cantidad de lineas y texto deseable, no usarás bulletpoints, no usarás caracterés de formateo a excepción del heading 1.

# Componentes ESP-IDF
Para componentes de ESP Component Registry: añade `fabricante/componente` con versión fija en `src/idf_component.yml`, declara `componente` en `PRIV_REQUIRES` de `src/CMakeLists.txt` (`REQUIRES` sólo si lo exponen headers públicos), conserva `dependencies.lock`, ignora `managed_components/`, no uses `lib_deps` y valida con `pio run` (`pio run -t clean` antes si la resolución quedó obsoleta).

# Mapa de pines
La fuente única del cableado es `include/board_pins.hpp`; consulta `README.md` para la tabla y las restricciones eléctricas.

# Herramienta de configuración
En Windows crea el entorno con python -m venv tools\.venv y actívalo con .\tools\.venv\Scripts\Activate.ps1; en Linux usa python3 -m venv tools/.venv y source tools/.venv/bin/activate. Instala con python -m pip install -r tools/requirements-gui.txt y ejecuta la GUI con python tools/rover_gui/main.py. Para elegir el puerto diagnostica con python tools/rover_cli.py --port PUERTO_SERIAL probe --seconds 4 o calibra con python tools/rover_cli.py --port PUERTO_SERIAL calibrate; omitir --port conserva la autodetección de un único CH340. No abras dos clientes serie simultáneos; en Linux concede al usuario acceso al grupo propietario del dispositivo serie y vuelve a iniciar sesión.
La GUI configura NVS, muestra sensores, MAC propia, Wi-Fi y orientación 3D; la calibración usa el marco del rover X=-Xchip, Y=-Ychip, Z=Zchip y debe completar las seis caras antes de guardar. Valida cambios con pio run, pytest test/test_rover_gui.py y una prueba serial física a 115200 baudios.

# Sistema de visión
Configura en `.env` `VISION_CHALLENGE_REPO`, `VISION_HOST`, `VISION_PORT`, `VISION_CAMERA_INDEX` y `VISION_CAMERA_PROFILE_INDEX`; los índices empiezan en cero y el perfil se resuelve según los JSON de `vision/calibraciones` ordenados. Ejecuta `python tools/vision_client.py` para usar cámara real, iniciar `python -m vision.sistema --ventana` sólo si el puerto está libre y comparar la ventana de cámara con el mapa TCP; no uses `--synthetic` salvo que la prueba lo pida ni termines un servidor preexistente.
Para obtener datos para agentes ejecuta `python tools/vision_client.py --count N`; stdout contiene NDJSON, stderr diagnósticos y los códigos 0, 2 y 3 indican respectivamente contrato v3 válido, contrato incompatible y error de configuración o conexión. Conserva la lectura fragmentada por líneas, valida contra `contrato/schema.py` del clon y no programes con tramas rechazadas.

# Diagnóstico del pivote y navegación, 5 de octubre de 2026
El cuerpo mide 95 mm en su eje longitudinal y el eje motor está a 20 mm del borde trasero: su coordenada respecto al centro es menos 27,5 mm, no más 27,5 mm. El centro del cuerpo traza un arco de radio 27,5 mm al girar; cerca del borde inicial puede sacar una esquina de la pista.
La navegación C++ todavía presupone el centro como referencia de giro al estimar y seguir rutas, y la planificación no comprueba toda la envolvente barrida del chasis y los brazos. Por eso una rotación o corrección de rumbo desplaza el centro respecto a la ruta, dispara recuperación o salida de pista y bloquea entregas. Falta integrar la geometría del eje en odometría y control, planear maniobras con margen de giro y validar la envolvente completa y la recuperación con visión y comunicación interrumpidas; ninguna propuesta parcial de esos cambios quedó en el código.
Herramientas de verificación: PlatformIO `pio run` para el firmware y pruebas físicas de sensores, navegación y competencia. Una compilación correcta no demuestra navegación funcional; la validación física sigue pendiente.
