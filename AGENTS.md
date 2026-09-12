# Documentación para desarrollo agentico
No puedes agregar reglas nuevas a AGENTS.md sin que el usuario lo pida o autorice explicitamente, Las reglas no deben tener un formato legible, estas deben tener la menor cantidad de lineas y texto deseable, no usarás bulletpoints, no usarás caracterés de formateo a excepción del heading 1.

# Componentes ESP-IDF
Para componentes de ESP Component Registry: añade `fabricante/componente` con versión fija en `src/idf_component.yml`, declara `componente` en `PRIV_REQUIRES` de `src/CMakeLists.txt` (`REQUIRES` sólo si lo exponen headers públicos), conserva `dependencies.lock`, ignora `managed_components/`, no uses `lib_deps` y valida con `pio run` (`pio run -t clean` antes si la resolución quedó obsoleta).

# Mapa de pines
La fuente única del cableado es `include/board_pins.h`; consulta `README.md` para la tabla y las restricciones eléctricas.

# Modelos TinyML
Conserva la partición `model` como `data/0x40`, con offset `0x600000` y tamaño máximo de 2 MiB según `partitions.csv`. La aplicación debe localizarla mediante `esp_partition_find_first()` usando el nombre y subtipo, sin depender del offset fijo.
Usa únicamente modelos TensorFlow Lite FlatBuffer con identificador `TFL3`. No flashees directamente el archivo `.tflite`: primero genera una imagen con cabecera `EIRM`, versión, longitud y CRC32 mediante `& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py build modelo.tflite .pio/build/model.bin --version 1`.
Flashea únicamente la imagen generada mediante tools/model_tool.py flash .pio/build/model.bin --port PUERTO_SERIAL. El comando flash valida cabecera, firma TFLite, tamaño y CRC antes de escribir; una carga normal actualiza la aplicación y conserva la partición del modelo.
Después de modificar `partitions.csv`, actualiza también las constantes de partición de `tools/model_tool.py`, ejecuta `pio run -t clean` y `pio run`, decodifica `.pio/build/esp32dev/partitions.bin` para verificar offsets y tamaños, y confirma que `firmware.bin` registre una flash de 8 MB.
Tras flashear un modelo, revisa el monitor serie a 115200 baudios. El firmware debe informar versión, longitud y CRC del modelo; si está ausente o corrupto debe continuar sin inferencia y registrar el error.

# Herramienta de configuración
En Windows crea el entorno con python -m venv tools\.venv y actívalo con .\tools\.venv\Scripts\Activate.ps1; en Linux usa python3 -m venv tools/.venv y source tools/.venv/bin/activate. Instala con python -m pip install -r tools/requirements-gui.txt y ejecuta la GUI con python tools/rover_gui/main.py. Para elegir el puerto diagnostica con python tools/rover_cli.py --port PUERTO_SERIAL probe --seconds 4 o calibra con python tools/rover_cli.py --port PUERTO_SERIAL calibrate; omitir --port conserva la autodetección de un único CH340. No abras dos clientes serie simultáneos; en Linux concede al usuario acceso al grupo propietario del dispositivo serie y vuelve a iniciar sesión.
La GUI configura NVS, muestra sensores, MAC propia, Wi-Fi y orientación 3D; la calibración usa el marco del rover X=-Xchip, Y=-Ychip, Z=Zchip y debe completar las seis caras antes de guardar. Valida cambios con pio run, pytest test/test_rover_gui.py y una prueba serial física a 115200 baudios.
