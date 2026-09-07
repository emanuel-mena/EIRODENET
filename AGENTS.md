Para componentes de ESP Component Registry: añade `fabricante/componente` con versión fija en `src/idf_component.yml`, declara `componente` en `PRIV_REQUIRES` de `src/CMakeLists.txt` (`REQUIRES` sólo si lo exponen headers públicos), conserva `dependencies.lock`, ignora `managed_components/`, no uses `lib_deps` y valida con `pio run` (`pio run -t clean` antes si la resolución quedó obsoleta).

## Modelos TinyML

Conserva la partición `model` como `data/0x40`, con offset `0x600000` y tamaño máximo de 2 MiB según `partitions.csv`. La aplicación debe localizarla mediante `esp_partition_find_first()` usando el nombre y subtipo, sin depender del offset fijo.

Usa únicamente modelos TensorFlow Lite FlatBuffer con identificador `TFL3`. No flashees directamente el archivo `.tflite`: primero genera una imagen con cabecera `EIRM`, versión, longitud y CRC32 mediante `& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py build modelo.tflite .pio/build/model.bin --version 1`.

Flashea únicamente la imagen generada, sustituyendo el puerto cuando corresponda, mediante `& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" tools/model_tool.py flash .pio/build/model.bin --port COM3`. El comando `flash` valida la cabecera, la firma TFLite, el tamaño y el CRC antes de escribir; una carga normal con `pio run -t upload` actualiza la aplicación y conserva la partición del modelo.

Después de modificar `partitions.csv`, actualiza también las constantes de partición de `tools/model_tool.py`, ejecuta `pio run -t clean` y `pio run`, decodifica `.pio/build/esp32dev/partitions.bin` para verificar offsets y tamaños, y confirma que `firmware.bin` registre una flash de 8 MB.

Tras flashear un modelo, revisa el monitor serie a 115200 baudios. El firmware debe informar versión, longitud y CRC del modelo; si está ausente o corrupto debe continuar sin inferencia y registrar el error.
