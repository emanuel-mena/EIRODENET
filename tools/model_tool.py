"""Construye y flashea imágenes EIRM para la partición TinyML del rover."""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path

MODEL_OFFSET = 0x600000
MODEL_PARTITION_SIZE = 0x200000
HEADER = struct.Struct("<4sIII")
MAGIC = b"EIRM"
TFLITE_IDENTIFIER = b"TFL3"
SUPPORTED_VERSION = 1


@dataclass(frozen=True)
class Image:
    version: int
    length: int
    crc32: int
    model: bytes


def validate_tflite(model: bytes) -> None:
    if len(model) < 8 or model[4:8] != TFLITE_IDENTIFIER:
        raise ValueError("el modelo no es un FlatBuffer TensorFlow Lite con firma TFL3")


def parse_image(data: bytes) -> Image:
    if len(data) < HEADER.size:
        raise ValueError("imagen EIRM truncada")
    magic, version, length, expected_crc = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError("cabecera EIRM inválida")
    if version != SUPPORTED_VERSION:
        raise ValueError(f"versión EIRM no soportada: {version}")
    if length > MODEL_PARTITION_SIZE - HEADER.size:
        raise ValueError("el modelo excede la partición de 2 MiB")
    if len(data) != HEADER.size + length:
        raise ValueError("la longitud declarada no coincide con la imagen")
    model = data[HEADER.size:]
    validate_tflite(model)
    actual_crc = zlib.crc32(model) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError(
            f"CRC32 inválido: esperado {expected_crc:08x}, calculado {actual_crc:08x}")
    return Image(version, length, actual_crc, model)


def build(source: Path, destination: Path, version: int) -> Image:
    if version != SUPPORTED_VERSION:
        raise ValueError(f"sólo se admite --version {SUPPORTED_VERSION}")
    model = source.read_bytes()
    validate_tflite(model)
    if len(model) > MODEL_PARTITION_SIZE - HEADER.size:
        raise ValueError("el modelo excede la partición de 2 MiB")
    crc = zlib.crc32(model) & 0xFFFFFFFF
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(HEADER.pack(MAGIC, version, len(model), crc) + model)
    return Image(version, len(model), crc, model)


def flash(image_path: Path, port: str, baud: int) -> Image:
    image = parse_image(image_path.read_bytes())
    packaged_esptool = Path.home() / ".platformio" / "packages" / "tool-esptoolpy" / "esptool.py"
    command = [sys.executable]
    if packaged_esptool.is_file():
        command.append(str(packaged_esptool))
    else:
        command.extend(("-m", "esptool"))
    command.extend([
        "--chip",
        "esp32",
        "--port",
        port,
        "--baud",
        str(baud),
        "write_flash",
        hex(MODEL_OFFSET),
        str(image_path),
    ])
    subprocess.run(command, check=True)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    build_parser = commands.add_parser("build", help="envolver un .tflite en una imagen EIRM")
    build_parser.add_argument("model", type=Path)
    build_parser.add_argument("output", type=Path)
    build_parser.add_argument("--version", type=int, default=SUPPORTED_VERSION)
    flash_parser = commands.add_parser("flash", help="validar y escribir una imagen EIRM")
    flash_parser.add_argument("image", type=Path)
    flash_parser.add_argument("--port", required=True)
    flash_parser.add_argument("--baud", type=int, default=460800)
    args = parser.parse_args()
    try:
        image = (build(args.model, args.output, args.version) if args.command == "build"
                 else flash(args.image, args.port, args.baud))
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f"model_tool: {exc}\n")
    print(f"EIRM v{image.version}: {image.length} bytes, CRC32={image.crc32:08x}")


if __name__ == "__main__":
    main()
