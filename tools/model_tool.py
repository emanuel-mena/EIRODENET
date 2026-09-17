#!/usr/bin/env python3
"""Build and flash EIRODENET TinyML partition images."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import struct
import subprocess
import sys
import zlib


MAGIC = 0x4D524945
HEADER_VERSION = 1
HEADER = struct.Struct("<IHHIII3I")
MODEL_OFFSET = "0x600000"
MODEL_PARTITION_SIZE = 0x200000


def build_image(model_path: Path, output_path: Path, model_version: int) -> None:
    model = model_path.read_bytes()
    if not model:
        raise ValueError("the model file is empty")
    if len(model) < 8 or model[4:8] != b"TFL3":
        raise ValueError("the input is not a TensorFlow Lite FlatBuffer (missing TFL3 identifier)")
    if len(model) > MODEL_PARTITION_SIZE - HEADER.size:
        raise ValueError(
            f"model is too large ({len(model)} bytes); maximum is "
            f"{MODEL_PARTITION_SIZE - HEADER.size} bytes"
        )

    header = HEADER.pack(
        MAGIC,
        HEADER_VERSION,
        HEADER.size,
        model_version,
        len(model),
        zlib.crc32(model) & 0xFFFFFFFF,
        0,
        0,
        0,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(header + model)
    print(
        f"Created {output_path} ({len(header) + len(model)} bytes, "
        f"model CRC32 {zlib.crc32(model) & 0xFFFFFFFF:08x})"
    )


def find_esptool() -> Path:
    core_dir = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    esptool = core_dir / "packages" / "tool-esptoolpy" / "esptool.py"
    if not esptool.is_file():
        raise FileNotFoundError(f"PlatformIO esptool was not found at {esptool}")
    return esptool


def validate_image(image_path: Path) -> bytes:
    image = image_path.read_bytes()
    if len(image) < HEADER.size:
        raise ValueError("model image is smaller than its header")

    magic, header_version, header_size, _, model_size, expected_crc, *_ = HEADER.unpack_from(image)
    if magic != MAGIC or header_version != HEADER_VERSION or header_size != HEADER.size:
        raise ValueError("model image header is invalid or unsupported")
    if model_size < 8 or len(image) != header_size + model_size:
        raise ValueError("model image length does not match its header")

    model = image[header_size:]
    if model[4:8] != b"TFL3":
        raise ValueError("model image does not contain a TensorFlow Lite FlatBuffer")
    actual_crc = zlib.crc32(model) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError(
            f"model image CRC32 mismatch: expected {expected_crc:08x}, got {actual_crc:08x}"
        )
    return image


def flash_image(image_path: Path, port: str) -> None:
    image = validate_image(image_path)
    if len(image) > MODEL_PARTITION_SIZE:
        raise ValueError("image is larger than the model partition")

    command = [
        sys.executable,
        str(find_esptool()),
        "--chip",
        "esp32",
        "--port",
        port,
        "write_flash",
        MODEL_OFFSET,
        str(image_path),
    ]
    subprocess.run(command, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = subparsers.add_parser("build", help="wrap a .tflite file with its header")
    build.add_argument("model", type=Path)
    build.add_argument("output", type=Path)
    build.add_argument("--version", type=int, default=1)

    flash = subparsers.add_parser("flash", help="flash a previously built model image")
    flash.add_argument("image", type=Path)
    flash.add_argument("--port", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "build":
            if not 0 <= args.version <= 0xFFFFFFFF:
                raise ValueError("version must fit in an unsigned 32-bit integer")
            build_image(args.model, args.output, args.version)
        else:
            flash_image(args.image, args.port)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
