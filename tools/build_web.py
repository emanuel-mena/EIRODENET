"""Integra el build de Vite y la imagen SPIFFS en `pio run -t upload`."""

from __future__ import annotations

import csv
import os
import subprocess
from pathlib import Path

Import("env")  # type: ignore[name-defined]  # noqa: F821

PROJECT = Path(env.subst("$PROJECT_DIR"))  # type: ignore[name-defined]  # noqa: F821
WEB = PROJECT / "web"
DATA = PROJECT / "data"
FS_IMAGE = Path(env.subst("$BUILD_DIR")) / "spiffs.bin"  # type: ignore[name-defined]  # noqa: F821


def parse_size(value: str) -> int:
    suffixes = {"K": 1024, "M": 1024 * 1024}
    value = value.strip().upper()
    if value[-1:] in suffixes:
        return int(value[:-1], 0) * suffixes[value[-1]]
    return int(value, 0)


def static_partition() -> tuple[str, int]:
    with (PROJECT / "partitions.csv").open(encoding="utf-8", newline="") as stream:
        for row in csv.reader(line for line in stream if not line.lstrip().startswith("#")):
            if row and row[0].strip() == "static":
                return row[3].strip(), parse_size(row[4])
    raise RuntimeError("partitions.csv no contiene la partición static")


STATIC_OFFSET, STATIC_SIZE = static_partition()


def run_checked(command: list[str], cwd: Path | None = None) -> None:
    result = subprocess.run(command, cwd=cwd, text=True, capture_output=True)
    output = (result.stdout + result.stderr).encode("ascii", "replace").decode("ascii")
    if output.strip():
        print(output, end="" if output.endswith("\n") else "\n")
    if result.returncode:
        raise subprocess.CalledProcessError(result.returncode, command)


def build_web_and_filesystem(source, target, env) -> None:
    del source, target
    npm = "npm.cmd" if os.name == "nt" else "npm"
    if not (WEB / "node_modules").is_dir():
        run_checked([npm, "ci"], cwd=WEB)
    run_checked([npm, "run", "build"], cwd=WEB)
    FS_IMAGE.parent.mkdir(parents=True, exist_ok=True)
    package_dir = Path(env.PioPlatform().get_package_dir("tool-mkspiffs"))
    executable = package_dir / (
        "mkspiffs_espressif32_espidf.exe" if os.name == "nt" else "mkspiffs_espressif32_espidf")
    run_checked([
        str(executable), "-c", str(DATA), "-s", str(STATIC_SIZE),
        "-p", "256", "-b", "4096", str(FS_IMAGE),
    ])


env.Append(FLASH_EXTRA_IMAGES=[(STATIC_OFFSET, str(FS_IMAGE))])  # type: ignore[name-defined]  # noqa: F821
env.AddPreAction("upload", build_web_and_filesystem)  # type: ignore[name-defined]  # noqa: F821
