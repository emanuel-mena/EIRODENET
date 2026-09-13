from __future__ import annotations

import importlib.util
import json
import os
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 2026
MAX_LINE_BYTES = 1024 * 1024


def load_dotenv(path: Path) -> dict[str, str]:
    """Lee el subconjunto portable de .env que necesita esta herramienta."""
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key, value = key.strip(), value.strip()
        if value[:1] == value[-1:] and value[:1] in {"'", '"'}:
            value = value[1:-1]
        if key:
            values[key] = value
    return values


@dataclass(frozen=True)
class VisionConfig:
    challenge_repo: Path
    vision_system: Path
    python: Path
    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    camera_index: int | None = None
    camera_profile_index: int | None = None

    @staticmethod
    def from_environment(env_file: Path | None = None) -> "VisionConfig":
        file_values = load_dotenv(env_file or ROOT / ".env")
        values = {**file_values, **os.environ}
        configured = values.get("VISION_CHALLENGE_REPO", "").strip()
        if not configured:
            raise ValueError(
                "Falta VISION_CHALLENGE_REPO en .env; debe apuntar al clon de "
                "Vision-Rover-Challenge."
            )
        repo = Path(configured).expanduser().resolve()
        vision_system = repo if repo.name == "vision-system" else repo / "vision-system"
        configured_python = values.get("VISION_PYTHON", "").strip()
        if configured_python:
            python = Path(configured_python).expanduser().resolve()
        else:
            suffix = Path("Scripts/python.exe") if os.name == "nt" else Path("bin/python")
            python = vision_system / ".venv" / suffix
        host = values.get("VISION_HOST", DEFAULT_HOST).strip() or DEFAULT_HOST
        try:
            port = int(values.get("VISION_PORT", str(DEFAULT_PORT)))
        except ValueError as exc:
            raise ValueError("VISION_PORT debe ser un entero.") from exc
        if not 1 <= port <= 65535:
            raise ValueError("VISION_PORT debe estar entre 1 y 65535.")
        camera_index = _optional_nonnegative_int(values, "VISION_CAMERA_INDEX")
        profile_index = _optional_nonnegative_int(values, "VISION_CAMERA_PROFILE_INDEX")
        return VisionConfig(repo, vision_system, python, host, port, camera_index, profile_index)

    def validate_installation(self) -> None:
        if not self.vision_system.is_dir():
            raise FileNotFoundError(f"No existe el sistema de visión: {self.vision_system}")
        if not (self.vision_system / "vision" / "sistema.py").is_file():
            raise FileNotFoundError(f"El clon no contiene vision/sistema.py: {self.vision_system}")
        if not self.python.is_file():
            raise FileNotFoundError(
                f"No existe el Python del sistema de visión: {self.python}. "
                "Cree su .venv o defina VISION_PYTHON."
            )
        self.camera_profile_name()

    def camera_profile_name(self) -> str | None:
        if self.camera_profile_index is None:
            return None
        profiles = sorted((self.vision_system / "vision" / "calibraciones").glob("*.json"))
        if self.camera_profile_index >= len(profiles):
            raise ValueError(
                f"VISION_CAMERA_PROFILE_INDEX={self.camera_profile_index} no existe; "
                f"hay {len(profiles)} perfiles, numerados de 0 a {max(0, len(profiles) - 1)}."
            )
        return profiles[self.camera_profile_index].stem


def _optional_nonnegative_int(values: dict[str, str], key: str) -> int | None:
    raw = values.get(key, "").strip()
    if not raw:
        return None
    try:
        value = int(raw)
    except ValueError as exc:
        raise ValueError(f"{key} debe ser un entero.") from exc
    if value < 0:
        raise ValueError(f"{key} debe ser mayor o igual que cero.")
    return value


class ContractValidator:
    """Carga el contrato desde el clon configurado: ese clon es la fuente de verdad."""

    def __init__(self, vision_system: Path) -> None:
        schema_path = vision_system / "contrato" / "schema.py"
        if not schema_path.is_file():
            raise FileNotFoundError(f"No existe el contrato de telemetría: {schema_path}")
        spec = importlib.util.spec_from_file_location("_eiro_vision_contract", schema_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"No se pudo cargar {schema_path}")
        module = importlib.util.module_from_spec(spec)
        # dataclasses resuelve anotaciones consultando sys.modules durante exec_module.
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        self._module: ModuleType = module
        self.version = int(module.PROTOCOL_VERSION)

    def validate(self, message: Any) -> str | None:
        return self._module.validate_message(message)


class NDJSONDecoder:
    """Reconstruye líneas aunque TCP entregue fragmentos o varias a la vez."""

    def __init__(self, max_line_bytes: int = MAX_LINE_BYTES) -> None:
        self.buffer = bytearray()
        self.max_line_bytes = max_line_bytes

    def feed(self, data: bytes) -> list[dict[str, Any]]:
        self.buffer.extend(data)
        if len(self.buffer) > self.max_line_bytes and b"\n" not in self.buffer:
            self.buffer.clear()
            raise ValueError("la línea NDJSON supera el límite de seguridad")
        messages: list[dict[str, Any]] = []
        while True:
            newline = self.buffer.find(b"\n")
            if newline < 0:
                break
            raw = bytes(self.buffer[:newline]).strip()
            del self.buffer[: newline + 1]
            if not raw:
                continue
            if len(raw) > self.max_line_bytes:
                raise ValueError("la línea NDJSON supera el límite de seguridad")
            try:
                value = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError(f"línea NDJSON inválida: {exc}") from exc
            if not isinstance(value, dict):
                raise ValueError("la línea NDJSON no contiene un objeto JSON")
            messages.append(value)
        return messages


def server_is_listening(host: str, port: int, timeout: float = 0.35) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class ServerProcess:
    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.process: subprocess.Popen[bytes] | None = None
        self._log = None

    @property
    def started_here(self) -> bool:
        return self.process is not None

    def ensure_running(
        self,
        *,
        window: bool = True,
        synthetic: bool = False,
        timeout: float = 25.0,
        status: Callable[[str], None] | None = None,
    ) -> bool:
        if server_is_listening(self.config.host, self.config.port):
            if status:
                status(f"Servidor existente en {self.config.host}:{self.config.port}")
            return False
        if self.config.host not in {"127.0.0.1", "localhost", "::1"}:
            raise ConnectionError("No se puede iniciar automáticamente un servidor remoto.")
        self.config.validate_installation()
        args = [str(self.config.python), "-m", "vision.sistema"]
        if window:
            args.append("--ventana")
        if synthetic:
            args.append("--sintetico")
        else:
            if self.config.camera_index is not None:
                args.extend(("--indice", str(self.config.camera_index)))
            profile = self.config.camera_profile_name()
            if profile is not None:
                args.extend(("--camara", profile))
        log_path = ROOT / ".pio" / "vision-server.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = log_path.open("ab", buffering=0)
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        if status:
            status(f"Iniciando servidor desde {self.config.vision_system}")
        self.process = subprocess.Popen(
            args,
            cwd=self.config.vision_system,
            env={**os.environ, "PYTHONIOENCODING": "utf-8"},
            stdin=subprocess.DEVNULL,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            creationflags=creationflags,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError(
                    f"El servidor terminó con código {self.process.returncode}; revise {log_path}"
                )
            if server_is_listening(self.config.host, self.config.port):
                if status:
                    status(f"Servidor listo en {self.config.host}:{self.config.port}")
                return True
            time.sleep(0.1)
        self.stop()
        raise TimeoutError(f"El servidor no abrió el puerto en {timeout:.0f} s; revise {log_path}")

    def stop(self) -> None:
        process, self.process = self.process, None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        if self._log is not None:
            self._log.close()
            self._log = None


class VisionConnection:
    def __init__(self, host: str, port: int, timeout: float = 5.0) -> None:
        self.host, self.port, self.timeout = host, port, timeout
        self.socket: socket.socket | None = None
        self.decoder = NDJSONDecoder()

    def connect(self) -> None:
        self.close()
        self.socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.socket.settimeout(1.0)
        self.decoder = NDJSONDecoder()

    def receive(self) -> list[dict[str, Any]]:
        if self.socket is None:
            raise ConnectionError("el cliente no está conectado")
        data = self.socket.recv(65536)
        if not data:
            raise ConnectionError("el servidor cerró la conexión")
        return self.decoder.feed(data)

    def close(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close()
            finally:
                self.socket = None
