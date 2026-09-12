from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass

PREFIX = b"@EIRO "
MAX_LINE_LENGTH = 1024
FACE_NAMES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
CALIBRATION_REJECTIONS = {
    1: "movimiento detectado por el acelerómetro",
    2: "movimiento detectado por el giroscopio",
    4: "la cara observada no coincide con la solicitada",
    8: "inclinación excesiva respecto al eje solicitado",
    16: "la magnitud de gravedad está fuera del intervalo esperado",
    32: "separación insuficiente entre las caras positiva y negativa",
}


class FrameParser:
    """Extrae exclusivamente tramas EIRO; ignora logs y recupera tras overflow."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._overflow = False

    def feed(self, chunk: bytes) -> list[dict]:
        messages: list[dict] = []
        for byte in chunk:
            if byte == 10:
                if not self._overflow:
                    line = bytes(self._buffer).rstrip(b"\r")
                    if line.startswith(PREFIX):
                        try:
                            value = json.loads(line[len(PREFIX) :].decode("utf-8"))
                            if isinstance(value, dict):
                                messages.append(value)
                        except (UnicodeDecodeError, json.JSONDecodeError):
                            pass
                self._buffer.clear()
                self._overflow = False
            elif not self._overflow:
                if len(self._buffer) < MAX_LINE_LENGTH:
                    self._buffer.append(byte)
                else:
                    self._buffer.clear()
                    self._overflow = True
        return messages


def calibration_rejection_messages(mask: int) -> list[str]:
    return [text for bit, text in CALIBRATION_REJECTIONS.items() if mask & bit]


def detected_face(accel: list[float] | tuple[float, ...]) -> str:
    axis = max(range(3), key=lambda index: abs(accel[index]))
    return ("+" if accel[axis] >= 0 else "-") + "XYZ"[axis]


def alignment_degrees(accel: list[float] | tuple[float, ...], face_index: int) -> float:
    norm = math.sqrt(sum(value * value for value in accel))
    if norm == 0:
        return 180.0
    axis = face_index // 2
    sign = 1.0 if face_index % 2 == 0 else -1.0
    cosine = max(-1.0, min(1.0, sign * accel[axis] / norm))
    return math.degrees(math.acos(cosine))


class TimeSeries:
    def __init__(self, seconds: float = 60.0) -> None:
        self.seconds = seconds
        self.points: deque[tuple[float, tuple[float, ...]]] = deque()

    def append(self, values: tuple[float, ...], now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        self.points.append((now, values))
        cutoff = now - self.seconds
        while self.points and self.points[0][0] < cutoff:
            self.points.popleft()


def quaternion_conjugate(q: tuple[float, float, float, float]) -> tuple[float, ...]:
    return q[0], -q[1], -q[2], -q[3]


def quaternion_multiply(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def normalize_quaternion(q: tuple[float, ...]) -> tuple[float, ...]:
    norm = math.sqrt(sum(value * value for value in q))
    return tuple(value / norm for value in q) if norm else (1.0, 0.0, 0.0, 0.0)


def relative_quaternion(current: tuple[float, ...], reference: tuple[float, ...]) -> tuple[float, ...]:
    return normalize_quaternion(quaternion_multiply(quaternion_conjugate(reference), current))


@dataclass
class SequenceTracker:
    last: int | None = None
    lost: int = 0

    def observe(self, sequence: int) -> None:
        if self.last is not None and sequence > self.last + 1:
            self.lost += sequence - self.last - 1
        if self.last is None or sequence > self.last:
            self.last = sequence
