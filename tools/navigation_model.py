"""Modelo puro de la geometría y el controlador del prototipo de navegación.

Se usa para pruebas deterministas en el host; las mismas convenciones y umbrales
están documentados junto al firmware en ``navigation_service.c``.
"""

from __future__ import annotations

import math

SENSOR_OFFSETS_MM = ((10.0, 20.0), (10.0, -20.0), (-10.0, 20.0), (-10.0, -20.0))


def vision_frame_is_new(previous_timestamp_ms: int | None, timestamp_ms: int) -> bool:
    """Distingue capturas de cámara de mensajes TCP repetidos."""
    return previous_timestamp_ms is None or timestamp_ms != previous_timestamp_ms


def navigation_inputs_ready(
    *, vision_connected: bool, protocol_valid: bool, pose_fresh: bool,
    imu_valid: bool, imu_calibrated: bool, infrared_valid: bool,
    ultrasonic_valid: bool,
) -> bool:
    """Replica las precondiciones de seguridad para aceptar un objetivo."""
    return all((vision_connected, protocol_valid, pose_fresh, imu_valid,
                imu_calibrated, infrared_valid, ultrasonic_valid))


def normalize_motor(command: int) -> int:
    if 0 < command < 700:
        return 700
    if -700 < command < 0:
        return -700
    return command


def wrap_degrees(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def sensor_positions(col: float, row: float, theta_deg: float, cell_mm: float = 20.0):
    radians = math.radians(theta_deg)
    for forward, left in SENSOR_OFFSETS_MM:
        yield (
            col + (forward * math.cos(radians) - left * math.sin(radians)) / cell_mm,
            row + (-forward * math.sin(radians) - left * math.cos(radians)) / cell_mm,
        )


def checker_pattern(col: float, row: float, theta_deg: float, cell_mm: float = 20.0) -> int:
    pattern = 0
    for index, (sensor_col, sensor_row) in enumerate(
        sensor_positions(col, row, theta_deg, cell_mm)
    ):
        if (math.floor(sensor_col) + math.floor(sensor_row)) & 1:
            pattern |= 1 << index
    return pattern


def controller_command(
    col: float,
    row: float,
    theta_deg: float,
    target_col: float,
    target_row: float,
    driving: bool = False,
) -> tuple[str, int, int]:
    dx, dy = target_col - col, target_row - row
    distance = math.hypot(dx, dy)
    if distance <= 0.75:
        return "arrived", 0, 0
    target_heading = math.degrees(math.atan2(-dy, dx))
    error = wrap_degrees(target_heading - theta_deg)
    if driving and abs(error) > 25.0:
        driving = False
    if not driving and abs(error) <= 8.0:
        driving = True
    if not driving:
        return "turning", (-700 if error > 0 else 700), (700 if error > 0 else -700)
    base = 700 if distance <= 3.0 else 850
    correction = max(-150, min(150, round(error * 8.0)))
    return "driving", normalize_motor(base - correction), normalize_motor(base + correction)
