"""Modelo puro de la geometría y el controlador del prototipo de navegación.

Se usa para pruebas deterministas en el host; las mismas convenciones y umbrales
están documentados junto al firmware en ``navigation_service.cpp``.
"""

from __future__ import annotations

import math
import heapq

SENSOR_OFFSETS_MM = ((10.0, 20.0), (10.0, -20.0), (-10.0, 20.0), (-10.0, -20.0))
DIRECTIONS = ((1, 0), (1, -1), (0, -1), (-1, -1),
              (-1, 0), (-1, 1), (0, 1), (1, 1))


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


def direction_heading(delta_col: int, delta_row: int) -> float:
    """Convierte uno de los ocho pasos de la cuadrícula al convenio de visión."""
    return float(45 * DIRECTIONS.index((delta_col, delta_row)))


def quantize_heading(theta_deg: float) -> float:
    """Cuantiza circularmente al múltiplo de 45 grados más cercano."""
    return float((round(theta_deg / 45.0) % 8) * 45)


def cell_for_position(col: float, row: float, cols: int, rows: int) -> tuple[int, int]:
    return min(cols - 1, max(0, math.floor(col))), min(rows - 1, max(0, math.floor(row)))


def stable_cell_axis(
    coordinate: float, confirmed: int, dimension: int, hysteresis: float = 0.12
) -> int:
    """Evita alternar de celda cuando visión oscila alrededor de una frontera."""
    candidate = min(dimension - 1, max(0, math.floor(coordinate)))
    if candidate == confirmed:
        return candidate
    if candidate == confirmed + 1 and coordinate < confirmed + 1 + hysteresis:
        return confirmed
    if candidate == confirmed - 1 and coordinate > confirmed - hysteresis:
        return confirmed
    return candidate


def a_star_route(
    cols: int,
    rows: int,
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: set[tuple[int, int]],
) -> list[tuple[int, int]]:
    """A* de ocho vecinos, costes 10/14 y diagonales sin cortar esquinas."""
    if goal in blocked:
        return []

    def heuristic(cell: tuple[int, int]) -> int:
        dx, dy = abs(goal[0] - cell[0]), abs(goal[1] - cell[1])
        return 14 * min(dx, dy) + 10 * abs(dx - dy)

    queue = [(heuristic(start), 0, start)]
    costs = {start: 0}
    parents: dict[tuple[int, int], tuple[int, int]] = {}
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost != costs.get(current):
            continue
        if current == goal:
            route = [current]
            while current != start:
                current = parents[current]
                route.append(current)
            return list(reversed(route))
        for dc, dr in DIRECTIONS:
            nxt = current[0] + dc, current[1] + dr
            if not (0 <= nxt[0] < cols and 0 <= nxt[1] < rows) or nxt in blocked:
                continue
            if dc and dr and ((current[0] + dc, current[1]) in blocked or
                              (current[0], current[1] + dr) in blocked):
                continue
            candidate = cost + (14 if dc and dr else 10)
            if candidate >= costs.get(nxt, math.inf):
                continue
            costs[nxt] = candidate
            parents[nxt] = current
            heapq.heappush(queue, (candidate + heuristic(nxt), candidate, nxt))
    return []


def queen_move_points(
    start: tuple[float, float], target: tuple[float, float]
) -> list[tuple[float, float]]:
    """Descompone un desplazamiento continuo en diagonal y recta."""
    dx, dy = target[0] - start[0], target[1] - start[1]
    diagonal = min(abs(dx), abs(dy))
    points: list[tuple[float, float]] = []
    cursor = start
    if diagonal > 1e-6:
        cursor = (start[0] + math.copysign(diagonal, dx),
                  start[1] + math.copysign(diagonal, dy))
        points.append(cursor)
    if math.dist(cursor, target) > 1e-6:
        points.append(target)
    return points


def lattice_route_points(
    route: list[tuple[int, int]],
    start_position: tuple[float, float],
    target: tuple[float, float],
) -> list[tuple[float, float]]:
    """Proyecta A* conservando el desplazamiento fraccional de la pose inicial."""
    if not route:
        return []
    offset_col = start_position[0] - route[0][0]
    offset_row = start_position[1] - route[0][1]
    points = [
        (cell[0] + offset_col, cell[1] + offset_row)
        for cell in route[1:]
    ]
    cursor = points[-1] if points else start_position
    if math.dist(cursor, target) > 0.4:
        points.extend(queen_move_points(cursor, target))
    return points


def ultrasonic_sample_action(invalid_samples: int, valid: bool) -> tuple[int, str]:
    """Un fallo aislado pausa; tres fallos consecutivos cancelan por seguridad."""
    if valid:
        return 0, "continue"
    invalid_samples += 1
    return invalid_samples, "error" if invalid_samples >= 3 else "pause"


def slow_drive_gate(
    distance: float, new_vision_frame: bool, pulse_ticks: int
) -> tuple[int, bool]:
    """Cerca del waypoint permite dos ciclos de motor por captura visual nueva."""
    if distance > 3.0:
        return 0, True
    if new_vision_frame and pulse_ticks == 0:
        pulse_ticks = 2
    if pulse_ticks == 0:
        return 0, False
    return pulse_ticks - 1, True


def arrival_confirmation(
    samples: int, distance: float, speed: float, new_vision_frame: bool
) -> tuple[int, bool]:
    """Confirma llegada sólo con cinco capturas frescas y el rover ya asentado."""
    if not new_vision_frame:
        return samples, False
    if distance > 0.4 or abs(speed) > 0.35:
        return 0, False
    samples += 1
    return samples, samples >= 5


def compress_route(route: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Conserva sólo los puntos donde cambia una de las ocho direcciones."""
    if len(route) <= 2:
        return route[:]
    result = [route[0]]
    previous = (route[1][0] - route[0][0], route[1][1] - route[0][1])
    for index in range(2, len(route)):
        direction = (route[index][0] - route[index - 1][0],
                     route[index][1] - route[index - 1][1])
        if direction != previous:
            result.append(route[index - 1])
            previous = direction
    result.append(route[-1])
    return result


def blind_crossing_allowed(crossings: int, grid_calibrated: bool) -> bool:
    return grid_calibrated and crossings < 2


def confirm_cell_crossing(
    confirmed: tuple[int, int],
    candidate: tuple[int, int],
    direction: tuple[int, int],
    observed_pattern: int,
    expected_pattern: int,
    stable_samples: int,
    blind_crossings: int,
    grid_calibrated: bool = True,
    confirmed_pattern: int | None = None,
) -> str:
    """Modelo de la compuerta que evita contar rebotes o cruces ambiguos."""
    delta = candidate[0] - confirmed[0], candidate[1] - confirmed[1]
    if (not grid_calibrated or stable_samples < 3 or observed_pattern in (0, 0x0F) or
            observed_pattern != expected_pattern or observed_pattern == confirmed_pattern or
            delta != direction):
        return "none"
    return "crossed" if blind_crossings < 2 else "wait_for_vision"


def occupied_cells(
    cols: int,
    rows: int,
    obstacles: list[tuple[int, int]],
    peer: tuple[int, int] | None,
) -> set[tuple[int, int]]:
    """Replica la ocupación circular del firmware, sin bloquear el borde."""
    rover_radius = math.hypot(5 / 2, 7.5 / 2)
    clearance = 0.4
    result: set[tuple[int, int]] = set()
    entities = [(item, rover_radius + 5 * math.sqrt(2) / 2 + clearance)
                for item in obstacles]
    if peer is not None:
        entities.append((peer, 2 * rover_radius + clearance))
    for center, radius in entities:
        for row in range(max(0, math.floor(center[1] - radius)),
                         min(rows, math.ceil(center[1] + radius) + 1)):
            for col in range(max(0, math.floor(center[0] - radius)),
                             min(cols, math.ceil(center[0] + radius) + 1)):
                if math.hypot(col + 0.5 - center[0], row + 0.5 - center[1]) <= radius:
                    result.add((col, row))
    return result


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
    if distance <= 0.4:
        return "arrived", 0, 0
    target_heading = math.degrees(math.atan2(-dy, dx))
    error = wrap_degrees(target_heading - theta_deg)
    if driving and abs(error) > 15.0:
        driving = False
    if not driving and abs(error) <= 8.0:
        driving = True
    if not driving:
        return "turning", (-700 if error > 0 else 700), (700 if error > 0 else -700)
    base = 700 if distance <= 3.0 else 850
    correction = max(-150, min(150, round(error * 8.0)))
    return "driving", normalize_motor(base - correction), normalize_motor(base + correction)
