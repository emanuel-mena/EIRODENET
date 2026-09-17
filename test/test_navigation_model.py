from __future__ import annotations

import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from navigation_model import (  # noqa: E402
    checker_pattern,
    controller_command,
    normalize_motor,
    sensor_positions,
    wrap_degrees,
)


def test_every_nonzero_motor_command_has_a_seventy_percent_floor() -> None:
    assert normalize_motor(0) == 0
    assert normalize_motor(1) == 700
    assert normalize_motor(699) == 700
    assert normalize_motor(-1) == -700
    assert normalize_motor(-699) == -700
    assert normalize_motor(850) == 850


def test_sensor_geometry_matches_physical_array_at_zero_degrees() -> None:
    positions = list(sensor_positions(10.0, 20.0, 0.0))
    assert positions == [(10.5, 19.0), (10.5, 21.0), (9.5, 19.0), (9.5, 21.0)]


def test_angle_wrap_uses_shortest_turn_across_180() -> None:
    assert wrap_degrees(181.0) == -179.0
    assert wrap_degrees(-181.0) == 179.0
    phase, left, right = controller_command(0, 0, 179, -10, 0)
    assert phase == "driving"
    assert left > 0 and right > 0


def test_controller_turns_then_drives_and_stops_inside_tolerance() -> None:
    assert controller_command(5, 5, 90, 10, 5)[0] == "turning"
    assert controller_command(5, 5, 0, 10, 5)[0] == "driving"
    assert controller_command(9.4, 5, 0, 10, 5) == ("arrived", 0, 0)


def test_checker_is_periodic_and_all_equal_patterns_are_detectable_as_ambiguous() -> None:
    base = checker_pattern(7.25, 9.5, 45.0)
    assert checker_pattern(9.25, 9.5, 45.0) == base
    assert checker_pattern(0.5, 0.5, 45.0) == 0x0F
    assert checker_pattern(0.5, 1.5, 45.0) == 0x00


def test_row_axis_points_down_while_positive_theta_points_up() -> None:
    phase, left, right = controller_command(10, 10, 90, 10, 5)
    assert phase == "driving"
    assert left == right == 850
