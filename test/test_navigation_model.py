from __future__ import annotations

import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from navigation_model import (  # noqa: E402
    DIRECTIONS,
    a_star_route,
    arrival_confirmation,
    blind_crossing_allowed,
    cell_for_position,
    checker_pattern,
    compress_route,
    confirm_cell_crossing,
    controller_command,
    direction_heading,
    lattice_route_points,
    navigation_inputs_ready,
    normalize_motor,
    occupied_cells,
    quantize_heading,
    queen_move_points,
    sensor_positions,
    slow_drive_gate,
    stable_cell_axis,
    ultrasonic_sample_action,
    vision_frame_is_new,
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
    assert controller_command(9.7, 5, 0, 10, 5) == ("arrived", 0, 0)


def test_checker_is_periodic_and_all_equal_patterns_are_detectable_as_ambiguous() -> None:
    base = checker_pattern(7.25, 9.5, 45.0)
    assert checker_pattern(9.25, 9.5, 45.0) == base
    assert checker_pattern(0.5, 0.5, 45.0) == 0x0F
    assert checker_pattern(0.5, 1.5, 45.0) == 0x00


def test_row_axis_points_down_while_positive_theta_points_up() -> None:
    phase, left, right = controller_command(10, 10, 90, 10, 5)
    assert phase == "driving"
    assert left == right == 850


def test_repeated_messages_from_one_camera_frame_are_not_new_observations() -> None:
    assert vision_frame_is_new(None, 1789680978545)
    assert not vision_frame_is_new(1789680978545, 1789680978545)
    assert vision_frame_is_new(1789680978545, 1789680978946)


def test_navigation_requires_every_safety_input() -> None:
    ready = dict(vision_connected=True, protocol_valid=True, pose_fresh=True,
                 imu_valid=True, imu_calibrated=True, infrared_valid=True,
                 ultrasonic_valid=True)
    assert navigation_inputs_ready(**ready)
    for input_name in ready:
        missing = {**ready, input_name: False}
        assert not navigation_inputs_ready(**missing)


def test_all_eight_grid_directions_match_vision_angles() -> None:
    assert [direction_heading(*direction) for direction in DIRECTIONS] == [
        0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
    assert quantize_heading(359.0) == 0.0
    assert quantize_heading(44.0) == 45.0
    assert quantize_heading(-46.0) == 315.0


def test_positions_map_to_floor_cells_and_clamp_outer_boundary() -> None:
    assert cell_for_position(3.75, 21.5, 43, 43) == (3, 21)
    assert cell_for_position(43.0, 43.0, 43, 43) == (42, 42)


def test_cell_hysteresis_rejects_vision_jitter_at_integer_boundary() -> None:
    assert stable_cell_axis(12.96, 13, 43) == 13
    assert stable_cell_axis(13.08, 13, 43) == 13
    assert stable_cell_axis(12.87, 13, 43) == 12
    assert stable_cell_axis(14.13, 13, 43) == 14


def test_a_star_uses_diagonal_and_compresses_straight_run() -> None:
    route = a_star_route(10, 10, (1, 1), (5, 5), set())
    assert route == [(1, 1), (2, 2), (3, 3), (4, 4), (5, 5)]
    assert compress_route(route) == [(1, 1), (5, 5)]


def test_a_star_detours_and_never_cuts_a_blocked_corner() -> None:
    blocked = {(2, 1), (1, 2)}
    route = a_star_route(6, 6, (1, 1), (4, 4), blocked)
    assert route
    assert route[1] != (2, 2)
    for current, nxt in zip(route, route[1:]):
        dc, dr = nxt[0] - current[0], nxt[1] - current[1]
        if dc and dr:
            assert (current[0] + dc, current[1]) not in blocked
            assert (current[0], current[1] + dr) not in blocked


def test_a_star_reports_no_route_or_blocked_goal() -> None:
    wall = {(2, row) for row in range(5)}
    assert a_star_route(5, 5, (1, 2), (3, 2), wall) == []
    assert a_star_route(5, 5, (1, 1), (3, 3), {(3, 3)}) == []


def test_fractional_target_is_reached_with_only_queen_directions() -> None:
    start, target = (3.5, 21.5), (7.75, 18.25)
    points = queen_move_points(start, target)
    assert points[-1] == target
    for origin, endpoint in zip([start, *points[:-1]], points):
        dc = 0 if endpoint[0] == origin[0] else (1 if endpoint[0] > origin[0] else -1)
        dr = 0 if endpoint[1] == origin[1] else (1 if endpoint[1] > origin[1] else -1)
        assert (dc, dr) in DIRECTIONS


def test_grid_route_keeps_fractional_offset_instead_of_centering_first() -> None:
    route = [(13, 39), (13, 38), (13, 37)]
    start = (13.02, 39.58)
    points = lattice_route_points(route, start, (13.0, 37.5))
    assert points[0] == (13.02, 38.58)
    assert points[0][0] == start[0]
    assert points == [(13.02, 38.58), (13.02, 37.58)]
    for origin, endpoint in zip([start, *points[:-1]], points):
        dc = 0 if abs(endpoint[0] - origin[0]) < 1e-9 else (1 if endpoint[0] > origin[0] else -1)
        dr = 0 if abs(endpoint[1] - origin[1]) < 1e-9 else (1 if endpoint[1] > origin[1] else -1)
        assert (dc, dr) in DIRECTIONS


def test_transient_ultrasonic_timeout_pauses_before_failing() -> None:
    samples, action = ultrasonic_sample_action(0, False)
    assert (samples, action) == (1, "pause")
    samples, action = ultrasonic_sample_action(samples, False)
    assert (samples, action) == (2, "pause")
    samples, action = ultrasonic_sample_action(samples, True)
    assert (samples, action) == (0, "continue")
    for expected in ("pause", "pause", "error"):
        samples, action = ultrasonic_sample_action(samples, False)
        assert action == expected


def test_slow_approach_pulses_only_after_fresh_vision() -> None:
    ticks, drive = slow_drive_gate(2.0, False, 0)
    assert (ticks, drive) == (0, False)
    ticks, drive = slow_drive_gate(2.0, True, ticks)
    assert (ticks, drive) == (1, True)
    ticks, drive = slow_drive_gate(2.0, False, ticks)
    assert (ticks, drive) == (0, True)
    assert slow_drive_gate(2.0, False, ticks) == (0, False)
    assert slow_drive_gate(4.0, False, 0) == (0, True)


def test_arrival_requires_five_fresh_stationary_observations() -> None:
    samples, arrived = arrival_confirmation(0, 0.2, 1.0, True)
    assert (samples, arrived) == (0, False)
    for expected in range(1, 6):
        samples, arrived = arrival_confirmation(samples, 0.2, 0.1, True)
        assert samples == expected
        assert arrived is (expected == 5)
    assert arrival_confirmation(samples, 0.2, 0.1, False) == (samples, False)
    assert arrival_confirmation(samples, 0.5, 0.0, True) == (0, False)


def test_blind_navigation_allows_two_crossings_only_with_calibrated_grid() -> None:
    assert blind_crossing_allowed(0, True)
    assert blind_crossing_allowed(1, True)
    assert not blind_crossing_allowed(2, True)
    assert not blind_crossing_allowed(0, False)


def test_crossing_gate_counts_cardinal_and_diagonal_once_patterns_are_stable() -> None:
    common = dict(observed_pattern=0b0110, expected_pattern=0b0110,
                  stable_samples=3, blind_crossings=0)
    assert confirm_cell_crossing((4, 4), (5, 4), (1, 0), **common) == "crossed"
    assert confirm_cell_crossing((4, 4), (5, 3), (1, -1), **common) == "crossed"
    assert confirm_cell_crossing((4, 4), (5, 4), (1, -1), **common) == "none"
    assert confirm_cell_crossing((4, 4), (5, 3), (1, -1),
                                 **{**common, "stable_samples": 2}) == "none"
    assert confirm_cell_crossing((4, 4), (5, 3), (1, -1),
                                 **{**common, "observed_pattern": 0x0F}) == "none"
    assert confirm_cell_crossing((4, 4), (5, 3), (1, -1),
                                 **common, confirmed_pattern=0b0110) == "none"


def test_third_blind_crossing_waits_for_vision() -> None:
    assert confirm_cell_crossing(
        (4, 4), (5, 4), (1, 0), 0b0110, 0b0110, 3, 2
    ) == "wait_for_vision"


def test_visual_entities_inflate_occupancy_and_force_a_detour() -> None:
    blocked = occupied_cells(20, 20, obstacles=[(10, 10)], peer=(5, 14))
    assert (10, 10) in blocked and (15, 15) in blocked
    assert (5, 14) in blocked and (9, 18) in blocked
    route = a_star_route(20, 20, (3, 3), (16, 3), blocked - {(3, 3)})
    assert route
    assert all(cell not in blocked for cell in route[1:])


def test_border_is_available_and_rover_clearance_uses_full_body() -> None:
    assert occupied_cells(20, 20, obstacles=[], peer=None) == set()
    blocked = occupied_cells(30, 30, obstacles=[], peer=(15, 15))
    assert (15, 23) in blocked
    assert (15, 25) not in blocked
