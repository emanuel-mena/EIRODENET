from __future__ import annotations

import math
import sys
from pathlib import Path

GUI_DIR = Path(__file__).resolve().parents[1] / "tools" / "rover_gui"
sys.path.insert(0, str(GUI_DIR))

from core import (  # noqa: E402
    FrameParser, SequenceTracker, TimeSeries, alignment_degrees,
    calibration_rejection_messages, detected_face, relative_quaternion,
)


def test_parser_accepts_partial_frame_and_ignores_logs() -> None:
    parser = FrameParser()
    assert parser.feed(b"I (22) rover: inicio\n@EIR") == []
    assert parser.feed(b'O {"v":1,"type":"telemetry","seq":7}\r\n') == [
        {"v": 1, "type": "telemetry", "seq": 7}
    ]


def test_parser_recovers_after_invalid_utf8_and_overflow() -> None:
    parser = FrameParser()
    wire = b"@EIRO \xff\n" + b"x" * 1100 + b"\n@EIRO {\"ok\":true}\n"
    assert parser.feed(wire) == [{"ok": True}]


def test_sequence_tracker_counts_only_forward_gaps() -> None:
    tracker = SequenceTracker()
    for sequence in (10, 11, 14, 13, 15):
        tracker.observe(sequence)
    assert tracker.last == 15
    assert tracker.lost == 2


def test_time_series_keeps_sixty_second_window() -> None:
    series = TimeSeries(60.0)
    series.append((1.0,), now=0.0)
    series.append((2.0,), now=59.0)
    series.append((3.0,), now=61.0)
    assert [row for _, row in series.points] == [(2.0,), (3.0,)]


def test_relative_quaternion_uses_manual_zero_reference() -> None:
    half = math.sqrt(0.5)
    reference = (half, half, 0.0, 0.0)
    result = relative_quaternion(reference, reference)
    assert result == (1.0, 0.0, 0.0, 0.0)


def test_calibration_diagnostics_identify_face_angle_and_reasons() -> None:
    accel = (-0.08, -0.09, 1.03)
    assert detected_face(accel) == "+Z"
    assert alignment_degrees(accel, 4) < 10.0
    assert calibration_rejection_messages(1 | 8) == [
        "movimiento detectado por el acelerómetro",
        "inclinación excesiva respecto al eje solicitado",
    ]
