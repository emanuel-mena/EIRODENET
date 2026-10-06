import json
import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinyml_trainer import (Candidate, candidate_sort_key, heuristic_action,
                            create_model, quantize_model, score_report,
                            write_leaderboard)
from rover_sim.runner import TFLitePolicy


def observation(phase, forward=.4, left=0.0):
    values = [0.0] * 68
    values[0] = 1
    values[1:7] = [-1.0] * 6
    values[1 + phase] = 1
    values[7:10] = [1.0, -1.0, -1.0]
    values[20:24] = [forward, left, forward, left]
    values[28:30] = [.2, .2]
    return values


def test_geometric_teacher_respects_phase_and_turn_direction():
    straight = heuristic_action(observation(0, .6, 0))
    left = heuristic_action(observation(1, .3, .3))
    retreat = heuristic_action(observation(4))
    yield_action = heuristic_action(observation(5))
    assert straight[0] > .5 and straight[1] == pytest.approx(0)
    assert left[0] > 0 and left[1] > 0
    assert retreat == pytest.approx((.72, 0))
    assert yield_action[0] > 0 and yield_action[1] < 0


def test_ranking_prioritizes_eligible_time_then_incomplete_progress():
    candidates = [
        dict(id="partial", eligible=False, deliveries=2, fitness=900_000),
        dict(id="slow", eligible=True, deliveries=3, completion_ms=70_000, fitness=1_100_000),
        dict(id="fast", eligible=True, deliveries=3, completion_ms=50_000, fitness=1_000_000),
        dict(id="weak", eligible=False, deliveries=1, fitness=2_000_000),
    ]
    candidates.sort(key=candidate_sort_key)
    assert [item["id"] for item in candidates] == ["fast", "slow", "partial", "weak"]


def test_score_rewards_delivery_and_rejects_unsafe_run():
    config = dict(cubes=[[0, 0, 0], [100, 0, 0], [200, 0, 0]],
                  depots=[[100, 0], [200, 0], [300, 0]])
    metrics = dict(outside_frames=0, rover_contacts=0, obstacle_contacts=0,
                   simultaneous_motion_ms=1500, effective_yields=1,
                   reassignments=0, blocked_mask=0, completion_ms=42_000)
    report = dict(final=dict(cubes=[[100, 0, 0], [200, 0, 0], [300, 0, 0]],
                             delivered=[True, True, True]),
                  metrics=metrics, eligible=True)
    safe = score_report(config, report, 60)
    report["metrics"] = dict(metrics, outside_frames=1)
    report["eligible"] = False
    unsafe = score_report(config, report, 60)
    assert safe["deliveries"] == 3 and safe["eligible"]
    assert safe["fitness"] > unsafe["fitness"]
    assert unsafe["unsafe_events"] == 1


def test_leaderboard_is_reloadable_and_excludes_weights(tmp_path):
    candidate = Candidate(metadata=dict(id="g001-c001", model="model.tflite",
                                        eligible=False, deliveries=0, fitness=1),
                          weights=[object()])
    path = write_leaderboard(tmp_path, [candidate], "running", {"seed": 7})
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["version"] == 1
    assert data["candidates"] == [candidate.metadata]
    assert not path.with_suffix(".json.tmp").exists()


def test_tinyml_window_starts_without_loading_tensorflow(monkeypatch):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("PySide6")
    from PySide6.QtWidgets import QApplication
    from tinyml_gui.main import Window
    app = QApplication.instance() or QApplication([])
    window = Window()
    try:
        assert window.table.columnCount() == 10
        assert window.sim is None
        assert "TinyML" in window.windowTitle()
        assert window.layout_seeds_label.text() == "11, 19"
    finally:
        window.close()
        app.processEvents()


def test_quantizer_produces_exact_int8_contract_when_tensorflow_is_available(tmp_path):
    tf = pytest.importorskip("tensorflow")
    np = pytest.importorskip("numpy")
    tf.keras.utils.set_random_seed(19)
    model = create_model(tf)
    representative = np.random.default_rng(19).uniform(
        -1, 1, size=(128, 68)).astype(np.float32)
    destination = tmp_path / "candidate.tflite"
    digest = quantize_model(tf, np, model.get_weights(), representative, destination)
    policy = TFLitePolicy(destination)
    action, quantized_input, quantized_output = policy([0.0] * 68)
    assert destination.read_bytes()[4:8] == b"TFL3"
    assert len(digest) == 64
    assert len(action) == len(quantized_output) == 2
    assert len(quantized_input) == 68
