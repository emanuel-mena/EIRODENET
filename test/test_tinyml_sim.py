import copy
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import rover_sim.runner as runner
from rover_sim.build import build
from rover_sim.layout import apply_domain_randomization, apply_seeded_entropy
from rover_sim.world import World, scenario


def test_domain_randomization_is_reproducible_and_bounded():
    base = scenario()
    first = apply_domain_randomization(copy.deepcopy(base), 1009)
    second = apply_domain_randomization(copy.deepcopy(base), 1009)
    other = apply_domain_randomization(copy.deepcopy(base), 1013)
    assert first == second
    assert first != other
    for name in ("rover_mass", "cube_mass", "full_speed_mm_s",
                 "traction_accel_mm_s2", "motor_response_s"):
        assert first["parameters"][name] == pytest.approx(
            base["parameters"][name], rel=.10)
    for name in ("contact_friction", "cube_floor_deceleration_mm_s2"):
        assert first["parameters"][name] == pytest.approx(
            base["parameters"][name], rel=.20)
    assert 0 <= first["parameters"]["motor_strength_difference"] <= .08
    assert 0 <= first["parameters"]["vision_delay_ms"] <= 250
    assert 10 <= first["parameters"]["peer_delay_ms"] <= 250


def test_seeded_entropy_changes_physical_cube_layout_reproducibly():
    base = scenario()
    first = apply_seeded_entropy(copy.deepcopy(base), 11)
    repeated = apply_seeded_entropy(copy.deepcopy(base), 11)
    other = apply_seeded_entropy(copy.deepcopy(base), 19)
    assert first == repeated
    assert first["cubes"] != base["cubes"]
    assert first["cubes"] != other["cubes"]
    assert first["challenge_layout"]["seed"] == 11
    assert first["domain_randomization"]["seed"] == 11
    assert first["seeded_entropy"] == {
        "seed": 11,
        "difficulty": .5,
        "layout_algorithm": "mulberry32-v1",
        "domain_algorithm": "eiro-domain-v1",
    }


def test_host_policy_emits_canonical_68_value_observation(tmp_path, monkeypatch):
    captured = []

    class CapturePolicy:
        def __init__(self, _model):
            pass

        def __call__(self, observation):
            captured.append(observation)
            return [0.0, 0.0], [0] * 68, [0, 0]

    monkeypatch.setattr(runner, "TFLitePolicy", CapturePolicy)
    controller = runner.Controller(build(), 10, tmp_path / "controller.log", model=tmp_path / "dummy.tflite")
    world = World(scenario())
    try:
        response = controller.step(dict(
            step=1,
            time_ms=3000,
            vision=world.frame(3000, 1),
            sensors=world.sensors(0),
            peer=dict(available=True, delivered=0, assignment_id=0,
                      assignment_color=0, assignment_result=0, phase=0,
                      model_available=True, model_version=1,
                      model_crc32=0x54464C33, ack={}),
            messages=[],
        ))
    finally:
        controller.close()
    assert response["step"] == 1
    assert response["messages"] and set(response["messages"][0]) == {"id", "color"}
    assert captured
    observation = captured[0]
    assert len(observation) == 68
    assert all(-1 <= value <= 1 for value in observation)
    assert observation[0] == 1
    assert observation[1:7].count(1) == 1
    assert observation[7:10].count(1) == 1


def test_simulator_without_model_keeps_autonomy_stopped(tmp_path):
    sim = runner.Simulation(scenario(), tmp_path / "no-model", build())
    try:
        for _ in range(400):
            sim.step()
        assert all(status["left"] == status["right"] == 0 for status in sim.status)
        assert all(status["model_version"] == 0 for status in sim.status)
    finally:
        sim.close()
