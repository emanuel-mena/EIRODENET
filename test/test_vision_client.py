from __future__ import annotations

import json
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from vision_client.core import NDJSONDecoder, VisionConfig, load_dotenv  # noqa: E402


def test_decoder_reassembles_fragmented_and_batched_messages() -> None:
    decoder = NDJSONDecoder()
    assert decoder.feed(b'{"seq":1') == []
    assert decoder.feed(b'}\n{"seq":2}\n') == [{"seq": 1}, {"seq": 2}]


def test_decoder_rejects_non_object_json() -> None:
    decoder = NDJSONDecoder()
    try:
        decoder.feed(json.dumps([1, 2]).encode() + b"\n")
    except ValueError as exc:
        assert "objeto JSON" in str(exc)
    else:
        raise AssertionError("debió rechazar una raíz que no es objeto")


def test_dotenv_accepts_spaces_quotes_and_comments(tmp_path: Path) -> None:
    path = tmp_path / ".env"
    path.write_text(' A = "uno dos"\n# comentario\nB=3\n', encoding="utf-8")
    assert load_dotenv(path) == {"A": "uno dos", "B": "3"}


def test_config_derives_vision_system_and_python(monkeypatch, tmp_path: Path) -> None:
    repo = tmp_path / "challenge"
    env_file = tmp_path / ".env"
    env_file.write_text(f"VISION_CHALLENGE_REPO={repo}\n", encoding="utf-8")
    for key in ("VISION_CHALLENGE_REPO", "VISION_PYTHON", "VISION_HOST", "VISION_PORT"):
        monkeypatch.delenv(key, raising=False)
    config = VisionConfig.from_environment(env_file)
    assert config.vision_system == repo.resolve() / "vision-system"
    assert config.port == 2026


def test_config_resolves_profile_index_using_challenge_menu_order(monkeypatch, tmp_path: Path) -> None:
    repo = tmp_path / "challenge"
    profiles = repo / "vision-system" / "vision" / "calibraciones"
    profiles.mkdir(parents=True)
    (profiles / "zeta.json").touch()
    (profiles / "alpha.json").touch()
    env_file = tmp_path / ".env"
    env_file.write_text(
        f"VISION_CHALLENGE_REPO={repo}\n"
        "VISION_CAMERA_INDEX=0\n"
        "VISION_CAMERA_PROFILE_INDEX=1\n",
        encoding="utf-8",
    )
    for key in (
        "VISION_CHALLENGE_REPO", "VISION_PYTHON", "VISION_HOST", "VISION_PORT",
        "VISION_CAMERA_INDEX", "VISION_CAMERA_PROFILE_INDEX",
    ):
        monkeypatch.delenv(key, raising=False)
    config = VisionConfig.from_environment(env_file)
    assert config.camera_index == 0
    assert config.camera_profile_name() == "zeta"
