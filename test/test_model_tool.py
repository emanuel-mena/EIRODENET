import struct
import zlib

import pytest

from tools import model_tool


def tflite(payload=b"tiny-policy"):
    return b"\x1c\x00\x00\x00TFL3" + payload


def test_build_and_parse_eirm_image(tmp_path):
    source = tmp_path / "policy.tflite"
    output = tmp_path / "model.bin"
    source.write_bytes(tflite())

    built = model_tool.build(source, output, version=1)
    parsed = model_tool.parse_image(output.read_bytes())

    assert parsed == built
    assert output.read_bytes()[:4] == b"EIRM"
    assert parsed.crc32 == zlib.crc32(parsed.model) & 0xFFFFFFFF


@pytest.mark.parametrize("data, message", [
    (b"", "truncada"),
    (struct.pack("<4sIII", b"NOPE", 1, 0, 0), "EIRM"),
    (struct.pack("<4sIII", b"EIRM", 2, 0, 0), "versión"),
])
def test_parse_rejects_invalid_headers(data, message):
    with pytest.raises(ValueError, match=message):
        model_tool.parse_image(data)


def test_parse_rejects_missing_tfl3_and_bad_crc():
    invalid = b"\x00" * 12
    bad_signature = model_tool.HEADER.pack(b"EIRM", 1, len(invalid), zlib.crc32(invalid)) + invalid
    with pytest.raises(ValueError, match="TFL3"):
        model_tool.parse_image(bad_signature)

    model = tflite()
    bad_crc = model_tool.HEADER.pack(b"EIRM", 1, len(model), 0) + model
    with pytest.raises(ValueError, match="CRC32"):
        model_tool.parse_image(bad_crc)


def test_parse_rejects_length_mismatch_and_partition_overflow():
    model = tflite()
    mismatch = model_tool.HEADER.pack(b"EIRM", 1, len(model) + 1, 0) + model
    with pytest.raises(ValueError, match="longitud declarada"):
        model_tool.parse_image(mismatch)

    overflow = model_tool.HEADER.pack(b"EIRM", 1, model_tool.MODEL_PARTITION_SIZE, 0)
    with pytest.raises(ValueError, match="excede"):
        model_tool.parse_image(overflow)


def test_flash_validates_before_invoking_esptool(tmp_path, monkeypatch):
    image = tmp_path / "model.bin"
    image.write_bytes(b"not-an-image")
    called = False

    def run(*_args, **_kwargs):
        nonlocal called
        called = True

    monkeypatch.setattr(model_tool.subprocess, "run", run)
    with pytest.raises(ValueError):
        model_tool.flash(image, "COM99", 460800)
    assert not called


def test_flash_writes_only_at_model_partition_offset(tmp_path, monkeypatch):
    source = tmp_path / "policy.tflite"
    image = tmp_path / "model.bin"
    source.write_bytes(tflite())
    model_tool.build(source, image, 1)
    calls = []
    monkeypatch.setattr(model_tool.subprocess, "run", lambda command, check: calls.append((command, check)))

    model_tool.flash(image, "COM99", 115200)

    assert len(calls) == 1 and calls[0][1] is True
    command = calls[0][0]
    assert command[command.index("write_flash") + 1] == "0x600000"
    assert command[-1] == str(image)
