from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent / "rover_gui"))
from core import (  # noqa: E402
    FACE_NAMES, FrameParser, alignment_degrees, calibration_rejection_messages,
    detected_face,
)


class RoverSerial:
    def __init__(self, port: str) -> None:
        self.serial = serial.Serial(port, 115200, timeout=0.1, write_timeout=1.0)
        self.parser = FrameParser()
        self.next_id = 1
        time.sleep(2.0)  # La apertura del CH340 reinicia normalmente el ESP32.

    def close(self) -> None:
        self.serial.close()

    def messages(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for message in self.parser.feed(self.serial.read(256)):
                yield message

    def request(self, command: str, timeout: float = 4.0, **fields) -> dict:
        request_id = self.next_id
        self.next_id += 1
        payload = {"v": 1, "id": request_id, "cmd": command, **fields}
        wire = b"@EIRO " + json.dumps(payload, separators=(",", ":")).encode() + b"\n"
        self.serial.write(wire)
        for message in self.messages(timeout):
            if message.get("type") == "response" and message.get("id") == request_id:
                if not message.get("ok"):
                    error = message.get("error", {})
                    raise RuntimeError(f"{error.get('code')}: {error.get('message')}")
                return message.get("data", {})
        raise TimeoutError(f"La placa no respondió a {command}")


def resolve_port(requested: str | None) -> str:
    if requested:
        return requested
    available = list(list_ports.comports())
    preferred = [port for port in available if port.vid == 0x1A86 and port.pid == 0x7523]
    candidates = preferred if preferred else available
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        raise RuntimeError("No se detectó ningún puerto serie; conecte el rover o use --port")
    names = ", ".join(port.device for port in candidates)
    raise RuntimeError(f"Hay varios puertos posibles ({names}); seleccione uno con --port")


def vector(values) -> str:
    return "[" + ", ".join(f"{float(value):+.5f}" for value in values) + "]"


def print_diagnostics(event: dict, face_index: int) -> None:
    accel = event.get("accel_mean", [0, 0, 0])
    accel_std = event.get("accel_stddev", [0, 0, 0])
    gyro = event.get("gyro_mean", [0, 0, 0])
    gyro_std = event.get("gyro_stddev", [0, 0, 0])
    norm = math.sqrt(sum(float(value) ** 2 for value in accel))
    print(f"  aceleración media : {vector(accel)} g")
    print(f"  aceleración sigma : {vector(accel_std)} g")
    print(f"  giroscopio medio    : {vector(gyro)} dps")
    print(f"  giroscopio sigma    : {vector(gyro_std)} dps")
    print(f"  |g|={norm:.4f}, cara detectada={detected_face(accel)}, "
          f"desalineación={alignment_degrees(accel, face_index):.1f}°")
    reasons = calibration_rejection_messages(int(event.get("rejection_mask", 0)))
    if reasons:
        print("  rechazo: " + "; ".join(reasons))


def wait_for_capture(client: RoverSerial, face_index: int) -> bool:
    for event in client.messages(6.0):
        if event.get("type") != "event" or event.get("topic") != "calibration":
            continue
        phase = int(event.get("phase", -1))
        if phase == 2:
            settling = int(event.get("settling_remaining", 0))
            text = f"estabilizando {settling:02d}/50" if settling else f"muestras {int(event.get('samples', 0)):03d}/200"
            print(f"\r  {text}", end="", flush=True)
        elif phase in (3, 4):
            print("\r" + " " * 40 + "\r", end="")
            print_diagnostics(event, face_index)
            return phase == 3
    raise TimeoutError("No se recibió el resultado de la captura")


def calibrate(client: RoverSerial) -> int:
    client.request("calibration.start")
    committed = False
    try:
        print("Marco del rover: X=-Xchip, Y=-Ychip, Z=Zchip.")
        for face_index, face in enumerate(FACE_NAMES):
            while True:
                print(f"\nCara {face}: coloque ese eje del rover apuntando hacia arriba y no lo toque.")
                answer = input("Pulse Enter para capturar, o escriba q para cancelar: ").strip().lower()
                if answer == "q":
                    return 1
                client.request("calibration.capture", face=face_index)
                if wait_for_capture(client, face_index):
                    print(f"  {face} aceptada.")
                    break
                print("  Ajuste el apoyo según el diagnóstico y vuelva a intentarlo.")
        answer = input("\nLas seis caras fueron aceptadas. ¿Guardar calibración? [s/N]: ").strip().lower()
        if answer != "s":
            return 1
        client.request("calibration.commit")
        committed = True
        print("Calibración guardada y aplicada correctamente.")
        return 0
    finally:
        if not committed:
            try: client.request("calibration.cancel")
            except Exception: pass


def probe(client: RoverSerial, seconds: float) -> int:
    client.request("stream.start")
    accel, gyro = [], []
    calibrated = False
    for message in client.messages(seconds):
        if (message.get("type") == "telemetry" and message.get("topic") == "imu" and
                message.get("valid") and "accel_g" in message and "gyro_dps" in message):
            accel.append(message["accel_g"]); gyro.append(message["gyro_dps"])
            calibrated = bool(message.get("calibrated"))
    if not accel:
        raise RuntimeError("No se recibieron muestras válidas del IMU")
    print(f"Muestras recibidas: {len(accel)} ({'calibradas' if calibrated else 'sin calibrar'})")
    for name, rows, unit in (("accel", accel, "g"), ("gyro", gyro, "dps")):
        mean = [statistics.fmean(row[axis] for row in rows) for axis in range(3)]
        std = [statistics.pstdev(row[axis] for row in rows) for axis in range(3)]
        print(f"{name:5s} media={vector(mean)} {unit}  sigma={vector(std)} {unit}")
    mean_accel = [statistics.fmean(row[axis] for row in accel) for axis in range(3)]
    print(f"cara dominante: {detected_face(mean_accel)}")
    return 0


def collect_imu(client: RoverSerial, seconds: float) -> tuple[list[list[float]], list[list[float]]]:
    accel, gyro = [], []
    for message in client.messages(seconds):
        if (message.get("type") == "telemetry" and message.get("topic") == "imu" and
                message.get("valid") and "accel_g" in message and "gyro_dps" in message):
            accel.append(message["accel_g"])
            gyro.append(message["gyro_dps"])
    if not accel:
        raise RuntimeError("No se recibieron muestras válidas del IMU")
    return accel, gyro


def deviation_norm(row: list[float], mean: list[float]) -> float:
    return math.sqrt(sum((float(row[axis]) - mean[axis]) ** 2 for axis in range(3)))


def axis_peaks(rows: list[list[float]], mean: list[float]) -> list[float]:
    return [max(abs(float(row[axis]) - mean[axis]) for row in rows) for axis in range(3)]


def test_motors(client: RoverSerial) -> int:
    client.request("stream.start")
    print("Midiendo referencia con el rover estacionario...")
    base_accel, base_gyro = collect_imu(client, 1.5)
    accel_mean = [statistics.fmean(row[axis] for row in base_accel) for axis in range(3)]
    gyro_mean = [statistics.fmean(row[axis] for row in base_gyro) for axis in range(3)]
    base_accel_peak = max(deviation_norm(row, accel_mean) for row in base_accel)
    base_gyro_peak = max(deviation_norm(row, gyro_mean) for row in base_gyro)

    print("Activando ambos motores al 70 % durante 1 segundo...")
    response = client.request("motors.test_forward")
    duration = max(float(response.get("duration_ms", 1000)) / 1000.0, 1.0)
    moving_accel, moving_gyro = collect_imu(client, duration + 0.5)
    moving_accel_peak = max(deviation_norm(row, accel_mean) for row in moving_accel)
    moving_gyro_peak = max(deviation_norm(row, gyro_mean) for row in moving_gyro)
    moving_accel_axes = axis_peaks(moving_accel, accel_mean)
    moving_gyro_axes = axis_peaks(moving_gyro, gyro_mean)
    accel_detected = moving_accel_peak > max(0.02, base_accel_peak * 3.0)
    gyro_detected = moving_gyro_peak > max(2.0, base_gyro_peak * 3.0)

    print(f"Ruido en reposo: accel={base_accel_peak:.4f} g, gyro={base_gyro_peak:.3f} dps")
    print(f"Pico en prueba : accel={moving_accel_peak:.4f} g, gyro={moving_gyro_peak:.3f} dps")
    print(f"Picos por eje  : accel={vector(moving_accel_axes)} g")
    print(f"                 gyro={vector(moving_gyro_axes)} dps")
    if accel_detected or gyro_detected:
        print("RESULTADO: el IMU detectó movimiento durante la activación de los motores.")
        return 0
    print("RESULTADO: no se detectó movimiento por encima del ruido de reposo.")
    return 1


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="CLI de diagnóstico y calibración EIRODENET")
    parser.add_argument("--port", help="Puerto serie; si se omite se detecta automáticamente el CH340")
    subparsers = parser.add_subparsers(dest="action", required=True)
    subparsers.add_parser("calibrate", help="Ejecuta el asistente interactivo de seis caras")
    subparsers.add_parser("test-motors", help="Activa ambos motores y comprueba movimiento con el IMU")
    probe_parser = subparsers.add_parser("probe", help="Resume las lecturas actuales del IMU")
    probe_parser.add_argument("--seconds", type=float, default=4.0)
    args = parser.parse_args()
    port_name = resolve_port(args.port)
    client = RoverSerial(port_name)
    print(f"Puerto: {port_name}")
    try:
        if args.action == "calibrate": return calibrate(client)
        if args.action == "test-motors": return test_motors(client)
        return probe(client, args.seconds)
    except KeyboardInterrupt:
        print("\nCancelado.")
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
