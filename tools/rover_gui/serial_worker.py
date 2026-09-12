from __future__ import annotations

import json
import queue
import threading

import serial
from PySide6.QtCore import QThread, Signal

from core import FrameParser


class SerialWorker(QThread):
    message_received = Signal(dict)
    connection_changed = Signal(bool, str)
    io_error = Signal(str)

    def __init__(self, port: str, parent=None) -> None:
        super().__init__(parent)
        self.port = port
        self._stop = threading.Event()
        self._outgoing: queue.Queue[bytes] = queue.Queue()

    def send(self, message: dict) -> None:
        wire = b"@EIRO " + json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n"
        self._outgoing.put(wire)

    def close(self) -> None:
        self._stop.set()

    def run(self) -> None:
        parser = FrameParser()
        try:
            with serial.Serial(self.port, 115200, timeout=0.05, write_timeout=1.0) as device:
                self.connection_changed.emit(True, self.port)
                while not self._stop.is_set():
                    try:
                        while True:
                            device.write(self._outgoing.get_nowait())
                    except queue.Empty:
                        pass
                    for message in parser.feed(device.read(256)):
                        self.message_received.emit(message)
        except (serial.SerialException, OSError) as exc:
            self.io_error.emit(str(exc))
        finally:
            self.connection_changed.emit(False, self.port)

