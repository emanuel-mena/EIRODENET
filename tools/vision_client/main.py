from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path
from typing import Any

from PySide6.QtCore import QThread, Signal, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtCore import QPointF, QRectF
from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow, QPlainTextEdit, QSplitter,
    QVBoxLayout, QWidget,
)

try:
    from .core import ContractValidator, ROOT, ServerProcess, VisionConfig, VisionConnection
except ImportError:
    from core import ContractValidator, ROOT, ServerProcess, VisionConfig, VisionConnection


COLORS = {
    "red": QColor("#ef4444"), "green": QColor("#22c55e"),
    "blue": QColor("#3b82f6"), "yellow": QColor("#facc15"),
}


class ArenaWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.message: dict[str, Any] | None = None
        self.setMinimumSize(620, 620)

    def set_message(self, message: dict[str, Any]) -> None:
        self.message = message
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#0f172a"))
        if not self.message:
            painter.setPen(QColor("#cbd5e1"))
            painter.drawText(self.rect(), Qt.AlignCenter, "Esperando telemetría…")
            return
        grid = self.message.get("grid", {})
        cols, rows = float(grid.get("cols", 1)), float(grid.get("rows", 1))
        margin = 42.0
        scale = min((self.width() - 2 * margin) / cols, (self.height() - 2 * margin) / rows)
        board = QRectF(margin, margin, cols * scale, rows * scale)
        painter.fillRect(board, QColor("#f8fafc"))
        painter.setPen(QPen(QColor("#dbe4ee"), 1))
        for col in range(int(cols) + 1):
            x = board.left() + col * scale
            painter.drawLine(QPointF(x, board.top()), QPointF(x, board.bottom()))
        for row in range(int(rows) + 1):
            y = board.top() + row * scale
            painter.drawLine(QPointF(board.left(), y), QPointF(board.right(), y))
        painter.setPen(QPen(QColor("#334155"), 2))
        painter.drawRect(board)
        painter.drawText(QPointF(board.left() - 22, board.top() - 12), "ID 0")
        painter.drawText(QPointF(board.right() - 12, board.top() - 12), "ID 1")
        painter.drawText(QPointF(board.right() - 12, board.bottom() + 22), "ID 2")
        painter.drawText(QPointF(board.left() - 22, board.bottom() + 22), "ID 3")

        def point(col: float, row: float) -> QPointF:
            return QPointF(board.left() + col * scale, board.top() + row * scale)

        depot_size = self.message.get("depot_size")
        for depot in self.message.get("depots", []):
            center = point(float(depot["col"]), float(depot["row"]))
            color = COLORS.get(depot.get("color"), QColor("#64748b"))
            if depot_size:
                distances = [
                    (float(depot["row"]), "horizontal"),
                    (rows - float(depot["row"]), "horizontal"),
                    (float(depot["col"]), "vertical"),
                    (cols - float(depot["col"]), "vertical"),
                ]
                orientation = min(distances)[1]
                length = float(depot_size["length"]) * scale
                depth = float(depot_size["depth"]) * scale
                width, height = (length, depth) if orientation == "horizontal" else (depth, length)
                rect = QRectF(center.x() - width / 2, center.y() - height / 2, width, height)
                fill = QColor(color); fill.setAlpha(55)
                painter.fillRect(rect, fill); painter.setPen(QPen(color, 2)); painter.drawRect(rect)
            else:
                painter.setPen(QPen(color, 2)); painter.drawEllipse(center, 7, 7)

        start = self.message.get("start")
        if isinstance(start, dict):
            center = point(float(start["col"]), float(start["row"]))
            painter.setBrush(QColor("#ffffff")); painter.setPen(QPen(QColor("#111827"), 2))
            painter.drawEllipse(center, 7, 7); painter.drawText(center + QPointF(9, -8), "Salida")

        for cube in self.message.get("cubes", []):
            center = point(float(cube["col"]), float(cube["row"]))
            side = float(self.message.get("cube_side", 2.2)) * scale
            painter.fillRect(QRectF(center.x() - side / 2, center.y() - side / 2, side, side),
                             COLORS.get(cube.get("color"), QColor("#64748b")))
        for obstacle in self.message.get("obstacles", []):
            center = point(float(obstacle["col"]), float(obstacle["row"]))
            painter.setBrush(COLORS["yellow"]); painter.setPen(QPen(QColor("#854d0e"), 2))
            painter.drawEllipse(center, 7, 7)
        for rover in self.message.get("rovers", []):
            center = point(float(rover["col"]), float(rover["row"]))
            theta = math.radians(float(rover["theta"]))
            direction = QPointF(math.cos(theta), -math.sin(theta))
            normal = QPointF(-direction.y(), direction.x())
            nose = center + direction * 16
            tail = center - direction * 11
            polygon = QPolygonF([nose, tail + normal * 8, tail - normal * 8])
            painter.setBrush(QColor("#111827")); painter.setPen(QPen(QColor("#f8fafc"), 2))
            painter.drawPolygon(polygon)
            painter.setPen(QColor("#111827")); painter.drawText(center + QPointF(12, -12), str(rover["id"]))


class StreamWorker(QThread):
    message_received = Signal(dict, object)
    status_changed = Signal(str)
    failed = Signal(str)

    def __init__(self, config: VisionConfig, validator: ContractValidator, args) -> None:
        super().__init__()
        self.config, self.validator, self.args = config, validator, args
        self.server = ServerProcess(config)
        self.connection: VisionConnection | None = None
        self.running = True

    def run(self) -> None:
        try:
            if self.args.no_start:
                if not self._connect_with_retries(1):
                    raise ConnectionError("No hay servidor y se indicó --no-start.")
            else:
                self.server.ensure_running(
                    window=not self.args.no_server_window,
                    synthetic=self.args.synthetic,
                    timeout=self.args.timeout,
                    status=self.status_changed.emit,
                )
            while self.running:
                if not self._connect_with_retries(20):
                    raise ConnectionError("No se pudo conectar con el servidor de visión.")
                try:
                    assert self.connection is not None
                    while self.running:
                        try:
                            messages = self.connection.receive()
                        except socket.timeout:
                            continue
                        for message in messages:
                            self.message_received.emit(message, self.validator.validate(message))
                except (ConnectionError, OSError, ValueError) as exc:
                    self.status_changed.emit(f"Conexión perdida: {exc}; reconectando…")
                    if self.connection:
                        self.connection.close()
                    time.sleep(0.25)
        except Exception as exc:  # noqa: BLE001
            self.failed.emit(f"{type(exc).__name__}: {exc}")
        finally:
            if self.connection:
                self.connection.close()
            if not self.args.keep_server:
                self.server.stop()

    def _connect_with_retries(self, attempts: int) -> bool:
        for _ in range(attempts):
            if not self.running:
                return False
            try:
                self.connection = VisionConnection(self.config.host, self.config.port, 1.0)
                self.connection.connect()
                self.status_changed.emit(f"Leyendo {self.config.host}:{self.config.port}")
                return True
            except OSError:
                time.sleep(0.25)
        return False

    def stop(self) -> None:
        self.running = False
        if self.connection:
            self.connection.close()


class MainWindow(QMainWindow):
    def __init__(self, config: VisionConfig, validator: ContractValidator, args) -> None:
        super().__init__()
        self.validator = validator
        self.last_seq: int | None = None
        self.lost = 0
        self.setWindowTitle("EIRODENET — Telemetría real de la cancha")
        self.resize(1180, 760)
        central = QWidget(); layout = QVBoxLayout(central)
        self.banner = QLabel("Preparando conexión…")
        self.banner.setWordWrap(True); self.banner.setFont(QFont("Segoe UI", 11, QFont.Bold))
        layout.addWidget(self.banner)
        splitter = QSplitter()
        self.arena = ArenaWidget(); splitter.addWidget(self.arena)
        side = QWidget(); side_layout = QVBoxLayout(side)
        self.summary = QLabel(); self.summary.setWordWrap(True); self.summary.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.raw = QPlainTextEdit(); self.raw.setReadOnly(True); self.raw.setLineWrapMode(QPlainTextEdit.NoWrap)
        side_layout.addWidget(self.summary); side_layout.addWidget(self.raw, 1)
        splitter.addWidget(side); splitter.setSizes([720, 430]); layout.addWidget(splitter, 1)
        self.setCentralWidget(central)
        self.worker = StreamWorker(config, validator, args)
        self.worker.status_changed.connect(self.show_status)
        self.worker.message_received.connect(self.show_message)
        self.worker.failed.connect(self.show_failure)
        self.worker.start()

    def show_status(self, text: str) -> None:
        self.statusBar().showMessage(text)

    def show_failure(self, text: str) -> None:
        print(text, file=sys.stderr, flush=True)
        self.banner.setText(text); self.banner.setStyleSheet("color:#b91c1c;background:#fee2e2;padding:8px")

    def show_message(self, message: dict[str, Any], error: str | None) -> None:
        self.arena.set_message(message)
        seq = message.get("seq")
        if isinstance(seq, int) and self.last_seq is not None and seq > self.last_seq + 1:
            self.lost += seq - self.last_seq - 1
        if isinstance(seq, int) and (self.last_seq is None or seq > self.last_seq):
            self.last_seq = seq
        latency = int(time.time() * 1000) - int(message.get("ts_ms", 0))
        self.summary.setText(
            f"Fase: {message.get('phase', '—')}\nSecuencia: {seq}\nLatencia: {latency} ms\n"
            f"Rovers: {len(message.get('rovers', []))} · Cubos: {len(message.get('cubes', []))} · "
            f"Perdidos: {self.lost}"
        )
        if error:
            self.banner.setText(
                f"CONTRATO INVÁLIDO — {error}. El visor dibuja el mensaje para diagnóstico, "
                f"pero no debe usarse para programar contra v{self.validator.version}."
            )
            self.banner.setStyleSheet("color:#991b1b;background:#fecaca;padding:8px")
        else:
            self.banner.setText(f"Contrato v{self.validator.version} válido — puntos recibidos por TCP")
            self.banner.setStyleSheet("color:#166534;background:#dcfce7;padding:8px")
        self.raw.setPlainText(json.dumps(message, ensure_ascii=False, indent=2))

    def closeEvent(self, event) -> None:  # noqa: N802
        self.worker.stop(); self.worker.wait(6000)
        event.accept()


def parse_args(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(
        description="Inicia, valida y visualiza el stream TCP del Vision-Rover-Challenge."
    )
    parser.add_argument("--count", type=int, default=0,
                        help="modo agente: imprime esta cantidad de mensajes JSON y sale")
    parser.add_argument("--timeout", type=float, default=25.0,
                        help="segundos máximos para iniciar/conectar")
    parser.add_argument("--no-start", action="store_true", help="no iniciar el servidor si falta")
    parser.add_argument("--synthetic", action="store_true", help="iniciar la fuente sintética")
    parser.add_argument("--no-server-window", action="store_true",
                        help="iniciar el servidor sin su vista de cámara")
    parser.add_argument("--keep-server", action="store_true",
                        help="no detener un servidor iniciado por esta herramienta al salir")
    return parser.parse_args(argv)


def run_headless(config: VisionConfig, validator: ContractValidator, args) -> int:
    server = ServerProcess(config)
    connection = VisionConnection(config.host, config.port, min(args.timeout, 5.0))
    invalid = False
    try:
        if args.no_start:
            if not server_is_available(config):
                print("No hay servidor y se indicó --no-start.", file=sys.stderr)
                return 3
        else:
            server.ensure_running(window=not args.no_server_window, synthetic=args.synthetic,
                                  timeout=args.timeout, status=lambda text: print(text, file=sys.stderr))
        connection.connect()
        deadline = time.monotonic() + args.timeout
        received = 0
        while received < args.count and time.monotonic() < deadline:
            try:
                messages = connection.receive()
            except socket.timeout:
                continue
            for message in messages:
                error = validator.validate(message)
                if error:
                    invalid = True
                    print(f"Contrato inválido: {error}", file=sys.stderr)
                print(json.dumps(message, ensure_ascii=False, separators=(",", ":")), flush=True)
                received += 1
                if received >= args.count:
                    break
        if received < args.count:
            print(f"Solo se recibieron {received} de {args.count} mensajes.", file=sys.stderr)
            return 3
        return 2 if invalid else 0
    finally:
        connection.close()
        if not args.keep_server:
            server.stop()


def server_is_available(config: VisionConfig) -> bool:
    try:
        probe = VisionConnection(config.host, config.port, 0.5)
        probe.connect(); probe.close()
        return True
    except OSError:
        return False


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.count < 0:
        print("--count no puede ser negativo.", file=sys.stderr); return 3
    try:
        config = VisionConfig.from_environment(ROOT / ".env")
        validator = ContractValidator(config.vision_system)
    except Exception as exc:  # noqa: BLE001
        print(f"Configuración inválida: {exc}", file=sys.stderr); return 3
    if args.count:
        return run_headless(config, validator, args)
    app = QApplication(sys.argv[:1])
    window = MainWindow(config, validator, args); window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
