from __future__ import annotations

import ipaddress
import re
import sys
import time
from pathlib import Path

from PySide6.QtCore import QTimer, QUrl, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QQuaternion
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QTabWidget, QVBoxLayout, QWidget,
)
from serial.tools import list_ports

from core import SequenceTracker, TimeSeries, calibration_rejection_messages, relative_quaternion
from serial_worker import SerialWorker

ROOT = Path(__file__).resolve().parents[2]
FACES = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]


class PlotWidget(QWidget):
    COLORS = [QColor("#38bdf8"), QColor("#f97316"), QColor("#22c55e"), QColor("#e879f9")]

    def __init__(self, title: str, names: tuple[str, ...], parent=None) -> None:
        super().__init__(parent)
        self.title = title
        self.names = names
        self.series = TimeSeries(60.0)
        self.setMinimumHeight(145)

    def add(self, values: tuple[float, ...]) -> None:
        self.series.append(values)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#111827"))
        painter.setPen(QColor("#e5e7eb"))
        painter.drawText(10, 18, self.title)
        for index, name in enumerate(self.names):
            painter.setPen(self.COLORS[index % len(self.COLORS)])
            painter.drawText(10 + index * 75, 36, name)
        points = list(self.series.points)
        if len(points) < 2:
            return
        values = [value for _, row in points for value in row]
        low, high = min(values), max(values)
        if high - low < 1e-6:
            low -= 1.0
            high += 1.0
        left, top, right, bottom = 8, 43, self.width() - 8, self.height() - 8
        first, last = points[0][0], points[-1][0]
        span = max(last - first, 1.0)
        for series_index in range(len(self.names)):
            painter.setPen(QPen(self.COLORS[series_index % len(self.COLORS)], 1.5))
            previous = None
            for timestamp, row in points:
                x = left + (timestamp - first) / span * (right - left)
                y = bottom - (row[series_index] - low) / (high - low) * (bottom - top)
                if previous is not None:
                    painter.drawLine(previous[0], previous[1], int(x), int(y))
                previous = (int(x), int(y))


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("EIRODENET — Configuración y diagnóstico")
        self.resize(1280, 820)
        self.worker: SerialWorker | None = None
        self.request_id = 0
        self.pending: dict[int, str] = {}
        self.sequence = SequenceTracker()
        self.current_quaternion = (1.0, 0.0, 0.0, 0.0)
        self.zero_quaternion = self.current_quaternion
        self._build_ui()
        self.refresh_ports()

    def _build_ui(self) -> None:
        central = QWidget()
        outer = QVBoxLayout(central)
        connection = QHBoxLayout()
        self.port = QComboBox()
        refresh = QPushButton("Actualizar puertos")
        refresh.clicked.connect(self.refresh_ports)
        self.connect_button = QPushButton("Conectar")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.connection_status = QLabel("Desconectado")
        self.loss_status = QLabel("Tramas perdidas: 0")
        connection.addWidget(QLabel("Puerto:")); connection.addWidget(self.port)
        connection.addWidget(refresh); connection.addWidget(self.connect_button)
        connection.addWidget(self.connection_status, 1); connection.addWidget(self.loss_status)
        outer.addLayout(connection)

        tabs = QTabWidget()
        tabs.addTab(self._dashboard(), "Sensores")
        tabs.addTab(self._configuration(), "Configuración")
        tabs.addTab(self._calibration(), "Calibración IMU")
        outer.addWidget(tabs)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Seleccione el puerto USB de la placa")

    def _dashboard(self) -> QWidget:
        page = QWidget(); layout = QHBoxLayout(page)
        left = QWidget(); left_layout = QVBoxLayout(left)
        values = QGroupBox("Lecturas actuales"); grid = QGridLayout(values)
        self.value_labels: dict[str, QLabel] = {}
        names = ["Temperatura", "Aceleración", "Giroscopio", "Distancia", "Color",
                 "IR", "Wi-Fi", "Calibración"]
        for row, name in enumerate(names):
            label = QLabel("—"); label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            grid.addWidget(QLabel(name + ":"), row, 0); grid.addWidget(label, row, 1)
            self.value_labels[name] = label
        left_layout.addWidget(values)
        self.accel_plot = PlotWidget("Aceleración [g]", ("X", "Y", "Z"))
        self.gyro_plot = PlotWidget("Giroscopio [dps]", ("X", "Y", "Z"))
        self.distance_plot = PlotWidget("Distancia [mm]", ("mm",))
        self.color_plot = PlotWidget("Sensor de color [ADC]", ("Amb", "R", "G", "B"))
        for plot in (self.accel_plot, self.gyro_plot, self.distance_plot, self.color_plot):
            left_layout.addWidget(plot)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setWidget(left)
        layout.addWidget(scroll, 3)

        right = QVBoxLayout()
        self.viewer = QQuickWidget()
        self.viewer.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.viewer.rootContext().setContextProperty(
            "roverModelUrl", QUrl.fromLocalFile(str(ROOT / "tools" / "assets" / "Mini Rover.glb")))
        self.viewer.setSource(QUrl.fromLocalFile(str(Path(__file__).with_name("RoverView.qml"))))
        right.addWidget(self.viewer, 1)
        zero = QPushButton("Poner orientación a cero")
        zero.clicked.connect(self.zero_orientation)
        right.addWidget(zero)
        warning = QLabel("⚠ El IMU no tiene magnetómetro: el yaw puede derivar con el tiempo.")
        warning.setWordWrap(True); warning.setStyleSheet("color: #b45309; font-weight: 600")
        right.addWidget(warning)
        right_widget = QWidget(); right_widget.setLayout(right)
        layout.addWidget(right_widget, 2)
        return page

    def _configuration(self) -> QWidget:
        page = QWidget(); layout = QVBoxLayout(page)
        box = QGroupBox("Parámetros persistentes"); form = QFormLayout(box)
        self.rover_mac = QLineEdit(); self.rover_mac.setReadOnly(True)
        self.rover_mac.setPlaceholderText("Se obtiene al conectar la placa")
        self.who_am_i = QComboBox()
        self.who_am_i.addItem("Sin configurar", 0)
        self.who_am_i.addItem("Rover 10", 10)
        self.who_am_i.addItem("Rover 11", 11)
        self.ssid = QLineEdit(); self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.Password)
        password_row = QWidget(); password_layout = QHBoxLayout(password_row)
        password_layout.setContentsMargins(0, 0, 0, 0)
        password_layout.addWidget(self.password, 1)
        self.password_visibility = QPushButton("Mostrar")
        self.password_visibility.setCheckable(True)
        self.password_visibility.toggled.connect(self.toggle_password_visibility)
        password_layout.addWidget(self.password_visibility)
        self.server_ip = QLineEdit(); self.server_port = QLineEdit()
        self.peer_mac = QLineEdit()
        self.server_ip.setPlaceholderText("192.168.1.10")
        self.server_port.setPlaceholderText("5000")
        self.peer_mac.setPlaceholderText("AA:BB:CC:DD:EE:FF")
        form.addRow("MAC de este rover", self.rover_mac)
        form.addRow("WHO_AM_I", self.who_am_i)
        form.addRow("SSID", self.ssid); form.addRow("Contraseña", password_row)
        form.addRow("IPv4 del servidor", self.server_ip); form.addRow("Puerto", self.server_port)
        form.addRow("MAC del compañero", self.peer_mac)
        layout.addWidget(box)
        buttons = QHBoxLayout()
        load = QPushButton("Leer de la placa"); load.clicked.connect(lambda: self.send_command("config.get"))
        save = QPushButton("Guardar y aplicar"); save.clicked.connect(self.save_configuration)
        buttons.addWidget(load); buttons.addWidget(save); buttons.addStretch()
        layout.addLayout(buttons)
        motor_test = QPushButton("Probar ambos motores hacia delante (1 s)")
        motor_test.clicked.connect(lambda: self.send_command("motors.test_forward"))
        layout.addWidget(motor_test)
        motor_warning = QLabel("⚠ Mantenga el rover suspendido y las ruedas libres durante la prueba.")
        motor_warning.setWordWrap(True)
        motor_warning.setStyleSheet("color: #b45309; font-weight: 600")
        layout.addWidget(motor_warning)
        drive_box = QGroupBox("Calibración de motores para competencia")
        drive_layout = QVBoxLayout(drive_box)
        drive_help = QLabel(
            "Suspenda el rover con las ruedas libres. La prueba mueve ambos motores "
            "durante 0,4 s para medir el drift del giroscopio Z y guarda el trim en la memoria del rover.")
        drive_help.setWordWrap(True); drive_layout.addWidget(drive_help)
        drive_buttons = QHBoxLayout()
        self.drive_calibration_start = QPushButton("Calibrar drift")
        self.drive_calibration_start.clicked.connect(
            lambda: self.send_command("drive_calibration.start"))
        self.drive_calibration_stop = QPushButton("Detener")
        self.drive_calibration_stop.clicked.connect(
            lambda: self.send_command("drive_calibration.stop"))
        self.drive_calibration_stop.setEnabled(False)
        drive_buttons.addWidget(self.drive_calibration_start)
        drive_buttons.addWidget(self.drive_calibration_stop)
        drive_layout.addLayout(drive_buttons)
        self.drive_calibration_status = QLabel("Trim guardado: consultar configuración")
        drive_layout.addWidget(self.drive_calibration_status)
        drive_layout.addWidget(QLabel("La celda se toma de la calibración de visión (cell_mm)."))
        layout.addWidget(drive_box)
        self.config_status = QLabel("")
        layout.addWidget(self.config_status); layout.addStretch()
        return page

    def toggle_password_visibility(self, visible: bool) -> None:
        self.password.setEchoMode(QLineEdit.Normal if visible else QLineEdit.Password)
        self.password_visibility.setText("Ocultar" if visible else "Mostrar")

    def _calibration(self) -> QWidget:
        page = QWidget(); page_layout = QHBoxLayout(page)
        left = QWidget(); layout = QVBoxLayout(left)
        intro = QLabel(
            "Coloque el rover inmóvil con el eje indicado apuntando hacia arriba. "
            "Cada cara recopila 200 muestras durante aproximadamente 2 segundos.")
        intro.setWordWrap(True); layout.addWidget(intro)
        self.face_labels: list[QLabel] = []
        grid = QGridLayout()
        descriptions = ["X positivo", "X negativo", "Y positivo", "Y negativo", "Z positivo", "Z negativo"]
        for row, (face, description) in enumerate(zip(FACES, descriptions)):
            grid.addWidget(QLabel(f"{face} — {description}"), row, 0)
            state = QLabel("Pendiente"); self.face_labels.append(state); grid.addWidget(state, row, 1)
        layout.addLayout(grid)
        controls = QHBoxLayout()
        start = QPushButton("Iniciar"); start.clicked.connect(lambda: self.send_command("calibration.start"))
        self.face_combo = QComboBox(); self.face_combo.addItems(FACES)
        capture = QPushButton("Capturar cara"); capture.clicked.connect(self.capture_face)
        commit = QPushButton("Guardar calibración"); commit.clicked.connect(lambda: self.send_command("calibration.commit"))
        cancel = QPushButton("Cancelar"); cancel.clicked.connect(lambda: self.send_command("calibration.cancel"))
        controls.addWidget(start); controls.addWidget(self.face_combo); controls.addWidget(capture)
        controls.addWidget(commit); controls.addWidget(cancel); controls.addStretch()
        layout.addLayout(controls)
        self.calibration_status = QLabel("Calibración inactiva")
        self.calibration_status.setWordWrap(True); layout.addWidget(self.calibration_status); layout.addStretch()
        page_layout.addWidget(left, 2)

        guide_layout = QVBoxLayout()
        guide_title = QLabel("Ejemplo visual de la posición seleccionada")
        guide_title.setStyleSheet("font-size: 15px; font-weight: 600")
        guide_layout.addWidget(guide_title)
        self.calibration_view = QQuickWidget()
        self.calibration_view.setMinimumSize(480, 520)
        self.calibration_view.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.calibration_view.rootContext().setContextProperty(
            "roverModelUrl", QUrl.fromLocalFile(str(ROOT / "tools" / "assets" / "Mini Rover.glb")))
        self.calibration_view.setSource(
            QUrl.fromLocalFile(str(Path(__file__).with_name("CalibrationGuide.qml"))))
        guide_layout.addWidget(self.calibration_view, 1)
        replay = QPushButton("Repetir animación")
        replay.clicked.connect(self.restart_calibration_animation)
        guide_layout.addWidget(replay)
        guide = QWidget(); guide.setLayout(guide_layout)
        page_layout.addWidget(guide, 3)
        self.face_combo.currentIndexChanged.connect(self.update_calibration_guide)
        self.update_calibration_guide(self.face_combo.currentIndex())
        return page

    def update_calibration_guide(self, face_index: int) -> None:
        root = self.calibration_view.rootObject()
        if root is not None:
            root.setProperty("faceIndex", face_index)

    def restart_calibration_animation(self) -> None:
        root = self.calibration_view.rootObject()
        if root is not None:
            root.restartAnimation()

    def refresh_ports(self) -> None:
        selected = self.port.currentText()
        detected = list(list_ports.comports())
        detected.sort(key=lambda item: (item.vid != 0x1A86 or item.pid != 0x7523, item.device))
        ports = [item.device for item in detected]
        self.port.clear(); self.port.addItems(ports)
        if selected in ports: self.port.setCurrentText(selected)

    def toggle_connection(self) -> None:
        if self.worker is not None:
            self.worker.close(); self.worker.wait(1500); self.worker = None
            return
        if not self.port.currentText():
            QMessageBox.warning(self, "Puerto", "No hay un puerto serie seleccionado.")
            return
        self.worker = SerialWorker(self.port.currentText(), self)
        self.worker.message_received.connect(self.handle_message)
        self.worker.connection_changed.connect(self.connection_changed)
        self.worker.io_error.connect(self.show_error)
        self.worker.finished.connect(self.worker_finished)
        self.worker.start()

    def worker_finished(self) -> None:
        if self.worker is not None and not self.worker.isRunning():
            self.worker = None

    def connection_changed(self, connected: bool, port: str) -> None:
        self.connection_status.setText(f"Conectado a {port}" if connected else "Desconectado")
        self.connect_button.setText("Desconectar" if connected else "Conectar")
        if connected:
            QTimer.singleShot(150, lambda: self.send_command("device.info"))
            QTimer.singleShot(250, lambda: self.send_command("config.get"))
            QTimer.singleShot(350, lambda: self.send_command("stream.start"))
        elif self.worker is not None and not self.worker.isRunning():
            self.worker = None

    def send_command(self, command: str, **fields) -> None:
        if self.worker is None or not self.worker.isRunning():
            self.show_error("La placa no está conectada."); return
        self.request_id += 1
        message = {"v": 1, "id": self.request_id, "cmd": command, **fields}
        self.pending[self.request_id] = command
        self.worker.send(message)

    def save_configuration(self) -> None:
        server = self.server_ip.text().strip()
        port_text = self.server_port.text().strip()
        peer = self.peer_mac.text().strip().upper()
        try:
            if server: ipaddress.IPv4Address(server)
            port = int(port_text) if port_text else 0
            if (server and not 1 <= port <= 65535) or (not server and port != 0):
                raise ValueError("IPv4 y puerto deben configurarse juntos")
            if peer and not re.fullmatch(r"[0-9A-F]{2}(?::[0-9A-F]{2}){5}", peer):
                raise ValueError("La MAC debe usar el formato AA:BB:CC:DD:EE:FF")
        except ValueError as exc:
            QMessageBox.warning(self, "Configuración inválida", str(exc)); return
        self.send_command("config.set", data={
            "who_am_i": self.who_am_i.currentData(),
            "wifi_ssid": self.ssid.text(), "wifi_password": self.password.text(),
            "server_ipv4": server, "server_port": port, "peer_mac": peer})

    def capture_face(self) -> None:
        self.send_command("calibration.capture", face=self.face_combo.currentIndex())

    def handle_message(self, message: dict) -> None:
        if isinstance(message.get("seq"), int):
            self.sequence.observe(message["seq"])
            self.loss_status.setText(f"Tramas perdidas: {self.sequence.lost}")
        if message.get("type") == "response": self.handle_response(message)
        elif message.get("type") == "telemetry": self.handle_telemetry(message)
        elif message.get("type") == "event" and message.get("topic") == "calibration":
            self.handle_calibration(message)

    def handle_response(self, message: dict) -> None:
        request_id = int(message.get("id", 0)); command = self.pending.pop(request_id, "orden")
        if not message.get("ok"):
            error = message.get("error", {})
            self.show_error(f"{command}: {error.get('code', 'error')} — {error.get('message', '')}")
            return
        data = message.get("data", {})
        if command == "device.info":
            self.rover_mac.setText(data.get("rover_mac", ""))
        elif command == "config.get":
            rover_index = self.who_am_i.findData(data.get("who_am_i", 0))
            self.who_am_i.setCurrentIndex(max(rover_index, 0))
            self.ssid.setText(data.get("wifi_ssid", ""))
            self.password.setText(data.get("wifi_password", ""))
            self.server_ip.setText(data.get("server_ipv4", ""))
            port = data.get("server_port", 0); self.server_port.setText(str(port) if port else "")
            self.peer_mac.setText(data.get("peer_mac", ""))
            self.drive_calibration_status.setText(
                f"Trim guardado: {data.get('motor_trim_pwm', 0):+.0f} PWM" if
                data.get("drive_calibration_valid") else "Sin calibración de drift guardada")
            self.config_status.setText("Configuración leída correctamente")
        elif command == "config.set":
            text = "Configuración guardada"
            if data.get("wifi_reconnecting"): text += "; reconectando Wi-Fi…"
            self.config_status.setText(text)
        elif command == "motors.test_forward":
            duration = int(data.get("duration_ms", 1000)) / 1000
            self.config_status.setText(
                f"Prueba iniciada por {duration:g} s; los motores se detendrán automáticamente")
        elif command == "drive_calibration.start":
            self.drive_calibration_start.setEnabled(False)
            self.drive_calibration_stop.setEnabled(True)
            self.drive_calibration_status.setText("Midiendo drift; los motores se detendrán automáticamente…")
        elif command == "drive_calibration.stop":
            self.drive_calibration_stop.setEnabled(False)
        self.statusBar().showMessage(f"{command}: correcto", 3000)

    @staticmethod
    def _vector_text(values) -> str:
        return ", ".join(f"{float(value):.3f}" for value in values)

    def handle_telemetry(self, message: dict) -> None:
        topic = message.get("topic")
        if topic == "imu":
            if not message.get("valid"):
                self.value_labels["Aceleración"].setText(f"Error {message.get('error')}"); return
            accel = tuple(message.get("accel_g", (0, 0, 0)))
            gyro = tuple(message.get("gyro_dps", (0, 0, 0)))
            self.value_labels["Temperatura"].setText(f"{message.get('temperature_c', 0):.2f} °C")
            self.value_labels["Aceleración"].setText(self._vector_text(accel) + " g")
            self.value_labels["Giroscopio"].setText(self._vector_text(gyro) + " dps")
            self.value_labels["Calibración"].setText("Aplicada" if message.get("calibrated") else "Sin calibrar")
            self.accel_plot.add(accel); self.gyro_plot.add(gyro)
            q = tuple(float(value) for value in message.get("quaternion", (1, 0, 0, 0)))
            if len(q) == 4: self.current_quaternion = q; self.update_rover_rotation()
        elif topic == "sensors":
            ultrasonic = message.get("ultrasonic", {})
            if ultrasonic.get("valid"):
                distance = float(ultrasonic.get("distance_mm", 0))
                self.value_labels["Distancia"].setText(f"{distance:.0f} mm"); self.distance_plot.add((distance,))
            else: self.value_labels["Distancia"].setText(f"Error {ultrasonic.get('error')}")
            infrared = message.get("infrared", {})
            if infrared.get("valid"):
                positions = (("FL", "front_left"), ("FR", "front_right"),
                             ("RL", "rear_left"), ("RR", "rear_right"))
                self.value_labels["IR"].setText("  ".join(
                    f"{label}={int(infrared.get(key, 0))}" for label, key in positions))
            else: self.value_labels["IR"].setText(f"Error {infrared.get('error')}")
            color = message.get("color", {})
            if color.get("valid"):
                row = tuple(float(color.get(key, 0)) for key in ("ambient", "red", "green", "blue"))
                self.value_labels["Color"].setText(" / ".join(str(int(value)) for value in row)); self.color_plot.add(row)
            else: self.value_labels["Color"].setText(f"Error {color.get('error')}")
        elif topic == "status":
            competition = message.get("competition", {})
            trim = competition.get("stored_motor_trim_pwm", 0.0)
            calibrated = competition.get("drive_calibration_valid", False)
            cell_mm = competition.get("cell_mm", 0.0)
            running = competition.get("drive_calibration_running", False)
            self.drive_calibration_start.setEnabled(not running)
            self.drive_calibration_stop.setEnabled(running)
            self.drive_calibration_status.setText(
                (f"Trim guardado: {trim:+.0f} PWM" if calibrated else "Sin calibración de drift guardada") +
                (f" · celda de visión: {cell_mm:.1f} mm" if cell_mm > 0 else " · celda de visión: sin dato"))
            if message.get("wifi_connected"):
                self.value_labels["Wi-Fi"].setText(
                    f"{message.get('local_ipv4')}  RSSI {message.get('rssi')} dBm")
                if "reconectando" in self.config_status.text(): self.config_status.setText("Configuración guardada; Wi-Fi conectado")
            else: self.value_labels["Wi-Fi"].setText("Desconectado")
            if not message.get("wifi_reconnecting") and not message.get("wifi_connected") and \
                    "reconectando" in self.config_status.text():
                self.config_status.setText(
                    f"Configuración guardada; falló la conexión ({message.get('reconnect_error')})")

    def handle_calibration(self, message: dict) -> None:
        mask = int(message.get("captured_mask", 0)); phase = int(message.get("phase", 0))
        for index, label in enumerate(self.face_labels):
            label.setText("Aceptada ✓" if mask & (1 << index) else "Pendiente")
        active = int(message.get("active_face", -1))
        texts = {0: "Calibración inactiva", 1: "Lista para capturar", 2: "Capturando",
                 3: "Cara aceptada", 4: "Captura rechazada",
                 5: "Calibración guardada y aplicada"}
        text = texts.get(phase, "Estado desconocido")
        if phase == 2 and 0 <= active < 6:
            settling = int(message.get("settling_remaining", 0))
            text += (f" {FACES[active]} (estabilizando: {settling}/50)" if settling else
                     f" {FACES[active]} ({message.get('samples', 0)}/200)")
        if phase == 4:
            reasons = calibration_rejection_messages(int(message.get("rejection_mask", 0)))
            if reasons: text += ": " + "; ".join(reasons)
            mean = message.get("accel_mean", [])
            if len(mean) == 3: text += "\nMedia accel: " + self._vector_text(mean) + " g"
        self.calibration_status.setText(text)

    def zero_orientation(self) -> None:
        self.zero_quaternion = self.current_quaternion
        self.update_rover_rotation()

    def update_rover_rotation(self) -> None:
        root = self.viewer.rootObject()
        if root is None: return
        relative = relative_quaternion(self.current_quaternion, self.zero_quaternion)
        sensor = QQuaternion(relative[0], relative[1], relative[2], relative[3])
        basis = QQuaternion.fromAxisAndAngle(1.0, 0.0, 0.0, -90.0)
        root.setProperty("roverRotation", basis * sensor * basis.conjugated())

    def show_error(self, text: str) -> None:
        self.statusBar().showMessage(text, 8000)

    def closeEvent(self, event) -> None:  # noqa: N802
        if self.worker is not None:
            self.worker.close(); self.worker.wait(1500)
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow(); window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
