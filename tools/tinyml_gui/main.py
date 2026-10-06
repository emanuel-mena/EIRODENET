"""GUI de escritorio para entrenar y desplegar políticas TinyML."""

from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QSpinBox,
    QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

TOOLS = Path(__file__).resolve().parents[1]
ROOT = TOOLS.parent
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from rover_sim.build import build
from rover_sim.gui import Arena
from rover_sim.layout import apply_seeded_entropy
from rover_sim.runner import Simulation
from rover_sim.world import SCENARIOS, scenario


PHASES = ("Tránsito", "Alineación", "Captura", "Empuje", "Retirada", "Cesión")
TABLE_COLUMNS = (
    "#", "Candidato", "Gen.", "Entregas", "Elegible", "Tiempo", "Fitness",
    "Progreso", "Seguridad", "SHA-256",
)


class Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("EIRODENET · Laboratorio TinyML")
        self.resize(1450, 900)
        self.training_process: QProcess | None = None
        self.active_training_output: Path | None = None
        self.flash_process: QProcess | None = None
        self.flash_stage = ""
        self.flash_image: Path | None = None
        self.flash_port = ""
        self.stdout_buffer = ""
        self.sim: Simulation | None = None
        self.sim_model: Path | None = None
        self.sim_run = 0
        self.closing_after_training = False
        self.candidates: list[dict] = []
        self.sim_timer = QTimer(self)
        self.sim_timer.setInterval(16)
        self.sim_timer.timeout.connect(self.simulation_tick)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._training_panel())
        splitter.addWidget(self._simulation_panel())
        splitter.setSizes([650, 800])
        self.setCentralWidget(splitter)
        self.refresh_ports()
        self.load_leaderboard(silent=True)

    def _training_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        title = QLabel("Entrenamiento evolutivo")
        title.setStyleSheet("font-size: 18px; font-weight: 600")
        layout.addWidget(title)
        description = QLabel(
            "Cada candidato se cuantiza a int8 y se puntúa con los servicios C++ reales, "
            "entropía de visión y aleatorización física reproducible.")
        description.setWordWrap(True)
        layout.addWidget(description)

        form = QFormLayout()
        default_run = (ROOT / ".pio" / "tinyml-training" /
                       datetime.now().strftime("run-%Y%m%d-%H%M%S-%f"))
        destination_row = QHBoxLayout()
        self.output_edit = QLineEdit(str(default_run))
        self.output_edit.editingFinished.connect(lambda: self.load_leaderboard(silent=True))
        browse = QPushButton("Elegir…")
        browse.clicked.connect(self.choose_output)
        destination_row.addWidget(self.output_edit, 1)
        destination_row.addWidget(browse)
        form.addRow("Carpeta de sesión", destination_row)

        self.scenario_combo = QComboBox()
        self.scenario_combo.addItems(SCENARIOS)
        form.addRow("Escenario", self.scenario_combo)
        self.seed_spin = QSpinBox()
        self.seed_spin.setRange(0, 2_147_483_647)
        self.seed_spin.setValue(20_261_006)
        form.addRow("Seed evolución", self.seed_spin)
        self.generations_spin = QSpinBox()
        self.generations_spin.setRange(1, 10_000)
        self.generations_spin.setValue(8)
        form.addRow("Generaciones", self.generations_spin)
        self.population_spin = QSpinBox()
        self.population_spin.setRange(2, 256)
        self.population_spin.setValue(8)
        form.addRow("Candidatos/generación", self.population_spin)
        self.episodes_spin = QSpinBox()
        self.episodes_spin.setRange(1, 16)
        self.episodes_spin.setValue(2)
        self.training_layout_seeds = json.loads(
            (TOOLS / "rover_sim" / "tinyml_splits.json").read_text(encoding="utf-8"))["training"]
        self.layout_seeds_label = QLabel()
        self.layout_seeds_label.setWordWrap(True)
        self.episodes_spin.valueChanged.connect(self.refresh_layout_seeds)
        form.addRow("Episodios/candidato", self.episodes_spin)
        form.addRow("Seeds de posiciones", self.layout_seeds_label)
        self.refresh_layout_seeds()
        self.seconds_spin = QDoubleSpinBox()
        self.seconds_spin.setRange(1, 180)
        self.seconds_spin.setValue(45)
        self.seconds_spin.setSuffix(" s")
        form.addRow("Límite por episodio", self.seconds_spin)
        self.sigma_spin = QDoubleSpinBox()
        self.sigma_spin.setDecimals(4)
        self.sigma_spin.setRange(.0001, 1)
        self.sigma_spin.setSingleStep(.005)
        self.sigma_spin.setValue(.035)
        form.addRow("Mutación σ", self.sigma_spin)
        layout.addLayout(form)

        buttons = QHBoxLayout()
        self.train_button = QPushButton("Iniciar entrenamiento")
        self.train_button.clicked.connect(self.start_training)
        self.stop_button = QPushButton("Detener al terminar el episodio")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_training)
        reload_button = QPushButton("Recargar top 10")
        reload_button.clicked.connect(self.load_leaderboard)
        buttons.addWidget(self.train_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(reload_button)
        layout.addLayout(buttons)
        self.training_progress = QProgressBar()
        self.training_progress.setRange(0, 1)
        self.training_progress.setValue(0)
        layout.addWidget(self.training_progress)
        self.training_status = QLabel("Listo")
        self.training_status.setWordWrap(True)
        layout.addWidget(self.training_status)

        layout.addWidget(QLabel("Diez mejores candidatos"))
        self.table = QTableWidget(0, len(TABLE_COLUMNS))
        self.table.setHorizontalHeaderLabels(TABLE_COLUMNS)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.itemSelectionChanged.connect(self.candidate_selected)
        self.table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.table, 1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)
        self.log.setMaximumHeight(150)
        layout.addWidget(self.log)
        return panel

    def refresh_layout_seeds(self) -> None:
        selected = self.training_layout_seeds[:self.episodes_spin.value()]
        self.layout_seeds_label.setText(", ".join(str(seed) for seed in selected))

    def _simulation_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        title_row = QHBoxLayout()
        title = QLabel("Simulación del candidato seleccionado")
        title.setStyleSheet("font-size: 18px; font-weight: 600")
        self.selected_label = QLabel("Ningún modelo seleccionado")
        self.selected_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(self.selected_label)
        layout.addLayout(title_row)

        options = QHBoxLayout()
        self.sim_scenario_combo = QComboBox()
        self.sim_scenario_combo.addItems(SCENARIOS)
        self.sim_seed_spin = QSpinBox()
        self.sim_seed_spin.setRange(0, 2_147_483_647)
        self.sim_seed_spin.setValue(20_261_006)
        self.sim_difficulty_spin = QDoubleSpinBox()
        self.sim_difficulty_spin.setRange(0, 1)
        self.sim_difficulty_spin.setSingleStep(.05)
        self.sim_difficulty_spin.setValue(.5)
        self.randomize_check = QCheckBox("Aleatorización de dominio")
        self.randomize_check.setChecked(True)
        self.steps_spin = QSpinBox()
        self.steps_spin.setRange(1, 20)
        self.steps_spin.setValue(2)
        options.addWidget(QLabel("Escenario"))
        options.addWidget(self.sim_scenario_combo)
        options.addWidget(QLabel("Seed"))
        options.addWidget(self.sim_seed_spin)
        options.addWidget(QLabel("Dificultad"))
        options.addWidget(self.sim_difficulty_spin)
        options.addWidget(self.randomize_check)
        options.addWidget(QLabel("Pasos/refresco"))
        options.addWidget(self.steps_spin)
        layout.addLayout(options)

        self.arena = Arena(self)
        layout.addWidget(self.arena, 1)
        self.sim_status = QLabel("Seleccione un candidato y pulse Cargar.")
        self.sim_status.setWordWrap(True)
        layout.addWidget(self.sim_status)
        controls = QHBoxLayout()
        self.load_sim_button = QPushButton("Cargar candidato")
        self.load_sim_button.clicked.connect(self.load_simulation)
        start = QPushButton("Iniciar")
        start.clicked.connect(lambda: self.sim_timer.start())
        pause = QPushButton("Pausar")
        pause.clicked.connect(self.sim_timer.stop)
        step = QPushButton("Un paso")
        step.clicked.connect(self.single_simulation_step)
        reset = QPushButton("Reiniciar")
        reset.clicked.connect(self.load_simulation)
        for button in (self.load_sim_button, start, pause, step, reset):
            controls.addWidget(button)
        layout.addLayout(controls)

        flash_box = QGroupBox("Flashear el candidato seleccionado")
        flash_layout = QVBoxLayout(flash_box)
        flash_row = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        refresh = QPushButton("Actualizar puertos")
        refresh.clicked.connect(self.refresh_ports)
        self.flash_button = QPushButton("Construir EIRM y flashear")
        self.flash_button.clicked.connect(self.start_flash)
        flash_row.addWidget(QLabel("Puerto"))
        flash_row.addWidget(self.port_combo, 1)
        flash_row.addWidget(refresh)
        flash_row.addWidget(self.flash_button)
        flash_layout.addLayout(flash_row)
        flash_layout.addWidget(QLabel(
            "Se valida TFL3, tamaño y CRC; sólo se escribe la partición model en 0x600000."))
        self.flash_log = QPlainTextEdit()
        self.flash_log.setReadOnly(True)
        self.flash_log.setMaximumBlockCount(500)
        self.flash_log.setMaximumHeight(105)
        flash_layout.addWidget(self.flash_log)
        layout.addWidget(flash_box)
        return panel

    def choose_output(self) -> None:
        directory = QFileDialog.getExistingDirectory(self, "Carpeta de entrenamiento",
                                                     self.output_edit.text())
        if directory:
            self.output_edit.setText(directory)
            self.load_leaderboard(silent=True)

    def selected_candidate(self) -> dict | None:
        row = self.table.currentRow()
        if row < 0:
            return None
        item = self.table.item(row, 1)
        return item.data(Qt.UserRole) if item else None

    def candidate_selected(self) -> None:
        candidate = self.selected_candidate()
        if not candidate:
            self.selected_label.setText("Ningún modelo seleccionado")
            return
        self.selected_label.setText(
            f'{candidate["id"]} · {candidate["sha256"][:12]} · {candidate["deliveries"]}/3 entregas')

    def update_table(self, candidates: list[dict]) -> None:
        selected = (self.selected_candidate() or {}).get("sha256")
        self.candidates = candidates[:10]
        self.table.setRowCount(len(self.candidates))
        for row, candidate in enumerate(self.candidates):
            completion = candidate.get("completion_ms")
            values = (
                str(row + 1), candidate.get("id", ""), str(candidate.get("generation", "")),
                f'{candidate.get("deliveries", 0)}/3', "Sí" if candidate.get("eligible") else "No",
                f"{completion / 1000:.2f} s" if completion is not None else "—",
                f'{candidate.get("fitness", 0):.1f}', f'{100 * candidate.get("progress", 0):.1f}% ',
                str(candidate.get("unsafe_events", 0)), candidate.get("sha256", "")[:16],
            )
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                if column == 1:
                    item.setData(Qt.UserRole, candidate)
                if column in (0, 2, 3, 4, 5, 6, 7, 8):
                    item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(row, column, item)
            if candidate.get("sha256") == selected:
                self.table.selectRow(row)
        self.table.resizeColumnsToContents()
        if self.candidates and self.table.currentRow() < 0:
            self.table.selectRow(0)

    def load_leaderboard(self, checked: bool = False, silent: bool = False) -> None:
        del checked
        path = Path(self.output_edit.text()).expanduser() / "leaderboard.json"
        if not path.is_file():
            if not silent:
                QMessageBox.information(self, "Top 10", "La carpeta aún no contiene leaderboard.json.")
            return
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self.update_table(data.get("candidates", []))
            self.training_status.setText(
                f'Estado guardado: {data.get("state", "desconocido")} · {data.get("updated_at", "")}')
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self, "Top 10 inválido", str(exc))

    def start_training(self) -> None:
        if self.training_process and self.training_process.state() != QProcess.NotRunning:
            return
        output = Path(self.output_edit.text()).expanduser().resolve()
        if (output / "leaderboard.json").exists():
            output = output.parent / datetime.now().strftime("run-%Y%m%d-%H%M%S-%f")
            self.output_edit.setText(str(output))
        output.mkdir(parents=True, exist_ok=True)
        (output / ".stop").unlink(missing_ok=True)
        arguments = [
            "-u", str(TOOLS / "tinyml_trainer.py"), "--output", str(output),
            "--scenario", self.scenario_combo.currentText(), "--seed", str(self.seed_spin.value()),
            "--generations", str(self.generations_spin.value()),
            "--population", str(self.population_spin.value()),
            "--episodes", str(self.episodes_spin.value()),
            "--seconds", str(self.seconds_spin.value()),
            "--mutation-sigma", str(self.sigma_spin.value()),
        ]
        process = QProcess(self)
        process.setWorkingDirectory(str(ROOT))
        process.setProgram(sys.executable)
        process.setArguments(arguments)
        process.readyReadStandardOutput.connect(self.read_training_stdout)
        process.readyReadStandardError.connect(self.read_training_stderr)
        process.finished.connect(self.training_finished)
        self.training_process = process
        self.active_training_output = output
        self.stdout_buffer = ""
        total = self.generations_spin.value() * self.population_spin.value()
        self.training_progress.setRange(0, total)
        self.training_progress.setValue(0)
        self.train_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.training_status.setText("Iniciando proceso de entrenamiento…")
        self.log.appendPlainText(f"$ {sys.executable} {' '.join(arguments)}")
        process.start()

    def stop_training(self) -> None:
        output = self.active_training_output or Path(self.output_edit.text()).expanduser()
        output.mkdir(parents=True, exist_ok=True)
        (output / ".stop").write_text("stop\n", encoding="ascii")
        self.stop_button.setEnabled(False)
        self.training_status.setText("Detención solicitada; cerrando el episodio actual…")

    def read_training_stdout(self) -> None:
        if not self.training_process:
            return
        self.stdout_buffer += bytes(self.training_process.readAllStandardOutput()).decode("utf-8", "replace")
        while "\n" in self.stdout_buffer:
            line, self.stdout_buffer = self.stdout_buffer.split("\n", 1)
            if not line.strip():
                continue
            try:
                self.handle_training_event(json.loads(line))
            except (ValueError, TypeError):
                self.log.appendPlainText(line)

    def read_training_stderr(self) -> None:
        if self.training_process:
            text = bytes(self.training_process.readAllStandardError()).decode("utf-8", "replace")
            if text:
                self.log.appendPlainText(text.rstrip())

    def handle_training_event(self, event: dict) -> None:
        kind = event.get("event")
        if kind == "progress":
            self.training_status.setText(event.get("message", "Entrenando…"))
            self.training_progress.setValue(int(event.get("completed", self.training_progress.value())))
        elif kind == "leaderboard":
            self.update_table(event.get("candidates", []))
            self.training_progress.setValue(int(event.get("completed", 0)))
        elif kind == "candidate":
            candidate = event.get("candidate", {})
            self.log.appendPlainText(
                f'{candidate.get("id")}: fitness {candidate.get("fitness")}, '
                f'entregas {candidate.get("deliveries")}/3, top10={event.get("retained")}')
        elif kind in ("finished", "stopped"):
            self.training_status.setText(
                "Entrenamiento terminado." if kind == "finished" else "Entrenamiento detenido limpiamente.")
            self.load_leaderboard(silent=True)
        elif kind == "error":
            self.training_status.setText(f'Error: {event.get("message", "desconocido")}')
            self.log.appendPlainText(self.training_status.text())
        elif kind == "started":
            self.training_status.setText("Entrenador activo; preparando el primer candidato…")

    def training_finished(self, exit_code: int, _status: QProcess.ExitStatus) -> None:
        self.train_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        if exit_code and not self.training_status.text().startswith("Error"):
            self.training_status.setText(f"El entrenador terminó con código {exit_code}.")
        self.load_leaderboard(silent=True)
        self.training_process = None
        self.active_training_output = None
        if self.closing_after_training:
            self.close()

    def _make_simulation(self, model: Path) -> Simulation:
        config = scenario(self.sim_scenario_combo.currentText())
        seed = self.sim_seed_spin.value()
        if self.randomize_check.isChecked():
            apply_seeded_entropy(config, seed, self.sim_difficulty_spin.value())
        else:
            from rover_sim.layout import apply_layout
            apply_layout(config, seed, self.sim_difficulty_spin.value())
        self.sim_run += 1
        output = (Path(self.output_edit.text()).expanduser() / "visualizations" /
                  f'{datetime.now().strftime("%Y%m%d-%H%M%S-%f")}-{self.sim_run:03d}')
        return Simulation(config, output, build(), model=model)

    def load_simulation(self) -> None:
        candidate = self.selected_candidate()
        if not candidate:
            QMessageBox.information(self, "Simulación", "Seleccione un candidato del top 10.")
            return
        model = Path(candidate["model"])
        if not model.is_file():
            QMessageBox.warning(self, "Modelo ausente", f"No existe {model}")
            return
        self.sim_timer.stop()
        if self.sim:
            self.sim.close()
            self.sim = None
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.sim_model = model
            self.sim = self._make_simulation(model)
            self.refresh_simulation()
        except Exception as exc:
            QMessageBox.critical(self, "No se pudo cargar el simulador", str(exc))
        finally:
            QApplication.restoreOverrideCursor()

    def refresh_simulation(self) -> None:
        if not self.sim:
            return
        statuses = []
        for index, status in enumerate(self.sim.status):
            phase = int(status.get("phase", 0))
            phase_name = PHASES[phase] if 0 <= phase < len(PHASES) else str(phase)
            statuses.append(
                f'Rover {index + 10}: {phase_name}, PWM {status.get("left", 0)}/{status.get("right", 0)}')
        metrics = self.sim.metrics
        self.sim_status.setText(
            f'{self.sim.world.time_ms / 1000:.2f} s · entregas {sum(self.sim.world.delivered())}/3 · '
            f'movimiento tándem {metrics["simultaneous_motion_ms"] / 1000:.2f} s · '
            f'cesiones efectivas {metrics["effective_yields"]}\n' + "  |  ".join(statuses))
        self.arena.update()

    def simulation_tick(self) -> None:
        if not self.sim or self.sim.closed:
            self.sim_timer.stop()
            return
        try:
            for _ in range(self.steps_spin.value()):
                self.sim.step()
            self.refresh_simulation()
        except Exception as exc:
            self.sim_timer.stop()
            self.sim.close()
            QMessageBox.critical(self, "Simulación detenida", str(exc))

    def single_simulation_step(self) -> None:
        self.sim_timer.stop()
        if self.sim:
            try:
                self.sim.step()
                self.refresh_simulation()
            except Exception as exc:
                self.sim.close()
                QMessageBox.critical(self, "Simulación detenida", str(exc))

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText() if hasattr(self, "port_combo") else ""
        if not hasattr(self, "port_combo"):
            return
        try:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
        except ImportError:
            ports = []
            self.flash_log.appendPlainText("pyserial no está instalado; puede escribir el puerto manualmente.")
        self.port_combo.clear()
        for port in ports:
            self.port_combo.addItem(f"{port.device} · {port.description}", port.device)
        if current and not ports:
            self.port_combo.setEditText(current.split(" · ", 1)[0])

    def _port(self) -> str:
        text = self.port_combo.currentText().strip()
        data = self.port_combo.currentData()
        if data and " · " in text:
            return str(data)
        return text.split(" · ", 1)[0]

    def start_flash(self) -> None:
        if self.flash_process and self.flash_process.state() != QProcess.NotRunning:
            return
        candidate = self.selected_candidate()
        port = self._port()
        if not candidate or not Path(candidate["model"]).is_file():
            QMessageBox.information(self, "Flasheo", "Seleccione un candidato disponible.")
            return
        if not port:
            QMessageBox.information(self, "Flasheo", "Seleccione o escriba el puerto serial.")
            return
        self.flash_port = port
        model = Path(candidate["model"])
        self.flash_image = ROOT / ".pio" / "build" / f'model-{candidate["sha256"][:12]}.bin'
        self.flash_stage = "build"
        self.flash_log.clear()
        self.flash_button.setEnabled(False)
        self._run_flash_process([
            str(TOOLS / "model_tool.py"), "build", str(model), str(self.flash_image),
            "--version", "1",
        ])

    def _run_flash_process(self, arguments: list[str]) -> None:
        process = QProcess(self)
        process.setWorkingDirectory(str(ROOT))
        process.setProgram(sys.executable)
        process.setArguments(arguments)
        process.readyReadStandardOutput.connect(self.read_flash_output)
        process.readyReadStandardError.connect(self.read_flash_output)
        process.finished.connect(self.flash_finished)
        self.flash_process = process
        self.flash_log.appendPlainText(f"$ {sys.executable} {' '.join(arguments)}")
        process.start()

    def read_flash_output(self) -> None:
        if not self.flash_process:
            return
        out = bytes(self.flash_process.readAllStandardOutput()).decode("utf-8", "replace")
        err = bytes(self.flash_process.readAllStandardError()).decode("utf-8", "replace")
        if out or err:
            self.flash_log.appendPlainText((out + err).rstrip())

    def flash_finished(self, exit_code: int, _status: QProcess.ExitStatus) -> None:
        stage = self.flash_stage
        self.flash_process = None
        if exit_code:
            self.flash_log.appendPlainText(f"Error en {stage}: código {exit_code}.")
            self.flash_button.setEnabled(True)
            return
        if stage == "build" and self.flash_image:
            self.flash_stage = "flash"
            self._run_flash_process([
                str(TOOLS / "model_tool.py"), "flash", str(self.flash_image),
                "--port", self.flash_port,
            ])
            return
        self.flash_log.appendPlainText(
            "Modelo flasheado. Reinicie la placa y verifique versión, longitud y CRC a 115200 baudios.")
        self.flash_button.setEnabled(True)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.flash_process and self.flash_process.state() != QProcess.NotRunning:
            QMessageBox.warning(self, "Flasheo activo", "Espere a que termine el flasheo antes de cerrar.")
            event.ignore()
            return
        if self.training_process and self.training_process.state() != QProcess.NotRunning:
            self.closing_after_training = True
            self.stop_training()
            self.hide()
            event.ignore()
            return
        self.sim_timer.stop()
        if self.sim:
            self.sim.close()
        event.accept()


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("EIRODENET TinyML")
    window = Window()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
