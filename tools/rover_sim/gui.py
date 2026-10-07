import math
import sys
from PySide6.QtCore import Qt, QPointF, QRectF, QTimer
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QApplication, QDoubleSpinBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QVBoxLayout, QWidget
from .layout import apply_layout
from .runner import Simulation
from .world import MARKERS

PHASES = ('Espera', 'Hacia cubo', 'Giro desvío', 'Avance desvío', 'Alinear depósito', 'Hacia depósito', 'Empuje', 'Terminado', 'Espera segura', 'Pausa por atasco', 'Espera visión', 'Retirada', 'Aproximación directa al cubo', 'Recolocar para reintento', 'Orientar parte trasera al centro', 'Retroceder hacia el centro', 'Esperar respuesta del depósito')
PALETTE = {'green':'#19845b','blue':'#2478d4','red':'#d75151','obstacle':'#e1b832'}


class Arena(QWidget):
    def __init__(self, window):
        super().__init__()
        self.window = window
        self.setMinimumSize(600,600)
        self.setAccessibleName('Pista de un metro con rovers, brazos, cubos y rutas')

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if not getattr(self.window, 'sim', None):
            p.fillRect(self.rect(), QColor('#f4f5f6'))
            p.setPen(QColor('#637080'))
            p.drawText(self.rect(), Qt.AlignCenter,
                       'Preparando simulación')
            p.end()
            return
        w = self.window.sim.world
        size = min(self.width(),self.height())-32
        p.translate((self.width()-size)/2,(self.height()-size)/2)
        p.scale(size/1000,size/1000)
        p.fillRect(QRectF(0,0,1000,1000),QColor('#f4f5f6'))
        p.setPen(Qt.NoPen)
        for row in range(50):
            for col in range(50):
                if (row+col)%2 == 0:
                    p.fillRect(QRectF(col*20,row*20,20,20),QColor('#e1e4e7'))
        for mx,my,bits in MARKERS:
            p.fillRect(QRectF(0 if mx==20 else 860,0 if my==20 else 860,140,140),QColor('#f4f5f6'))
            for row,line in enumerate(bits):
                for col,value in enumerate(line):
                    if value=='1':p.fillRect(QRectF(mx+col*100/6,my+row*100/6,100/6,100/6),QColor('#637080'))
        p.setPen(QPen(QColor('#63758c'),2,Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(*w.origin,w.grid['cols']*w.grid['cell_mm'],w.grid['rows']*w.grid['cell_mm']))
        p.drawEllipse(QPointF(*w.start),12,12)
        p.drawText(QRectF(w.start[0]-35,w.start[1]+15,90,25),'Salida')
        for color,(x,y) in zip(PALETTE,w.config['depots']):
            dx,dy = w.depot_extent(x,y)
            fill = QColor(PALETTE[color]);fill.setAlpha(70)
            p.fillRect(QRectF(x-dx/2,y-dy/2,dx,dy),fill)
        for status in self.window.sim.status:
            if status['phase'] not in (0,7) and status['target_color'] < 3:
                x,y = w.from_grid(status['target_col'],status['target_row'])
                p.setPen(QPen(QColor('#63758c'),2,Qt.DashLine))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QPointF(x,y),6,6)
        for s,label in w.labels.items():
            color = PALETTE.get(label,'#45526b' if label.startswith('rover-10') else '#97713a')
            p.setPen(QPen(QColor(color),1))
            p.setBrush(QColor(color))
            p.drawPolygon(QPolygonF([QPointF(x,y) for x,y in w.polygon(s)]))
        for i,b in enumerate(w.rovers):
            x,y,angle = w.pose(b)
            p.setPen(QPen(QColor('#ffffff'),3))
            p.drawLine(QPointF(x,y),QPointF(x+35*math.cos(b.angle),y-35*math.sin(b.angle)))
            p.drawText(QRectF(x-30,y+10,60,25),Qt.AlignCenter,str(i+10))
        p.setPen(Qt.NoPen);p.setBrush(QColor('#ee4b8d'))
        for contact in w.contacts:
            for x,y in contact['points']:p.drawEllipse(QPointF(x,y),3,3)
        p.end()


class Window(QMainWindow):
    def __init__(self, config, output, exe):
        super().__init__()
        self.config,self.output,self.exe = config,output,exe
        self.run_number = 0
        self.sim = Simulation(config,output/'run-000',exe)
        self.setWindowTitle(f'EIRODENET · Simulación del firmware · {self.sim.world.grid["cols"]} × {self.sim.world.grid["rows"]}')
        self.resize(900,850)
        root = QWidget();layout = QVBoxLayout(root)
        title = QLabel('Dos rovers · brazos rígidos 3 × 55 mm · hueco 94 mm')
        layout.addWidget(title)
        controls = QHBoxLayout()
        saved = config.get('challenge_layout',{})
        self.seed_input = QLineEdit(str(saved.get('seed',config['seed'])))
        self.seed_input.setAccessibleName('Seed de Vision Rover Challenge')
        self.seed_input.setMaximumWidth(130)
        self.difficulty_input = QDoubleSpinBox()
        self.difficulty_input.setRange(0,1);self.difficulty_input.setSingleStep(.01)
        self.difficulty_input.setValue(saved.get('difficulty',.5))
        controls.addWidget(QLabel('Seed:'));controls.addWidget(self.seed_input)
        controls.addWidget(QLabel('Dificultad:'));controls.addWidget(self.difficulty_input)
        generate = QPushButton('Aplicar posiciones');generate.clicked.connect(self.generate_layout)
        controls.addWidget(generate);controls.addStretch()
        layout.addLayout(controls)
        self.arena = Arena(self);layout.addWidget(self.arena,1)
        self.status_label = QLabel();self.status_label.setWordWrap(True);layout.addWidget(self.status_label)
        row = QHBoxLayout()
        for text,callback in [('Iniciar',self.start),('Pausar',self.pause),('Un paso (10 ms)',self.single_step),('Reiniciar',self.reset)]:
            button = QPushButton(text);button.clicked.connect(callback);row.addWidget(button)
        layout.addLayout(row)
        layout.addWidget(QLabel('Física estimada · inicio de competencia verificado y sensores calibrados · contactos en rosa'))
        self.setCentralWidget(root)
        self.timer = QTimer(self);self.timer.timeout.connect(self.step)
        self.refresh()

    def refresh(self):
        texts = [f'Rover {i+10}: {PHASES[s["phase"]]} · PWM {s["left"]}/{s["right"]}' for i,s in enumerate(self.sim.status)]
        self.status_label.setText(f'{self.sim.world.time_ms/1000:.2f} s  |  Entregas físicas: {sum(self.sim.world.delivered())}/3\n'+'   |   '.join(texts))
        self.arena.update()

    def start(self): self.timer.start(10)
    def pause(self): self.timer.stop()
    def single_step(self):
        self.pause();self.step()

    def step(self):
        if self.sim.closed:
            self.pause()
            return
        try:
            self.sim.step();self.refresh()
        except Exception as exc:
            self.pause();self.sim.close()
            QMessageBox.critical(self,'Simulación detenida',str(exc))

    def reset(self):
        self.pause();self.sim.close();self.run_number += 1
        self.sim = Simulation(self.config,self.output/f'run-{self.run_number:03d}',self.exe)
        self.refresh()

    def generate_layout(self):
        import copy
        try:
            config = apply_layout(copy.deepcopy(self.config),int(self.seed_input.text()),self.difficulty_input.value())
        except ValueError as exc:
            QMessageBox.warning(self,'Seed inválida',str(exc))
            return
        self.config = config
        self.reset()

    def closeEvent(self,event):
        self.pause();self.sim.close();event.accept()


def run(config,output,exe):
    app = QApplication.instance() or QApplication(sys.argv)
    window = Window(config,output,exe);window.show()
    app.exec()
