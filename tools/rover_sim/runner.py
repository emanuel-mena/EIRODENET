import copy
import hashlib
import json
import os
import platform
from pathlib import Path
import queue
import subprocess
import threading
from importlib.metadata import version

from vision_client.core import ContractValidator, VisionConfig
from .build import ROOT, build
from .world import World


class Controller:
    def __init__(self, exe, identity, log):
        self.stderr = log.open('w', encoding='utf-8')
        env = dict(os.environ)
        if os.name == 'nt':
            env['PATH'] = str(Path(os.environ.get('CXX', 'C:/msys64/ucrt64/bin/g++.exe')).parent) + os.pathsep + env['PATH']
        self.p = subprocess.Popen([str(exe),str(identity)], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=self.stderr,
                                  text=True, encoding='utf-8', env=env,
                                  creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
        self.lines = queue.Queue()
        def read():
            for line in self.p.stdout:
                self.lines.put(line)
            self.lines.put(None)
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()

    def step(self, data):
        self.p.stdin.write(json.dumps(data,separators=(',',':'))+'\n')
        self.p.stdin.flush()
        try:
            line = self.lines.get(timeout=5)
        except queue.Empty as exc:
            raise RuntimeError('El controlador no respondió en 5 segundos.') from exc
        if line is None:
            raise RuntimeError(f'El controlador terminó: {self.p.poll()}; consulte el registro C++.')
        response = json.loads(line)
        if response['step'] != data['step']:
            raise RuntimeError('Respuesta de controlador fuera de secuencia.')
        return response

    def close(self):
        if self.p.poll() is None:
            self.p.stdin.close()
            try:
                self.p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.p.kill()
                self.p.wait(timeout=2)
        self.reader.join(timeout=2)
        self.p.stdout.close()
        self.stderr.close()


class Simulation:
    def __init__(self, config, output, exe=None):
        self.config = copy.deepcopy(config)
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.world = World(self.config)
        vc = VisionConfig.from_environment()
        self.validator = ContractValidator(vc.vision_system)
        if self.validator.version != 3:
            raise ValueError('El simulador requiere el contrato de visión v3.')
        exe = exe or build()
        sources = [ROOT/'src/services'/f'{n}.cpp' for n in ('navigation_service','competition_runtime','grid_planner','motion_control','vision_contract')]
        sources += [vc.vision_system/'contrato/schema.py', ROOT/'tools/vision_client/core.py', exe]
        if self.config.get('layout_source'):
            sources.append(Path(self.config['layout_source']))
        sources += sorted((ROOT/'include').glob('*.hpp'))
        sources += sorted(p for p in (ROOT/'tools/rover_sim').rglob('*')
                          if p.suffix in ('.py','.hpp','.h','.cpp'))
        manifest = dict(scenario=self.config, sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                        runtime=dict(python=platform.python_version(),system=platform.platform(),pymunk=version('pymunk'),pyside6=version('PySide6')),
                        assumptions='Physical parameters estimated; calibrated sensors; verified competition startup; no radio or camera emulation.')
        (self.output/'scenario.json').write_text(json.dumps(self.config,indent=2),encoding='utf-8')
        (self.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        self.controllers = []
        try:
            for identity in (10,11):
                self.controllers.append(Controller(exe,identity,self.output/f'rover-{identity}.log'))
        except Exception:
            self.close()
            raise
        self.trace = (self.output/'trace.ndjson').open('w',encoding='utf-8')
        self.step_id = 0
        self.vision_queue = []
        self.peer_queue = [[],[]]
        self.status = [dict(available=True,delivered=0,ack={},messages=[],phase=0,left=0,right=0,route=[]) for _ in range(2)]
        self.acks = [{},{}]
        self.events = []
        self.previous = None
        self.closed = False

    def fault(self, kind, now):
        return any(f['kind']==kind and f['start_ms']<=now<f['end_ms'] for f in self.config['faults'])

    def step(self):
        self.step_id += 1
        now = self.step_id*10
        if self.step_id==1 or now % self.world.p.vision_period_ms == 0:
            frame = self.world.frame(now,self.step_id)
            error = self.validator.validate(frame)
            if error:
                raise ValueError(f'Trama sintética rechazada: {error}')
            if not self.fault('vision-loss',now):
                self.vision_queue.append((now+self.world.p.vision_delay_ms,frame))
        frame = None
        while self.vision_queue and self.vision_queue[0][0]<=now:
            _,frame = self.vision_queue.pop(0)
        replies = []
        inputs = []
        for i in range(2):
            remote = None
            messages = []
            while self.peer_queue[i] and self.peer_queue[i][0][0]<=now:
                _,remote = self.peer_queue[i].pop(0)
                messages.extend(remote.get('messages',[]))
            if self.step_id == 1:
                remote = self.status[1-i]
            data = dict(step=self.step_id,time_ms=now,vision=frame, sensors=self.world.sensors(i),peer=remote,messages=messages)
            inputs.append(data)
            replies.append(self.controllers[i].step(data))
        for i,reply in enumerate(replies):
            if reply['ack']:
                self.acks[i] = reply['ack']
            reply['ack'] = self.acks[i]
            if not self.fault('peer-loss',now):
                self.peer_queue[1-i].append((now+max(10,self.world.p.peer_delay_ms),copy.deepcopy(reply)))
        self.status = replies
        self.world.advance([(r['left'],r['right']) for r in replies])
        snapshot = self.world.snapshot()
        declared = replies[0]['delivered'] | replies[1]['delivered']
        mismatches = [i for i,inside in enumerate(snapshot['delivered']) if bool(declared & (1<<i)) != inside]
        signature = ([r['phase'] for r in replies],[r.get('failure',0) for r in replies],snapshot['delivered'],mismatches,bool(snapshot['outside']))
        if signature != self.previous:
            self.events.append(dict(time_ms=now,phases=signature[0],failures=signature[1],
                                    physical_delivery=signature[2],delivery_disagreement=mismatches,outside=snapshot['outside']))
            self.previous = signature
        self.trace.write(json.dumps(dict(step=self.step_id,inputs=inputs,outputs=replies,world=snapshot),separators=(',',':'))+'\n')
        return snapshot

    def close(self):
        if getattr(self,'closed',False):
            return
        self.closed = True
        for controller in self.controllers:
            controller.close()
        if hasattr(self,'trace'):
            self.trace.close()
            report = dict(steps=self.step_id,events=self.events,final=self.world.snapshot(),controllers=self.status)
            (self.output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


def replay(trace, exe, output):
    """Replay exact saved controller inputs, independent of physics version."""
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    controllers = []
    count = 0
    try:
        for i in range(2):
            controllers.append(Controller(exe,10+i,output/f'replay-{10+i}.log'))
        with Path(trace).open(encoding='utf-8') as stream:
            for line in stream:
                record = json.loads(line)
                for i in range(2):
                    actual = controllers[i].step(record['inputs'][i])
                    expected = dict(record['outputs'][i])
                    # Persistent radio acknowledgement is added by the orchestrator.
                    if not actual['ack']:
                        expected['ack'] = {}
                    if actual != expected:
                        raise AssertionError(f'Replay divergente: paso {record["step"]}, rover {10+i}')
                count += 1
    finally:
        for c in controllers:
            c.close()
    return count
