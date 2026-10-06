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


class TFLitePolicy:
    """Intérprete de entrenamiento para el contrato int8 [1,68] -> [1,2]."""
    def __init__(self, model):
        try:
            import numpy as np
            import tensorflow as tf
        except ImportError as exc:
            raise RuntimeError('Instale tools/requirements-tinyml.txt para usar --model.') from exc
        if Path(model).read_bytes()[4:8] != b'TFL3':
            raise ValueError('El modelo no contiene la firma FlatBuffer TFL3.')
        self.np = np
        self.interpreter = tf.lite.Interpreter(
            model_path=str(model), num_threads=1,
            experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_REF)
        self.interpreter.allocate_tensors()
        inputs, outputs = self.interpreter.get_input_details(), self.interpreter.get_output_details()
        if len(inputs) != 1 or len(outputs) != 1 or tuple(inputs[0]['shape']) != (1,68) or tuple(outputs[0]['shape']) != (1,2):
            raise ValueError('El modelo debe tener tensores [1,68] y [1,2].')
        if inputs[0]['dtype'] != np.int8 or outputs[0]['dtype'] != np.int8:
            raise ValueError('El modelo debe estar cuantizado completamente a int8.')
        operations = self.interpreter._get_ops_details()
        if [op['op_name'] for op in operations] != [
                'FULLY_CONNECTED','FULLY_CONNECTED','FULLY_CONNECTED','TANH']:
            raise ValueError('La arquitectura debe ser Dense(64)-Dense(64)-Dense(2,tanh).')
        tensors = {item['index']:item for item in self.interpreter.get_tensor_details()}
        widths = [tuple(tensors[op['outputs'][0]]['shape']) for op in operations[:3]]
        if widths != [(1,64),(1,64),(1,2)]:
            raise ValueError('Las capas densas deben producir 64, 64 y 2 valores.')
        self.input, self.output = inputs[0], outputs[0]

    def __call__(self, observation):
        np = self.np
        scale, zero = self.input['quantization']
        if not scale:
            raise ValueError('La entrada int8 no tiene parámetros de cuantización.')
        values = np.clip(np.rint(np.asarray(observation, dtype=np.float32)/scale + zero),-128,127).astype(np.int8)[None,:]
        self.interpreter.set_tensor(self.input['index'], values)
        self.interpreter.invoke()
        quantized = self.interpreter.get_tensor(self.output['index'])[0]
        out_scale, out_zero = self.output['quantization']
        if not out_scale:
            raise ValueError('La salida int8 no tiene parámetros de cuantización.')
        action = np.clip((quantized.astype(np.float32)-out_zero)*out_scale,-1,1)
        return action.tolist(), values[0].tolist(), quantized.tolist()


class Controller:
    def __init__(self, exe, identity, log, model=None):
        self.stderr = log.open('w', encoding='utf-8')
        env = dict(os.environ)
        if os.name == 'nt':
            env['PATH'] = str(Path(os.environ.get('CXX', 'C:/msys64/ucrt64/bin/g++.exe')).parent) + os.pathsep + env['PATH']
        command = [str(exe),str(identity)] + (['--policy-rpc'] if model else [])
        self.policy = TFLitePolicy(model) if model else None
        self.last_inferences = []
        self.p = subprocess.Popen(command, stdin=subprocess.PIPE,
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
        self.last_inferences = []
        self.p.stdin.write(json.dumps(data,separators=(',',':'))+'\n')
        self.p.stdin.flush()
        while True:
            try:
                line = self.lines.get(timeout=5)
            except queue.Empty as exc:
                raise RuntimeError('El controlador no respondió en 5 segundos.') from exc
            if line is None:
                raise RuntimeError(f'El controlador terminó: {self.p.poll()}; consulte el registro C++.')
            response = json.loads(line)
            if response.get('type') != 'inference':
                break
            if self.policy is None:
                raise RuntimeError('El controlador solicitó inferencia sin un modelo.')
            action, quantized_input, quantized_output = self.policy(response['observation'])
            self.last_inferences.append(dict(observation=response['observation'],
                                             input_int8=quantized_input,
                                             output_int8=quantized_output,action=action))
            self.p.stdin.write(json.dumps(dict(action=action),separators=(',',':'))+'\n')
            self.p.stdin.flush()
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
    def __init__(self, config, output, exe=None, model=None):
        self.config = copy.deepcopy(config)
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.world = World(self.config)
        vc = VisionConfig.from_environment()
        self.validator = ContractValidator(vc.vision_system)
        if self.validator.version != 3:
            raise ValueError('El simulador requiere el contrato de visión v3.')
        exe = exe or build()
        sources = [ROOT/'src/services'/f'{n}.cpp' for n in ('navigation_service','competition_runtime','tinyml_policy','grid_planner','motion_control','vision_contract')]
        sources += [vc.vision_system/'contrato/schema.py', ROOT/'tools/vision_client/core.py', exe]
        if self.config.get('layout_source'):
            sources.append(Path(self.config['layout_source']))
        sources += sorted((ROOT/'include').glob('*.hpp'))
        sources += sorted(p for p in (ROOT/'tools/rover_sim').rglob('*')
                          if p.suffix in ('.py','.hpp','.h','.cpp') or p.name == 'vision_entropy.json')
        realized = dict(motor_strengths=[dict(left=left,right=right)
                                         for left,right in self.world.motor_strengths])
        model_path = Path(model) if model else None
        if model_path:
            realized['model'] = dict(path=str(model_path),sha256=hashlib.sha256(model_path.read_bytes()).hexdigest())
        self.model_sha256 = realized.get('model',{}).get('sha256')
        manifest = dict(scenario=self.config, realized=realized,
                        sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
                        runtime=dict(python=platform.python_version(),system=platform.platform(),pymunk=version('pymunk'),pyside6=version('PySide6')),
                        assumptions='Physical parameters estimated; calibrated sensors; verified competition startup; no radio or camera emulation.')
        (self.output/'scenario.json').write_text(json.dumps(self.config,indent=2),encoding='utf-8')
        (self.output/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        self.controllers = []
        try:
            for identity in (10,11):
                self.controllers.append(Controller(exe,identity,self.output/f'rover-{identity}.log',model_path))
        except Exception:
            self.close()
            raise
        self.trace = (self.output/'trace.ndjson').open('w',encoding='utf-8')
        self.step_id = 0
        self.vision_queue = []
        self.peer_queue = [[],[]]
        self.status = [dict(available=True,delivered=0,ack={},messages=[],phase=0,left=0,right=0,
                            model_available=bool(model_path),route=[]) for _ in range(2)]
        self.acks = [{},{}]
        self.events = []
        self.previous = None
        self.metrics = dict(simultaneous_motion_ms=0,yields=0,effective_yields=0,reassignments=0,
                            blocked_mask=0,
                            recovery_after_faults={f['kind']:False for f in config['faults']},
                            outside_frames=0,rover_contacts=0,obstacle_contacts=0,
                            completion_ms=None)
        self.closed = False
        self.yield_start_distance = None
        self.fault_recovery = {f['kind']:[False,False] for f in config['faults']}

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
        if all(r['left'] or r['right'] for r in replies):
            self.metrics['simultaneous_motion_ms'] += 10
        self.metrics['yields'] = max(self.metrics['yields'],sum(r.get('yields',0) for r in replies))
        peer_distance = ((snapshot['rovers'][0][0]-snapshot['rovers'][1][0])**2 +
                         (snapshot['rovers'][0][1]-snapshot['rovers'][1][1])**2)**.5
        if replies[1]['phase'] == 5 and self.yield_start_distance is None:
            self.yield_start_distance = peer_distance
        elif replies[1]['phase'] != 5 and self.yield_start_distance is not None:
            if peer_distance >= self.yield_start_distance + self.world.grid['cell_mm'] * .5:
                self.metrics['effective_yields'] += 1
            self.yield_start_distance = None
        self.metrics['reassignments'] = max(self.metrics['reassignments'],sum(r.get('reassignments',0) for r in replies))
        self.metrics['blocked_mask'] |= replies[0].get('blocked_mask',0) | replies[1].get('blocked_mask',0)
        for fault in self.config['faults']:
            if now >= fault['end_ms']:
                for rover, reply in enumerate(replies):
                    if reply['left'] or reply['right']:
                        self.fault_recovery[fault['kind']][rover] = True
                self.metrics['recovery_after_faults'][fault['kind']] = all(
                    self.fault_recovery[fault['kind']])
        if snapshot['outside']:
            self.metrics['outside_frames'] += 1
        for contact in snapshot['contacts']:
            names = contact['objects']
            if 'obstacle' in names:self.metrics['obstacle_contacts'] += 1
            if (any(n.startswith('rover-10') for n in names) and any(n.startswith('rover-11') for n in names)):
                self.metrics['rover_contacts'] += 1
        if self.metrics['completion_ms'] is None and all(snapshot['delivered']) and all(r['available'] for r in replies):
            self.metrics['completion_ms'] = now
        declared = replies[0]['delivered'] | replies[1]['delivered']
        mismatches = [i for i,inside in enumerate(snapshot['delivered']) if bool(declared & (1<<i)) != inside]
        signature = ([r['phase'] for r in replies],[r.get('failure',0) for r in replies],snapshot['delivered'],mismatches,bool(snapshot['outside']))
        if signature != self.previous:
            self.events.append(dict(time_ms=now,phases=signature[0],failures=signature[1],
                                    physical_delivery=signature[2],delivery_disagreement=mismatches,outside=snapshot['outside']))
            self.previous = signature
        inferences = [controller.last_inferences for controller in self.controllers]
        self.trace.write(json.dumps(dict(step=self.step_id,inputs=inputs,outputs=replies,
                                         policy=inferences,world=snapshot),separators=(',',':'))+'\n')
        return snapshot

    def close(self):
        if getattr(self,'closed',False):
            return
        self.closed = True
        for controller in self.controllers:
            controller.close()
        if hasattr(self,'trace'):
            self.trace.close()
            final = self.world.snapshot()
            declared = self.status[0]['delivered'] | self.status[1]['delivered']
            eligible = (all(final['delivered']) and declared == 7 and
                        self.metrics['outside_frames'] == 0 and
                        self.metrics['rover_contacts'] == 0 and self.metrics['obstacle_contacts'] == 0 and
                        all(r['available'] and r['left']==r['right']==0 for r in self.status))
            report = dict(steps=self.step_id,events=self.events,final=final,
                          controllers=self.status,metrics=self.metrics,
                          physical_delivery_mask=sum((1<<i) for i,value in enumerate(final['delivered']) if value),
                          declared_delivery_mask=declared,model_sha256=self.model_sha256,
                          eligible=eligible)
            (self.output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')


def replay(trace, exe, output, model=None):
    """Replay exact saved controller inputs, independent of physics version."""
    output = Path(output)
    output.mkdir(parents=True,exist_ok=True)
    controllers = []
    count = 0
    try:
        for i in range(2):
            controllers.append(Controller(exe,10+i,output/f'replay-{10+i}.log',model))
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
