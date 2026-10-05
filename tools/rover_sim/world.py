"""Millimetres, kilograms, seconds. World Y points up; vision row points down."""
import math
import random
import json
from dataclasses import asdict, dataclass

import pymunk
from vision_client.core import VisionConfig

COLORS = ('green', 'blue', 'red')
# Six-by-six marker cells transcribed from the supplied one-metre PDF.
MARKERS = ((20,20,('111111','111001','100111','101111','111001','111111')),
           (880,20,('111111','111101','111011','100101','110001','111111')),
           (20,880,('111111','101111','101111','110001','101001','111111')),
           (880,880,('111111','100011','111011','101011','110011','111111')))


def black_at(x,row):
    if not (0<=x<1000 and 0<=row<1000):
        return False
    if (x<140 or x>=860) and (row<140 or row>=860):
        for mx,my,bits in MARKERS:
            if mx<=x<mx+100 and my<=row<my+100:
                return bits[int((row-my)*6/100)][int((x-mx)*6/100)]=='1'
        return False
    return (math.floor(x/20)+math.floor(row/20))%2==0


@dataclass
class Parameters:
    # Estimates, not measurements. Geometry above the drivetrain is confirmed.
    rover_mass: float = 0.6
    cube_mass: float = 0.08
    wheel_track_mm: float = 80.0
    full_speed_mm_s: float = 180.0
    traction_accel_mm_s2: float = 1200.0
    motor_response_s: float = 0.10
    contact_friction: float = 0.5
    cube_floor_deceleration_mm_s2: float = 200.0
    vision_period_ms: int = 50
    ultrasonic_period_ms: int = 200
    vision_delay_ms: int = 0
    peer_delay_ms: int = 10
    noise_mm: float = 0.0
    substeps: int = 10


def scenario(name='delivery'):
    source = VisionConfig.from_environment().vision_system / 'vision/config_vision.json'
    official = json.loads(source.read_text(encoding='utf-8'))
    grid = {k:official['tablero'][k] for k in ('cols','rows','cell_mm')}
    cell = grid['cell_mm']
    # The origin is marker 0's centre, inset from the physical mat's corner.
    origin = [(1000-grid['cols']*cell)/2, (1000-grid['rows']*cell)/2]
    places = official['lugares']
    centers = {p['color']:[origin[0]+p['col']*cell,origin[1]+p['row']*cell]
               for p in places['depots']}
    start = [origin[0]+places['start']['col']*cell,origin[1]+places['start']['row']*cell]
    s = dict(name=name, seed=1, parameters=asdict(Parameters()),
             rovers=[[start[0],start[1]-150,0], [start[0],start[1]+150,0]],
             cubes=[[250, 450, 0], [750, 450, 0], [500, 550, 0]],
             depots=[centers[c] for c in COLORS], obstacles=[], faults=[],
             grid=grid, origin_mm=origin,
             start=start,
             depot_size_mm=[places['tamano_deposito_mm']['largo'],places['tamano_deposito_mm']['fondo']],
             layout_source=str(source))
    if name == 'crossing':
        s['cubes'] = [[700, 400, 0], [300, 400, 0], [500, 550, 0]]
    elif name == 'obstacles':
        s['obstacles'] = [[250, 610], [650, 600]]
    elif name == 'blocked':
        s['obstacles'] = [[150, 550], [250, 550], [350, 550]]
    elif name == 'edge':
        s['cubes'][0] = [origin[0]+45, 450, 0]
    elif name in ('vision-loss', 'peer-loss'):
        s['faults'] = [dict(kind=name, start_ms=3200, end_ms=5500)]
    elif name == 'delays':
        s['parameters'].update(vision_delay_ms=250, peer_delay_ms=250)
    elif name != 'delivery':
        raise ValueError(f'Escenario desconocido: {name}')
    return s


SCENARIOS = ('delivery', 'crossing', 'obstacles', 'blocked', 'edge',
             'vision-loss', 'peer-loss', 'delays')


class World:
    def __init__(self, config):
        self.config = config
        # Saved old scenarios retain their original coordinate system for replay.
        self.grid = config.get('grid',dict(cols=50,rows=50,cell_mm=20.0))
        self.origin = config.get('origin_mm',[0,0])
        self.depot_size = config.get('depot_size_mm',[200,150])
        self.start = config.get('start',[500,925])
        self.p = Parameters(**config['parameters'])
        values = asdict(self.p)
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values.values()):
            raise ValueError('Los parámetros deben ser números finitos.')
        if any(values[k]<=0 for k in ('rover_mass','cube_mass','wheel_track_mm','traction_accel_mm_s2','vision_period_ms')):
            raise ValueError('Masa, separación de ruedas, aceleración y período deben ser positivos.')
        if any(values[k]<0 for k in ('vision_delay_ms','peer_delay_ms','noise_mm','contact_friction','cube_floor_deceleration_mm_s2')):
            raise ValueError('Retardos, ruido y fricción no pueden ser negativos.')
        if self.p.vision_period_ms % 10 or int(self.p.substeps)!=self.p.substeps:
            raise ValueError('El período debe ser múltiplo de 10 ms y los subpasos enteros.')
        if self.p.ultrasonic_period_ms <= 0 or self.p.ultrasonic_period_ms % 10:
            raise ValueError('El período ultrasónico debe ser un múltiplo positivo de 10 ms.')
        if len(config['rovers'])!=2 or len(config['cubes'])!=3 or len(config['depots'])!=3 or len(config['obstacles'])>32:
            raise ValueError('Se requieren dos rovers, tres cubos, tres depósitos y máximo 32 obstáculos.')
        if self.p.substeps < 10 or self.p.full_speed_mm_s <= 0 or self.p.motor_response_s <= 0:
            raise ValueError('Se requieren >=10 subpasos y velocidad/respuesta positivas.')
        self.rng = random.Random(config['seed'])
        self.space = pymunk.Space()
        self.space.iterations = 30
        self.space.collision_slop = 0.02
        self.contacts = []
        self.labels = {}
        self.rovers = []
        self.cubes = []
        self.obstacles = []
        self.time_ms = 0
        self.ultrasonic_samples = [None, None]
        for i, pose in enumerate(config['rovers']):
            b = pymunk.Body(self.p.rover_mass, pymunk.moment_for_box(self.p.rover_mass, (150, 100)))
            b.position = pose[0], 1000-pose[1]
            b.angle = math.radians(pose[2])
            self.space.add(b)
            self.rovers.append(b)
            self._rect(b, -47.5, -50, 47.5, 50, f'rover-{i+10}:body', i+1)
            self._rect(b, 47.5, -50, 102.5, -47, f'rover-{i+10}:right-arm', i+1)
            self._rect(b, 47.5, 47, 102.5, 50, f'rover-{i+10}:left-arm', i+1)
        for i, pose in enumerate(config['cubes']):
            b = pymunk.Body(self.p.cube_mass, pymunk.moment_for_box(self.p.cube_mass, (60, 60)))
            b.position = pose[0], 1000-pose[1]
            b.angle = math.radians(pose[2])
            self.space.add(b)
            self._rect(b, -30, -30, 30, 30, COLORS[i], 0)
            self.cubes.append(b)
        for x, row in config['obstacles']:
            b = pymunk.Body(body_type=pymunk.Body.STATIC)
            b.position = x, 1000-row
            self.space.add(b)
            self._rect(b, -50, -50, 50, 50, 'obstacle', 0)
            self.obstacles.append(b)
        # The PDF is a flat mat, not a walled arena. Departures are diagnosed.
        self.space.add_default_collision_handler().post_solve = self._contact

    def _rect(self, body, x0, y0, x1, y1, label, group):
        s = pymunk.Poly(body, [(x0,y0),(x1,y0),(x1,y1),(x0,y1)])
        s.friction = self.p.contact_friction
        s.elasticity = 0
        s.filter = pymunk.ShapeFilter(group=group)
        self.labels[s] = label
        self.space.add(s)
        return s

    def _contact(self, arbiter, space, data):
        names = [self.labels[s] for s in arbiter.shapes]
        points = [[p.point_a.x, 1000-p.point_a.y] for p in arbiter.contact_point_set.points]
        self.contacts.append(dict(objects=names, points=points))

    @staticmethod
    def pose(b):
        return [b.position.x, 1000-b.position.y, math.degrees(b.angle) % 360]

    @staticmethod
    def polygon(s):
        return [[p.x, 1000-p.y] for p in (s.body.local_to_world(v) for v in s.get_vertices())]

    def sensors(self, i):
        b = self.rovers[i]
        ir = []
        for x,y in ((10,20),(10,-20),(-10,20),(-10,-20)):
            p = b.local_to_world((x,y))
            row = 1000-p.y
            black = black_at(p.x,row)
            ir.append(3200 if black else 600)
        stamp = self.time_ms // self.p.ultrasonic_period_ms * self.p.ultrasonic_period_ms
        cached = self.ultrasonic_samples[i]
        if cached is None or cached['timestamp_ms'] != stamp:
            origin = b.local_to_world((47.5,0))
            end = origin + pymunk.Vec2d(2000,0).rotated(b.angle)
            hit = self.space.segment_query_first(origin, end, 0, pymunk.ShapeFilter(group=i+1))
            cached = dict(timestamp_ms=stamp, ultrasonic_valid=hit is not None,
                          distance_mm=round(hit.alpha*2000) if hit else 2000)
            self.ultrasonic_samples[i] = cached
        return dict(ir=ir, gyro=math.degrees(b.angular_velocity), **cached)

    def advance(self, commands):
        self.contacts = []
        # Bound tip displacement below 0.3 mm even if speed estimates are raised.
        steps = max(self.p.substeps, math.ceil(self.p.full_speed_mm_s * .01 *
                    (1+205/self.p.wheel_track_mm)/.3))
        dt = .01/steps
        for _ in range(steps):
            for b, (left,right) in zip(self.rovers, commands):
                v = (left+right)/2000*self.p.full_speed_mm_s
                w = (right-left)/1000*self.p.full_speed_mm_s/self.p.wheel_track_mm
                target = pymunk.Vec2d(v,0).rotated(b.angle)
                delta = (target-b.velocity)*min(1,dt/self.p.motor_response_s)
                cap = self.p.traction_accel_mm_s2*dt
                if delta.length > cap:
                    delta = delta.normalized()*cap
                b.apply_impulse_at_world_point(delta*b.mass,b.position)
                dw = (w-b.angular_velocity)*min(1,dt/self.p.motor_response_s)
                b.angular_velocity += max(-cap/self.p.wheel_track_mm, min(cap/self.p.wheel_track_mm,dw))
            for b in self.cubes:
                speed = b.velocity.length
                if speed:
                    b.velocity *= max(0,1-self.p.cube_floor_deceleration_mm_s2*dt/speed)
                b.angular_velocity *= math.exp(-8*dt)
            self.space.step(dt)
        self.time_ms += 10

    def delivered(self):
        result = []
        for b, (x,row) in zip(self.cubes, self.config['depots']):
            width,height = self.depot_extent(x,row)
            hx,hy = width/2,height/2
            result.append(all(abs(px-x)<=hx and abs(py-row)<=hy
                              for s in b.shapes for px,py in self.polygon(s)))
        return result

    def to_grid(self,x,row):
        cell = self.grid['cell_mm']
        return (x-self.origin[0])/cell,(row-self.origin[1])/cell

    def from_grid(self,col,row):
        cell = self.grid['cell_mm']
        return self.origin[0]+col*cell,self.origin[1]+row*cell

    def depot_extent(self,x,row):
        col,r = self.to_grid(x,row)
        horizontal = min(r,self.grid['rows']-r)<min(col,self.grid['cols']-col)
        length,depth = self.depot_size
        return (length,depth) if horizontal else (depth,length)

    def snapshot(self):
        return dict(time_ms=self.time_ms, rovers=[self.pose(b) for b in self.rovers],
                    cubes=[self.pose(b) for b in self.cubes], delivered=self.delivered(),
                    contacts=self.contacts,
                    outside=[self.labels[s] for s in self.space.shapes
                             if any(not (0<=self.to_grid(x,y)[0]<=self.grid['cols'] and
                                         0<=self.to_grid(x,y)[1]<=self.grid['rows']) for x,y in self.polygon(s))])

    def frame(self, time_ms, sequence):
        elapsed = max(0,time_ms-3000)
        def pos(b):
            x,row,theta = self.pose(b)
            col,row = self.to_grid(x+self.rng.gauss(0,self.p.noise_mm),row+self.rng.gauss(0,self.p.noise_mm))
            return dict(col=col,row=row,age_ms=0)
        return dict(v=2, seq=sequence, ts_ms=time_ms,
                    phase='READY' if time_ms<3000 else 'RUNNING',
                    clock=dict(elapsed_ms=elapsed,remaining_ms=max(0,180000-elapsed),total_ms=max(180000,elapsed)),
                    grid=dict(self.grid), cube_side=60/self.grid['cell_mm'],
                    depot_size=dict(length=self.depot_size[0]/self.grid['cell_mm'],depth=self.depot_size[1]/self.grid['cell_mm']),
                    start=dict(zip(('col','row'),self.to_grid(*self.start))),
                    rovers=[dict(id=10+i,theta=self.pose(b)[2],**pos(b)) for i,b in enumerate(self.rovers)],
                    cubes=[dict(color=COLORS[i],**pos(b)) for i,b in enumerate(self.cubes)],
                    obstacles=[pos(b) for b in self.obstacles],
                    depots=[dict(color=c,**dict(zip(('col','row'),self.to_grid(x,y)))) for c,(x,y) in zip(COLORS,self.config['depots'])])
