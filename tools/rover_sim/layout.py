"""Seeded version of the cube generator in Vision Rover Challenge docs/index.html."""
import math


class Mulberry32:
    def __init__(self, seed):
        if type(seed) is not int or not 0 <= seed <= 0xffffffff:
            raise ValueError('La seed debe ser un entero entre 0 y 4294967295.')
        self.state = seed

    def __call__(self):
        self.state = (self.state + 0x6d2b79f5) & 0xffffffff
        t = self.state
        t = ((t ^ (t >> 15)) * (t | 1)) & 0xffffffff
        t ^= (t + (((t ^ (t >> 7)) * (t | 61)) & 0xffffffff)) & 0xffffffff
        return ((t ^ (t >> 14)) & 0xffffffff) / 4294967296


GOALS = {'red': (21, 1, 8, 6), 'green': (1, 21, 6, 8), 'blue': (43, 21, 6, 8)}
KEYS = ('red', 'blue', 'green')


def _overlap(a, b, gap=0):
    x, y, w, h = a
    bx, by, bw, bh = b
    return x < bx+bw+gap and x+w+gap > bx and y < by+bh+gap and y+h+gap > by


def _valid(cube, peers):
    _, x, y = cube
    box = (x, y, 3, 3)
    return (5 <= x <= 42 and 5 <= y <= 42
            and not any(_overlap(box, r) for r in (*GOALS.values(), (19,46,4,3), (27,46,4,3)))
            and not any(_overlap(box, (p[1],p[2],3,3), 2) for p in peers))


def _segment_distance(a, b, c, d):
    def orient(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    def on(p, q, r):
        return all(min(p[i],q[i])-1e-8 <= r[i] <= max(p[i],q[i])+1e-8 for i in (0,1))
    o1,o2,o3,o4 = orient(a,b,c),orient(a,b,d),orient(c,d,a),orient(c,d,b)
    if (o1*o2 < 0 and o3*o4 < 0) or any(abs(o)<1e-8 and on(p,q,r) for o,p,q,r in
            ((o1,a,b,c),(o2,a,b,d),(o3,c,d,a),(o4,c,d,b))):
        return 0
    def point(p, q, r):
        dx,dy = r[0]-q[0],r[1]-q[1]
        length = dx*dx+dy*dy
        t = max(0,min(1,((p[0]-q[0])*dx+(p[1]-q[1])*dy)/length)) if length else 0
        return math.hypot(p[0]-(q[0]+t*dx),p[1]-(q[1]+t*dy))
    return min(point(a,c,d),point(b,c,d),point(c,a,b),point(d,a,b))


def challenge_layout(seed, difficulty=.5):
    """Return page coordinates (top-left corner, 50x50 board), in page order."""
    rng = Mulberry32(seed)
    if not math.isfinite(difficulty) or not 0 <= difficulty <= 1:
        raise ValueError('La dificultad debe estar entre 0 y 1.')
    def rand(lo, hi):
        return lo + rng()*(hi-lo)
    def target(key):
        x,y,w,h = GOALS[key]
        return x+w/2,y+h/2
    def center(c):
        return c[1]+1.5,c[2]+1.5
    wave = math.sin(math.pi*difficulty)
    jitter = 2 if difficulty < .1 else 2.4 if difficulty > .9 else 3.6
    best, best_loss = None, math.inf
    for attempt in range(700):
        anchors = {
            'red': (23.5+rand(-jitter,jitter),8.5+31*difficulty+rand(-jitter,jitter)),
            'green': (8.5+29*difficulty+rand(-jitter,jitter),23+5*wave+rand(-jitter,jitter)),
            'blue': (38.5-29*difficulty+rand(-jitter,jitter),23-5*wave+rand(-jitter,jitter))}
        layout = [(key,*(max(5,min(42,math.floor(v+.5))) for v in anchors[key])) for key in KEYS]
        if not all(_valid(c,[p for j,p in enumerate(layout) if i != j]) for i,c in enumerate(layout)):
            continue
        distance = sum(abs(center(c)[0]-target(c[0])[0])+abs(center(c)[1]-target(c[0])[1]) for c in layout)/3
        crossings = sum(_segment_distance(center(layout[i]),target(layout[i][0]),center(layout[j]),target(layout[j][0])) < 4
                        for i in range(3) for j in range(i+1,3))
        desired = 0 if difficulty < .28 else 1 if difficulty < .53 else 2 if difficulty < .77 else 3
        loss = abs(distance-(6.5+35*difficulty))/36*.55 + abs(crossings-desired)/3*.45 + rand(0,.06)
        if loss < best_loss:
            best,best_loss = layout,loss
        if best_loss < .035 and attempt > 40:
            break
    if best is None:
        best = []
        for key in KEYS:
            for _ in range(10000):
                c = (key,math.floor(rand(5,43)),math.floor(rand(5,43)))
                if _valid(c,best):
                    best.append(c)
                    break
            else:
                raise ValueError('No se encontró espacio válido para los tres cubos.')
    return [dict(key=k,x=x,y=y) for k,x,y in best]


def apply_layout(config, seed, difficulty=.5):
    layout = challenge_layout(seed,difficulty)
    by_color = {c['key']:c for c in layout}
    # Rotate the page clockwise: its lower start becomes the simulator's left start.
    config['cubes'] = [[1000-(by_color[k]['y']+1.5)*20,(by_color[k]['x']+1.5)*20,0]
                       for k in ('green','blue','red')]
    config['seed'] = seed
    config['challenge_layout'] = dict(seed=seed,difficulty=difficulty,cubes=layout,algorithm='mulberry32-v1')
    return config
