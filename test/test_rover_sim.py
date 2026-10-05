import copy
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from rover_sim.world import World, scenario, SCENARIOS, black_at
from rover_sim.runner import Simulation, replay
from rover_sim.build import build


@pytest.fixture(scope='session')
def host_exe():
    return build()


def contact_world(offset=0):
    s = scenario()
    s['rovers'][0] = [200,500,0]
    s['cubes'][0] = [277.5,500+offset,0]
    return World(s)


def test_geometry_and_coordinates():
    w = contact_world()
    shapes = {label:s for s,label in w.labels.items() if label.startswith('rover-10')}
    body = shapes['rover-10:body'].bb
    left = shapes['rover-10:left-arm'].bb
    right = shapes['rover-10:right-arm'].bb
    assert body.right-body.left == 95
    assert body.top-body.bottom == 100
    assert left.right-left.left == 55
    assert left.top-left.bottom == 3
    assert left.bottom-right.top == 94
    assert left.right-body.left == 150
    w.rovers[0].angle = math.pi/2
    assert w.pose(w.rovers[0]) == [200,500,90]


@pytest.mark.parametrize('offset',[0,12])
def test_push_and_retreat_does_not_glue_cube(offset):
    w = contact_world(offset)
    touched = False
    for _ in range(100):
        w.advance([(700,700),(0,0)])
        touched |= any('green' in c['objects'] for c in w.contacts)
    assert touched
    assert w.cubes[0].position.x > 330
    start = w.cubes[0].position.x
    for _ in range(100): w.advance([(-700,-700),(0,0)])
    assert w.cubes[0].position.x >= start-2
    assert w.rovers[0].position.x < w.cubes[0].position.x-100


def test_arm_side_contact_and_turn():
    w = contact_world(18)
    touched_arm = False
    for _ in range(100):
        w.advance([(-700,700),(0,0)])
        touched_arm |= any('green' in c['objects'] and any('arm' in n for n in c['objects']) for c in w.contacts)
    assert touched_arm
    assert abs(w.rovers[0].angle) > .2
    assert (w.cubes[0].position-(277.5,482)).length > 2


def test_physical_delivery_checks_rotated_corners():
    w = World(scenario())
    b = w.cubes[0]
    x,row = w.config['depots'][0]
    b.position = x+70,1000-row
    assert w.delivered()[0]
    b.angle = math.pi/4
    assert not w.delivered()[0]


def test_seed_repeatability():
    a,b = contact_world(),contact_world()
    for _ in range(50):
        a.advance([(700,750),(0,0)]);b.advance([(700,750),(0,0)])
    assert a.snapshot() == b.snapshot()


def test_floor_markers_and_clearance():
    assert black_at(21,21)
    assert not black_at(5,5)
    assert black_at(201,201)
    assert not black_at(221,201)
    assert not black_at(-1,500)
    w = contact_world()
    # A centred cube fits between the arms, without an initial lateral contact.
    arms = [s for s,n in w.labels.items() if n.startswith('rover-10') and 'arm' in n]
    cube = next(iter(w.cubes[0].shapes))
    assert all(not a.shapes_collide(cube).points for a in arms)


@pytest.mark.parametrize('other',['obstacle','rover'])
def test_body_collision_blocks_forward_motion(other):
    s = scenario()
    s['rovers'][0] = [200,300,0]
    if other=='obstacle':s['obstacles'] = [[400,300]]
    else:s['rovers'][1] = [400,300,180]
    w = World(s)
    contacts = []
    for _ in range(250):
        w.advance([(700,700),(0,0)])
        contacts.extend(w.contacts)
    assert any(any(n.startswith('rover-10') for n in c['objects']) for c in contacts)
    if other=='obstacle':
        assert w.rovers[0].position.x < 360
    else:
        # Another rover is dynamic: it can be pushed, but must not be penetrated.
        assert w.rovers[1].position.x > 400
        assert w.rovers[0].position.x < 450
        for a in w.rovers[0].shapes:
            for b in w.rovers[1].shapes:
                assert all(p.distance >= -.5 for p in a.shapes_collide(b).points)


def test_gui_controls_and_reset(tmp_path,host_exe,monkeypatch):
    monkeypatch.setenv('QT_QPA_PLATFORM','offscreen')
    from PySide6.QtWidgets import QApplication
    from rover_sim.gui import Window
    app = QApplication.instance() or QApplication([])
    w = Window(scenario(),tmp_path,host_exe)
    w.single_step()
    assert w.sim.step_id==1
    w.start();assert w.timer.isActive()
    w.pause();assert not w.timer.isActive()
    old_pids = [c.p.pid for c in w.sim.controllers]
    w.reset()
    assert w.sim.step_id==0
    assert old_pids != [c.p.pid for c in w.sim.controllers]
    w.close()
    assert all(c.p.poll() is not None for c in w.sim.controllers)


def test_firmware_diagnostic_and_exact_replay(tmp_path,host_exe):
    # Historical diagnostic fixture, not the official arena layout.
    config = scenario()
    config.update(grid=dict(cols=50,rows=50,cell_mm=20.0),origin_mm=[0,0],
                  rovers=[[250,750,90],[750,750,90]],
                  depots=[[250,75],[750,75],[925,500]],start=[500,925])
    sim = Simulation(config,tmp_path/'run',host_exe)
    try:
        assert sim.controllers[0].p.pid != sim.controllers[1].p.pid
        for _ in range(800):sim.step()
        assert [r['phase'] for r in sim.status] == [8,8]
        assert not any(sim.world.delivered())
        assert sim.status[0]['route'] != sim.status[1]['route']
    finally:
        sim.close()
    assert replay(tmp_path/'run/trace.ndjson',host_exe,tmp_path/'replay') == 800


def test_official_depots_and_coordinate_frame():
    w = World(scenario())
    assert w.grid == dict(cols=43,rows=43,cell_mm=20.0)
    assert w.origin == [70,70]
    assert w.config['depots'] == [[500,145],[500,855],[855,500]]
    assert w.start == [145,500]
    assert w.to_grid(70,70) == (0,0)
    assert w.from_grid(43,43) == (930,930)
    f = w.frame(10,1)
    assert {p['color']:(p['col'],p['row']) for p in f['depots']} == {
        'green':(21.5,3.75),'blue':(21.5,39.25),'red':(39.25,21.5)}
    assert f['start'] == dict(col=3.75,row=21.5)
    assert [w.depot_extent(*p) for p in w.config['depots']] == [(200,150),(200,150),(150,200)]


@pytest.mark.parametrize('kind',['vision-loss','peer-loss'])
def test_loss_stops_and_recovers_inputs(tmp_path,host_exe,kind):
    s = scenario()
    s['faults'] = [dict(kind=kind,start_ms=3200,end_ms=5500)]
    sim = Simulation(s,tmp_path/kind,host_exe)
    try:
        for _ in range(510): sim.step()
        assert all(r['left']==r['right']==0 for r in sim.status)
        for _ in range(100): sim.step()
        assert sim.step_id==610
    finally:
        sim.close()
    records = [json.loads(line) for line in (tmp_path/kind/'trace.ndjson').read_text().splitlines()]
    key = 'vision' if kind=='vision-loss' else 'peer'
    assert any(row['inputs'][0][key] is not None for row in records[-50:])


@pytest.mark.parametrize('name',['crossing','obstacles','blocked','edge','delays'])
def test_scenarios_reach_running_with_valid_contract(tmp_path,host_exe,name):
    sim = Simulation(scenario(name),tmp_path/name,host_exe)
    try:
        for _ in range(350):sim.step()
        assert sim.step_id==350
        assert all(math.isfinite(v) for b in sim.world.rovers for v in sim.world.pose(b))
    finally:
        sim.close()


def test_invalid_contract_never_reaches_controller(tmp_path,host_exe):
    sim = Simulation(scenario(),tmp_path,host_exe)
    sim.world.frame = lambda *args: {'v':999}
    try:
        with pytest.raises(ValueError,match='rechazada'):sim.step()
        assert all(c.p.poll() is None for c in sim.controllers)
        assert sim.world.time_ms==0
    finally:
        sim.close()
