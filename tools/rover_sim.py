"""Launch with tools/.venv Python after installing requirements-sim.txt."""
import argparse
import json
from pathlib import Path
from rover_sim.build import ROOT, build
from rover_sim.runner import Simulation, replay
from rover_sim.world import SCENARIOS, scenario


def main():
    p = argparse.ArgumentParser(description='Simulación del firmware real de dos rovers')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--scenario',choices=SCENARIOS,default='delivery')
    p.add_argument('--config',type=Path)
    p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--output',type=Path,default=ROOT/'.pio/sim/run')
    p.add_argument('--replay',type=Path)
    args = p.parse_args()
    if args.seconds <= 0:
        p.error('--seconds debe ser positivo')
    exe = build()
    if args.replay:
        print(f'Replay exacto: {replay(args.replay,exe,args.output)} pasos')
        return
    config = json.loads(args.config.read_text(encoding='utf-8')) if args.config else scenario(args.scenario)
    if args.headless:
        sim = Simulation(config,args.output,exe)
        try:
            for _ in range(round(args.seconds*100)):
                sim.step()
        finally:
            sim.close()
        print(json.dumps(dict(report=str(args.output/'report.json'),seconds=sim.world.time_ms/1000,
                              delivered=sim.world.delivered(),phases=[s['phase'] for s in sim.status])))
    else:
        from rover_sim.gui import run
        run(config,args.output,exe)


if __name__=='__main__':
    main()
