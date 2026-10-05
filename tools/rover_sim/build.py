from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def build():
    out = ROOT / '.pio' / 'sim'
    out.mkdir(parents=True, exist_ok=True)
    compiler = os.environ.get('CXX') or shutil.which('g++')
    if not compiler:
        raise RuntimeError('Instale g++ (MSYS2 UCRT64 en Windows) o configure CXX.')
    cjson = Path(os.environ.get('PLATFORMIO_CORE_DIR', Path.home() / '.platformio')) / 'packages/framework-espidf/components/json/cJSON'
    if not (cjson / 'cJSON.c').exists():
        raise RuntimeError('Falta cJSON de ESP-IDF; ejecute pio run primero.')
    exe = out / ('rover_host.exe' if os.name == 'nt' else 'rover_host')
    sources = [ROOT / 'src/services' / (name + '.cpp') for name in
               ('navigation_service', 'competition_runtime', 'grid_planner', 'motion_control')]
    sources += [ROOT / 'tools/rover_sim/host/main.cpp', cjson / 'cJSON.c']
    subprocess.run([compiler, '-std=gnu++20', '-O2', '-DEIRO_HOST_SIM',
                    '-I' + str(ROOT / 'tools/rover_sim/host'), '-I' + str(ROOT / 'include'),
                    '-I' + str(cjson), *map(str, sources), '-o', str(exe)], check=True)
    return exe


if __name__ == '__main__':
    print(build())
