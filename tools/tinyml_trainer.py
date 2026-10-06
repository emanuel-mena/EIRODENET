"""Entrenamiento evolutivo de la política TinyML usando el simulador real.

El proceso emite un evento JSON por línea para que la GUI pueda mostrar el
progreso sin importar TensorFlow dentro del proceso gráfico.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import random
import shutil
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

from rover_sim.build import ROOT, build
from rover_sim.layout import apply_seeded_entropy
from rover_sim.runner import Simulation, TFLitePolicy
from rover_sim.world import SCENARIOS, scenario


LEADERBOARD_VERSION = 1
INPUTS = 68
OUTPUTS = 2
DEFAULT_OUTPUT = ROOT / ".pio" / "tinyml-training"
SPLITS = ROOT / "tools" / "rover_sim" / "tinyml_splits.json"


class TrainingStopped(RuntimeError):
    pass


@dataclass
class Candidate:
    metadata: dict[str, Any]
    weights: list[Any]


def emit(kind: str, **payload: Any) -> None:
    print(json.dumps({"event": kind, **payload}, separators=(",", ":")), flush=True)


def candidate_sort_key(item: dict[str, Any]) -> tuple[Any, ...]:
    """Elegibles por tiempo; los demás por entregas y progreso/fitness."""
    if item.get("eligible"):
        return (0, float(item.get("completion_ms") or math.inf),
                -float(item.get("fitness", -math.inf)))
    return (1, -int(item.get("deliveries", 0)),
            -float(item.get("fitness", -math.inf)), item.get("id", ""))


def public_candidate(item: Candidate | dict[str, Any]) -> dict[str, Any]:
    return dict(item.metadata if isinstance(item, Candidate) else item)


def write_leaderboard(destination: Path, candidates: list[Candidate], state: str,
                      settings: dict[str, Any]) -> Path:
    data = {
        "version": LEADERBOARD_VERSION,
        "state": state,
        "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "settings": settings,
        "candidates": [public_candidate(candidate) for candidate in candidates],
    }
    path = destination / "leaderboard.json"
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
    temporary.replace(path)
    return path


def _phase(observation: Any) -> int:
    return max(range(6), key=lambda index: float(observation[1 + index]))


def heuristic_action(observation: Any) -> tuple[float, float]:
    """Profesor geométrico sólo para inicializar la búsqueda evolutiva."""
    phase = _phase(observation)
    if phase == 4:  # retirada: C++ convierte una salida positiva en reversa
        return 0.72, 0.0
    if phase == 5:  # cesión: separarse del compañero
        peer_forward, peer_left = float(observation[28]), float(observation[29])
        turn = -0.8 if peer_left >= 0 else 0.8
        return 0.55, turn if abs(peer_forward) + abs(peer_left) > .02 else 0.0
    if phase == 3:  # empuje: el objetivo es el depósito
        forward, left = float(observation[22]), float(observation[23])
    else:  # tránsito, alineación y captura: el objetivo es el cubo
        forward, left = float(observation[20]), float(observation[21])
    angle = math.atan2(left, forward)
    angular = max(-1.0, min(1.0, angle / (math.pi / 2)))
    distance = min(1.0, math.hypot(forward, left) * 5.0)
    precision = .32 if phase in (1, 2, 3) else .78
    linear = distance * max(.08, 1.0 - precision * abs(angular))
    if forward < -.02:
        linear *= .15
    if phase == 2:
        linear = min(linear, .38)
    if phase == 3:
        linear = min(linear, .62)
    return max(-1.0, min(1.0, linear)), angular


def bootstrap_dataset(np: Any, samples: int, seed: int) -> tuple[Any, Any]:
    rng = np.random.default_rng(seed)
    x = rng.uniform(-1.0, 1.0, size=(samples, INPUTS)).astype(np.float32)
    y = np.zeros((samples, OUTPUTS), dtype=np.float32)
    for row in range(samples):
        phase = row % 6
        color = (row // 6) % 3
        x[row, 1:7] = -1
        x[row, 1 + phase] = 1
        x[row, 7:10] = -1
        x[row, 7 + color] = 1
        x[row, 20:24] = rng.uniform(-.85, .85, size=4)
        x[row, 26] = x[row, 27] = 1 if phase == 4 else -1
        x[row, 30:36] = rng.uniform(-1, 1, size=6)
        x[row, 44:56:3] = rng.uniform(-.8, .8, size=4)
        x[row, 46:56:3] = rng.choice((-1, 1), size=4)
        x[row, 56:60] = rng.uniform(.1, 1, size=4)
        y[row] = heuristic_action(x[row])
    return x, y


def create_model(tf: Any) -> Any:
    inputs = tf.keras.Input(shape=(INPUTS,), name="observation")
    value = tf.keras.layers.Dense(64, activation="relu", name="dense_1")(inputs)
    value = tf.keras.layers.Dense(64, activation="relu", name="dense_2")(value)
    outputs = tf.keras.layers.Dense(OUTPUTS, activation="tanh", name="action")(value)
    return tf.keras.Model(inputs, outputs, name="eiro_tinyml_policy")


def bootstrap_weights(tf: Any, np: Any, seed: int, epochs: int) -> tuple[list[Any], Any]:
    tf.keras.utils.set_random_seed(seed)
    model = create_model(tf)
    x, y = bootstrap_dataset(np, 4096, seed)
    model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=.002), loss="mse")
    model.fit(x, y, batch_size=128, epochs=epochs, verbose=0, shuffle=True)
    return model.get_weights(), x


def mutate_weights(np: Any, weights: list[Any], rng: Any, sigma: float) -> list[Any]:
    mutated = []
    for index, values in enumerate(weights):
        local_sigma = sigma * (.35 if index % 2 else 1.0)
        mutated.append((values + rng.normal(0, local_sigma, values.shape)).astype(values.dtype))
    return mutated


def quantize_model(tf: Any, np: Any, weights: list[Any], representative: Any,
                   destination: Path) -> str:
    model = create_model(tf)
    model.set_weights(weights)
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    def representative_dataset():
        for sample in representative[:512]:
            yield [sample[None, :].astype(np.float32)]

    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    payload = converter.convert()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(payload)
    # La misma validación estricta que usa el simulador antes de puntuar.
    TFLitePolicy(destination)
    return hashlib.sha256(payload).hexdigest()


def _distance(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def score_report(config: dict[str, Any], report: dict[str, Any], seconds: float) -> dict[str, Any]:
    final = report["final"]
    metrics = report["metrics"]
    initial = [_distance(cube, depot) for cube, depot in zip(config["cubes"], config["depots"])]
    remaining = [_distance(cube, depot) for cube, depot in zip(final["cubes"], config["depots"])]
    progress = sum(max(-.25, min(1.0, (before - after) / max(before, 1.0)))
                   for before, after in zip(initial, remaining)) / 3.0
    deliveries = sum(bool(value) for value in final["delivered"])
    unsafe = (metrics["outside_frames"] + metrics["rover_contacts"] +
              metrics["obstacle_contacts"])
    completion = metrics["completion_ms"]
    fitness = deliveries * 100_000 + progress * 20_000
    fitness += min(10_000, metrics["simultaneous_motion_ms"] / 4)
    fitness += metrics["effective_yields"] * 1_000
    fitness -= unsafe * 20_000 + metrics["reassignments"] * 2_000
    fitness -= bin(int(metrics["blocked_mask"])).count("1") * 10_000
    if report["eligible"]:
        fitness += 1_000_000 + max(0, seconds * 1000 - completion)
    return {
        "fitness": round(fitness, 3),
        "progress": round(progress, 6),
        "deliveries": deliveries,
        "eligible": bool(report["eligible"]),
        "completion_ms": completion,
        "unsafe_events": unsafe,
        "simultaneous_motion_ms": metrics["simultaneous_motion_ms"],
        "effective_yields": metrics["effective_yields"],
    }


def run_episode(model: Path, scenario_name: str, seed: int, seconds: float,
                output: Path, exe: Path, stop_file: Path) -> dict[str, Any]:
    if stop_file.exists():
        raise TrainingStopped()
    config = scenario(scenario_name)
    apply_seeded_entropy(config, seed)
    sim = Simulation(config, output, exe, model=model)
    try:
        for step in range(round(seconds * 100)):
            if step % 100 == 0 and stop_file.exists():
                raise TrainingStopped()
            sim.step()
            if sim.metrics["completion_ms"] is not None and all(
                    status["available"] and status["left"] == status["right"] == 0
                    for status in sim.status):
                break
    finally:
        sim.close()
    report = json.loads((output / "report.json").read_text(encoding="utf-8"))
    result = score_report(config, report, seconds)
    result["report"] = str((output / "report.json").resolve())
    return result


def aggregate_episodes(results: list[dict[str, Any]]) -> dict[str, Any]:
    eligible = all(item["eligible"] for item in results)
    completions = [item["completion_ms"] for item in results if item["completion_ms"] is not None]
    return {
        "fitness": round(sum(item["fitness"] for item in results) / len(results), 3),
        "progress": round(sum(item["progress"] for item in results) / len(results), 6),
        "deliveries": min(item["deliveries"] for item in results),
        "eligible": eligible,
        "completion_ms": (round(sum(completions) / len(completions))
                          if eligible and len(completions) == len(results) else None),
        "unsafe_events": sum(item["unsafe_events"] for item in results),
        "simultaneous_motion_ms": round(sum(item["simultaneous_motion_ms"] for item in results) / len(results)),
        "effective_yields": sum(item["effective_yields"] for item in results),
        "reports": [item["report"] for item in results],
    }


def train(args: argparse.Namespace) -> int:
    try:
        import numpy as np
        import tensorflow as tf
    except ImportError as exc:
        raise RuntimeError(
            "Falta TensorFlow. Instale tools/requirements-tinyml-gui.txt.") from exc

    destination = args.output.resolve()
    candidates_dir = destination / "candidates"
    runs_dir = destination / "runs"
    destination.mkdir(parents=True, exist_ok=True)
    if (destination / "leaderboard.json").exists():
        raise ValueError(
            "la carpeta ya contiene un entrenamiento; elija una carpeta nueva para conservarlo")
    candidates_dir.mkdir(exist_ok=True)
    runs_dir.mkdir(exist_ok=True)
    stop_file = destination / ".stop"
    settings = {
        "scenario": args.scenario,
        "seed": args.seed,
        "generations": args.generations,
        "population": args.population,
        "episodes": args.episodes,
        "seconds": args.seconds,
        "mutation_sigma": args.mutation_sigma,
        "bootstrap_epochs": args.bootstrap_epochs,
    }
    training_seeds = json.loads(SPLITS.read_text(encoding="utf-8"))["training"]
    if args.episodes > len(training_seeds):
        raise ValueError(
            f"episodes excede las {len(training_seeds)} seeds reservadas para entrenamiento")
    episode_seeds = training_seeds[:args.episodes]
    settings["layout_seeds"] = episode_seeds
    write_leaderboard(destination, [], "initializing", settings)
    emit("started", output=str(destination), settings=settings)
    if stop_file.exists():
        write_leaderboard(destination, [], "stopped", settings)
        emit("stopped", path=str(destination / "leaderboard.json"), candidates=0,
             completed=0, total=args.generations * args.population)
        return 0
    emit("progress", stage="building", message="Compilando controlador C++ del simulador")
    exe = build()
    emit("progress", stage="bootstrap", message="Inicializando política geométrica")
    base_weights, representative = bootstrap_weights(tf, np, args.seed, args.bootstrap_epochs)
    rng = np.random.default_rng(args.seed ^ 0x45564F4C)
    random.seed(args.seed)
    leaderboard: list[Candidate] = []
    generated_paths: set[Path] = set()
    completed = 0
    total = args.generations * args.population
    try:
        for generation in range(args.generations):
            parents = leaderboard[:max(1, min(4, len(leaderboard)))]
            generation_weights: list[list[Any]] = []
            for index in range(args.population):
                if generation == 0 and index == 0:
                    weights = [values.copy() for values in base_weights]
                elif index == 0 and parents:
                    weights = [values.copy() for values in parents[0].weights]
                else:
                    source = (parents[index % len(parents)].weights if parents else base_weights)
                    weights = mutate_weights(np, source, rng, args.mutation_sigma)
                generation_weights.append(weights)
            for index, weights in enumerate(generation_weights):
                if stop_file.exists():
                    raise TrainingStopped()
                candidate_id = f"g{generation + 1:03d}-c{index + 1:03d}"
                model_path = candidates_dir / f"{candidate_id}.tflite"
                emit("progress", stage="quantizing", generation=generation + 1,
                     candidate=index + 1, completed=completed, total=total,
                     message=f"Cuantizando {candidate_id}")
                digest = quantize_model(tf, np, weights, representative, model_path)
                generated_paths.add(model_path.resolve())
                episode_results = []
                for episode, episode_seed in enumerate(episode_seeds):
                    run_output = runs_dir / candidate_id / f"episode-{episode + 1:02d}"
                    emit("progress", stage="simulating", generation=generation + 1,
                         candidate=index + 1, episode=episode + 1, completed=completed,
                         total=total, message=f"Simulando {candidate_id}, episodio {episode + 1}/{args.episodes}")
                    episode_results.append(run_episode(model_path, args.scenario, episode_seed,
                                                       args.seconds, run_output, exe, stop_file))
                aggregate = aggregate_episodes(episode_results)
                metadata = {
                    "id": candidate_id,
                    "generation": generation + 1,
                    "candidate": index + 1,
                    "model": str(model_path.resolve()),
                    "sha256": digest,
                    **aggregate,
                }
                completed += 1
                candidate = Candidate(metadata=metadata, weights=weights)
                leaderboard.append(candidate)
                leaderboard.sort(key=lambda item: candidate_sort_key(item.metadata))
                leaderboard = leaderboard[:10]
                retained = {Path(item.metadata["model"]).resolve() for item in leaderboard}
                for stale in list(generated_paths - retained):
                    stale.unlink(missing_ok=True)
                    generated_paths.discard(stale)
                board_path = write_leaderboard(destination, leaderboard, "running", settings)
                emit("candidate", candidate=metadata, retained=model_path.resolve() in retained,
                     completed=completed, total=total)
                emit("leaderboard", path=str(board_path),
                     candidates=[public_candidate(item) for item in leaderboard],
                     completed=completed, total=total)
        board_path = write_leaderboard(destination, leaderboard, "finished", settings)
        emit("finished", path=str(board_path), candidates=len(leaderboard))
        return 0
    except TrainingStopped:
        board_path = write_leaderboard(destination, leaderboard, "stopped", settings)
        emit("stopped", path=str(board_path), candidates=len(leaderboard),
             completed=completed, total=total)
        return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    result.add_argument("--scenario", choices=SCENARIOS, default="delivery")
    result.add_argument("--seed", type=int, default=20261006)
    result.add_argument("--generations", type=int, default=8)
    result.add_argument("--population", type=int, default=8)
    result.add_argument("--episodes", type=int, default=2)
    result.add_argument("--seconds", type=float, default=45)
    result.add_argument("--mutation-sigma", type=float, default=.035)
    result.add_argument("--bootstrap-epochs", type=int, default=6)
    return result


def main() -> int:
    args = parser().parse_args()
    if args.generations < 1 or args.population < 2 or args.episodes < 1:
        raise SystemExit("generations y episodes deben ser >=1; population debe ser >=2")
    if args.seconds <= 0 or not 0 < args.mutation_sigma <= 1 or args.bootstrap_epochs < 1:
        raise SystemExit("seconds, mutation-sigma y bootstrap-epochs deben ser positivos")
    try:
        return train(args)
    except Exception as exc:
        emit("error", message=str(exc), type=type(exc).__name__)
        return 1


if __name__ == "__main__":
    sys.exit(main())
