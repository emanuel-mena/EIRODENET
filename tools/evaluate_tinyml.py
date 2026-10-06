"""Califica y clasifica políticas TFLite sin modificar ni conservar modelos."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from rover_sim.build import ROOT, build
from rover_sim.layout import apply_seeded_entropy
from rover_sim.runner import Simulation
from rover_sim.world import scenario


SCENARIOS = ("delivery", "crossing", "vision-loss", "peer-loss", "delays")
SPLITS = Path(__file__).parent / "rover_sim" / "tinyml_splits.json"


def run_candidate(model: Path, split: str, seeds: list[int], output: Path,
                  seconds: float, exe: Path) -> dict:
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    runs = []
    for seed in seeds:
        for scenario_name in SCENARIOS:
            config = scenario(scenario_name)
            apply_seeded_entropy(config, seed)
            destination = output / digest[:12] / f"{split}-{seed}-{scenario_name}"
            sim = Simulation(config, destination, exe, model=model)
            try:
                for _ in range(round(seconds * 100)):
                    sim.step()
            finally:
                sim.close()
            report = json.loads((destination / "report.json").read_text(encoding="utf-8"))
            metrics = report["metrics"]
            eligible = bool(report["eligible"])
            if scenario_name == "delivery":
                eligible &= metrics["simultaneous_motion_ms"] >= 1000
            if scenario_name == "crossing":
                eligible &= metrics["effective_yields"] > 0
            if scenario_name in ("vision-loss", "peer-loss"):
                eligible &= metrics["recovery_after_faults"].get(scenario_name, False)
            runs.append(dict(seed=seed, scenario=scenario_name, eligible=eligible,
                             completion_ms=metrics["completion_ms"], report=str(destination / "report.json")))
    times = [run["completion_ms"] for run in runs if run["completion_ms"] is not None]
    eligible = len(times) == len(runs) and all(run["eligible"] for run in runs)
    ordered = sorted(times)
    p95 = ordered[max(0, math.ceil(.95 * len(ordered)) - 1)] if ordered else None
    return dict(model=str(model), sha256=digest, eligible=eligible,
                mean_completion_ms=(sum(times) / len(times) if times else None),
                p95_completion_ms=p95, runs=runs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("models", nargs="+", type=Path)
    parser.add_argument("--split", choices=("training", "classification", "qualification"),
                        default="classification")
    parser.add_argument("--seconds", type=float, default=120)
    parser.add_argument("--output", type=Path, default=ROOT / ".pio" / "sim" / "tinyml-evaluation")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds debe ser positivo")
    seeds = json.loads(SPLITS.read_text(encoding="utf-8"))[args.split]
    exe = build()
    results = [run_candidate(model.resolve(), args.split, seeds, args.output, args.seconds, exe)
               for model in args.models]
    results.sort(key=lambda item: (not item["eligible"],
                                   item["mean_completion_ms"] if item["mean_completion_ms"] is not None else math.inf,
                                   item["p95_completion_ms"] if item["p95_completion_ms"] is not None else math.inf))
    summary = dict(split=args.split, seeds=seeds, scenarios=SCENARIOS, ranking=results)
    args.output.mkdir(parents=True, exist_ok=True)
    destination = args.output / "ranking.json"
    destination.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(dict(ranking=str(destination), eligible=sum(r["eligible"] for r in results),
                          candidates=len(results))))


if __name__ == "__main__":
    main()
