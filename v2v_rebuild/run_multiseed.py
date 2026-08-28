from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path
from statistics import mean, stdev
from typing import Callable


MODES = ["perfect_v2v", "lossy_v2v", "sensor_only"]
METRICS = ["collision_rate", "arrival_rate", "near_miss_rate", "mean_min_distance", "mean_reward"]


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def train_model(
    project_dir: Path,
    environment: str,
    mode: str,
    seed: int,
    timesteps: int,
    output_path: Path,
    pdr_scale: float,
) -> None:
    script_name = "train_simple_ppo.py" if environment == "simple" else "train_sumo_ppo.py"
    command = [
        sys.executable,
        str(project_dir / script_name),
        "--timesteps",
        str(timesteps),
        "--mode",
        mode,
        "--pdr-scale",
        str(pdr_scale),
        "--seed",
        str(seed),
        "--out",
        str(output_path),
    ]
    print("\nTraining:", " ".join(command))
    subprocess.run(command, cwd=project_dir, check=True)


def aggregate(
    rows: list[dict[str, object]],
    seeds: list[int],
    episodes: int,
    modes: list[str],
    timesteps: int,
) -> list[dict[str, object]]:
    summaries: list[dict[str, object]] = []
    for mode in modes:
        mode_rows = [row for row in rows if row["eval_mode"] == mode]
        summary: dict[str, object] = {
            "eval_mode": mode,
            "seeds": ",".join(str(seed) for seed in seeds),
            "num_seeds": len(mode_rows),
            "episodes_per_seed": episodes,
            "timesteps_per_model": timesteps,
        }
        for metric in METRICS:
            values = [float(row[metric]) for row in mode_rows]
            metric_mean = mean(values)
            metric_std = stdev(values) if len(values) > 1 else 0.0
            ci95 = 1.96 * metric_std / math.sqrt(len(values)) if values else 0.0
            summary[f"{metric}_mean"] = metric_mean
            summary[f"{metric}_std"] = metric_std
            summary[f"{metric}_ci95_half_width"] = ci95
        summaries.append(summary)
    return summaries


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train and evaluate matched PPO models across multiple seeds."
    )
    parser.add_argument("--environment", choices=["simple", "sumo"], default="simple")
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--pdr-scale", type=float, default=1.0)
    parser.add_argument("--out-dir", default=None)
    parser.add_argument("--force-retrain", action="store_true")
    parser.add_argument(
        "--modes",
        nargs="+",
        choices=MODES,
        default=MODES,
        help="Observation modes to run. Defaults to all three modes.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    project_dir = Path(__file__).resolve().parent
    out_dir = Path(args.out_dir) if args.out_dir else project_dir / "runs" / "multiseed" / args.environment
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.environment == "simple":
        from evaluate_simple import evaluate
    else:
        from evaluate_sumo import evaluate

    evaluate_fn: Callable[..., dict[str, object]] = evaluate
    raw_rows: list[dict[str, object]] = []
    for seed in args.seeds:
        for mode in args.modes:
            model_path = out_dir / f"ppo_seed{seed}_{mode}.zip"
            if args.force_retrain or not model_path.exists():
                train_model(
                    project_dir,
                    args.environment,
                    mode,
                    seed,
                    args.timesteps,
                    model_path,
                    args.pdr_scale,
                )
            else:
                print(f"\nUsing existing model: {model_path}")

            eval_seed = 100_000 + seed * 10_000
            if args.environment == "simple":
                result = evaluate_fn(
                    str(model_path),
                    mode,
                    args.episodes,
                    args.pdr_scale,
                    eval_seed=eval_seed,
                )
            else:
                result = evaluate_fn(
                    str(model_path),
                    mode,
                    args.episodes,
                    pdr_scale=args.pdr_scale,
                    eval_seed=eval_seed,
                )
            result["environment"] = args.environment
            result["seed"] = seed
            result["eval_mode"] = mode
            raw_rows.append(result)

    raw_path = out_dir / "raw_results.csv"
    summary_path = out_dir / "summary.csv"
    write_csv(raw_path, raw_rows)
    write_csv(
        summary_path,
        aggregate(raw_rows, args.seeds, args.episodes, args.modes, args.timesteps),
    )
    print(f"\nsaved: {raw_path}")
    print(f"saved: {summary_path}")


if __name__ == "__main__":
    main()
