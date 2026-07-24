from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys


MODELS = [
    ("perfect_v2v", "runs/sumo_ppo_50k_multi_perfect.zip"),
    ("sensor_only", "runs/sumo_ppo_50k_multi_sensor.zip"),
    ("lossy_v2v", "runs/sumo_ppo_50k_multi_lossy.zip"),
]


def main() -> None:
    parser = argparse.ArgumentParser(description="Train+eval intermediate multi-NPC SUMO")
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--skip-train", action="store_true")
    parser.add_argument("--out", default="runs/sumo_multi_npc_comparison.csv")
    args = parser.parse_args()

    if not args.skip_train:
        for mode, out_path in MODELS:
            print(f"\n=== train multi_npc {mode} ===")
            subprocess.run(
                [
                    sys.executable,
                    "train_sumo_ppo.py",
                    "--timesteps", str(args.timesteps),
                    "--mode", mode,
                    "--multi-npc",
                    "--out", out_path,
                    "--seed", "42",
                ],
                check=True,
            )

    from evaluate_sumo import evaluate

    rows: list[dict[str, float | int | str]] = []
    for mode, model_path in MODELS:
        print(f"\n=== eval multi_npc {mode} ===")
        row = evaluate(
            model_path,
            mode,
            args.episodes,
            pdr_scale=1.0,
            sensor_range=35.0,
            multi_npc=True,
        )
        row["train_model"] = mode
        row["eval_mode"] = mode
        row["scenario"] = "multi_npc"
        rows.append(row)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nsaved: {args.out}")


if __name__ == "__main__":
    main()
