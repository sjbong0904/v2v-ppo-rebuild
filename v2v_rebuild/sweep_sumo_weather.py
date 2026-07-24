from __future__ import annotations

import argparse
import csv
import os

from evaluate_sumo import evaluate


# Weather proxy: shorter onboard sensor range (fog/rain).
RANGES = [35.0, 25.0, 15.0, 10.0]

MODELS = [
    ("perfect_v2v", "runs/sumo_ppo_50k_perfect.zip", "perfect_v2v"),
    ("lossy_v2v", "runs/sumo_ppo_50k_lossy.zip", "lossy_v2v"),
    ("sensor_only", "runs/sumo_ppo_50k_sensor.zip", "sensor_only"),
]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SUMO weather proxy sweep via sensor_range reduction"
    )
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--out", default="runs/sumo_weather_sensor_range_sweep.csv")
    args = parser.parse_args()

    rows: list[dict[str, float | int | str]] = []
    for train_name, model_path, mode in MODELS:
        for sensor_range in RANGES:
            print(f"\n=== {train_name} sensor_range={sensor_range} ===")
            row = evaluate(
                model_path,
                mode,
                args.episodes,
                pdr_scale=1.0,
                sensor_range=sensor_range,
            )
            row["train_model"] = train_name
            row["eval_mode"] = mode
            rows.append(row)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nsaved: {args.out}")


if __name__ == "__main__":
    main()
