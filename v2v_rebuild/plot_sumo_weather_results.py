from __future__ import annotations

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt


RUNS = Path("runs/sumo_weather_sensor_range_sweep.csv")
OUT_DIR = Path("presentation")


def main() -> None:
    if not RUNS.exists():
        raise FileNotFoundError(f"missing {RUNS}; run sweep_sumo_weather.py first")

    with open(RUNS, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    by_model: dict[str, list[tuple[float, float, float]]] = {}
    for row in rows:
        name = row["train_model"]
        by_model.setdefault(name, []).append((
            float(row["sensor_range"]),
            float(row["collision_rate"]) * 100.0,
            float(row["near_miss_rate"]) * 100.0,
        ))

    os.makedirs(OUT_DIR, exist_ok=True)

    plt.figure(figsize=(7.2, 4.2))
    for name, points in by_model.items():
        points = sorted(points, key=lambda x: x[0], reverse=True)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        plt.plot(xs, ys, marker="o", linewidth=2.2, label=name)
    plt.gca().invert_xaxis()
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.xlabel("Sensor range (m)  [lower = worse weather]")
    plt.ylabel("Collision rate (%)")
    plt.title("SUMO Weather Proxy: Collision vs Sensor Range")
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "sumo_weather_collision_rate.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")

    plt.figure(figsize=(7.2, 4.2))
    for name, points in by_model.items():
        points = sorted(points, key=lambda x: x[0], reverse=True)
        xs = [p[0] for p in points]
        ys = [p[2] for p in points]
        plt.plot(xs, ys, marker="o", linewidth=2.2, label=name)
    plt.gca().invert_xaxis()
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.xlabel("Sensor range (m)  [lower = worse weather]")
    plt.ylabel("Near miss rate (%)")
    plt.title("SUMO Weather Proxy: Near Miss vs Sensor Range")
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "sumo_weather_near_miss_rate.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")


if __name__ == "__main__":
    main()
