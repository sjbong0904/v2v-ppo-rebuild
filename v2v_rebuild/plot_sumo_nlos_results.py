from __future__ import annotations

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt


RUNS = Path("runs/sumo_nlos_comparison.csv")
OUT_DIR = Path("presentation")


def main() -> None:
    if not RUNS.exists():
        raise FileNotFoundError(f"missing {RUNS}; run eval_sumo_nlos.py first")

    with open(RUNS, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    order = ["perfect_v2v", "lossy_v2v", "sensor_only"]
    los = {r["train_model"]: r for r in rows if r["scenario"] == "los"}
    nlos = {r["train_model"]: r for r in rows if r["scenario"] == "nlos"}

    labels = order
    x = list(range(len(labels)))
    width = 0.36
    los_c = [float(los[n]["collision_rate"]) * 100 for n in labels]
    nlos_c = [float(nlos[n]["collision_rate"]) * 100 for n in labels]

    os.makedirs(OUT_DIR, exist_ok=True)
    plt.figure(figsize=(7.4, 4.3))
    plt.bar([i - width / 2 for i in x], los_c, width, label="LOS (no blocker)")
    plt.bar([i + width / 2 for i in x], nlos_c, width, label="NLOS (blocker)")
    plt.xticks(x, labels)
    plt.ylabel("Collision rate (%)")
    plt.title("SUMO NLOS Occlusion: Collision Comparison")
    ymax = max(los_c + nlos_c)
    plt.ylim(0, ymax * 1.25 + 1)
    plt.grid(axis="y", linestyle="--", alpha=0.35)
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "sumo_nlos_collision_comparison.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")

    # sensor visibility fraction under NLOS
    vis = [float(nlos[n]["sensor_visible_step_frac"]) * 100 for n in labels]
    occ = [float(nlos[n]["occluded_step_frac"]) * 100 for n in labels]
    plt.figure(figsize=(7.4, 4.3))
    plt.bar([i - width / 2 for i in x], vis, width, label="Sensor visible steps")
    plt.bar([i + width / 2 for i in x], occ, width, label="Occluded steps")
    plt.xticks(x, labels)
    plt.ylabel("Step fraction (%)")
    plt.title("SUMO NLOS: Sensor Visibility vs Occlusion")
    plt.grid(axis="y", linestyle="--", alpha=0.35)
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "sumo_nlos_visibility.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")


if __name__ == "__main__":
    main()
