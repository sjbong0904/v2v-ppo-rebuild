from __future__ import annotations

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt


RUNS = Path("runs/sumo_multi_npc_comparison.csv")
OUT_DIR = Path("presentation")


def main() -> None:
    if not RUNS.exists():
        raise FileNotFoundError(f"missing {RUNS}")

    with open(RUNS, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    order = ["perfect_v2v", "lossy_v2v", "sensor_only"]
    by_name = {r["train_model"]: r for r in rows}
    labels = order
    collision = [float(by_name[n]["collision_rate"]) * 100 for n in labels]
    near = [float(by_name[n]["near_miss_rate"]) * 100 for n in labels]

    x = list(range(len(labels)))
    width = 0.36
    os.makedirs(OUT_DIR, exist_ok=True)
    plt.figure(figsize=(7.4, 4.3))
    plt.bar([i - width / 2 for i in x], collision, width, label="Collision")
    plt.bar([i + width / 2 for i in x], near, width, label="Near miss")
    plt.xticks(x, labels)
    plt.ylabel("Rate (%)")
    plt.title("Intermediate Multi-NPC SUMO Safety")
    plt.ylim(0, max(max(collision), max(near)) * 1.25 + 1)
    plt.grid(axis="y", linestyle="--", alpha=0.35)
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "sumo_multi_npc_safety.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")


if __name__ == "__main__":
    main()
