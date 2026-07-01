from __future__ import annotations

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt


RUNS_DIR = Path("runs")
OUT_DIR = Path("presentation")

PDR_FILES = {
    1.0: RUNS_DIR / "stage3_3x3_comparison.csv",
    0.5: RUNS_DIR / "stage3_lossy_pdr_0_5.csv",
    0.25: RUNS_DIR / "stage3_lossy_pdr_0_25.csv",
    0.1: RUNS_DIR / "stage3_lossy_pdr_0_1.csv",
}


def _read_rows(path: Path) -> list[dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _lossy_row(path: Path) -> dict[str, str]:
    rows = _read_rows(path)
    for row in rows:
        if row["train_model"] == "lossy_v2v" and row["eval_mode"] == "lossy_v2v":
            return row
    raise RuntimeError(f"lossy_v2v row not found: {path}")


def collect_pdr_sweep() -> list[dict[str, float]]:
    data = []
    for pdr_scale, path in PDR_FILES.items():
        row = _lossy_row(path)
        data.append({
            "pdr_scale": pdr_scale,
            "collision_rate": float(row["collision_rate"]),
            "arrival_rate": float(row["arrival_rate"]),
            "near_miss_rate": float(row["near_miss_rate"]),
            "mean_final_aoi": float(row["mean_final_aoi"]),
            "mean_v2v_rx": float(row["mean_v2v_rx"]),
            "mean_reward": float(row["mean_reward"]),
        })
    return sorted(data, key=lambda x: x["pdr_scale"], reverse=True)


def save_summary_csv(data: list[dict[str, float]]) -> None:
    OUT_DIR.mkdir(exist_ok=True)
    path = OUT_DIR / "stage3_pdr_sweep_summary.csv"
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(data[0].keys()))
        writer.writeheader()
        writer.writerows(data)
    print(f"saved: {path}")


def line_plot(
    data: list[dict[str, float]],
    y_key: str,
    ylabel: str,
    title: str,
    out_name: str,
    percent: bool = False,
) -> None:
    xs = [row["pdr_scale"] for row in data]
    ys = [row[y_key] * 100.0 if percent else row[y_key] for row in data]

    plt.figure(figsize=(7.2, 4.2))
    plt.plot(xs, ys, marker="o", linewidth=2.5)
    plt.gca().invert_xaxis()
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.xlabel("PDR scale (higher is better)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.tight_layout()
    path = OUT_DIR / out_name
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")


def grouped_safety_bar() -> None:
    rows = _read_rows(RUNS_DIR / "stage3_3x3_comparison.csv")
    selected = [
        row for row in rows
        if (row["train_model"], row["eval_mode"]) in {
            ("perfect_v2v", "perfect_v2v"),
            ("lossy_v2v", "lossy_v2v"),
            ("sensor_only", "sensor_only"),
        }
    ]
    labels = [row["eval_mode"] for row in selected]
    collision = [float(row["collision_rate"]) * 100.0 for row in selected]
    near_miss = [float(row["near_miss_rate"]) * 100.0 for row in selected]

    x = list(range(len(labels)))
    width = 0.36
    plt.figure(figsize=(7.2, 4.2))
    plt.bar([i - width / 2 for i in x], collision, width, label="Collision")
    plt.bar([i + width / 2 for i in x], near_miss, width, label="Near miss")
    plt.xticks(x, labels)
    plt.ylabel("Rate (%)")
    plt.title("Stage 3 Safety Comparison")
    plt.ylim(0, max(max(collision), max(near_miss)) * 1.2 + 1)
    plt.grid(axis="y", linestyle="--", alpha=0.35)
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "stage3_safety_comparison.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    data = collect_pdr_sweep()
    save_summary_csv(data)
    grouped_safety_bar()
    line_plot(
        data,
        "collision_rate",
        "Collision rate (%)",
        "Collision Rate vs PDR Scale",
        "stage3_pdr_collision_rate.png",
        percent=True,
    )
    line_plot(
        data,
        "near_miss_rate",
        "Near miss rate (%)",
        "Near Miss Rate vs PDR Scale",
        "stage3_pdr_near_miss_rate.png",
        percent=True,
    )
    line_plot(
        data,
        "mean_final_aoi",
        "Mean final AoI (s)",
        "AoI vs PDR Scale",
        "stage3_pdr_aoi.png",
    )
    line_plot(
        data,
        "mean_v2v_rx",
        "Mean V2V messages / episode",
        "V2V Reception vs PDR Scale",
        "stage3_pdr_v2v_rx.png",
    )


if __name__ == "__main__":
    main()

