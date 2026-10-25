from __future__ import annotations

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt


RUNS_DIR = Path("runs")
OUT_DIR = Path("presentation")
SUMMARY = RUNS_DIR / "sumo_pdr_sweep_summary.csv"
COMPARE = RUNS_DIR / "sumo_stage4_3x3_comparison.csv"
# Canonical final metrics (preferred source for the README safety bar).
FINAL_SUMMARY = OUT_DIR / "테스트지표2" / "summary_final.csv"


def _read_rows(path: Path) -> list[dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def collect_pdr_sweep() -> list[dict[str, float]]:
    rows = _read_rows(SUMMARY)
    data = []
    for row in rows:
        data.append({
            "pdr_scale": float(row["pdr_scale"]),
            "collision_rate": float(row["collision_rate"]),
            "arrival_rate": float(row["arrival_rate"]),
            "near_miss_rate": float(row["near_miss_rate"]),
            "mean_final_aoi": float(row["mean_final_aoi"]),
            "mean_v2v_rx": float(row["mean_v2v_rx"]),
            "mean_reward": float(row["mean_reward"]),
        })
    return sorted(data, key=lambda x: x["pdr_scale"], reverse=True)


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
    """README bar: sensor / lossy / lossy@PDR0.3 (no perfect)."""
    # Prefer canonical final metrics; fall back to stage4 matched diagonal.
    labels: list[str]
    collision: list[float]
    near_miss: list[float]

    if FINAL_SUMMARY.exists():
        by_name = {row["scenario"]: row for row in _read_rows(FINAL_SUMMARY)}
        order = [
            ("sensor", "sensor_only"),
            ("lossy", "lossy_v2v"),
            ("lossy_pdr03", "lossy_v2v\n(pdr=0.3)"),
        ]
        missing = [key for key, _ in order if key not in by_name]
        if missing:
            raise KeyError(f"missing scenarios in {FINAL_SUMMARY}: {missing}")
        labels = [label for _, label in order]
        collision = [float(by_name[key]["collision_rate"]) * 100.0 for key, _ in order]
        near_miss = [float(by_name[key]["near_miss_rate"]) * 100.0 for key, _ in order]
    else:
        rows = _read_rows(COMPARE)
        selected = [
            row for row in rows
            if (row["train_model"], row["eval_mode"]) in {
                ("sensor_only", "sensor_only"),
                ("lossy_v2v", "lossy_v2v"),
            }
        ]
        # PDR 0.3 from sweep summary if available.
        labels = [row["eval_mode"] for row in selected]
        collision = [float(row["collision_rate"]) * 100.0 for row in selected]
        near_miss = [float(row["near_miss_rate"]) * 100.0 for row in selected]
        if SUMMARY.exists():
            sweep = {float(r["pdr_scale"]): r for r in _read_rows(SUMMARY)}
            if 0.3 in sweep:
                labels.append("lossy_v2v\n(pdr=0.3)")
                collision.append(float(sweep[0.3]["collision_rate"]) * 100.0)
                near_miss.append(float(sweep[0.3]["near_miss_rate"]) * 100.0)

    x = list(range(len(labels)))
    width = 0.36
    plt.figure(figsize=(7.2, 4.2))
    plt.bar([i - width / 2 for i in x], collision, width, label="Collision")
    plt.bar([i + width / 2 for i in x], near_miss, width, label="Near miss")
    plt.xticks(x, labels)
    plt.ylabel("Rate (%)")
    plt.title("SUMO Safety Comparison")
    plt.ylim(0, max(max(collision), max(near_miss)) * 1.2 + 1)
    plt.grid(axis="y", linestyle="--", alpha=0.35)
    plt.legend()
    plt.tight_layout()
    path = OUT_DIR / "sumo_stage4_safety_comparison.png"
    plt.savefig(path, dpi=180)
    plt.close()
    print(f"saved: {path}")


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    if not SUMMARY.exists():
        raise FileNotFoundError(f"missing {SUMMARY}; run sweep_sumo_pdr.py first")
    data = collect_pdr_sweep()
    out_summary = OUT_DIR / "sumo_pdr_sweep_summary.csv"
    with open(out_summary, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(data[0].keys()))
        writer.writeheader()
        writer.writerows(data)
    print(f"saved: {out_summary}")

    if COMPARE.exists():
        grouped_safety_bar()

    line_plot(
        data,
        "collision_rate",
        "Collision rate (%)",
        "SUMO Collision Rate vs PDR Scale",
        "sumo_pdr_collision_rate.png",
        percent=True,
    )
    line_plot(
        data,
        "near_miss_rate",
        "Near miss rate (%)",
        "SUMO Near Miss Rate vs PDR Scale",
        "sumo_pdr_near_miss_rate.png",
        percent=True,
    )
    line_plot(
        data,
        "mean_final_aoi",
        "Mean final AoI (s)",
        "SUMO AoI vs PDR Scale",
        "sumo_pdr_aoi.png",
    )
    line_plot(
        data,
        "mean_v2v_rx",
        "Mean V2V messages / episode",
        "SUMO V2V Reception vs PDR Scale",
        "sumo_pdr_v2v_rx.png",
    )


if __name__ == "__main__":
    main()
