from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


METRICS = [
    ("collision_rate", "Collision rate"),
    ("arrival_rate", "Arrival rate"),
    ("near_miss_rate", "Near-miss rate"),
]


def read_summary(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot multiseed safety summaries.")
    parser.add_argument("--summary", required=True, help="Path to multiseed summary.csv")
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args()

    summary_path = Path(args.summary)
    rows = read_summary(summary_path)
    if not rows:
        raise ValueError(f"No rows found in {summary_path}")

    modes = [row["eval_mode"] for row in rows]
    figure, axes = plt.subplots(1, len(METRICS), figsize=(12, 4), constrained_layout=True)
    for axis, (metric, title) in zip(axes, METRICS):
        means = [float(row[f"{metric}_mean"]) for row in rows]
        errors = [float(row[f"{metric}_std"]) for row in rows]
        axis.bar(modes, means, yerr=errors, capsize=4, color=["#4472c4", "#70ad47", "#ed7d31"])
        axis.set_title(title)
        axis.set_ylim(0.0, 1.0)
        axis.set_ylabel("Rate")
        axis.tick_params(axis="x", rotation=25)
        axis.grid(axis="y", alpha=0.25)

    figure.suptitle(
        f"Matched PPO results: {summary_path.parent.name} "
        f"(mean ± standard deviation)"
    )
    output_dir = Path(args.out_dir) if args.out_dir else summary_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "multiseed_safety_summary.png"
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    print(f"saved: {output_path}")


if __name__ == "__main__":
    main()
