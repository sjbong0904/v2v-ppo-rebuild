from __future__ import annotations

import argparse
import csv
import os

from evaluate_sumo import evaluate


SCALES = [1.0, 0.5, 0.25, 0.1]


def scale_tag(scale: float) -> str:
    return str(scale).replace(".", "_")


def main() -> None:
    parser = argparse.ArgumentParser(description="SUMO lossy_v2v PDR scale sweep")
    parser.add_argument("--model", default="runs/sumo_ppo_50k_lossy.zip")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--out-dir", default="runs")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    summary_rows: list[dict[str, float | int | str]] = []

    for scale in SCALES:
        print(f"\n=== SUMO lossy_v2v pdr_scale={scale} ===")
        row = evaluate(args.model, "lossy_v2v", args.episodes, pdr_scale=scale)
        row["train_model"] = "lossy_v2v"
        row["eval_mode"] = "lossy_v2v"
        row["pdr_scale"] = scale
        summary_rows.append(row)

        out_csv = os.path.join(args.out_dir, f"sumo_lossy_pdr_{scale_tag(scale)}.csv")
        with open(out_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            writer.writeheader()
            writer.writerow(row)
        print(f"saved: {out_csv}")

    summary_path = os.path.join(args.out_dir, "sumo_pdr_sweep_summary.csv")
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        writer.writeheader()
        writer.writerows(summary_rows)
    print(f"\nsaved: {summary_path}")


if __name__ == "__main__":
    main()
