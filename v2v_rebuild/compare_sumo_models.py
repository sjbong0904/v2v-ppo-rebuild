from __future__ import annotations

import argparse
import csv
import os

from evaluate_sumo import evaluate


MODES = ["perfect_v2v", "lossy_v2v", "sensor_only"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        action="append",
        nargs=2,
        metavar=("NAME", "PATH"),
        required=True,
        help="Model label and .zip path. Can be passed multiple times.",
    )
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--pdr-scale", type=float, default=1.0)
    parser.add_argument("--out", default="runs/sumo_stage4_3x3_comparison.csv")
    args = parser.parse_args()

    rows = []
    for model_name, model_path in args.model:
        for mode in MODES:
            print(f"\n[{model_name}] eval_mode={mode}")
            row = evaluate(model_path, mode, args.episodes, args.pdr_scale)
            row["train_model"] = model_name
            row["eval_mode"] = mode
            row["pdr_scale"] = args.pdr_scale
            rows.append(row)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fieldnames = ["train_model", "eval_mode"] + [
        key for key in rows[0].keys() if key not in {"train_model", "eval_mode"}
    ]
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nsaved: {args.out}")


if __name__ == "__main__":
    main()
