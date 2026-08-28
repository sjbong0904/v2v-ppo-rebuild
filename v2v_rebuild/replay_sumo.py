from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

from stable_baselines3 import PPO

from sumo_intersection_env import SumoIntersectionEnv, find_sumo_binary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay one SUMO episode and save the control/visibility trace."
    )
    parser.add_argument("--model", default=None, help="PPO .zip path. Omit to replay the rule policy.")
    parser.add_argument(
        "--mode",
        default="perfect_v2v",
        choices=["perfect_v2v", "lossy_v2v", "sensor_only"],
    )
    parser.add_argument("--episode-seed", type=int, default=7)
    parser.add_argument("--pdr-scale", type=float, default=1.0)
    parser.add_argument("--sensor-range", type=float, default=35.0)
    parser.add_argument("--nlos-blocker", action="store_true")
    parser.add_argument("--multi-npc", action="store_true")
    parser.add_argument("--gui", action="store_true", help="Use sumo-gui instead of headless SUMO.")
    parser.add_argument("--delay", type=float, default=0.0, help="Seconds to wait between GUI steps.")
    parser.add_argument("--out", default="runs/replay/sumo_replay.csv")
    args = parser.parse_args()

    sumo_binary = find_sumo_binary("sumo-gui" if args.gui else "sumo")
    env = SumoIntersectionEnv(
        observation_mode=args.mode,
        pdr_scale=args.pdr_scale,
        sensor_range=args.sensor_range,
        nlos_blocker=args.nlos_blocker,
        multi_npc=args.multi_npc,
        seed=args.episode_seed,
        sumo_binary=sumo_binary,
    )
    model = PPO.load(args.model) if args.model else None
    rows: list[dict[str, object]] = []

    try:
        obs, _ = env.reset(seed=args.episode_seed)
        done = False
        step = 0
        while not done:
            action = env.rule_based_action() if model is None else model.predict(obs, deterministic=True)[0]
            action = int(action)
            next_obs, reward, terminated, truncated, info = env.step(action)
            rows.append(
                {
                    "episode_seed": args.episode_seed,
                    "step": step,
                    "action": action,
                    "reward": float(reward),
                    "ego_y": float(env.ego_y),
                    "ego_speed": float(env.ego_speed),
                    "target_x": float(env.target_x),
                    "target_speed": float(env.target_speed),
                    "visible": int(info.get("visible", False)),
                    "sensor_visible": int(info.get("sensor_visible", False)),
                    "occluded": int(info.get("occluded", False)),
                    "aoi": float(info.get("aoi", 0.0)),
                    "v2v_rx_count": int(info.get("v2v_rx_count", 0)),
                    "min_distance": float(info.get("min_distance", 0.0)),
                    "collision": int(info.get("collision", False)),
                    "arrived": int(info.get("arrived", False)),
                    "near_miss": int(info.get("near_miss", False)),
                }
            )
            obs = next_obs
            done = terminated or truncated
            step += 1
            if args.delay > 0:
                time.sleep(args.delay)
    finally:
        env.close()

    output_path = Path(args.out)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved: {output_path}")
    print(
        f"mode={args.mode} model={args.model or 'rule'} "
        f"seed={args.episode_seed} steps={len(rows)} "
        f"collision={rows[-1]['collision']} arrived={rows[-1]['arrived']}"
    )


if __name__ == "__main__":
    main()
