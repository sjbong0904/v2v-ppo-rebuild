from __future__ import annotations

import argparse
import csv
import os

from stable_baselines3 import PPO

from sumo_intersection_env import SumoIntersectionEnv


def evaluate(
    model_path: str | None,
    mode: str,
    episodes: int,
    pdr_scale: float = 1.0,
) -> dict[str, float | int | str]:
    env = SumoIntersectionEnv(observation_mode=mode, pdr_scale=pdr_scale, seed=123)
    model = PPO.load(model_path) if model_path else None

    collisions = 0
    arrivals = 0
    near_misses = 0
    total_reward = 0.0
    min_distances: list[float] = []
    action_counts = [0, 0, 0, 0, 0]
    total_steps = 0
    episode_steps: list[int] = []
    aoi_values: list[float] = []
    pdr_values: list[float] = []
    v2v_rx_total = 0

    try:
        for ep in range(episodes):
            obs, _ = env.reset(seed=ep)
            done = False
            ep_reward = 0.0
            info: dict = {}
            steps = 0
            while not done:
                if model is None:
                    action = env.rule_based_action()
                else:
                    action, _ = model.predict(obs, deterministic=True)
                action_counts[int(action)] += 1
                total_steps += 1
                obs, reward, terminated, truncated, info = env.step(int(action))
                ep_reward += reward
                steps += 1
                done = terminated or truncated

            collisions += int(info.get("collision", False))
            arrivals += int(info.get("arrived", False))
            near_misses += int(info.get("near_miss", False))
            min_distances.append(float(info.get("min_distance", 0.0)))
            total_reward += ep_reward
            episode_steps.append(steps)
            aoi_values.append(float(info.get("aoi", 0.0)))
            pdr_values.append(float(info.get("pdr", 0.0)))
            v2v_rx_total += int(info.get("v2v_rx_count", 0))
    finally:
        env.close()

    result: dict[str, float | int | str] = {
        "model": model_path or "rule",
        "mode": mode,
        "episodes": episodes,
        "collision_rate": collisions / episodes,
        "arrival_rate": arrivals / episodes,
        "near_miss_rate": near_misses / episodes,
        "mean_min_distance": sum(min_distances) / len(min_distances),
        "mean_steps": sum(episode_steps) / len(episode_steps),
        "mean_reward": total_reward / episodes,
        "mean_final_aoi": sum(aoi_values) / len(aoi_values),
        "mean_final_pdr": sum(pdr_values) / len(pdr_values),
        "mean_v2v_rx": v2v_rx_total / episodes,
        "act_hard_brake_pct": action_counts[0] / max(total_steps, 1),
        "act_brake_pct": action_counts[1] / max(total_steps, 1),
        "act_hold_pct": action_counts[2] / max(total_steps, 1),
        "act_accel_pct": action_counts[3] / max(total_steps, 1),
        "act_full_accel_pct": action_counts[4] / max(total_steps, 1),
    }
    for key, value in result.items():
        if isinstance(value, float):
            print(f"{key:17s}: {value:.3f}")
        else:
            print(f"{key:17s}: {value}")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=None, help="Path to PPO .zip model. Omit for rule policy.")
    parser.add_argument(
        "--mode",
        default="perfect_v2v",
        choices=["perfect_v2v", "lossy_v2v", "sensor_only"],
    )
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--pdr-scale", type=float, default=1.0)
    parser.add_argument("--csv", default=None)
    args = parser.parse_args()
    result = evaluate(args.model, args.mode, args.episodes, args.pdr_scale)
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(result.keys()))
            writer.writeheader()
            writer.writerow(result)
        print(f"saved: {args.csv}")


if __name__ == "__main__":
    main()
