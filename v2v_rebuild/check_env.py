from __future__ import annotations

import argparse
import csv
import os

from simple_intersection_env import SimpleIntersectionEnv


POLICIES = ["random", "hold", "full", "rule"]
MODES = ["perfect_v2v", "lossy_v2v", "sensor_only"]


def choose_action(env: SimpleIntersectionEnv, name: str) -> int:
    if name == "rule":
        return env.rule_based_action()
    if name == "hold":
        return 2
    if name == "full":
        return 4
    return env.action_space.sample()


def run_policy(
    name: str,
    mode: str,
    episodes: int = 200,
) -> tuple[dict[str, float], list[dict[str, float | int | str]]]:
    env = SimpleIntersectionEnv(observation_mode=mode, seed=7)
    collisions = 0
    arrivals = 0
    near_misses = 0
    total_reward = 0.0
    min_distances = []
    episode_rows = []

    for ep in range(episodes):
        _, _ = env.reset(seed=ep)
        done = False
        ep_reward = 0.0
        info = {}
        steps = 0
        while not done:
            action = choose_action(env, name)
            _, reward, terminated, truncated, info = env.step(action)
            ep_reward += reward
            steps += 1
            done = terminated or truncated

        collisions += int(info.get("collision", False))
        arrivals += int(info.get("arrived", False))
        near_misses += int(info.get("near_miss", False))
        min_distance = float(info.get("min_distance", 0.0))
        min_distances.append(min_distance)
        total_reward += ep_reward
        episode_rows.append({
            "mode": mode,
            "policy": name,
            "episode": ep,
            "steps": steps,
            "collision": int(info.get("collision", False)),
            "arrived": int(info.get("arrived", False)),
            "near_miss": int(info.get("near_miss", False)),
            "min_distance": min_distance,
            "reward": ep_reward,
        })

    summary = {
        "mode": mode,
        "policy": name,
        "episodes": episodes,
        "collision_rate": collisions / episodes,
        "arrival_rate": arrivals / episodes,
        "near_miss_rate": near_misses / episodes,
        "mean_min_distance": sum(min_distances) / episodes,
        "mean_reward": total_reward / episodes,
    }
    return summary, episode_rows


def write_csv(path: str, rows: list[dict[str, float | int | str]]) -> None:
    if not rows:
        return
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--out-dir", default="runs/baseline")
    args = parser.parse_args()

    summaries = []
    details = []
    for mode in MODES:
        for policy in POLICIES:
            summary, rows = run_policy(policy, mode, args.episodes)
            summaries.append(summary)
            details.extend(rows)
            print(f"{mode:12s} {policy:6s} {summary}")

    write_csv(os.path.join(args.out_dir, "summary.csv"), summaries)
    write_csv(os.path.join(args.out_dir, "episodes.csv"), details)
    print(f"saved: {args.out_dir}")
