"""Improved per-episode metric evaluation for V2V thesis claims.

Runs 6 scenarios (perfect excluded, PDR 0.3 included), 200 episodes each,
and writes per-episode CSVs plus a summary CSV.

Fixes vs teammate logs:
- first_*_ttc uses ego time-to-conflict at the event step (not legacy |arrival gap|)
- first_brake_* is captured only on the first brake action step
- perception_to_brake_latency_s is logged separately
- aggregate rates go only into summary.csv (not repeated on every episode row)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import PPO

from sumo_intersection_env import DT, SumoIntersectionEnv


BRAKE_ACTIONS = {0, 1}  # hard brake / brake
COAST_ACTION = 2
ACCEL_ACTIONS = {3, 4}

SCENARIOS: list[dict[str, Any]] = [
    {
        "name": "sensor",
        "model": "runs/sumo_ppo_50k_sensor.zip",
        "mode": "sensor_only",
        "multi_npc": False,
        "pdr_scale": 1.0,
    },
    {
        "name": "multi_sensor",
        "model": "runs/sumo_ppo_50k_multi_sensor.zip",
        "mode": "sensor_only",
        "multi_npc": True,
        "pdr_scale": 1.0,
    },
    {
        "name": "lossy",
        "model": "runs/sumo_ppo_50k_lossy.zip",
        "mode": "lossy_v2v",
        "multi_npc": False,
        "pdr_scale": 1.0,
    },
    {
        "name": "multi_lossy",
        "model": "runs/sumo_ppo_50k_multi_lossy.zip",
        "mode": "lossy_v2v",
        "multi_npc": True,
        "pdr_scale": 1.0,
    },
    {
        "name": "lossy_pdr03",
        "model": "runs/sumo_ppo_50k_lossy.zip",
        "mode": "lossy_v2v",
        "multi_npc": False,
        "pdr_scale": 0.3,
    },
    {
        "name": "multi_lossy_pdr03",
        "model": "runs/sumo_ppo_50k_multi_lossy.zip",
        "mode": "lossy_v2v",
        "multi_npc": True,
        "pdr_scale": 0.3,
    },
]

EPISODE_FIELDS = [
    "scenario",
    "episode",
    "seed",
    "observation_mode",
    "multi_npc",
    "pdr_scale",
    "episode_return",
    "episode_length_steps",
    "episode_length_seconds",
    "collision",
    "successful_passage",
    "near_miss",
    "min_distance_m",
    "min_arrival_gap_s",
    "min_ego_ttc_s",
    # event snapshots (captured at the event step)
    "first_valid_perception_step",
    "first_valid_perception_distance_m",
    "first_valid_perception_ego_ttc_s",
    "first_valid_perception_arrival_gap_s",
    "first_sensor_detection_step",
    "first_sensor_detection_distance_m",
    "first_sensor_detection_ego_ttc_s",
    "first_sensor_detection_arrival_gap_s",
    "first_brake_step",
    "first_brake_distance_m",
    "first_brake_ego_ttc_s",
    "first_brake_arrival_gap_s",
    "perception_to_brake_latency_s",
    "v2v_early_before_sensor",
    # channel / behavior aggregates
    "sensor_visible_step_fraction",
    "occluded_step_fraction",
    "v2v_rx_count",
    "v2v_rx_step_fraction",
    "mean_aoi_s",
    "mean_pdr",
    "mean_ego_speed_mps",
    "mean_target_speed_mps",
    "mean_npc_speed_mps",
    "brake_action_fraction",
    "coast_action_fraction",
    "accelerate_action_fraction",
    "ego_speed_scenario_mps",
    "target_speed_scenario_mps",
    "target_start_offset_m",
    "npc_speed_scenario_mps",
]


def _nan() -> float:
    return float("nan")


def _finite(x: Any) -> bool:
    try:
        return x is not None and math.isfinite(float(x))
    except (TypeError, ValueError):
        return False


@dataclass
class EpisodeTracker:
    """Capture first-event metrics at the correct step (not episode finals)."""

    first_valid_perception_step: int | None = None
    first_valid_perception_distance_m: float = field(default_factory=_nan)
    first_valid_perception_ego_ttc_s: float = field(default_factory=_nan)
    first_valid_perception_arrival_gap_s: float = field(default_factory=_nan)

    first_sensor_detection_step: int | None = None
    first_sensor_detection_distance_m: float = field(default_factory=_nan)
    first_sensor_detection_ego_ttc_s: float = field(default_factory=_nan)
    first_sensor_detection_arrival_gap_s: float = field(default_factory=_nan)

    first_brake_step: int | None = None
    first_brake_distance_m: float = field(default_factory=_nan)
    first_brake_ego_ttc_s: float = field(default_factory=_nan)
    first_brake_arrival_gap_s: float = field(default_factory=_nan)

    min_arrival_gap_s: float = field(default_factory=lambda: float("inf"))
    min_ego_ttc_s: float = field(default_factory=lambda: float("inf"))

    sensor_visible_steps: int = 0
    occluded_steps: int = 0
    v2v_rx_steps: int = 0
    aoi_sum: float = 0.0
    pdr_sum: float = 0.0
    ego_speed_sum: float = 0.0
    target_speed_sum: float = 0.0
    npc_speed_sum: float = 0.0
    npc_speed_n: int = 0
    brake_steps: int = 0
    coast_steps: int = 0
    accel_steps: int = 0
    total_steps: int = 0

    def observe(self, step_idx: int, action: int | None, info: dict[str, Any]) -> None:
        distance = float(info.get("distance", info.get("min_distance", _nan())))
        ego_ttc = float(info.get("ego_ttc", _nan()))
        arrival_gap = float(info.get("arrival_gap", info.get("ttc", _nan())))

        if _finite(arrival_gap):
            self.min_arrival_gap_s = min(self.min_arrival_gap_s, arrival_gap)
        if _finite(ego_ttc):
            self.min_ego_ttc_s = min(self.min_ego_ttc_s, ego_ttc)

        visible = bool(info.get("visible", False))
        sensor_visible = bool(info.get("sensor_visible", False))

        if visible and self.first_valid_perception_step is None:
            self.first_valid_perception_step = step_idx
            self.first_valid_perception_distance_m = distance
            self.first_valid_perception_ego_ttc_s = ego_ttc
            self.first_valid_perception_arrival_gap_s = arrival_gap

        if sensor_visible and self.first_sensor_detection_step is None:
            self.first_sensor_detection_step = step_idx
            self.first_sensor_detection_distance_m = distance
            self.first_sensor_detection_ego_ttc_s = ego_ttc
            self.first_sensor_detection_arrival_gap_s = arrival_gap

        if action is not None:
            self.total_steps += 1
            self.sensor_visible_steps += int(sensor_visible)
            self.occluded_steps += int(bool(info.get("occluded", False)))
            self.v2v_rx_steps += int(bool(info.get("v2v_rx_this_step", False)))
            self.aoi_sum += float(info.get("aoi", 0.0))
            self.pdr_sum += float(info.get("pdr", 0.0))
            self.ego_speed_sum += float(info.get("ego_speed", 0.0))
            self.target_speed_sum += float(info.get("target_speed", 0.0) or 0.0)
            npc = info.get("npc_speed", None)
            if npc is not None and _finite(npc):
                self.npc_speed_sum += float(npc)
                self.npc_speed_n += 1

            a = int(action)
            if a in BRAKE_ACTIONS:
                self.brake_steps += 1
                if self.first_brake_step is None:
                    self.first_brake_step = step_idx
                    self.first_brake_distance_m = distance
                    self.first_brake_ego_ttc_s = ego_ttc
                    self.first_brake_arrival_gap_s = arrival_gap
            elif a == COAST_ACTION:
                self.coast_steps += 1
            elif a in ACCEL_ACTIONS:
                self.accel_steps += 1

    def perception_to_brake_latency_s(self) -> float:
        if self.first_valid_perception_step is None or self.first_brake_step is None:
            return _nan()
        return (self.first_brake_step - self.first_valid_perception_step) * DT

    def v2v_early_before_sensor(self) -> bool:
        if self.first_valid_perception_step is None:
            return False
        if self.first_sensor_detection_step is None:
            return True
        return self.first_valid_perception_step < self.first_sensor_detection_step


def _safe_div(num: float, den: float) -> float:
    return float(num / den) if den else 0.0


def run_episode(
    env: SumoIntersectionEnv,
    model: PPO,
    *,
    scenario_name: str,
    episode: int,
    seed: int,
) -> dict[str, Any]:
    tracker = EpisodeTracker()
    obs, _ = env.reset(seed=seed)
    # Capture perception that may already be available at t=0 (after reset V2V).
    tracker.observe(step_idx=0, action=None, info=env._build_info(collision=False, arrived=False))

    done = False
    ep_return = 0.0
    steps = 0
    info: dict[str, Any] = {}

    while not done:
        action, _ = model.predict(obs, deterministic=True)
        action_i = int(action)
        obs, reward, terminated, truncated, info = env.step(action_i)
        steps += 1
        ep_return += float(reward)
        tracker.observe(step_idx=steps, action=action_i, info=info)
        done = bool(terminated or truncated)

    collision = bool(info.get("collision", False))
    arrived = bool(info.get("arrived", False))
    successful = bool(arrived and not collision)
    n = max(tracker.total_steps, 1)

    min_gap = tracker.min_arrival_gap_s
    if not math.isfinite(min_gap):
        min_gap = _nan()
    min_ego = tracker.min_ego_ttc_s
    if not math.isfinite(min_ego):
        min_ego = _nan()

    return {
        "scenario": scenario_name,
        "episode": episode,
        "seed": seed,
        "observation_mode": env.observation_mode,
        "multi_npc": bool(env.multi_npc),
        "pdr_scale": float(env.pdr_scale),
        "episode_return": ep_return,
        "episode_length_steps": steps,
        "episode_length_seconds": steps * DT,
        "collision": collision,
        "successful_passage": successful,
        "near_miss": bool(info.get("near_miss", False)),
        "min_distance_m": float(info.get("min_distance", _nan())),
        "min_arrival_gap_s": min_gap,
        "min_ego_ttc_s": min_ego,
        "first_valid_perception_step": tracker.first_valid_perception_step
        if tracker.first_valid_perception_step is not None
        else "",
        "first_valid_perception_distance_m": tracker.first_valid_perception_distance_m,
        "first_valid_perception_ego_ttc_s": tracker.first_valid_perception_ego_ttc_s,
        "first_valid_perception_arrival_gap_s": tracker.first_valid_perception_arrival_gap_s,
        "first_sensor_detection_step": tracker.first_sensor_detection_step
        if tracker.first_sensor_detection_step is not None
        else "",
        "first_sensor_detection_distance_m": tracker.first_sensor_detection_distance_m,
        "first_sensor_detection_ego_ttc_s": tracker.first_sensor_detection_ego_ttc_s,
        "first_sensor_detection_arrival_gap_s": tracker.first_sensor_detection_arrival_gap_s,
        "first_brake_step": tracker.first_brake_step if tracker.first_brake_step is not None else "",
        "first_brake_distance_m": tracker.first_brake_distance_m,
        "first_brake_ego_ttc_s": tracker.first_brake_ego_ttc_s,
        "first_brake_arrival_gap_s": tracker.first_brake_arrival_gap_s,
        "perception_to_brake_latency_s": tracker.perception_to_brake_latency_s(),
        "v2v_early_before_sensor": tracker.v2v_early_before_sensor(),
        "sensor_visible_step_fraction": _safe_div(tracker.sensor_visible_steps, n),
        "occluded_step_fraction": _safe_div(tracker.occluded_steps, n),
        "v2v_rx_count": int(info.get("v2v_rx_count", 0)),
        "v2v_rx_step_fraction": _safe_div(tracker.v2v_rx_steps, n),
        "mean_aoi_s": _safe_div(tracker.aoi_sum, n),
        "mean_pdr": _safe_div(tracker.pdr_sum, n),
        "mean_ego_speed_mps": _safe_div(tracker.ego_speed_sum, n),
        "mean_target_speed_mps": _safe_div(tracker.target_speed_sum, n),
        "mean_npc_speed_mps": _safe_div(tracker.npc_speed_sum, tracker.npc_speed_n)
        if tracker.npc_speed_n
        else "",
        "brake_action_fraction": _safe_div(tracker.brake_steps, n),
        "coast_action_fraction": _safe_div(tracker.coast_steps, n),
        "accelerate_action_fraction": _safe_div(tracker.accel_steps, n),
        "ego_speed_scenario_mps": float(info.get("scenario_ego_speed", _nan())),
        "target_speed_scenario_mps": float(info.get("scenario_target_speed", _nan())),
        "target_start_offset_m": float(info.get("scenario_target_offset", _nan())),
        "npc_speed_scenario_mps": info.get("scenario_npc_speed", "")
        if info.get("scenario_npc_speed") is not None
        else "",
    }


def _quantile(values: list[float], q: float) -> float:
    if not values:
        return _nan()
    return float(np.quantile(np.asarray(values, dtype=float), q))


def summarize_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    def col(name: str) -> list[float]:
        out: list[float] = []
        for r in rows:
            v = r.get(name, "")
            if v == "" or v is None:
                continue
            try:
                fv = float(v)
            except (TypeError, ValueError):
                continue
            if math.isfinite(fv):
                out.append(fv)
        return out

    n = len(rows)
    coll = [bool(r["collision"]) for r in rows]
    succ = [bool(r["successful_passage"]) for r in rows]
    early = [bool(r["v2v_early_before_sensor"]) for r in rows]
    near = [bool(r["near_miss"]) for r in rows]

    def rate(flags: list[bool]) -> float:
        return sum(flags) / n if n else _nan()

    dist = col("min_distance_m")
    ego_ttc_first = col("first_valid_perception_ego_ttc_s")
    brake_ttc = col("first_brake_ego_ttc_s")
    perc_dist = col("first_valid_perception_distance_m")
    latency = col("perception_to_brake_latency_s")

    return {
        "scenario": rows[0]["scenario"] if rows else "",
        "n": n,
        "observation_mode": rows[0]["observation_mode"] if rows else "",
        "multi_npc": rows[0]["multi_npc"] if rows else "",
        "pdr_scale": rows[0]["pdr_scale"] if rows else "",
        "collision_rate": rate(coll),
        "collision_count": int(sum(coll)),
        "successful_passage_rate": rate(succ),
        "near_miss_rate": rate(near),
        "v2v_early_before_sensor_rate": rate(early),
        "first_valid_perception_distance_mean_m": float(np.mean(perc_dist)) if perc_dist else _nan(),
        "first_valid_perception_distance_median_m": _quantile(perc_dist, 0.5),
        "first_valid_perception_ego_ttc_mean_s": float(np.mean(ego_ttc_first)) if ego_ttc_first else _nan(),
        "first_valid_perception_ego_ttc_median_s": _quantile(ego_ttc_first, 0.5),
        "first_brake_ego_ttc_mean_s": float(np.mean(brake_ttc)) if brake_ttc else _nan(),
        "first_brake_ego_ttc_median_s": _quantile(brake_ttc, 0.5),
        "perception_to_brake_latency_mean_s": float(np.mean(latency)) if latency else _nan(),
        "perception_to_brake_latency_median_s": _quantile(latency, 0.5),
        "min_distance_mean_m": float(np.mean(dist)) if dist else _nan(),
        "min_distance_median_m": _quantile(dist, 0.5),
        "min_distance_p05_m": _quantile(dist, 0.05),
        "min_distance_p95_m": _quantile(dist, 0.95),
        "mean_v2v_rx": float(np.mean(col("v2v_rx_count"))) if col("v2v_rx_count") else 0.0,
        "mean_aoi_s": float(np.mean(col("mean_aoi_s"))) if col("mean_aoi_s") else _nan(),
        "mean_pdr": float(np.mean(col("mean_pdr"))) if col("mean_pdr") else _nan(),
        "mean_brake_fraction": float(np.mean(col("brake_action_fraction")))
        if col("brake_action_fraction")
        else _nan(),
        "mean_return": float(np.mean(col("episode_return"))) if col("episode_return") else _nan(),
        "mean_steps": float(np.mean(col("episode_length_steps")))
        if col("episode_length_steps")
        else _nan(),
    }


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            out = {}
            for k in fieldnames:
                v = row.get(k, "")
                if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
                    out[k] = ""
                else:
                    out[k] = v
            writer.writerow(out)


def run_scenario(
    cfg: dict[str, Any],
    *,
    episodes: int,
    seed_start: int,
    out_dir: Path,
    sensor_range: float,
) -> list[dict[str, Any]]:
    model_path = Path(cfg["model"])
    if not model_path.is_file():
        raise FileNotFoundError(f"Model not found: {model_path}")

    print(
        f"\n=== {cfg['name']} | mode={cfg['mode']} multi={cfg['multi_npc']} "
        f"pdr={cfg['pdr_scale']} episodes={episodes} ==="
    )
    model = PPO.load(str(model_path))
    env = SumoIntersectionEnv(
        observation_mode=cfg["mode"],
        pdr_scale=float(cfg["pdr_scale"]),
        sensor_range=sensor_range,
        multi_npc=bool(cfg["multi_npc"]),
        seed=seed_start,
    )

    rows: list[dict[str, Any]] = []
    try:
        for i in range(episodes):
            seed = seed_start + i
            row = run_episode(
                env,
                model,
                scenario_name=cfg["name"],
                episode=i + 1,
                seed=seed,
            )
            rows.append(row)
            if (i + 1) % 20 == 0 or i == 0 or i + 1 == episodes:
                print(
                    f"  ep {i+1}/{episodes} seed={seed} "
                    f"coll={row['collision']} success={row['successful_passage']} "
                    f"perc_dist={row['first_valid_perception_distance_m']:.1f} "
                    f"perc_ego_ttc={row['first_valid_perception_ego_ttc_s']:.2f} "
                    f"brake_ego_ttc={row['first_brake_ego_ttc_s'] if _finite(row['first_brake_ego_ttc_s']) else 'NA'} "
                    f"latency={row['perception_to_brake_latency_s'] if _finite(row['perception_to_brake_latency_s']) else 'NA'}"
                )
    finally:
        env.close()

    out_path = out_dir / f"{cfg['name']}_results.csv"
    write_csv(out_path, rows, EPISODE_FIELDS)
    print(f"saved: {out_path}")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run improved 6-scenario metric evaluation (200 eps default)."
    )
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument(
        "--out-dir",
        default="presentation/테스트지표_v2",
        help="Output directory for per-scenario CSVs + summary",
    )
    parser.add_argument("--sensor-range", type=float, default=35.0)
    parser.add_argument(
        "--scenarios",
        nargs="*",
        default=None,
        help="Optional subset of scenario names",
    )
    parser.add_argument(
        "--smoke",
        type=int,
        default=0,
        help="If >0, override episodes to this small number for a quick check",
    )
    args = parser.parse_args()

    episodes = args.smoke if args.smoke > 0 else args.episodes
    out_dir = Path(args.out_dir)
    selected = SCENARIOS
    if args.scenarios:
        wanted = set(args.scenarios)
        selected = [s for s in SCENARIOS if s["name"] in wanted]
        missing = wanted - {s["name"] for s in selected}
        if missing:
            raise SystemExit(f"Unknown scenarios: {sorted(missing)}")

    all_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    for cfg in selected:
        rows = run_scenario(
            cfg,
            episodes=episodes,
            seed_start=args.seed_start,
            out_dir=out_dir,
            sensor_range=args.sensor_range,
        )
        all_rows.extend(rows)
        summaries.append(summarize_rows(rows))

    if all_rows:
        write_csv(out_dir / "all_episodes.csv", all_rows, EPISODE_FIELDS)
    if summaries:
        summary_fields = list(summaries[0].keys())
        write_csv(out_dir / "summary.csv", summaries, summary_fields)
        print("\n=== SUMMARY ===")
        for s in summaries:
            print(
                f"{s['scenario']:18s} coll={s['collision_rate']*100:5.1f}% "
                f"success={s['successful_passage_rate']*100:5.1f}% "
                f"early={s['v2v_early_before_sensor_rate']*100:5.1f}% "
                f"perc_dist={s['first_valid_perception_distance_mean_m']:6.1f}m "
                f"perc_ego_ttc={s['first_valid_perception_ego_ttc_mean_s']:5.2f}s "
                f"brake_ego_ttc={s['first_brake_ego_ttc_mean_s']:5.2f}s "
                f"latency={s['perception_to_brake_latency_mean_s']:5.2f}s"
            )
        print(f"\nsaved: {out_dir / 'summary.csv'}")


if __name__ == "__main__":
    # Avoid OpenMP / SUMO thread contention surprises on Windows.
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    main()
