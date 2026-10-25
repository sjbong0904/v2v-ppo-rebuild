from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import traci
from stable_baselines3 import PPO

from sumo_intersection_env import EGO_ID, TARGET_ID, SumoIntersectionEnv, find_sumo_binary


ACTION_NAME = {
    0: "HARD_BRK",
    1: "BRAKE",
    2: "HOLD",
    3: "ACCEL",
    4: "FULL_ACC",
}

# RGBA 0-255 for SUMO vehicle coloring (visibility cue in GUI).
COLOR_VISIBLE = (40, 200, 80, 255)
COLOR_OCCLUDED = (220, 160, 40, 255)
COLOR_BLIND = (220, 60, 60, 255)
COLOR_TARGET = (60, 100, 255, 255)


def _bool_tag(flag: bool, yes: str = "Y", no: str = "N") -> str:
    return yes if flag else no


def _print_hud(
    step: int,
    action: int,
    reward: float,
    env: SumoIntersectionEnv,
    info: dict,
) -> None:
    vis = bool(info.get("visible", False))
    sens = bool(info.get("sensor_visible", False))
    occ = bool(info.get("occluded", False))
    lines = [
        "",
        f"── step {step:4d}  t={step * 0.1:5.1f}s  action={ACTION_NAME.get(action, action):8s}  "
        f"reward={reward:+6.2f}",
        f"  ego    y={env.ego_y:7.1f}m  v={env.ego_speed:5.2f}m/s",
        f"  threat x={env.target_x:7.1f}m  v={env.target_speed:5.2f}m/s  "
        f"id={info.get('primary_threat', TARGET_ID)}",
        f"  dist={float(info.get('min_distance', 0.0)):6.2f}m  "
        f"ttc={float(info.get('ttc', 0.0)):5.2f}s",
        f"  visible={_bool_tag(vis)}  sensor={_bool_tag(sens)}  "
        f"occluded={_bool_tag(occ)}  "
        f"aoi={float(info.get('aoi', 0.0)):4.2f}s  "
        f"v2v_rx={int(info.get('v2v_rx_count', 0))}",
        f"  collision={_bool_tag(bool(info.get('collision')))}  "
        f"arrived={_bool_tag(bool(info.get('arrived')))}  "
        f"near_miss={_bool_tag(bool(info.get('near_miss')))}",
    ]
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def _update_gui_colors(info: dict) -> None:
    """Tint ego by current sensing state so GUI matches the HUD."""
    ids = set(traci.vehicle.getIDList())
    if EGO_ID not in ids:
        return
    if info.get("sensor_visible"):
        color = COLOR_VISIBLE
    elif info.get("occluded"):
        color = COLOR_OCCLUDED
    else:
        color = COLOR_BLIND
    traci.vehicle.setColor(EGO_ID, color)
    if TARGET_ID in ids:
        traci.vehicle.setColor(TARGET_ID, COLOR_TARGET)


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
    parser.add_argument(
        "--hud",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Print live metrics each step (default: on when --gui).",
    )
    parser.add_argument(
        "--colorize",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Color ego by sensor state in GUI (default: on when --gui).",
    )
    parser.add_argument("--out", default="runs/replay/sumo_replay.csv")
    args = parser.parse_args()

    show_hud = args.gui if args.hud is None else args.hud
    colorize = args.gui if args.colorize is None else args.colorize
    if args.delay <= 0 and args.gui:
        args.delay = 0.08

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

    if show_hud:
        print(
            f"HUD on | mode={args.mode} nlos={int(args.nlos_blocker)} "
            f"multi_npc={int(args.multi_npc)} model={args.model or 'rule'}"
        )
        print(
            "ego color: green=sensor sees | orange=occluded | red=blind/out of range"
        )
        if args.gui:
            print("SUMO-GUI: press Play if the simulation is paused.")

    try:
        obs, _ = env.reset(seed=args.episode_seed)
        done = False
        step = 0
        while not done:
            action = (
                env.rule_based_action()
                if model is None
                else int(model.predict(obs, deterministic=True)[0])
            )
            action = int(action)
            next_obs, reward, terminated, truncated, info = env.step(action)
            row = {
                "episode_seed": args.episode_seed,
                "step": step,
                "action": action,
                "action_name": ACTION_NAME.get(action, str(action)),
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
                "ttc": float(info.get("ttc", 0.0)),
                "collision": int(info.get("collision", False)),
                "arrived": int(info.get("arrived", False)),
                "near_miss": int(info.get("near_miss", False)),
            }
            rows.append(row)
            if colorize:
                try:
                    _update_gui_colors(info)
                except Exception:
                    pass
            if show_hud:
                _print_hud(step, action, float(reward), env, info)
            obs = next_obs
            done = terminated or truncated
            step += 1
            if args.delay > 0:
                time.sleep(args.delay)
    finally:
        env.close()

    if not rows:
        print("no steps recorded")
        return

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
