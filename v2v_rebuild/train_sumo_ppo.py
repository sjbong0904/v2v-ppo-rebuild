from __future__ import annotations

import argparse
import os

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from sumo_intersection_env import SumoIntersectionEnv


RUN_DIR = "runs"
MODEL_PATH = os.path.join(RUN_DIR, "sumo_ppo.zip")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=50_000)
    parser.add_argument(
        "--mode",
        default="perfect_v2v",
        choices=["perfect_v2v", "lossy_v2v", "sensor_only"],
    )
    parser.add_argument("--pdr-scale", type=float, default=1.0)
    parser.add_argument("--sensor-range", type=float, default=35.0)
    parser.add_argument("--nlos-blocker", action="store_true")
    parser.add_argument("--multi-npc", action="store_true")
    parser.add_argument("--out", default=MODEL_PATH)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--tensorboard", action="store_true")
    args = parser.parse_args()

    out_dir = os.path.dirname(args.out) or "."
    os.makedirs(out_dir, exist_ok=True)

    raw_env = SumoIntersectionEnv(
        observation_mode=args.mode,
        pdr_scale=args.pdr_scale,
        sensor_range=args.sensor_range,
        nlos_blocker=args.nlos_blocker,
        multi_npc=args.multi_npc,
        seed=args.seed,
    )
    env = Monitor(raw_env)

    try:
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=1024,
            batch_size=128,
            gamma=0.98,
            gae_lambda=0.95,
            ent_coef=0.01,
            verbose=1,
            tensorboard_log=os.path.join(out_dir, "tb") if args.tensorboard else None,
            seed=args.seed,
        )
        model.learn(total_timesteps=args.timesteps)
        model.save(args.out)
        print(f"saved: {args.out}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
