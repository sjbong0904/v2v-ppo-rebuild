# /home/ubuntu/v2x-project/new_v2v/train_ppo_0512.py
import os
import math
import time
import glob
import logging
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import (
    BaseCallback, CheckpointCallback, CallbackList
)
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

from v2x_env_0512 import RealWorldV2XEnv

DATE_PREFIX = "0512"
SUMO_CONFIG_PATH = "../sumo_data_multi/map.sumocfg"
CHECKPOINT_DIR = f"./checkpoints_{DATE_PREFIX}"
BEST_MODEL_DIR = f"./best_model_{DATE_PREFIX}"
LOG_DIR = f"./logs_{DATE_PREFIX}"
TOTAL_TIMESTEPS = 2000000
SEED = 42
SMOOTH_WINDOW = 50  # 시각화 이동평균 윈도우 크기

TRAIN_LOG_FILE = f"{DATE_PREFIX}_train_log.log"
OUTPUT_CSV_FILE = f"{DATE_PREFIX}_metrics.csv"
PLOT_REWARD_FILE = f"{DATE_PREFIX}_plot_1_reward.png"
PLOT_LENGTH_FILE = f"{DATE_PREFIX}_plot_2_length.png"
PLOT_ENTROPY_FILE = f"{DATE_PREFIX}_plot_3_entropy.png"
PLOT_LOSS_FILE = f"{DATE_PREFIX}_plot_4_loss.png"
PLOT_VALUE_FILE = f"{DATE_PREFIX}_plot_5_value_loss.png"
PLOT_KL_FILE = f"{DATE_PREFIX}_plot_6_approx_kl.png"

MODEL_PREFIX = DATE_PREFIX
FINAL_MODEL_NAME = f"{DATE_PREFIX}_fin"
INTERRUPT_MODEL_NAME = f"{DATE_PREFIX}_interrupted"
PRETRAINED_PATH = None  # 이어 학습 시 체크포인트 경로 지정 ex) "./checkpoints_0512/0512_500000_steps"

# 로거
logger = logging.getLogger(f"train_ppo_{DATE_PREFIX}")
logger.setLevel(logging.INFO)

_fh = logging.FileHandler(TRAIN_LOG_FILE, mode="w")
_fh.setLevel(logging.INFO)
_fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))

_sh = logging.StreamHandler()
_sh.setLevel(logging.WARNING)
_sh.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))

logger.addHandler(_fh)
logger.addHandler(_sh)


def cosine_schedule(initial_value: float, floor_ratio: float = 0.2):
    # 학습률 코사인 감쇠 스케줄러. floor_ratio 비율까지 점진적으로 감소
    def func(progress_remaining: float) -> float:
        progress = 1.0 - progress_remaining
        cosine_decay = 0.5 * (1.0 + math.cos(math.pi * progress))
        decayed = (1.0 - floor_ratio) * cosine_decay + floor_ratio
        return initial_value * decayed
    return func


def initial_cleanup():
    # 학습 시작 전 잔여 Mininet 프로세스 정리
    os.system("sudo mn -c > /dev/null 2>&1")
    time.sleep(1.0)


def get_latest_log_dir(base_dir: str = LOG_DIR):
    # 가장 최근에 수정된 PPO 로그 디렉토리 반환
    if not os.path.exists(base_dir):
        return None
    dirs = [d for d in glob.glob(os.path.join(base_dir, "PPO_*"))
            if os.path.isdir(d)]
    return max(dirs, key=os.path.getmtime) if dirs else None


class NavIntentCallback(BaseCallback):
    # 에피소드 종료 시마다 의도 분포, 충돌률, 최근 50회 평균 보상 로그로 남김

    def __init__(self, train_env: Monitor, log_interval: int = 10, verbose: int = 0):
        super().__init__(verbose)
        self._train_env = train_env
        self._log_interval = log_interval
        self._episode_count = 0
        self._intent_counts = {"straight": 0, "left": 0, "right": 0}
        self._collision_count = 0
        self._ep_rewards = []

    def _get_inner_env(self) -> RealWorldV2XEnv:
        env = self._train_env
        while hasattr(env, "env"):
            env = env.env
        return env

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])

        for done, info in zip(dones, infos):
            if not done:
                continue

            self._episode_count += 1
            inner = self._get_inner_env()
            cur_intent = inner.nav_intent

            if cur_intent in self._intent_counts:
                self._intent_counts[cur_intent] += 1

            if info.get("collision", False):
                self._collision_count += 1

            ep_rew = info.get("episode", {}).get("r", None)
            if ep_rew is not None:
                self._ep_rewards.append(ep_rew)

            # log_interval 에피소드마다 통계 출력
            if self._episode_count % self._log_interval == 0:
                cnt = self._intent_counts
                recent_rews = self._ep_rewards[-50:] if self._ep_rewards else [0]
                mean_rew = np.mean(recent_rews)
                collision_rate = (self._collision_count / self._episode_count
                                  if self._episode_count > 0 else 0.0)
                logger.info(
                    f"[ep={self._episode_count}] "
                    f"intent={cur_intent} | "
                    f"S={cnt['straight']} L={cnt['left']} R={cnt['right']} | "
                    f"mean_rew(최근50)={mean_rew:.2f} | "
                    f"누적충돌률={collision_rate:.1%}"
                )
        return True

# 시각화(테스트용)
def _plot_metric(df: pd.DataFrame, tag: str, title: str,
                 ylabel: str, color: str, out_path: str,
                 window: int = SMOOTH_WINDOW):
    sub = df[df["Tag"] == tag].copy()
    if sub.empty:
        logger.warning(f"태그 '{tag}' 데이터 없음. 건너뜀.")
        return
    plt.figure(figsize=(12, 5))
    plt.plot(sub["Step"], sub["Value"],
             color=color, alpha=0.25, linewidth=0.8, label="Raw")
    smoothed = sub["Value"].rolling(window=window, min_periods=1).mean()
    plt.plot(sub["Step"], smoothed,
             color=color, linewidth=2.0, label=f"MA-{window}")
    plt.title(title, fontsize=14, fontweight="bold")
    plt.xlabel("Steps", fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend(fontsize=10)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    logger.info(f" {out_path} 저장 완료")


def merge_and_visualize_logs(log_dir: str, output_csv: str):
    logger.info(f"[1/4] 로그: {log_dir}")
    event_files = [f for f in os.listdir(log_dir)
                   if "events.out.tfevents" in f]
    if not event_files:
        logger.error(f"이벤트 파일 없음: {log_dir}")
        return

    all_data = []
    for fname in event_files:
        ea = EventAccumulator(os.path.join(log_dir, fname),
                              size_guidance={"scalars": 0})
        ea.Reload()
        for tag in ea.Tags().get("scalars", []):
            events = ea.Scalars(tag)
            all_data.append(pd.DataFrame({
                "Step": [e.step for e in events],
                "Value": [e.value for e in events],
                "Wall_Time": [e.wall_time for e in events],
                "Tag": tag,
            }))

    if not all_data:
        logger.error("추출할 스칼라 데이터 없음")
        return

    df = pd.concat(all_data, ignore_index=True)
    df.sort_values(["Tag", "Step"], inplace=True)
    df.drop_duplicates(subset=["Tag", "Step"], keep="last", inplace=True)  # 중복 스텝 제거

    logger.info(f"[3/4] CSV 저장: {output_csv} ({len(df):,} 행)")
    df.to_csv(output_csv, index=False)

    logger.info("[4/4] 차트 생성")
    _plot_metric(df, "rollout/ep_rew_mean", "Episode Reward Mean", "Mean Reward", "tab:blue", PLOT_REWARD_FILE)
    _plot_metric(df, "rollout/ep_len_mean", "Episode Length Mean", "Steps / Episode", "tab:green", PLOT_LENGTH_FILE)
    _plot_metric(df, "train/entropy_loss", "Entropy Loss", "Entropy Loss", "tab:orange", PLOT_ENTROPY_FILE)
    _plot_metric(df, "train/loss", "Total Training Loss", "Loss", "tab:red", PLOT_LOSS_FILE)
    _plot_metric(df, "train/value_loss", "Value Function Loss", "Value Loss", "tab:purple", PLOT_VALUE_FILE)
    _plot_metric(df, "train/approx_kl", "Approx KL Divergence", "KL Divergence", "tab:brown", PLOT_KL_FILE)

    logger.info("완료")
    print(f" CSV : {output_csv}")
    print(f" 차트 : {PLOT_REWARD_FILE} ~ {PLOT_KL_FILE}")


if __name__ == "__main__":

    initial_cleanup()
    for d in [CHECKPOINT_DIR, BEST_MODEL_DIR, LOG_DIR]:
        os.makedirs(d, exist_ok=True)

    train_env = Monitor(
        RealWorldV2XEnv(
            config_file=SUMO_CONFIG_PATH,
            nav_intent="random",
            randomize_intent=True,
            build_network=False,
        ),
        filename=os.path.join(LOG_DIR, f"monitor_train_{DATE_PREFIX}.csv"),
    )

    # 50k 스텝마다 체크포인트 저장
    checkpoint_cb = CheckpointCallback(
        save_freq=50000,
        save_path=CHECKPOINT_DIR,
        name_prefix=MODEL_PREFIX,
        verbose=0,
    )
    nav_intent_cb = NavIntentCallback(train_env=train_env, log_interval=10, verbose=0)
    callbacks = CallbackList([checkpoint_cb, nav_intent_cb])

    if PRETRAINED_PATH:
        # 이어 학습: 기존 가중치 로드 후 학습
        model = PPO.load(
            PRETRAINED_PATH,
            env=train_env,
            tensorboard_log=LOG_DIR,
        )
        logger.info(f"체크포인트 로드: {PRETRAINED_PATH}")
        print(f">>> 이어 학습 : {PRETRAINED_PATH}")
    else:
        # 신규 학습: PPO 하이퍼파라미터 설정
        model = PPO(
            policy="MlpPolicy",
            env=train_env,
            verbose=0,
            seed=SEED,
            learning_rate=cosine_schedule(3e-4, floor_ratio=0.1),
            n_steps=4096, # rollout 버퍼 크기
            batch_size=256,
            n_epochs=10, # PPO 업데이트 반복 횟수
            ent_coef=0.02, # 탐험 촉진 엔트로피 계수
            vf_coef=0.7, # value function 손실 가중치
            max_grad_norm=0.5,
            clip_range=0.2, # PPO clipping Range
            gae_lambda=0.95,
            policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256])),
            tensorboard_log=LOG_DIR,
        )

    logger.info("=" * 64)
    logger.info(f" {DATE_PREFIX} 학습 시작 | seed={SEED} | total_steps={TOTAL_TIMESTEPS:,}")
    logger.info("=" * 64)

    print(f" >>> {DATE_PREFIX} 학습 시작 (총 {TOTAL_TIMESTEPS:,} 스텝)")
    print(f" 로그 : {TRAIN_LOG_FILE}")
    print(f" 체크포인트 : {CHECKPOINT_DIR}/ (50k 스텝마다)")
    print(f" TensorBoard : tensorboard --logdir {LOG_DIR}")

    interrupted = False
    try:
        t0 = time.time()
        model.learn(
            total_timesteps=TOTAL_TIMESTEPS,
            callback=callbacks,
            progress_bar=True,
        )
        elapsed = (time.time() - t0) / 60
        final_path = os.path.join(CHECKPOINT_DIR, FINAL_MODEL_NAME)
        model.save(final_path)
        logger.info(f"학습 완료 | {elapsed:.1f}분 소요 | 저장: {final_path}")
        print(f">>> 학습 완료 | 소요 시간: {elapsed:.1f}분")
        print(f"최종 모델: {final_path}.zip")

    except KeyboardInterrupt:
        interrupted = True
        intr_path = os.path.join(CHECKPOINT_DIR, INTERRUPT_MODEL_NAME)
        model.save(intr_path)
        logger.warning(f"학습 중단 | 저장: {intr_path}")
        print(f">>> 학습 중단 | 저장: {intr_path}.zip")

    finally:
        train_env.close()
        logger.info("환경 종료 완료")

    latest_log = get_latest_log_dir()
    if not latest_log:
        logger.error("로그 디렉토리 없음. 시각화 건너뜀.")
    elif interrupted:
        # 중단된 경우 시각화 수동 실행 안내
        print(f">>> 시각화 수동 실행:")
        print(f"from train_ppo_{DATE_PREFIX} import merge_and_visualize_logs")
        print(f"merge_and_visualize_logs('{latest_log}', '{OUTPUT_CSV_FILE}')")
    else:
        print(">>> TensorBoard 데이터 추출 및 차트 생성 중...")
        merge_and_visualize_logs(latest_log, OUTPUT_CSV_FILE)
        print(">>> 모든 작업 완료.")
