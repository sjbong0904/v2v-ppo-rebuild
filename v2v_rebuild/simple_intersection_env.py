from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces


DT = 0.1
TARGET_SPEED = 12.0
MAX_SPEED = 16.0
EGO_START_Y = -80.0
EGO_GOAL_Y = 45.0
TARGET_START_X = 65.0
TARGET_Y = 0.0
COLLISION_RADIUS = 4.5
NEAR_MISS_RADIUS = 8.0
MAX_STEPS = 700
V2V_MAX_RANGE = 120.0
BSM_INTERVAL = 0.1
AOI_TIMEOUT = 1.0

ACTION_ACCEL = {
    0: -8.0,
    1: -4.0,
    2: 0.0,
    3: 2.0,
    4: 4.0,
}


@dataclass
class Scenario:
    ego_speed: float
    target_speed: float
    target_start_x: float


class SimpleIntersectionEnv(gym.Env):
    """Small kinematic crossing scenario for reward/debug-first PPO work.

    Observation:
        [ego_y, ego_speed, target_x, target_speed, signed_time_gap, visible]

    Coordinate system:
        Ego moves north along x=0.
        Target moves west along y=0.
        The conflict point is (0, 0).
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        observation_mode: str = "perfect_v2v",
        sensor_range: float = 35.0,
        pdr_scale: float = 1.0,
        seed: int | None = None,
    ):
        super().__init__()
        if observation_mode not in {"perfect_v2v", "sensor_only", "lossy_v2v"}:
            raise ValueError(
                "observation_mode must be 'perfect_v2v', 'sensor_only', or 'lossy_v2v'"
            )

        self.observation_mode = observation_mode
        self.sensor_range = sensor_range
        self.pdr_scale = pdr_scale
        self.rng = np.random.default_rng(seed)

        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(
            low=np.array([-1.0, 0.0, -1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.ego_y = EGO_START_Y
        self.ego_speed = TARGET_SPEED
        self.target_x = TARGET_START_X
        self.target_speed = TARGET_SPEED
        self.steps = 0
        self.min_distance = float("inf")
        self.near_miss = False
        self.prev_action = 2
        self.last_bsm: dict[str, float] | None = None
        self.bsm_aoi = AOI_TIMEOUT
        self.v2v_rx_count = 0

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        scenario = self._sample_scenario()
        self.ego_y = EGO_START_Y
        self.ego_speed = scenario.ego_speed
        self.target_x = scenario.target_start_x
        self.target_speed = scenario.target_speed
        self.steps = 0
        self.min_distance = float("inf")
        self.near_miss = False
        self.prev_action = 2
        self.last_bsm = None
        self.bsm_aoi = AOI_TIMEOUT
        self.v2v_rx_count = 0
        return self._get_obs(), {}

    def step(self, action: int):
        action = int(action)
        accel = ACTION_ACCEL[action]

        self.ego_speed = float(np.clip(self.ego_speed + accel * DT, 0.0, MAX_SPEED))
        self.ego_y += self.ego_speed * DT
        self.target_x -= self.target_speed * DT
        self.steps += 1
        self._update_v2v_message()

        distance = self._distance_to_target()
        self.min_distance = min(self.min_distance, distance)
        if distance < NEAR_MISS_RADIUS:
            self.near_miss = True

        collision = distance <= COLLISION_RADIUS
        arrived = self.ego_y >= EGO_GOAL_Y
        timeout = self.steps >= MAX_STEPS
        terminated = collision or arrived
        truncated = timeout and not terminated

        reward = self._reward(action, collision, arrived)
        self.prev_action = action
        info = {
            "collision": collision,
            "arrived": arrived,
            "near_miss": self.near_miss,
            "min_distance": self.min_distance,
            "ttc": self._ttc_to_conflict(),
            "visible": self._target_visible(),
            "aoi": self.bsm_aoi,
            "pdr": self._calc_pdr(),
            "v2v_rx_count": self.v2v_rx_count,
        }
        return self._get_obs(), reward, terminated, truncated, info

    def rule_based_action(self) -> int:
        """Simple TTC policy used as a sanity-check oracle."""
        ttc = self._ttc_to_conflict()
        signed_gap = self._signed_time_gap()
        visible = self._target_visible()
        if visible and signed_gap > -0.3 and ttc < 3.0:
            return 0
        if visible and signed_gap > 0.0 and ttc < 5.0:
            return 1
        if visible and signed_gap <= -0.3 and ttc < 3.0:
            return 4
        if self.ego_speed < TARGET_SPEED:
            return 3
        return 2

    def _sample_scenario(self) -> Scenario:
        return Scenario(
            ego_speed=float(self.rng.uniform(10.0, 14.0)),
            target_speed=float(self.rng.uniform(9.0, 13.5)),
            target_start_x=float(self.rng.uniform(55.0, 75.0)),
        )

    def _target_visible(self) -> bool:
        if self.observation_mode == "perfect_v2v":
            return True
        if self.observation_mode == "lossy_v2v":
            return self._sensor_visible() or self._fresh_bsm_available()
        return self._sensor_visible()

    def _sensor_visible(self) -> bool:
        return self._distance_to_target() <= self.sensor_range

    def _fresh_bsm_available(self) -> bool:
        return self.last_bsm is not None and self.bsm_aoi <= AOI_TIMEOUT

    def _calc_pdr(self) -> float:
        distance = self._distance_to_target()
        if distance > V2V_MAX_RANGE:
            return 0.0
        base = 1.0 - 0.75 * (distance / V2V_MAX_RANGE)
        return float(np.clip(base * self.pdr_scale, 0.05, 1.0))

    def _update_v2v_message(self) -> None:
        if self.observation_mode != "lossy_v2v":
            self.bsm_aoi = 0.0 if self.observation_mode == "perfect_v2v" else AOI_TIMEOUT
            return

        self.bsm_aoi = min(self.bsm_aoi + DT, AOI_TIMEOUT * 2.0)
        if self._sensor_visible():
            return

        if self.rng.random() <= self._calc_pdr():
            self.last_bsm = {
                "target_x": self.target_x,
                "target_speed": self.target_speed,
                "signed_gap": self._signed_time_gap(),
            }
            self.bsm_aoi = 0.0
            self.v2v_rx_count += 1

    def _distance_to_target(self) -> float:
        return float(np.hypot(self.target_x, TARGET_Y - self.ego_y))

    def _ttc_to_conflict(self) -> float:
        ego_t = (0.0 - self.ego_y) / max(self.ego_speed, 0.1)
        target_t = self.target_x / max(self.target_speed, 0.1)
        if ego_t < 0.0 or target_t < 0.0:
            return 10.0
        return float(np.clip(abs(ego_t - target_t), 0.0, 10.0))

    def _signed_time_gap(self) -> float:
        """Return ego_arrival_time - target_arrival_time at the conflict point."""
        ego_t = (0.0 - self.ego_y) / max(self.ego_speed, 0.1)
        target_t = self.target_x / max(self.target_speed, 0.1)
        if ego_t < 0.0 or target_t < 0.0:
            return 10.0
        return float(np.clip(ego_t - target_t, -10.0, 10.0))

    def _get_obs(self) -> np.ndarray:
        visible = self._target_visible()
        if self.observation_mode == "lossy_v2v" and not self._sensor_visible() and self._fresh_bsm_available():
            target_x = float(self.last_bsm["target_x"]) if self.last_bsm else TARGET_START_X
            target_speed = float(self.last_bsm["target_speed"]) if self.last_bsm else 0.0
            signed_gap = float(self.last_bsm["signed_gap"]) if self.last_bsm else 10.0
        else:
            target_x = self.target_x if visible else TARGET_START_X
            target_speed = self.target_speed if visible else 0.0
            signed_gap = self._signed_time_gap() if visible else 10.0

        return np.array(
            [
                np.clip(self.ego_y / 100.0, -1.0, 1.0),
                np.clip(self.ego_speed / MAX_SPEED, 0.0, 1.0),
                np.clip(target_x / 100.0, -1.0, 1.0),
                np.clip(target_speed / MAX_SPEED, 0.0, 1.0),
                np.clip(signed_gap / 10.0, -1.0, 1.0),
                1.0 if visible else 0.0,
            ],
            dtype=np.float32,
        )

    def _reward(self, action: int, collision: bool, arrived: bool) -> float:
        if collision:
            return -100.0
        if arrived:
            return 50.0

        ttc = self._ttc_to_conflict()
        signed_gap = self._signed_time_gap()
        visible = self._target_visible()
        reward = -0.02

        target_first_conflict = signed_gap > -0.3 and ttc < 3.0
        ego_first_conflict = signed_gap <= -0.3 and ttc < 3.0

        target_has_cleared = self.target_x < -COLLISION_RADIUS
        ego_is_waiting = self.ego_speed < 1.0

        if visible and target_first_conflict and not target_has_cleared:
            if ego_is_waiting:
                if action in {2, 3}:
                    reward += 1.0
                else:
                    reward -= 1.5
            elif action <= 1:
                reward += 3.0
            elif action == 2:
                reward -= 2.0
            else:
                reward -= 6.0
        elif visible and ego_first_conflict:
            if action >= 3:
                reward += 2.0
            elif action == 2:
                reward -= 0.5
            else:
                reward -= 3.0
        elif visible and signed_gap > 0.0 and ttc < 5.0:
            if action <= 1:
                reward += 1.0
            elif action >= 3:
                reward -= 1.5
        else:
            if self.ego_speed < TARGET_SPEED * 0.7:
                reward -= 1.0
                if action >= 3:
                    reward += 1.0
            elif action in {3, 4} and self.ego_speed < TARGET_SPEED:
                reward += 0.3
            elif action == 0:
                reward -= 0.8

        action_change = abs(action - self.prev_action)
        reward -= 0.05 * action_change

        progress = self.ego_speed * DT / max(EGO_GOAL_Y - EGO_START_Y, 1.0)
        reward += 3.0 * progress
        return float(np.clip(reward, -10.0, 10.0))
