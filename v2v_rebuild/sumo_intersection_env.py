from __future__ import annotations

import os
import shutil
import socket
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
import traci
from gymnasium import spaces


DT = 0.1
TARGET_SPEED = 12.0
MAX_SPEED = 16.0
EGO_START_OFFSET = 80.0  # meters before junction on S_in
EGO_GOAL_Y = 45.0
TARGET_START_OFFSET = 65.0  # meters before junction on E_in
COLLISION_RADIUS = 4.5
NEAR_MISS_RADIUS = 8.0
MAX_STEPS = 700
V2V_MAX_RANGE = 120.0
AOI_TIMEOUT = 1.0
EGO_ID = "ego"
TARGET_ID = "target"

ACTION_ACCEL = {
    0: -8.0,
    1: -4.0,
    2: 0.0,
    3: 2.0,
    4: 4.0,
}

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SUMOCFG = os.path.join(HERE, "sumo_data", "map.sumocfg")


@dataclass
class Scenario:
    ego_speed: float
    target_speed: float
    target_start_offset: float


def _get_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", 0))
        return int(sock.getsockname()[1])


def find_sumo_binary(preferred: str = "sumo") -> str:
    """Resolve sumo / sumo-gui binary from PATH or SUMO_HOME / eclipse-sumo."""
    if preferred and os.path.isfile(preferred):
        return preferred

    which = shutil.which(preferred)
    if which:
        return which

    env_home = os.environ.get("SUMO_HOME", "").strip()
    names = [preferred, f"{preferred}.exe"]
    search_roots: list[str] = []
    if env_home:
        search_roots.append(os.path.join(env_home, "bin"))

    try:
        import sumo as sumo_pkg

        pkg_home = getattr(sumo_pkg, "SUMO_HOME", None) or os.path.dirname(sumo_pkg.__file__)
        search_roots.append(os.path.join(pkg_home, "bin"))
        search_roots.append(pkg_home)
        if not os.environ.get("SUMO_HOME"):
            os.environ["SUMO_HOME"] = pkg_home
    except Exception:
        pass

    for root in search_roots:
        for name in names:
            path = os.path.join(root, name)
            if os.path.isfile(path):
                return path

    raise FileNotFoundError(
        f"SUMO binary '{preferred}' not found. Install SUMO / eclipse-sumo "
        "and set SUMO_HOME, or pass sumo_binary=..."
    )


class SumoIntersectionEnv(gym.Env):
    """Minimal SUMO port of SimpleIntersectionEnv (ego S->N, target E->W).

    Observation:
        [ego_y_norm, ego_speed, target_x_norm, target_speed, signed_time_gap, visible]

    V2V loss is simulated (no Mininet): perfect_v2v / sensor_only / lossy_v2v.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        observation_mode: str = "perfect_v2v",
        sensor_range: float = 35.0,
        pdr_scale: float = 1.0,
        seed: int | None = None,
        sumo_binary: str = "sumo",
        config_file: str = DEFAULT_SUMOCFG,
        sumo_port: int | None = None,
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
        self.sumo_binary = find_sumo_binary(sumo_binary)
        self.config_file = os.path.abspath(config_file)
        self.sumo_port = sumo_port
        self.traci_started = False

        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(
            low=np.array([-1.0, 0.0, -1.0, 0.0, -1.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        self.ego_y = -EGO_START_OFFSET
        self.ego_speed = TARGET_SPEED
        self.target_x = TARGET_START_OFFSET
        self.target_speed = TARGET_SPEED
        self.steps = 0
        self.min_distance = float("inf")
        self.near_miss = False
        self.prev_action = 2
        self.last_bsm: dict[str, float] | None = None
        self.bsm_aoi = AOI_TIMEOUT
        self.v2v_rx_count = 0
        self._scenario = Scenario(TARGET_SPEED, TARGET_SPEED, TARGET_START_OFFSET)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self._close_traci()
        self._scenario = self._sample_scenario()
        self._start_sumo()
        self._spawn_vehicles()

        self.steps = 0
        self.min_distance = float("inf")
        self.near_miss = False
        self.prev_action = 2
        self.last_bsm = None
        self.bsm_aoi = AOI_TIMEOUT
        self.v2v_rx_count = 0
        self._sync_kinematics()
        return self._get_obs(), {}

    def step(self, action: int):
        action = int(action)
        accel = ACTION_ACCEL[action]

        if EGO_ID not in traci.vehicle.getIDList():
            info = self._empty_info(collision=False, arrived=True)
            return self._get_obs(), 50.0, True, False, info

        self.ego_speed = float(np.clip(self.ego_speed + accel * DT, 0.0, MAX_SPEED))
        traci.vehicle.setSpeed(EGO_ID, self.ego_speed)

        if TARGET_ID in traci.vehicle.getIDList():
            traci.vehicle.setSpeed(TARGET_ID, self.target_speed)

        traci.simulationStep()
        self.steps += 1
        self._sync_kinematics()
        self._update_v2v_message()

        distance = self._distance_to_target()
        self.min_distance = min(self.min_distance, distance)
        if distance < NEAR_MISS_RADIUS:
            self.near_miss = True

        colliding = set(traci.simulation.getCollidingVehiclesIDList())
        collision = (
            distance <= COLLISION_RADIUS
            or EGO_ID in colliding
            or TARGET_ID in colliding
        )
        arrived = self.ego_y >= EGO_GOAL_Y or EGO_ID not in traci.vehicle.getIDList()
        if collision:
            arrived = False
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

    def close(self):
        self._close_traci()
        super().close()

    def _sample_scenario(self) -> Scenario:
        return Scenario(
            ego_speed=float(self.rng.uniform(10.0, 14.0)),
            target_speed=float(self.rng.uniform(9.0, 13.5)),
            target_start_offset=float(self.rng.uniform(55.0, 75.0)),
        )

    def _start_sumo(self) -> None:
        if not os.path.isfile(self.config_file):
            raise FileNotFoundError(f"SUMO config not found: {self.config_file}")
        net_file = os.path.join(os.path.dirname(self.config_file), "map.net.xml")
        if not os.path.isfile(net_file):
            raise FileNotFoundError(
                f"Network file missing: {net_file}. Run: python sumo_data/build_net.py"
            )

        port = self.sumo_port if self.sumo_port is not None else _get_free_port()
        cmd = [
            self.sumo_binary,
            "-c", self.config_file,
            "--start",
            "--quit-on-end",
            "--step-length", str(DT),
            "--collision.action", "warn",
            "--collision.check-junctions", "true",
            "--time-to-teleport", "-1",
            "--no-warnings", "true",
            "--no-step-log", "true",
        ]
        traci.start(cmd, port=port, numRetries=10)
        self.traci_started = True

    def _spawn_vehicles(self) -> None:
        scenario = self._scenario
        # Lane position: distance from start of lane. S_in length ~200m.
        # Place ego so remaining distance to junction ~= EGO_START_OFFSET.
        s_in_len = traci.lane.getLength("S_in_0")
        e_in_len = traci.lane.getLength("E_in_0")
        ego_pos = max(0.1, s_in_len - EGO_START_OFFSET)
        target_pos = max(0.1, e_in_len - scenario.target_start_offset)

        traci.vehicle.add(
            EGO_ID,
            "ego_SN",
            typeID="ego_car",
            depart="now",
            departLane="0",
            departPos=str(ego_pos),
            departSpeed=str(scenario.ego_speed),
        )
        traci.vehicle.add(
            TARGET_ID,
            "target_EW",
            typeID="target_car",
            depart="now",
            departLane="0",
            departPos=str(target_pos),
            departSpeed=str(scenario.target_speed),
        )
        traci.simulationStep()

        for vid, speed in (
            (EGO_ID, scenario.ego_speed),
            (TARGET_ID, scenario.target_speed),
        ):
            if vid in traci.vehicle.getIDList():
                traci.vehicle.setSpeedMode(vid, 0)
                traci.vehicle.setSpeed(vid, speed)

        self.ego_speed = scenario.ego_speed
        self.target_speed = scenario.target_speed

    def _close_traci(self) -> None:
        if self.traci_started:
            try:
                traci.close()
            except Exception:
                pass
            self.traci_started = False

    def _sync_kinematics(self) -> None:
        if EGO_ID in traci.vehicle.getIDList():
            x, y = traci.vehicle.getPosition(EGO_ID)
            self.ego_y = float(y)
            self.ego_speed = float(traci.vehicle.getSpeed(EGO_ID))
        if TARGET_ID in traci.vehicle.getIDList():
            x, y = traci.vehicle.getPosition(TARGET_ID)
            self.target_x = float(x)
            self.target_speed = float(traci.vehicle.getSpeed(TARGET_ID))
        elif self.target_x > -50.0:
            # Keep last known target state if it left the network past junction.
            self.target_x = min(self.target_x, -COLLISION_RADIUS - 1.0)

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
        if TARGET_ID in traci.vehicle.getIDList() and EGO_ID in traci.vehicle.getIDList():
            ex, ey = traci.vehicle.getPosition(EGO_ID)
            tx, ty = traci.vehicle.getPosition(TARGET_ID)
            return float(np.hypot(tx - ex, ty - ey))
        return float(np.hypot(self.target_x, 0.0 - self.ego_y))

    def _ttc_to_conflict(self) -> float:
        ego_t = (0.0 - self.ego_y) / max(self.ego_speed, 0.1)
        target_t = self.target_x / max(self.target_speed, 0.1)
        if ego_t < 0.0 or target_t < 0.0:
            return 10.0
        return float(np.clip(abs(ego_t - target_t), 0.0, 10.0))

    def _signed_time_gap(self) -> float:
        ego_t = (0.0 - self.ego_y) / max(self.ego_speed, 0.1)
        target_t = self.target_x / max(self.target_speed, 0.1)
        if ego_t < 0.0 or target_t < 0.0:
            return 10.0
        return float(np.clip(ego_t - target_t, -10.0, 10.0))

    def _get_obs(self) -> np.ndarray:
        visible = self._target_visible()
        if (
            self.observation_mode == "lossy_v2v"
            and not self._sensor_visible()
            and self._fresh_bsm_available()
        ):
            target_x = float(self.last_bsm["target_x"]) if self.last_bsm else TARGET_START_OFFSET
            target_speed = float(self.last_bsm["target_speed"]) if self.last_bsm else 0.0
            signed_gap = float(self.last_bsm["signed_gap"]) if self.last_bsm else 10.0
        else:
            target_x = self.target_x if visible else TARGET_START_OFFSET
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

        progress = self.ego_speed * DT / max(EGO_GOAL_Y + EGO_START_OFFSET, 1.0)
        reward += 3.0 * progress
        return float(np.clip(reward, -10.0, 10.0))

    def _empty_info(self, collision: bool, arrived: bool) -> dict[str, Any]:
        return {
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
