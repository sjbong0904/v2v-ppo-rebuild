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
NPC_WE_ID = "npc_we"
PARKED_TRUCK_ID = "parked_truck"  # east / SE shoulder (legacy id)
PARKED_TRUCK_WEST_ID = "parked_truck_w"
NO_PRIMARY_ID = ""

# NLOS occluders (sensor only; V2V is unaffected).
# East = SE block for ego↔EW target. West = SW block (east rotated 90° CW) for ego↔WE npc.
NLOS_BUILDING_EAST_AABB = (5.0, -50.0, 50.0, -5.0)  # xmin, ymin, xmax, ymax
NLOS_BUILDING_WEST_AABB = (-50.0, -50.0, -5.0, -5.0)
NLOS_BUILDING_AABB = NLOS_BUILDING_EAST_AABB  # backward-compatible alias

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
        nlos_blocker: bool = False,
        nlos_east: bool = True,
        nlos_west: bool = False,
        multi_npc: bool = False,
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
        self.nlos_blocker = nlos_blocker
        self.nlos_east = bool(nlos_east)
        self.nlos_west = bool(nlos_west)
        self.multi_npc = multi_npc
        self.primary_threat_id = TARGET_ID
        self.npc_speed = TARGET_SPEED
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
        self.v2v_rx_this_step = False
        self.last_pdr = 0.0
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
        self.v2v_rx_this_step = False
        self.last_pdr = 0.0
        self._sync_ego_pose()
        # V2V before primary pick so lossy can bootstrap an occluded threat.
        self._update_v2v_message()
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
            traci.vehicle.setSpeed(TARGET_ID, self._scenario.target_speed)
        if self.multi_npc and NPC_WE_ID in traci.vehicle.getIDList():
            traci.vehicle.setSpeed(NPC_WE_ID, self.npc_speed)

        traci.simulationStep()
        self.steps += 1
        self._sync_ego_pose()
        self._update_v2v_message()
        self._sync_kinematics()

        distance = self._distance_to_target()
        self.min_distance = min(self.min_distance, distance)
        if distance < NEAR_MISS_RADIUS:
            self.near_miss = True

        colliding = set(traci.simulation.getCollidingVehiclesIDList())
        collision = distance <= COLLISION_RADIUS or EGO_ID in colliding
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
            "sensor_visible": self._sensor_visible(),
            "occluded": self._is_occluded(),
            "primary_threat": self.primary_threat_id,
            "aoi": self.bsm_aoi,
            "pdr": self.last_pdr,
            "v2v_rx_count": self.v2v_rx_count,
            "v2v_rx_this_step": self.v2v_rx_this_step,
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
        scenario = Scenario(
            ego_speed=float(self.rng.uniform(10.0, 14.0)),
            target_speed=float(self.rng.uniform(9.0, 13.5)),
            target_start_offset=float(self.rng.uniform(55.0, 75.0)),
        )
        self.npc_speed = float(self.rng.uniform(9.0, 13.0))
        return scenario

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
        if self.multi_npc:
            w_in_len = traci.lane.getLength("W_in_0")
            # Approach from the west; offset slightly different from target for asynchrony.
            npc_offset = float(self.rng.uniform(50.0, 80.0))
            npc_pos = max(0.1, w_in_len - npc_offset)
            traci.vehicle.add(
                NPC_WE_ID,
                "npc_WE",
                typeID="target_car",
                depart="now",
                departLane="0",
                departPos=str(npc_pos),
                departSpeed=str(self.npc_speed),
            )
        traci.simulationStep()
        # Parked trucks use parking=true; SUMO relocates them off-lane on the next step.
        if (
            PARKED_TRUCK_ID in traci.vehicle.getIDList()
            or PARKED_TRUCK_WEST_ID in traci.vehicle.getIDList()
        ):
            traci.simulationStep()

        controlled = [
            (EGO_ID, scenario.ego_speed),
            (TARGET_ID, scenario.target_speed),
        ]
        if self.multi_npc:
            controlled.append((NPC_WE_ID, self.npc_speed))
        for vid, speed in controlled:
            if vid in traci.vehicle.getIDList():
                traci.vehicle.setSpeedMode(vid, 0)
                traci.vehicle.setSpeed(vid, speed)

        self.ego_speed = scenario.ego_speed
        self.target_speed = scenario.target_speed
        self.primary_threat_id = TARGET_ID

    def _close_traci(self) -> None:
        if self.traci_started:
            try:
                traci.close()
            except Exception:
                pass
            self.traci_started = False

    def _sync_ego_pose(self) -> None:
        if EGO_ID not in traci.vehicle.getIDList():
            return
        _, y = traci.vehicle.getPosition(EGO_ID)
        self.ego_y = float(y)
        self.ego_speed = float(traci.vehicle.getSpeed(EGO_ID))

    def _approaching_candidates(self) -> list[tuple[str, float, float, float]]:
        """Return (vid, approach_dist, speed, ttc) for NPCs still approaching the junction."""
        candidates: list[tuple[str, float, float, float]] = []
        if TARGET_ID in traci.vehicle.getIDList():
            x, _ = traci.vehicle.getPosition(TARGET_ID)
            speed = max(float(traci.vehicle.getSpeed(TARGET_ID)), 0.1)
            # East->West: positive x means still approaching junction.
            approach = float(x)
            if approach > 0.0:
                candidates.append((TARGET_ID, approach, speed, approach / speed))
        if self.multi_npc and NPC_WE_ID in traci.vehicle.getIDList():
            x, _ = traci.vehicle.getPosition(NPC_WE_ID)
            speed = max(float(traci.vehicle.getSpeed(NPC_WE_ID)), 0.1)
            # West->East: negative x means still approaching junction.
            approach = float(-x)
            if approach > 0.0:
                candidates.append((NPC_WE_ID, approach, speed, approach / speed))
        return candidates

    def _sync_kinematics(self) -> None:
        self._sync_ego_pose()
        candidates = self._approaching_candidates()
        if candidates:
            picked = self._pick_primary_candidate(candidates)
            if picked is None:
                self._clear_primary_threat()
            else:
                vid, approach, speed, _ = picked
                self.primary_threat_id = vid
                self.target_x = approach
                self.target_speed = speed
        else:
            self._clear_primary_threat()

    def _clear_primary_threat(self) -> None:
        """No detected approaching threat in the observation slot."""
        self.primary_threat_id = NO_PRIMARY_ID
        self.target_x = min(self.target_x, -COLLISION_RADIUS - 1.0)
        self.target_speed = 0.0

    def _pick_primary_candidate(
        self, candidates: list[tuple[str, float, float, float]]
    ) -> tuple[str, float, float, float] | None:
        """Choose primary only among vehicles the ego can currently detect.

        perfect_v2v: most urgent TTC (V2V always provides all candidates).
        sensor_only: most urgent among sensor-visible only; None if none visible.
        lossy_v2v: sensor-visible first; else adopt via fresh BSM (prior/BSM vid /
        most-urgent bootstrap); otherwise None.
        """
        if self.observation_mode == "perfect_v2v":
            return min(candidates, key=lambda c: c[3])

        visible = [c for c in candidates if self._sensor_visible_vehicle(c[0])]
        if visible:
            return min(visible, key=lambda c: c[3])

        if self.observation_mode == "lossy_v2v" and self._fresh_bsm_available():
            # Prefer the vehicle whose BSM we just hold, then prior primary,
            # otherwise bootstrap the most urgent approaching threat via V2V.
            bsm_vid = None
            if self.last_bsm is not None:
                bsm_vid = self.last_bsm.get("vid")
            if bsm_vid:
                for cand in candidates:
                    if cand[0] == bsm_vid:
                        return cand
            for cand in candidates:
                if cand[0] == self.primary_threat_id:
                    return cand
            return min(candidates, key=lambda c: c[3])
        return None

    def _target_visible(self) -> bool:
        if not self.primary_threat_id:
            return False
        if self.observation_mode == "perfect_v2v":
            return True
        if self.observation_mode == "lossy_v2v":
            return self._sensor_visible() or self._fresh_bsm_available()
        return self._sensor_visible()

    def _sensor_visible(self) -> bool:
        if not self.primary_threat_id:
            return False
        return self._sensor_visible_vehicle(self.primary_threat_id)

    def _sensor_visible_vehicle(self, vid: str) -> bool:
        """True if ego can sense this vehicle by range + geometric LOS."""
        if not vid:
            return False
        if EGO_ID not in traci.vehicle.getIDList() or vid not in traci.vehicle.getIDList():
            return False
        ex, ey = traci.vehicle.getPosition(EGO_ID)
        tx, ty = traci.vehicle.getPosition(vid)
        if float(np.hypot(tx - ex, ty - ey)) > self.sensor_range:
            return False
        if self.nlos_blocker and self._los_occluded_between(
            float(ex), float(ey), float(tx), float(ty)
        ):
            return False
        return True

    def _is_occluded(self) -> bool:
        """True if ego-primary LOS hits an enabled NLOS occluder."""
        if not self.nlos_blocker or not self.primary_threat_id:
            return False
        threat = self.primary_threat_id
        if EGO_ID in traci.vehicle.getIDList() and threat in traci.vehicle.getIDList():
            ex, ey = traci.vehicle.getPosition(EGO_ID)
            tx, ty = traci.vehicle.getPosition(threat)
        else:
            return False
        return self._los_occluded_between(float(ex), float(ey), float(tx), float(ty))

    def _los_occluded_between(
        self, ex: float, ey: float, tx: float, ty: float
    ) -> bool:
        for aabb in self._nlos_occluder_aabbs():
            if _segment_intersects_aabb(ex, ey, tx, ty, aabb):
                return True
        return False

    def _nlos_occluder_aabbs(self) -> list[tuple[float, float, float, float]]:
        """Enabled building polys + parked-truck footprints."""
        if not self.nlos_blocker:
            return []
        occluders: list[tuple[float, float, float, float]] = []
        if self.nlos_east:
            occluders.append(NLOS_BUILDING_EAST_AABB)
            truck = self._parked_truck_aabb(PARKED_TRUCK_ID)
            if truck is not None:
                occluders.append(truck)
        if self.nlos_west:
            occluders.append(NLOS_BUILDING_WEST_AABB)
            truck = self._parked_truck_aabb(PARKED_TRUCK_WEST_ID)
            if truck is not None:
                occluders.append(truck)
        return occluders

    def _parked_truck_aabb(
        self, truck_id: str = PARKED_TRUCK_ID
    ) -> tuple[float, float, float, float] | None:
        """Axis-aligned footprint of a parked truck from TraCI pose/size."""
        if truck_id not in traci.vehicle.getIDList():
            return None
        x, y = traci.vehicle.getPosition(truck_id)
        length = float(traci.vehicle.getLength(truck_id))
        width = float(traci.vehicle.getWidth(truck_id))
        # SUMO angle: 0=north, 90=east (degrees clockwise from north).
        angle_rad = np.deg2rad(float(traci.vehicle.getAngle(truck_id)))
        # Forward unit (north=0): dx=sin(angle), dy=cos(angle)
        fx, fy = float(np.sin(angle_rad)), float(np.cos(angle_rad))
        lx, ly = -fy, fx  # left lateral
        half_l, half_w = 0.5 * length, 0.5 * width
        corners = [
            (x + s_l * half_l * fx + s_w * half_w * lx,
             y + s_l * half_l * fy + s_w * half_w * ly)
            for s_l in (-1.0, 1.0)
            for s_w in (-1.0, 1.0)
        ]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return (min(xs), min(ys), max(xs), max(ys))

    def _fresh_bsm_available(self) -> bool:
        return self.last_bsm is not None and self.bsm_aoi <= AOI_TIMEOUT

    def _calc_pdr(self, distance: float | None = None) -> float:
        if distance is None:
            distance = self._distance_to_target()
        if distance > V2V_MAX_RANGE:
            return 0.0
        base = 1.0 - 0.75 * (distance / V2V_MAX_RANGE)
        return float(np.clip(base * float(self.pdr_scale), 0.05, 1.0))

    def _radio_listen_candidate(
        self,
    ) -> tuple[str, float, float, float] | None:
        """Choose which approaching vehicle to attempt a BSM RX for."""
        candidates = self._approaching_candidates()
        if not candidates:
            return None
        for cand in candidates:
            if cand[0] == self.primary_threat_id:
                if self._sensor_visible_vehicle(cand[0]):
                    return None
                return cand
        occluded = [c for c in candidates if not self._sensor_visible_vehicle(c[0])]
        if not occluded:
            return None
        return min(occluded, key=lambda c: c[3])

    def _update_v2v_message(self) -> None:
        self.v2v_rx_this_step = False
        if self.observation_mode != "lossy_v2v":
            self.bsm_aoi = 0.0 if self.observation_mode == "perfect_v2v" else AOI_TIMEOUT
            self.last_pdr = 1.0 if self.observation_mode == "perfect_v2v" else 0.0
            return

        self.bsm_aoi = min(self.bsm_aoi + DT, AOI_TIMEOUT * 2.0)
        cand = self._radio_listen_candidate()
        if cand is None:
            self.last_pdr = 0.0
            return

        vid, approach, speed, _ = cand
        if EGO_ID not in traci.vehicle.getIDList() or vid not in traci.vehicle.getIDList():
            self.last_pdr = 0.0
            return
        ex, ey = traci.vehicle.getPosition(EGO_ID)
        tx, ty = traci.vehicle.getPosition(vid)
        distance = float(np.hypot(tx - ex, ty - ey))
        self.last_pdr = self._calc_pdr(distance)
        if self.rng.random() > self.last_pdr:
            return

        ego_t = (0.0 - self.ego_y) / max(self.ego_speed, 0.1)
        target_t = approach / max(speed, 0.1)
        if ego_t < 0.0 or target_t < 0.0:
            signed_gap = 10.0
        else:
            signed_gap = float(np.clip(ego_t - target_t, -10.0, 10.0))
        self.last_bsm = {
            "vid": vid,
            "target_x": approach,
            "target_speed": speed,
            "signed_gap": signed_gap,
        }
        self.bsm_aoi = 0.0
        self.v2v_rx_count += 1
        self.v2v_rx_this_step = True

    def _distance_to_target(self) -> float:
        threat = self.primary_threat_id
        if threat in traci.vehicle.getIDList() and EGO_ID in traci.vehicle.getIDList():
            ex, ey = traci.vehicle.getPosition(EGO_ID)
            tx, ty = traci.vehicle.getPosition(threat)
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
            "sensor_visible": self._sensor_visible(),
            "occluded": self._is_occluded(),
            "primary_threat": self.primary_threat_id,
            "aoi": self.bsm_aoi,
            "pdr": self.last_pdr,
            "v2v_rx_count": self.v2v_rx_count,
            "v2v_rx_this_step": self.v2v_rx_this_step,
        }


def _segment_intersects_aabb(
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    aabb: tuple[float, float, float, float],
) -> bool:
    """Liang-Barsky style segment vs axis-aligned box test."""
    xmin, ymin, xmax, ymax = aabb
    dx = x2 - x1
    dy = y2 - y1
    p = [-dx, dx, -dy, dy]
    q = [x1 - xmin, xmax - x1, y1 - ymin, ymax - y1]
    u1, u2 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return False
            continue
        t = qi / pi
        if pi < 0:
            u1 = max(u1, t)
        else:
            u2 = min(u2, t)
        if u1 > u2:
            return False
    return True
