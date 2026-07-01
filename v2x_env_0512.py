import os
import socket
import time
import random
import subprocess
import numpy as np
import traci
import traci.constants as tc
import gymnasium as gym
from gymnasium import spaces
from mn_wifi.net import Mininet_wifi

# 상수
TARGET_SPEED = 16.67 # 목표 속도 (60km/h)
AEB_DECEL = 8.0 # 긴급 제동 감속도 (m/s²)
NORMAL_DECEL = 4.0
NORMAL_ACCEL = 2.0
MAX_ACCEL = 4.0
DT = 0.1 # 시뮬레이션 타임스텝 (초)
SENSOR_MAX_RANGE = 150.0 # 센서 최대 탐지 거리 (m)
V2X_MAX_RANGE = 300.0 # V2X 통신 최대 범위 (m)
SENSOR_FOV_DEG = 120.0 # 센서 시야각 (도)
BSM_HZ = 10.0 # BSM 메시지 전송 주파수 (Hz)
BSM_INTERVAL = 1.0 / BSM_HZ
AOI_THRESHOLD = BSM_INTERVAL * 5.0  # 이 시간 이상 갱신 없으면 이웃 테이블에서 제거
HALF_LANE_WIDTH = 1.6
N_NEIGHBORS = 3 # 상태 벡터에 포함할 인접 차량 수
EGO_VEHICLE_ID = "ego"
TTC_CRITICAL_MIN = 0.5 # 최소 보장 크리티컬 TTC 임계값 (초)
TTC_CAUTION_MIN = 1.0 # 최소 보장 주의 TTC 임계값 (초)
JUNCTION_APPROACH_DIST = 60.0 # 교차로 접근으로 간주할 잔여 거리 (m)
SAME_DIR_HDIFF_THRESH = 25.0 # 동일 방향 차량으로 판단하는 헤딩 차이 임계값 (도)
TIME_MARGIN = 2.5 # 교차로 진입 순서 판단 시 시간 여유 (초)
REAR_CLOSING_RATIO = 0.5
JUNCTION_CP_DIST = 60.0 # 충돌 예상 지점(CP) 탐색 최대 거리
FRONT_CLOSING_DIST = 40.0
REAR_CLOSING_DIST = 30.0
PREDICT_TIME = 2.0 # 미래 위치 예측 시간 (초)
N_NEIGHBOR_FEATS = 8 # 이웃 차량 1대당 특성 벡터 차원 수
STATE_DIM = 4 + 3 + N_NEIGHBOR_FEATS * N_NEIGHBORS  # = 31 (ego 4 + nav 3 + neighbor 24)

WARMUP_STEPS = 250 # 배경차량 충분히 배포 후 ego 출발 (25초)
MIN_BG_VEHICLES = 4 # ego 출발 시 최소 배경 차량 수

# 루트, 의도
_ROUTE_MAP = {
    "straight": "ego_straight",
    "left": "ego_left",
    "right": "ego_right",
}

# left 가중치 2: 좌회전 상황을 더 자주 학습
INTENT_WEIGHTS = {"straight": 1, "left": 2, "right": 1}
_INTENT_POOL = [k for k, w in INTENT_WEIGHTS.items() for _ in range(w)]

# TraCI 구독 변수 (배경차량용)
_SUB_VARS = [
    tc.VAR_POSITION,
    tc.VAR_SPEED,
    tc.VAR_ANGLE,
    tc.VAR_ACCELERATION,
    tc.VAR_LANE_ID,
]

# TraCI 구독 변수 (ego 전용 - 주행 거리 추가)
_EGO_SUB_VARS = [
    tc.VAR_POSITION,
    tc.VAR_SPEED,
    tc.VAR_ANGLE,
    tc.VAR_ACCELERATION,
    tc.VAR_LANE_ID,
    tc.VAR_DISTANCE,
]


# 유틸 함수
def _get_free_port() -> int:
    # 사용 가능한 랜덤 포트를 찾아 반환
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", 0))
        return s.getsockname()[1]


def _infer_intent_from_route(vid: str, cache: dict) -> str:
    # 차량의 라우트 마지막 엣지 방향을 분석해 의도(직진/좌/우)를 추론, 결과는 캐싱
    if vid in cache:
        return cache[vid]
    try:
        route = traci.vehicle.getRoute(vid)
        if len(route) < 2:
            intent = "straight"
        else:
            cur_heading = traci.vehicle.getAngle(vid)
            last_edge = route[-1]
            shape = traci.lane.getShape(f"{last_edge}_0")
            if len(shape) >= 2:
                dx = shape[-1][0] - shape[-2][0]
                dy = shape[-1][1] - shape[-2][1]
                dest_heading = float(np.degrees(np.arctan2(dx, dy))) % 360
                hdiff = (dest_heading - cur_heading + 180) % 360 - 180
                if hdiff >  20: intent = "right"
                elif hdiff < -20: intent = "left"
                else: intent = "straight"
            else:
                intent = "straight"
    except Exception:
        intent = "unknown"
    cache[vid] = intent
    return intent


# 환경
class RealWorldV2XEnv(gym.Env):
    # SUMO + Mininet-WiFi 기반 V2X 자율주행 강화학습 환경
    # action: 0=AEB, 1=감속, 2=유지, 3=가속, 4=최대가속 (Discrete 5)
    # observation: STATE_DIM=31 차원 ([-1, 1] 정규화)

    def __init__(self,
                 config_file: str = "../sumo_data_multi/map.sumocfg",
                 ego_id: str = EGO_VEHICLE_ID,
                 sumo_port: int | None = None,
                 build_network: bool = False,
                 nav_intent: str = "straight",
                 randomize_intent: bool = True):
        super().__init__()
        self.config_file = config_file
        self.ego_id = ego_id
        self.sumo_port = sumo_port
        self.nav_intent = nav_intent
        self.randomize_intent = randomize_intent
        self.build_network = build_network

        # 행동: 5단계
        self.action_space = spaces.Discrete(5)
        # 관측: 31차원 벡터
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(STATE_DIM,),
            dtype=np.float32,
        )

        self.net = None
        self.traci_started = False
        self._sumo_proc = None
        self.step_count = 0
        self.current_network_load = 0.0
        self.neighbor_table: dict = {}
        self.ego_spawned = False
        self.collision_occurred = False
        self._ghost_brake_count = 0  # 유령 제동(오탐) 연속 횟수 카운터
        self._route_length = 500.0
        self._junction_entered = False
        self._junction_exited = False
        self._intent_cache: dict = {}
        self._stop_time = 0.0  # 정차 시간 누적 변수


    # Mininet
    def _cleanup_mininet(self):
        # Mininet-WiFi 네트워크 종료 및 잔여 프로세스 정리

        if self.net:
            try:
                self.net.stop()
            except Exception:
                pass
            self.net = None
        subprocess.run(
            ["sudo", "mn", "-c"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)

    def _build_network(self):
        # ego + 인접 차량(tgt_1~3) 가상 WiFi 노드 생성 및 전파 모델 설정

        self._cleanup_mininet()
        self.net = Mininet_wifi(noise_th=-91)
        self.net.addStation(self.ego_id, ip="10.0.0.1/8", position="0,0,0")
        for i in range(1, N_NEIGHBORS + 1):
            self.net.addStation(f"tgt_{i}", ip=f"10.0.0.{i+1}/8", position="0,0,0")
        try:
            self.net.setPropagationModel(
                model="logDistance", exp=3.5, fading_coefficient=3)
        except Exception:
            self.net.setPropagationModel(model="logDistance", exp=3.5)
        self.net.configureWifiNodes()
        self.net.build()
        self.net.start()


    # 채널 모델
    def _calc_pdr(self, dist: float, is_occluded: bool = False) -> float:
        # 패킷 수신률(PDR) 계산
        # 거리, 차폐 여부, 채널 혼잡도를 반영해 [0.05, 1.0] 범위로 반환.

        base = 1.0 - (dist / V2X_MAX_RANGE) * 0.8
        if is_occluded:
            base *= 0.5 # 센서 사각지대는 PDR 절반
        base *= (1.0 - self.current_network_load * 0.5)
        return float(np.clip(base, 0.05, 1.0))

    def _calc_channel_load(self, n_vehicles: int) -> float:
        # 채널 혼잡도: 차량 50대 기준 정규화 [0, 1]
        return float(np.clip(n_vehicles / 50.0, 0.0, 1.0))


    # 내비게이션 인코딩
    def _encode_nav_intent(self) -> np.ndarray:
        #주행 의도를 원핫 벡터(3차원)로 변환: [직진, 좌회전, 우회전]
        table = {
            "straight": [1.0, 0.0, 0.0],
            "left": [0.0, 1.0, 0.0],
            "right": [0.0, 0.0, 1.0],
        }
        return np.array(table.get(self.nav_intent, [1.0, 0.0, 0.0]), dtype=np.float32)


    # 교차로 상태 판단
    def _is_in_junction(self, veh_id: str) -> bool:
        # lane ID가 ':'으로 시작하면 교차로 내부에 있는 것으로 판단
        try:
            return traci.vehicle.getLaneID(veh_id).startswith(":")
        except Exception:
            return False

    def _is_approaching_junction(self) -> bool:
        # 잔여 경로 거리가 JUNCTION_APPROACH_DIST 이하이면 교차로 접근 중으로 판단
        try:
            dist_traveled  = traci.vehicle.getDistance(self.ego_id)
            dist_remaining = self._route_length - dist_traveled
            return dist_remaining < JUNCTION_APPROACH_DIST
        except Exception:
            return False

    def _compute_junction_order(self, ego_pos, ego_speed, ego_dir, threats) -> int:
        #교차로 진입 우선순위 계산.
        #위협 차량 중 ego보다 먼저 CP에 도달하는 차량이 있으면 순위 증가
        #반환값이 1이면 ego가 먼저 진입 가능, 2 이상이면 대기 필요

        if ego_speed < 0.1:
            return 99  # 정차 중이면 최하위 순위

        ego_order = 1
        for t in threats:
            if t["tgt_speed"] < 0.1:
                continue
            if t["lon_ttc"] == float("inf"):
                continue

            cp = self._compute_conflict_point(
                ego_pos, ego_dir, t["tgt_pos"], t["tgt_dir"])

            if cp is None:
                # CP 계산 불가 = 상대방이 교차로 통과 중
                # 근거리에 있는 상대방이 이미 교차로를 점유 중이면 대기
                d_tgt_to_ego = float(np.linalg.norm(t["tgt_pos"] - ego_pos))
                if d_tgt_to_ego < JUNCTION_CP_DIST and t["tgt_speed"] > 0.5:
                    ego_order += 1
                continue

            d_ego = float(np.linalg.norm(cp - ego_pos))
            d_tgt = float(np.linalg.norm(cp - t["tgt_pos"]))
            t_ego = d_ego / max(ego_speed, 0.5)
            t_tgt = d_tgt / max(t["tgt_speed"], 0.5)

            # 상대방이 TIME_MARGIN 이상 먼저 도착하면 ego는 대기
            if t_tgt < t_ego - TIME_MARGIN:
                ego_order += 1

        return ego_order

    def _is_same_or_adjacent_lane(self, ego_id: str, vid: str) -> bool:
        # 두 차량이 동일 또는 인접 차선인지 확인
        try:
            ego_lane = traci.vehicle.getLaneID(ego_id)
            tgt_lane = traci.vehicle.getLaneID(vid)
            if ego_lane == tgt_lane:
                return True
            return ego_lane.split("_")[:-1] == tgt_lane.split("_")[:-1]
        except Exception:
            return False


    # 동적 TTC 임계값
    def _get_dynamic_ttc_thresholds(self, ego_v: float):
        # 속도에 비례해 TTC 임계값을 동적으로 계산, 고속일수록 임계값 커짐
        margin = np.clip(ego_v / 10.0, 0.5, 2.0)
        caution_ttc = (ego_v / (2 * NORMAL_DECEL)) + margin
        critical_ttc = (ego_v / (2 * AEB_DECEL))   + (margin * 0.5)
        return max(critical_ttc, TTC_CRITICAL_MIN), max(caution_ttc, TTC_CAUTION_MIN)


    # 위협 필터링
    def _filter_threats(self, threats: list) -> list:
        # 이미 지나친 차량(lon_ttc < 0)을 위협 목록에서 제거
        return [t for t in threats if t.get("lon_ttc", 999) >= 0]


    # 충돌 후보 판단
    def _is_collision_candidate(self, ego_id, ego_pos, ego_heading,
                                 vid, data, ego_speed: float) -> bool:
        
        # 대상 차량이 ego의 실제 위협(충돌 후보)인지 판단.
        # - 동일 방향 차량: 전방 접근 여부 확인
        # - 교차로 외부: 반대 차선, 측면 차량 필터링
        # - 교차로 근처: 충돌 예상 지점(CP) 기반 판단

        tgt_heading = data["heading"]
        tgt_pos = data["pos"]
        tgt_speed = data["speed"]
        dist = data["est_dist"]

        rad = np.radians(ego_heading)
        ego_dir = np.array([np.sin(rad), np.cos(rad)])
        rel_pos = tgt_pos - ego_pos
        longitudinal = float(np.dot(rel_pos, ego_dir))
        hdiff = abs((ego_heading - tgt_heading + 180) % 360 - 180)

        ego_in_j = self._is_in_junction(ego_id)
        ego_near_j = self._is_approaching_junction()

        # 동일 방향 차량 처리
        if hdiff < SAME_DIR_HDIFF_THRESH:
            rad_tgt = np.radians(tgt_heading)
            tgt_dir = np.array([np.sin(rad_tgt), np.cos(rad_tgt)])
            ego_v_vec = ego_dir * ego_speed
            tgt_v_vec = tgt_dir * tgt_speed

            if longitudinal > 0.5:   # 전방 차량: 접근 속도가 양수이면 위협
                rel_v = float(np.dot(
                    ego_v_vec - tgt_v_vec, rel_pos / (dist + 1e-6)))
                if rel_v > 0.5 and dist < FRONT_CLOSING_DIST:
                    return True

            if dist < 10.0:
                return True
            return False

        # 교차로 외부: 불필요한 차량 필터링
        if not ego_in_j and not ego_near_j:
            if longitudinal < -3.0:
                return False
            if hdiff < 120.0:
                return False
            # 반대 차선 차량 필터 (hdiff = 180도 + 측면 분리)
            lateral = float(np.cross(ego_dir, rel_pos))
            if hdiff > 150.0 and abs(lateral) > HALF_LANE_WIDTH * 1.0:
                return False  # 반대 차선 통행 차량은 위협 아님
            if abs(lateral) > HALF_LANE_WIDTH * 3.0:
                return False
            return True

        if longitudinal < -3.0 and not ego_in_j:
            return False

        # 교차로 근처: 진행 방향 벡터로 CP 계산 후 도달 시간 비교
        rad_tgt = np.radians(tgt_heading)
        tgt_dir = np.array([np.sin(rad_tgt), np.cos(rad_tgt)])
        A = np.array([ego_dir, -tgt_dir]).T
        det = np.linalg.det(A)

        if abs(det) > 1e-3:
            b = tgt_pos - ego_pos
            t = np.linalg.solve(A, b)
            d_ego_to_cp = float(t[0])
            d_tgt_to_cp = float(t[1])
            if d_ego_to_cp > 0 and d_tgt_to_cp > 0:
                if (d_ego_to_cp < JUNCTION_CP_DIST and
                        d_tgt_to_cp < JUNCTION_CP_DIST):
                    t_ego = d_ego_to_cp / max(ego_speed, 0.5)
                    t_tgt = d_tgt_to_cp / max(tgt_speed, 0.5)
                    if abs(t_ego - t_tgt) < TIME_MARGIN:
                        return True  # 거의 동시에 CP 도달: 충돌 위험

        if dist < HALF_LANE_WIDTH * 3.0 and hdiff > 60.0:
            return True

        return False


    # 충돌 예상 지점 계산
    def _compute_conflict_point(self, ego_pos, ego_dir, tgt_pos, tgt_dir):
        # 두 차량의 진행 방향 직선 교점(충돌 예상 지점, CP) 반환
        # 교점이 두 차량 후방에 있거나 직선이 평행하면 None 반환

        A = np.array([ego_dir, -tgt_dir]).T
        b = tgt_pos - ego_pos
        if abs(np.linalg.det(A)) < 1e-3:
            return None
        t = np.linalg.solve(A, b)
        t_ego = float(t[0])
        t_tgt = float(t[1])
        if t_ego < 0 or t_tgt < 0:
            return None
        return ego_pos + ego_dir * t_ego

    def _compute_ttcf(self, ego_pos, ego_speed, tgt_pos, tgt_speed,
                      ego_dir, tgt_dir) -> float:
        # CP 기반 TTC 계산 (교차로용)
        # 두 차량이 CP에 2.5초 이내 차이로 도달하면 충돌 위험으로 판단해 min(t_ego, t_tgt) 반환

        cp = self._compute_conflict_point(ego_pos, ego_dir, tgt_pos, tgt_dir)
        if cp is None:
            return float("inf")
        d_ego = float(np.linalg.norm(cp - ego_pos))
        d_tgt = float(np.linalg.norm(cp - tgt_pos))
        t_ego = d_ego / ego_speed if ego_speed > 0.1 else float("inf")
        t_tgt = d_tgt / tgt_speed if tgt_speed > 0.1 else float("inf")
        if t_ego == float("inf") or t_tgt == float("inf"):
            return float("inf")
        pet = abs(t_ego - t_tgt)
        if pet <= 2.5:
            return min(t_ego, t_tgt)
        return float("inf")


    # TraCI 구독 설정
    def _setup_subscriptions(self):
        traci.vehicle.subscribeContext(
            self.ego_id,
            tc.CMD_GET_VEHICLE_VARIABLE,
            V2X_MAX_RANGE,
            _SUB_VARS,
        )
        traci.vehicle.subscribe(self.ego_id, _EGO_SUB_VARS)


    # 이웃 테이블 업데이트
    def _update_neighbor_table(self):
        # 매 스텝마다 이웃 테이블 갱신:
        # 1. AoI 초과 차량 삭제 (오래된 정보 폐기)
        # 2. 갱신 없는 차량은 위치 추정
        # 3. BSM 수신 성공한 차량만 테이블에 추가/갱신

        if self.ego_id not in traci.vehicle.getIDList():
            return

        ctx = traci.vehicle.getContextSubscriptionResults(self.ego_id) or {}
        ego_data = traci.vehicle.getSubscriptionResults(self.ego_id)
        if ego_data is None:
            return

        ego_pos = np.array(ego_data[tc.VAR_POSITION])
        n_vehicles = len(traci.vehicle.getIDList())
        self.current_network_load = self._calc_channel_load(n_vehicles)

        # AoI 초과 차량 제거 또는 위치 갱신
        dead = []
        for vid, data in self.neighbor_table.items():
            data["aoi"] += DT
            if data["aoi"] > AOI_THRESHOLD:
                dead.append(vid)
            else:
                rad = np.radians(data["heading"])
                data["pos"][0] += data["speed"] * np.sin(rad) * DT
                data["pos"][1] += data["speed"] * np.cos(rad) * DT
                data["est_dist"] = float(np.linalg.norm(data["pos"] - ego_pos))
        for vid in dead:
            del self.neighbor_table[vid]
            self._intent_cache.pop(vid, None)

        # BSM 수신 성공 차량만 테이블에 등록
        for vid, vdata in ctx.items():
            if vid == self.ego_id:
                continue
            try:
                t_pos = np.array(vdata[tc.VAR_POSITION], dtype=np.float64)
                real_dist = float(np.linalg.norm(t_pos - ego_pos))
                if real_dist > V2X_MAX_RANGE:
                    continue

                is_occluded = real_dist > SENSOR_MAX_RANGE  # 센서 범위 밖 = V2V 통신
                pdr = self._calc_pdr(real_dist, is_occluded)
                if random.random() >= pdr:
                    continue

                speed = float(vdata[tc.VAR_SPEED])
                heading = float(vdata[tc.VAR_ANGLE])
                accel = float(vdata[tc.VAR_ACCELERATION])
                intent = _infer_intent_from_route(vid, self._intent_cache)

                self.neighbor_table[vid] = {
                    "pos": t_pos.copy(),
                    "speed": speed,
                    "heading": heading,
                    "is_aeb": accel < -7.5, # 급감속 중이면 AEB 발동으로 간주
                    "aoi": 0.0,
                    "est_dist": real_dist,
                    "pdr": pdr,
                    "is_v2v": is_occluded,   # 센서 사각지대 차량 = V2V로만 인지
                    "intent": intent,
                }
            except (KeyError, TypeError):
                continue


    # 위협 목록 생성
    def _get_threats(self):
        """
        이웃 테이블에서 위협 차량 목록을 추출.
        - 정지 차량: 간단한 TTC로 처리
        - 이동 차량: _is_collision_candidate()로 필터 후 교차로 여부에 따라
          일반 TTC 또는 CP 기반 TTC(ttcf) 선택
        - 위험도(risk_score) 기준 상위 N_NEIGHBORS개만 반환
        """
        if self.ego_id not in traci.vehicle.getIDList():
            return SENSOR_MAX_RANGE, 0.0, 0, 0.0, []

        ego_data = traci.vehicle.getSubscriptionResults(self.ego_id)
        if ego_data is None:
            return SENSOR_MAX_RANGE, 0.0, 0, 0.0, []

        ego_pos = np.array(ego_data[tc.VAR_POSITION])
        ego_speed = float(ego_data[tc.VAR_SPEED])
        ego_heading = float(ego_data[tc.VAR_ANGLE])
        ego_accel = float(ego_data[tc.VAR_ACCELERATION])

        rad_ego = np.radians(ego_heading)
        ego_dir = np.array([np.sin(rad_ego), np.cos(rad_ego)])
        ego_v_vec = ego_dir * ego_speed
        ego_in_j = self._is_in_junction(self.ego_id)
        ego_near_j = self._is_approaching_junction()

        # 전방 최근접 차량 거리 계산 (FOV 내)
        sensor_dist = SENSOR_MAX_RANGE
        ctx = traci.vehicle.getContextSubscriptionResults(self.ego_id) or {}
        for vid, vdata in ctx.items():
            if vid == self.ego_id:
                continue
            try:
                t_pos = np.array(vdata[tc.VAR_POSITION])
                dist  = float(np.linalg.norm(t_pos - ego_pos))
                if dist > SENSOR_MAX_RANGE:
                    continue
                if not ego_in_j and not ego_near_j:
                    dx = float(t_pos[0]) - float(ego_pos[0])
                    dy = float(t_pos[1]) - float(ego_pos[1])
                    angle_to_t = float(np.degrees(np.arctan2(dx, dy))) % 360.0
                    hdiff_fov  = float(abs((angle_to_t - ego_heading + 180.0) % 360.0 - 180.0))
                    if dist > 5.0 and hdiff_fov > SENSOR_FOV_DEG:
                        continue
                if dist < sensor_dist:
                    sensor_dist = dist
            except (KeyError, TypeError):
                continue

        threat_list = []
        total_aoi = 0.0
        valid_cnt = 0
        emergency_flag = 0
        _APPROACH_EDGES = {"N_in", "S_in", "E_in", "W_in"}

        for vid, data in self.neighbor_table.items():
            is_static = data["speed"] < 0.1

            # 정지 차량 처리 (30m 이내만)
            if is_static:
                dist_static = data["est_dist"]
                if dist_static > 30.0:
                    continue
                rel_pos    = data["pos"] - ego_pos
                lon_ttc_st = dist_static / max(ego_speed, 0.5)
                lateral    = float(np.cross(ego_dir, rel_pos))
                lat_overlap = 1.0 if abs(lateral) <= HALF_LANE_WIDTH * 1.5 else 0.0
                fut_ego_pos = ego_pos + ego_dir * ego_speed * PREDICT_TIME
                fut_rel_dist = float(np.linalg.norm(data["pos"] - fut_ego_pos))
                valid_cnt += 1
                total_aoi += data["aoi"]
                threat_list.append({
                    "rel_dist": dist_static,
                    "rel_speed": ego_speed,
                    "lon_ttc": lon_ttc_st,
                    "lat_overlap": lat_overlap,
                    "is_v2v": float(data["is_v2v"]),
                    "fut_rel_dist": fut_rel_dist,
                    "att_weight": 0.0,
                    "tgt_pos": data["pos"].copy(),
                    "tgt_speed": 0.0,
                    "tgt_intent": data.get("intent", "unknown"),
                    "tgt_dir": ego_dir.copy(),
                })
                continue

            if not self._is_collision_candidate(
                    self.ego_id, ego_pos, ego_heading, vid, data, ego_speed):
                continue

            valid_cnt += 1
            total_aoi += data["aoi"]
            if data["is_aeb"]:
                emergency_flag = 1 # 주변에 AEB 발동 차량 있음

            tgt_pos = data["pos"]
            tgt_speed = data["speed"]
            rad_tgt = np.radians(data["heading"])
            tgt_dir = np.array([np.sin(rad_tgt), np.cos(rad_tgt)])
            dist = data["est_dist"]

            tgt_lane_id = ""
            try:
                tgt_lane_id = traci.vehicle.getLaneID(vid)
            except Exception:
                pass

            tgt_on_approach = tgt_lane_id.rsplit("_", 1)[0] in _APPROACH_EDGES

            # 교차로 상황이면 CP 기반 TTC, 일반 도로면 상대속도 기반 TTC
            use_cp_ttc = (ego_in_j or ego_near_j
                          or self._is_in_junction(vid)
                          or tgt_on_approach)
            if not use_cp_ttc:
                rel_pos_vec = tgt_pos - ego_pos
                rel_v_vec = ego_v_vec - tgt_dir * tgt_speed
                closing_speed = (
                    -np.dot(rel_pos_vec, rel_v_vec) / dist
                    if dist > 1e-3 else 0.0
                )
                lon_ttc = dist / closing_speed if closing_speed > 0.0 else float("inf")
            else:
                lon_ttc = self._compute_ttcf(
                    ego_pos, ego_speed, tgt_pos, tgt_speed, ego_dir, tgt_dir)

            rel_pos = tgt_pos - ego_pos
            lateral = float(np.cross(ego_dir, rel_pos))
            lat_overlap = 1.0 if abs(lateral) <= HALF_LANE_WIDTH * 1.5 else 0.0

            rel_pos_n = rel_pos / (dist + 1e-6)
            rel_spd = float(np.dot(ego_v_vec - tgt_dir * tgt_speed, rel_pos_n))

            fut_ego_pos = ego_pos + ego_dir * ego_speed * PREDICT_TIME
            fut_tgt_pos = tgt_pos + tgt_dir * tgt_speed * PREDICT_TIME
            fut_rel_dist = float(np.linalg.norm(fut_tgt_pos - fut_ego_pos))

            threat_list.append({
                "rel_dist": dist,
                "rel_speed": rel_spd,
                "lon_ttc": lon_ttc,
                "lat_overlap": lat_overlap,
                "is_v2v": float(data["is_v2v"]),
                "fut_rel_dist": fut_rel_dist,
                "att_weight": 0.0,
                "tgt_pos": tgt_pos.copy(),
                "tgt_speed": tgt_speed,
                "tgt_intent": data.get("intent", "unknown"),
                "tgt_dir": tgt_dir.copy(),
            })

        if threat_list:
            # 위험도 = TTC 역수 + 거리 역수 가중합 -> attention 가중치 계산
            risk_scores = [
                1.0 / (t["lon_ttc"] + 0.1) + 1.0 / (t["rel_dist"] / 10.0 + 1.0)
                for t in threat_list
            ]
            total_risk = sum(risk_scores) + 1e-6
            for t, r in zip(threat_list, risk_scores):
                t["att_weight"] = r / total_risk

            # TTC 오름차순 정렬 후 상위 N_NEIGHBORS개만 선택
            threat_list.sort(key=lambda x: x["lon_ttc"])
            threat_list = threat_list[:N_NEIGHBORS]

        avg_aoi = total_aoi / valid_cnt if valid_cnt > 0 else 0.0
        return sensor_dist, avg_aoi, emergency_flag, float(ego_accel), threat_list


    # reset
    def reset(self, seed=None, options=None):
        """
        에피소드 초기화:
        1. 기존 SUMO/TraCI 종료
        2. SUMO 재시작 및 WARMUP_STEPS 동안 배경차량 배포
        3. ego 차량 생성 및 MIN_BG_VEHICLES 확보 대기
        4. 구독 설정 후 초기 상태 반환
        """
        super().reset(seed=seed)

        if self.build_network:
            self._cleanup_mininet()

        if self.randomize_intent:
            self.nav_intent = random.choice(_INTENT_POOL)

        # 내부 상태 초기화
        self.step_count = 0
        self.current_network_load = 0.0
        self.neighbor_table.clear()
        self._intent_cache.clear()
        self.collision_occurred = False
        self.ego_spawned = False
        self._ghost_brake_count = 0
        self._junction_entered = False
        self._junction_exited = False
        self._stop_time = 0.0

        # 이전 TraCI/SUMO 프로세스 종료
        if self.traci_started:
            try:
                traci.close()
            except Exception:
                pass
            self.traci_started = False

        if self._sumo_proc is not None:
            try:
                self._sumo_proc.terminate()
                self._sumo_proc.wait(timeout=3)
            except Exception:
                pass
            self._sumo_proc = None

        time.sleep(0.1)
        port = self.sumo_port if self.sumo_port is not None else _get_free_port()

        sumo_cmd = [
            "sumo", "-c", self.config_file,
            "--no-step-log", "true",
            "--no-warnings", "true",
            "--start", "true",
            "--step-length", str(DT),
            "--random", "true",
            "--collision.action", "remove",
            "--collision.check-junctions", "true",
        ]
        traci.start(sumo_cmd, port=port, numRetries=10)
        self.traci_started = True

        # 배경 차량 충분히 생성될 때까지 워밍업
        for _ in range(WARMUP_STEPS):
            traci.simulationStep()

        route_id = _ROUTE_MAP.get(self.nav_intent, "ego_straight")
        try:
            traci.vehicle.add(
                self.ego_id,
                route_id,
                typeID="ego_car",
                depart="now",
                departLane="best",
                departSpeed="7.0",
            )
        except traci.exceptions.TraCIException:
            pass

        for _ in range(100):
            traci.simulationStep()
            if self.ego_id in traci.vehicle.getIDList():
                traci.vehicle.setSpeedMode(self.ego_id, 0)  # SUMO 내부 속도 제어 비활성화
                traci.vehicle.setColor(self.ego_id, (255, 0, 0))
                self.ego_spawned = True
                break

        if not self.ego_spawned:
            traci.close()
            self.traci_started = False
            raise RuntimeError("ego vehicle did not spawn.")

        # 최소 배경 차량 확보
        for _ in range(5):
            bg_count = len([v for v in traci.vehicle.getIDList() if v != self.ego_id])
            if bg_count >= MIN_BG_VEHICLES:
                break
            for _ in range(50):
                traci.simulationStep()
            if self.ego_id not in traci.vehicle.getIDList():
                break # 대기 중 ego 소멸 시 조기 종료 

        # 총 경로 길이 계산 (거리 정규화에 사용)
        try:
            route = traci.vehicle.getRoute(self.ego_id)
            self._route_length = max(
                sum(traci.lane.getLength(f"{e}_0") for e in route), 1.0)
        except Exception:
            self._route_length = 500.0

        if self.build_network:
            self._build_network()

        if self.ego_id not in traci.vehicle.getIDList():
            traci.close()
            self.traci_started = False
            return self.reset(seed=seed, options=options)

        self._setup_subscriptions()
        self._update_neighbor_table()
        return self._get_state(), {}


    # step
    def step(self, action):
        """
        1 스텝 실행:
        1. action에 따라 ego 속도 설정
        2. SUMO 시뮬레이션 1스텝 진행
        3. 충돌/도착 감지, TTC 계산, 교차로 진입·통과 감지
        4. 보상 계산 및 상태 반환
        """
        self.step_count += 1
        drive_action = int(action)

        was_in_junction = self._is_in_junction(self.ego_id)
        self._update_neighbor_table()

        if self.ego_id not in traci.vehicle.getIDList():
            empty_obs = np.zeros(STATE_DIM, dtype=np.float32)
            return (
                empty_obs, 0.0, True, False,
                {"termination": "ego_disappeared", "collision": False,
                 "arrived": 0, "speed": 0.0, "sensor_dist": SENSOR_MAX_RANGE,
                 "min_ttc": 10.0, "emergency": 0, "drive_action": drive_action,
                 "n_threats": 0, "cbr": 0.0, "table_size": 0, "reward": 0.0},
            )

        # 이산 행동 → 목표 속도 변환
        current_speed = traci.vehicle.getSpeed(self.ego_id)
        if   drive_action == 0: new_speed = max(0.0, current_speed - AEB_DECEL * DT)
        elif drive_action == 1: new_speed = max(0.0, current_speed - NORMAL_DECEL * DT)
        elif drive_action == 2: new_speed = current_speed
        elif drive_action == 3: new_speed = min(TARGET_SPEED, current_speed + NORMAL_ACCEL * DT)
        else: new_speed = min(TARGET_SPEED, current_speed + MAX_ACCEL * DT)
        traci.vehicle.slowDown(self.ego_id, new_speed, 0.0)

        traci.simulationStep()

        # 정차 시간 누적
        current_speed_after = traci.vehicle.getSpeed(self.ego_id) if self.ego_id in traci.vehicle.getIDList() else 0.0
        if current_speed_after <= 0.1:
            self._stop_time += DT
        else:
            self._stop_time = 0.0

        collisions = traci.simulation.getCollidingVehiclesIDList()
        self.collision_occurred = self.ego_id in collisions

        sensor_dist, avg_aoi, emergency_flag, ego_accel, raw_threats = self._get_threats()
        threats = self._filter_threats(raw_threats)

        ego_in_sim = self.ego_id in traci.vehicle.getIDList()
        arrived = self.ego_spawned and not ego_in_sim and not self.collision_occurred

        now_in_junction = self._is_in_junction(self.ego_id) if ego_in_sim else False

        # 교차로 첫 진입 감지
        junction_entered_now = (
            not was_in_junction and now_in_junction and not self._junction_entered
        )
        if junction_entered_now:
            self._junction_entered = True

        # 교차로 통과 완료 감지
        junction_exited_now = (
            was_in_junction and not now_in_junction
            and self._junction_entered and not self._junction_exited
        )
        if junction_exited_now:
            self._junction_exited = True

        min_ttc = float(np.clip(
            min((t["lon_ttc"] for t in threats), default=10.0), 0.0, 10.0))

        # 교차로 진입 순위 계산
        j_order = 1
        if ego_in_sim and (now_in_junction or self._is_approaching_junction()):
            try:
                e_data = traci.vehicle.getSubscriptionResults(self.ego_id)
                e_pos = np.array(e_data[tc.VAR_POSITION])
                e_spd = float(e_data[tc.VAR_SPEED])
                rad = np.radians(float(e_data[tc.VAR_ANGLE]))
                e_dir = np.array([np.sin(rad), np.cos(rad)])
                j_order = self._compute_junction_order(e_pos, e_spd, e_dir, threats)
            except Exception:
                j_order = 1

        if arrived:
            reward = 20.0  # 목적지 도착 보너스
        else:
            reward = self._calculate_reward(
                drive_action, sensor_dist, min_ttc,
                avg_aoi, self.collision_occurred, emergency_flag,
                threats, self._stop_time, junction_entered_now, junction_exited_now,
                j_order,
            )

        done = (self.collision_occurred
                or arrived
                or traci.simulation.getTime() >= 300)

        next_state = self._get_state(sensor_dist, avg_aoi, emergency_flag,
                                     ego_accel, threats, j_order)

        info = {
            "cbr": self.current_network_load,
            "table_size": len(self.neighbor_table),
            "reward": reward,
            "collision": self.collision_occurred,
            "arrived": int(arrived),
            "speed": traci.vehicle.getSpeed(self.ego_id) if ego_in_sim else 0.0,
            "sensor_dist": sensor_dist,
            "min_ttc": min_ttc,
            "emergency": emergency_flag,
            "drive_action": drive_action,
            "n_threats": len(threats),
            "stop_time": self._stop_time,
            "junction_order": j_order,
        }
        return next_state, reward, done, False, info


    # 상태 벡터 구성
    def _get_state(self, sensor_dist=None, avg_aoi=None,
                   emergency_flag=None, ego_accel=None, threats=None, j_order: int = 1):
        """
        31차원 상태 벡터 구성:
        0   ego 속도 / TARGET_SPEED
        1   ego 가속도 / 8.0
        2   잔여 경로 비율
        3   교차로 진입 순위 (정규화)
        4-6 nav_intent 원핫 벡터
        7-30 위협 차량 3대 × 특성 8개 (rel_dist, rel_speed, ttc, lat_overlap,
               is_v2v, fut_rel_dist, att_weight, intent_enc)
        """
        _INTENT_ENC = {"left": -1.0, "straight": 0.0, "right": 1.0, "unknown": 0.0}

        if sensor_dist is None:
            sensor_dist, avg_aoi, emergency_flag, ego_accel, raw_threats = self._get_threats()
            threats = self._filter_threats(raw_threats)

        if self.ego_id not in traci.vehicle.getIDList():
            return np.zeros(STATE_DIM, dtype=np.float32)

        ego_data = traci.vehicle.getSubscriptionResults(self.ego_id)
        if ego_data is None:
            return np.zeros(STATE_DIM, dtype=np.float32)

        ego_speed = float(ego_data[tc.VAR_SPEED])
        try:
            dist_traveled  = float(ego_data[tc.VAR_DISTANCE])
            dist_remaining = float(np.clip(
                self._route_length - dist_traveled, 0.0, self._route_length))
        except (KeyError, TypeError):
            dist_remaining = self._route_length

        ego_state = np.array([
            np.clip(ego_speed / TARGET_SPEED, 0.0, 1.0),
            np.clip(ego_accel / 8.0, -1.0, 1.0),
            np.clip(dist_remaining / self._route_length, 0.0, 1.0),
            np.clip((j_order - 1) / 3.0, 0.0, 1.0), # 교차로 순위 정규화
        ], dtype=np.float32)

        nav_intent    = self._encode_nav_intent()
        neighbor_vecs = []
        for i in range(N_NEIGHBORS):
            if i < len(threats):
                t   = threats[i]
                vec = np.array([
                    np.clip(t["rel_dist"] / V2X_MAX_RANGE, 0.0, 1.0),
                    np.clip(t["rel_speed"] / TARGET_SPEED, -1.0, 1.0),
                    np.clip(t["lon_ttc"] / 10.0, 0.0, 1.0),
                    t["lat_overlap"],
                    t["is_v2v"],
                    np.clip(t["fut_rel_dist"] / V2X_MAX_RANGE, 0.0, 1.0),
                    float(np.clip(t["att_weight"], 0.0, 1.0)),
                    float(_INTENT_ENC.get(t.get("tgt_intent", "unknown"), 0.0)),
                ], dtype=np.float32)
            else:
                # 위협 없음 -> 안전 기본값 (먼 거리, TTC 최대)
                vec = np.array([1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                               dtype=np.float32)
            neighbor_vecs.append(vec)

        state = np.concatenate([ego_state, nav_intent] + neighbor_vecs)
        return np.nan_to_num(state.astype(np.float32), nan=0.0)


    # 보상 함수
    def _calculate_reward(self, drive_a, s_dist, v_ttc,
                          aoi, coll, emg, threats, stop_time,
                          junction_entered_now: bool = False,
                          junction_exited_now:  bool = False,
                          junction_order: int = 1) -> float:
        """
        보상:
        - 충돌: -20 (즉시 종료)
        - 기본 페널티: -0.01/step (정체 방지)
        - TTC 기반: 위험도에 따라 지수 함수 패널티
        - 유령 제동: 안전 상황에서 제동 반복 시 누적 패널티
        - 속도: 안전 상황에서 목표 속도 접근 보상
        - 교차로 진입: 상황별 차등 보상
        - 교차로 통과: +4.0
        """
        if coll:
            return -20.0

        reward = -0.01  # 매 스텝 기본 패널티

        if self.ego_id not in traci.vehicle.getIDList():
            return 0.0

        ego_v = traci.vehicle.getSpeed(self.ego_id)
        ego_in_j = self._is_in_junction(self.ego_id)
        ego_near_j = self._is_approaching_junction()

        dyn_critical, dyn_caution = self._get_dynamic_ttc_thresholds(ego_v)

        critical = (v_ttc < dyn_critical) or (emg == 1)
        caution = (dyn_critical <= v_ttc < dyn_caution) and not critical
        safe = (v_ttc >= dyn_caution) and (s_dist > 20.0) and (emg == 0)
        has_threat = len(threats) > 0
        occlusion = (s_dist >= SENSOR_MAX_RANGE
                       and any(t["is_v2v"] for t in threats))
        waiting_for_order = (junction_order >= 2)  # 교차로 대기 중 여부

        # 불필요한 정차 패널티 (위협/대기 없는데 멈춘 경우)
        if ego_v <= 0.1 and safe and not has_threat and not waiting_for_order:
            reward -= 0.5
            if stop_time > 10.0:
                reward -= 0.5

        # 과도한 교차로 대기 패널티
        max_wait = 30.0 if waiting_for_order else 15.0
        if ego_v <= 0.1 and stop_time >= max_wait and has_threat:
            reward -= 2.0

        # 정차 중 불필요한 제동 패널티
        if ego_v <= 0.1 and drive_a <= 1 and not waiting_for_order:
            reward -= 0.5

        # TTC 기반 위험 패널티 (지수 감쇠)
        if v_ttc < dyn_caution:
            reward += float(-3.0 * np.exp(-v_ttc / 1.5))

        for t in threats:
            if t["lat_overlap"] and t["lon_ttc"] < dyn_caution:
                reward += float(-1.0 * np.exp(-t["lon_ttc"] / 2.0))

        # 유령 제동 방지: 안전 상황에서 제동 반복 시 누적 패널티
        if safe:
            if drive_a <= 1:
                self._ghost_brake_count += 1
                reward -= min(self._ghost_brake_count, 10) * 0.2
            else:
                self._ghost_brake_count = max(0, self._ghost_brake_count - 1)
        else:
            self._ghost_brake_count = max(0, self._ghost_brake_count - 1)

        if drive_a == 0: reward -= 0.5 # 긴급제동 남용 방지
        elif drive_a == 4: reward -= 0.2 # 풀가속 남용 방지

        # 위험 수준별 행동 보상/패널티
        if critical:
            if drive_a <= 1: reward -= 0.50 # 올바른 제동
            elif drive_a == 2: reward -= 1.50 # 유지 -> 위험
            else: reward -= 3.00 # 가속 -> 매우 위험

        elif caution:
            if drive_a <= 1: reward += 0.20
            elif drive_a == 2: reward -= 0.20
            else: reward -= 0.80

        elif safe:
            speed_ratio = ego_v / TARGET_SPEED
            if ego_near_j or ego_in_j:
                # 교차로 근처: 적정 속도(50~80%) 유지
                if 0.5 <= speed_ratio <= 0.8: reward += 0.20
                elif speed_ratio > 0.9: reward -= 0.20
            else:
                if ego_v > 2.0: reward += 0.40 * speed_ratio
                else: reward -= 0.25

            if drive_a == 3 and ego_v < TARGET_SPEED * 0.95:
                reward += 0.20 # 목표 속도 미달 시 가속

        # V2V로만 인지한 위협(사각지대)에 제동하면 보상
        if occlusion and (critical or caution) and drive_a <= 1:
            reward += 0.20

        # 교차로 진입 보상 (상황 차등)
        if junction_entered_now:
            if safe and not has_threat: reward += 2.0
            elif safe and has_threat and drive_a >= 2: reward += 3.0
            elif caution and drive_a <= 1: reward += 0.5
            else: reward -= 1.0

        # 교차로 통과 보상
        if junction_exited_now:
            reward += 4.0

        return float(np.clip(reward, -5.0, 5.0))


    # render / close
    def render(self, mode: str = "human"):
        pass

    def close(self):
        self._cleanup_mininet()
        if self.traci_started:
            try:
                traci.close()
            except Exception:
                pass
            self.traci_started = False
        if self._sumo_proc is not None:
            try:
                self._sumo_proc.terminate()
                self._sumo_proc.wait(timeout=3)
            except Exception:
                pass
            self._sumo_proc = None
