# /home/ubuntu/v2x-project/new_v2v/evaluate_0512.py

# 사용법:
# 세 방향 한 번에
# sudo -E ~/v2x-env/bin/python3 evaluate_0512.py --nav_intent all

# 모델 경로 직접 지정
# sudo -E ~/v2x-env/bin/python3 evaluate_0512.py \
# --model ./checkpoints_0512/0512_2000000_steps \
# --nav_intent all

import os
import sys
import time
import logging
import argparse
import numpy as np
import pandas as pd
import traci

from stable_baselines3 import PPO

from v2x_env_0512 import (
    RealWorldV2XEnv,
    SENSOR_MAX_RANGE, V2X_MAX_RANGE, TARGET_SPEED,
    TTC_CRITICAL_MIN, TTC_CAUTION_MIN,
    HALF_LANE_WIDTH,
    BSM_HZ, N_NEIGHBORS,
    AEB_DECEL, NORMAL_DECEL,
)

# 날짜/체크포인트 태그로 출력 파일명 일괄 관리
DATE_PREFIX = "0512"
STEP_TAG = "1.3M"
SUMO_CONFIG_PATH = "../sumo_data_multi/map.sumocfg"
CHECKPOINT_DIR = "./checkpoints_0512"

DEFAULT_MODEL = os.path.join(CHECKPOINT_DIR, f"{DATE_PREFIX}_1300000_steps")

N_EPISODES = 100
LOG_INTERVAL = 30 # 매 N스텝마다 로그 출력

DRIVE_ACTIONS = {
    0: "긴급제동(AEB)",
    1: "일반감속",
    2: "속도유지",
    3: "일반가속",
    4: "최고속도",
}


def _get_dynamic_ttc_thresholds(ego_v: float) -> tuple:
    margin = np.clip(ego_v / 10.0, 0.5, 2.0)
    caution_ttc = (ego_v / (2 * NORMAL_DECEL)) + margin
    critical_ttc = (ego_v / (2 * AEB_DECEL)) + (margin * 0.5)
    return max(critical_ttc, TTC_CRITICAL_MIN), max(caution_ttc, TTC_CAUTION_MIN)


def _make_result_file(intent: str) -> str:
    return f"{DATE_PREFIX}_results_{STEP_TAG}_{intent}.csv"

def _make_log_file(intent: str) -> str:
    return f"{DATE_PREFIX}_details_{STEP_TAG}_{intent}.log"


def _make_logger(intent: str) -> logging.Logger:
    log_file = _make_log_file(intent)
    name = f"evaluate_{DATE_PREFIX}_{intent}"
    log = logging.getLogger(name)
    log.setLevel(logging.INFO)
    if log.handlers:
        log.handlers.clear()
    fh = logging.FileHandler(log_file, mode="w", encoding="utf-8")
    fh.setLevel(logging.INFO)
    fh.setFormatter(logging.Formatter("%(message)s"))
    sh = logging.StreamHandler()
    sh.setLevel(logging.WARNING)
    sh.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
    log.addHandler(fh)
    log.addHandler(sh)
    return log


def initial_cleanup():
    # 평가 시작 전 잔여 Mininet 프로세스 정리
    os.system("sudo mn -c > /dev/null 2>&1")
    time.sleep(1.0)


def _safe_pct(num: int, den: int) -> float:
    # 0 나누기 방지
    return (num / den * 100) if den > 0 else 0.0


def _fresh_ep_data() -> dict:
    # 에피소드 시작 시 지표 수집용 딕셔너리 초기화
    return {
        "speeds": [],
        "ttcs": [],
        "cbrs": [],
        "n_threats": [],
        "stop_times": [],
        "j_orders": [],
        "steps": 0,
        "arrived": 0,
        "collision": 0,
        "timeout": 0,
        "stop_steps": 0,
        "hard_brakes": 0,
        "near_misses": 0,
        "ghost_braking": 0,
        "v2v_detects": 0,
        "emergencies_rcvd": 0,
        "ego_disappeared": 0,
        "ego_route": "알수없음",
        "nav_intent": "알수없음",
        "drive_dist": {i: 0 for i in range(5)},
        "state_steps": {"critical": 0, "caution": 0, "safe": 0, "other": 0},
        "collision_details": [],
        "pre_collision_ttc": 10.0,
        "freeze_events": 0, # stop_time >= 30s & has_threat (j_order>=2면 기준 완화)
        "opportunity_loss": 0, # stop_time > 10s & safe & no_threat & j_order==1
        "valid_waits": 0, # j_order >= 2로 정차한 정상 대기 횟수
        "ghost_brake_j1": 0, # j_order==1인데 안전 상황에서 제동 (실제 유령 제동)
    }


def _get_threat_details(env: RealWorldV2XEnv) -> list:
    # 이웃 테이블에서 충돌 후보 차량만 상세 정보 반환 (로그용)
    details = []
    if env.ego_id not in traci.vehicle.getIDList():
        return details
    try:
        ego_pos = np.array(traci.vehicle.getPosition(env.ego_id))
        ego_heading = traci.vehicle.getAngle(env.ego_id)
        rad_ego = np.radians(ego_heading)
        ego_dir = np.array([np.sin(rad_ego), np.cos(rad_ego)])
        ego_speed = traci.vehicle.getSpeed(env.ego_id)

        for vid, data in env.neighbor_table.items():
            if not env._is_collision_candidate(
                    env.ego_id, ego_pos, ego_heading, vid, data, ego_speed):
                continue
            try:
                route = traci.vehicle.getRoute(vid)
                intent = f"{' -> '.join(route)}" if route else "알수없음"
            except Exception:
                intent = "알수없음"
            rel_pos = data["pos"] - ego_pos
            lateral = float(np.cross(ego_dir, rel_pos))
            lat_overlap = abs(lateral) <= HALF_LANE_WIDTH * 1.5
            source = "V2V" if data["is_v2v"] else "Sensor"
            details.append({
                "vid": vid,
                "dist": data["est_dist"],
                "speed": data["speed"],
                "intent": intent,
                "lat_overlap": lat_overlap,
                "lateral_m": lateral,
                "source": source,
                "is_aeb": data["is_aeb"],
            })
        details.sort(key=lambda x: x["dist"])
    except Exception:
        pass
    return details


def _make_step_log(env: RealWorldV2XEnv, info: dict,
                   step: int, threat_details: list,
                   dyn_critical: float, dyn_caution: float) -> str:
    # 스텝별 상태/위협 로그 생성
    drive_name = DRIVE_ACTIONS[info["drive_action"]]
    ttc = info["min_ttc"]
    emg = info["emergency"]
    s_dist = info["sensor_dist"]
    ego_kmh = info["speed"] * 3.6
    stop_time = info.get("stop_time", 0.0)
    j_order = info.get("junction_order", 1)

    if emg == 1 or ttc < dyn_critical:
        state_str = f"CRITICAL TTC:{ttc:.2f}s (임계:{dyn_critical:.2f}s)"
        if emg == 1:
            state_str += " [긴급이벤트수신]"
    elif ttc < dyn_caution:
        state_str = f"CAUTION TTC:{ttc:.2f}s (임계:{dyn_caution:.2f}s)"
    else:
        state_str = f"SAFE 전방:{s_dist:.1f}m"

    j_str = f" | 진입순서:{j_order}" if j_order > 1 else ""

    lines = [
        f"  [{step:04d}] {drive_name} | 에고:{ego_kmh:5.1f}km/h"
        + (f" | 정차:{stop_time:.1f}s" if stop_time > 0.5 else "")
        + j_str,
        f"  상태 : {state_str}",
    ]
    if threat_details:
        lines.append(f"  위협 : {len(threat_details)}대 (필터 후)")
        for i, t in enumerate(threat_details):
            is_last = (i == len(threat_details) - 1)
            prefix = "└─" if is_last else "├─"
            overlap = "겹침" if t["lat_overlap"] else "없음"
            aeb_flag = " 긴급" if t["is_aeb"] else ""
            vid_short = t["vid"][:12]
            lines.append(
                f"  {prefix} [{vid_short:<12s}]"
                f" 거리:{t['dist']:6.1f}m"
                f" 속도:{t['speed']:5.1f}m/s"
                f" 횡:{overlap}"
                f" 소스:{t['source']:<6s}"
                f" 의도:{t['intent']}"
                f"{aeb_flag}"
            )
    else:
        lines.append("  위협 : 없음")
    return "\n".join(lines)


def _record_collision_detail(env: RealWorldV2XEnv,
                              step: int, ttc: float) -> str:
    # 충돌 발생 시 ego 및 상대 차량 라우트/TTC 문자열 반환
    ego_id = env.ego_id
    try:
        colliders = traci.simulation.getCollidingVehiclesIDList()
        others = [v for v in colliders if v != ego_id]
        try:
            ego_route = traci.vehicle.getRoute(ego_id)
            ego_path = f"{' → '.join(ego_route)}" if ego_route else "알수없음"
        except Exception:
            ego_path = "알수없음(제거됨)"
        if others:
            detail_parts = []
            for other_id in others:
                try:
                    other_route = traci.vehicle.getRoute(other_id)
                    other_path = f"{' -> '.join(other_route)}" if other_route else "알수없음"
                except Exception:
                    other_path = "알수없음"
                detail_parts.append(
                    f"Ego({ego_path}) vs 상대({other_path}, id={other_id})"
                    f" 충돌TTC:{ttc:.2f}s step={step}"
                )
            detail = " | ".join(detail_parts)
        else:
            detail = f"Ego({ego_path}) 단독/원인불명 step={step}"
    except Exception as e:
        detail = f"충돌(경로 조회 실패: {e}) step={step}"
    return detail


def _get_ep_start_info(env: RealWorldV2XEnv) -> tuple:
    # 에피소드 시작 시 ego 라우트 문자열과 nav_intent 반환
    nav = env.nav_intent
    try:
        route = traci.vehicle.getRoute(env.ego_id)
        route_str = " -> ".join(route) if route else "알수없음"
    except Exception:
        route_str = "알수없음"
    return route_str, nav


def evaluate_single_intent(model: PPO, env: RealWorldV2XEnv,
                            intent: str, n_episodes: int) -> pd.DataFrame:
    logger = _make_logger(intent)
    result_file = _make_result_file(intent)
    log_file = _make_log_file(intent)

    env.nav_intent = intent

    print(f"\n{'='*60}")
    print(f" 방향: {intent.upper():8s} | 에피소드: {n_episodes}회 | 모델: {STEP_TAG}")
    print(f" 결과: {result_file}")
    print(f" 로그: {log_file}")
    print(f"{'='*60}")

    logger.info("=" * 70)
    logger.info(f" V2V PPO 에이전트 평가 [{DATE_PREFIX}] | 체크포인트: {STEP_TAG}")
    logger.info(f" nav_intent: {intent} | 에피소드: {n_episodes}회")
    logger.info("=" * 70)

    results = []
    route_str = "알수없음"

    for ep in range(n_episodes):
        obs, _ = env.reset()
        done = False
        ep_d = _fresh_ep_data()

        route_str, nav = _get_ep_start_info(env)
        ep_d["ego_route"] = route_str
        ep_d["nav_intent"] = nav

        logger.info(f"\n[Episode {ep+1:02d}/{n_episodes}] {'─'*50}")
        logger.info(f"  nav_intent : {nav}")
        logger.info(f"  에고 라우트: {route_str}")
        logger.info(f"  {'─'*50}")

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, done, _, info = env.step(action)

            # ego 소실: 시뮬레이션 도중 사라진 경우
            if info.get("termination") == "ego_disappeared":
                ep_d["ego_disappeared"] = 1
                logger.info("  에고 차량 소실 — 에피소드 즉시 종료")
                break

            if info.get("arrived", 0) == 1:
                ep_d["arrived"] = 1

            ep_d["steps"] += 1
            step = ep_d["steps"]
            ttc = info["min_ttc"]
            emg = info["emergency"]
            s_dist = info["sensor_dist"]
            ego_v = info["speed"]
            stop_time = info.get("stop_time", 0.0)
            j_order = info.get("junction_order", 1)

            dyn_critical, dyn_caution = _get_dynamic_ttc_thresholds(ego_v)

            # 기본 지표 수집
            ep_d["speeds"].append(ego_v)
            ep_d["ttcs"].append(ttc)
            ep_d["cbrs"].append(info["cbr"])
            ep_d["n_threats"].append(info["n_threats"])
            ep_d["stop_times"].append(stop_time)
            ep_d["j_orders"].append(j_order)
            ep_d["drive_dist"][info["drive_action"]] += 1

            # 센서 범위 밖 차량을 V2V로 탐지한 경우
            if info["n_threats"] > 0 and s_dist >= SENSOR_MAX_RANGE:
                ep_d["v2v_detects"] += 1

            # 위험 상태 분류 (동적 TTC 기준)
            if emg == 1 or ttc < dyn_critical:
                ep_d["state_steps"]["critical"] += 1
            elif ttc < dyn_caution:
                ep_d["state_steps"]["caution"] += 1
            elif s_dist > 20.0 and emg == 0:
                ep_d["state_steps"]["safe"] += 1
            else:
                ep_d["state_steps"]["other"] += 1

            # 이벤트 카운트
            if ego_v < 0.5:
                ep_d["stop_steps"] += 1
            if info["drive_action"] == 0:
                ep_d["hard_brakes"] += 1
            if ttc < 2.0:
                ep_d["near_misses"] += 1
            if emg == 1:
                ep_d["emergencies_rcvd"] += 1

            safe_state = (ttc >= dyn_caution and emg == 0 and s_dist > 20.0)
            has_threat = info["n_threats"] > 0
            waiting = (j_order >= 2)  # 교차로 진입 대기 중

            # 유령 제동: 안전 상황에서 제동
            if info["drive_action"] <= 1 and safe_state:
                ep_d["ghost_braking"] += 1
                if not waiting:
                    ep_d["ghost_brake_j1"] += 1

            # 정상 대기: j_order >= 2로 정차한 경우
            if ego_v <= 0.1 and waiting:
                ep_d["valid_waits"] += 1

            # Freeze: 대기 순위 아닌데 장시간 정차
            freeze_threshold = 30.0 if waiting else 15.0
            if ego_v <= 0.1 and stop_time >= freeze_threshold and has_threat:
                ep_d["freeze_events"] += 1

            # 기회 상실: j_order==1 + 위협 없는데 10초보다 길게 정차
            if ego_v <= 0.1 and safe_state and not has_threat and not waiting and stop_time > 10.0:
                ep_d["opportunity_loss"] += 1

            # 충돌 처리 (최초 1회만 기록)
            if info["collision"] and ep_d["collision"] == 0:
                ep_d["collision"] = 1
                detail = _record_collision_detail(env, step, ep_d["pre_collision_ttc"])
                ep_d["collision_details"].append(detail)
                logger.info(f"\n  [{step:04d}] 사고 발생")
                logger.info(f"   {detail}")
                td = _get_threat_details(env)
                logger.info(_make_step_log(env, info, step, td, dyn_critical, dyn_caution))

            # 스텝 로그: 1번 스텝 / 위험 상태 / 유령 제동 / 주기 로그
            should_log = (
                step == 1
                or emg == 1
                or ttc < dyn_caution
                or (info["drive_action"] <= 1 and safe_state)
                or ep_d["freeze_events"] > 0
                or ep_d["opportunity_loss"] > 0
                or (step % LOG_INTERVAL == 0)
            )
            if should_log:
                td = _get_threat_details(env)
                logger.info(_make_step_log(env, info, step, td, dyn_critical, dyn_caution))
                if info["drive_action"] <= 1 and safe_state:
                    if not waiting:
                        logger.info(
                            f"   유령 제동 (j_order=1, 누적 {ep_d['ghost_brake_j1']}회)"
                        )
                    else:
                        logger.info(
                            f"   순위 대기 정상 제동 (j_order={j_order}, 누적 {ep_d['valid_waits']}회)"
                        )
                if ego_v <= 0.1 and stop_time >= freeze_threshold and has_threat:
                    logger.info(
                        f"   과도 대기(Freeze) (정차:{stop_time:.1f}s, 기준:{freeze_threshold:.0f}s)"
                    )
                if ego_v <= 0.1 and safe_state and not has_threat and not waiting and stop_time > 10.0:
                    logger.info(
                        f"   기회 상실(Opportunity Loss) (정차:{stop_time:.1f}s, j_order=1)"
                    )

            if not info["collision"]:
                ep_d["pre_collision_ttc"] = ttc # 충돌 직전 TTC 갱신

        # 에피소드 종료 처리
        completed = int(
            ep_d["collision"] == 0
            and ep_d["ego_disappeared"] == 0
            and ep_d["arrived"] == 1
        )
        if not completed and ep_d["collision"] == 0 and ep_d["ego_disappeared"] == 0:
            ep_d["timeout"] = 1

        s = ep_d["steps"]
        avg_speed = np.mean(ep_d["speeds"]) * 3.6 if ep_d["speeds"] else 0.0
        avg_ttc = np.mean(ep_d["ttcs"]) if ep_d["ttcs"] else 10.0
        min_ttc = np.min(ep_d["ttcs"]) if ep_d["ttcs"] else 10.0
        travel_t = s * 0.1
        stop_t = ep_d["stop_steps"] * 0.1
        max_stop_t = max(ep_d["stop_times"]) if ep_d["stop_times"] else 0.0
        avg_j_ord = np.mean(ep_d["j_orders"]) if ep_d["j_orders"] else 1.0
        avg_threats = np.mean(ep_d["n_threats"]) if ep_d["n_threats"] else 0.0

        if ep_d["collision"]: mark = "충돌"
        elif ep_d["timeout"]: mark = "타임아웃"
        elif ep_d["ego_disappeared"]: mark = "소실"
        else: mark = "완주"

        logger.info(
            f"\n  종료 상태: {mark}"
            f" | {travel_t:.1f}s"
            f" | 급제동:{ep_d['hard_brakes']}회"
            f" | MinTTC:{min_ttc:.2f}s"
            f" | 유령제동(j1):{ep_d['ghost_brake_j1']}회"
            f" | 정상대기:{ep_d['valid_waits']}회"
            f" | 정차시간:{stop_t:.1f}s"
            f" | 최대연속정차:{max_stop_t:.1f}s"
            f" | Freeze:{ep_d['freeze_events']}회"
            f" | 기회상실:{ep_d['opportunity_loss']}회"
            f" | 평균j_order:{avg_j_ord:.2f}"
        )
        print(
            f"  Ep{ep+1:02d} {mark:5s}"
            f" nav={nav:<8s}"
            f" 시간:{travel_t:4.1f}s"
            f" 급제동:{ep_d['hard_brakes']:3d}회"
            f" MinTTC:{min_ttc:4.2f}s"
            f" 정차:{stop_t:4.1f}s"
            f" Freeze:{ep_d['freeze_events']}회"
            f" 정상대기:{ep_d['valid_waits']}회"
        )

        # 에피소드 결과를 CSV로 추가
        results.append({
            "Episode": ep + 1,
            "Nav_Intent": nav,
            "Ego_Route": route_str,
            "Steps": s,
            "Collision": ep_d["collision"],
            "Timeout": ep_d["timeout"],
            "Completed": completed,
            "Ego_Disappeared": ep_d["ego_disappeared"],
            "Travel_Time(s)": round(travel_t, 1),
            "Stop_Time(s)": round(stop_t, 1),
            "Max_Consec_Stop(s)": round(max_stop_t, 1),
            "Freeze_Events": ep_d["freeze_events"],
            "Opportunity_Loss": ep_d["opportunity_loss"],
            "Valid_Waits": ep_d["valid_waits"],
            "Ghost_Brake_Total": ep_d["ghost_braking"],
            "Ghost_Brake_J1": ep_d["ghost_brake_j1"],
            "Avg_Junction_Order": round(avg_j_ord, 2),
            "Hard_Brakes": ep_d["hard_brakes"],
            "Min_TTC(s)": round(min_ttc, 2),
            "Avg_TTC(s)": round(avg_ttc, 2),
            "Near_Misses": ep_d["near_misses"],
            "V2V_Detects": ep_d["v2v_detects"],
            "Emergencies_Rcvd": ep_d["emergencies_rcvd"],
            "Avg_Speed(km/h)": round(avg_speed, 2),
            "Avg_N_Threats": round(avg_threats, 1),
            "Avg_CBR(%)": round(np.mean(ep_d["cbrs"]) * 100 if ep_d["cbrs"] else 0.0, 1),
            "State_Critical(%)": round(_safe_pct(ep_d["state_steps"]["critical"], s), 1),
            "State_Caution(%)": round(_safe_pct(ep_d["state_steps"]["caution"], s), 1),
            "State_Safe(%)": round(_safe_pct(ep_d["state_steps"]["safe"], s), 1),
            "Act_AEB(%)": round(_safe_pct(ep_d["drive_dist"][0], s), 1),
            "Act_Decel(%)": round(_safe_pct(ep_d["drive_dist"][1], s), 1),
            "Act_Hold(%)": round(_safe_pct(ep_d["drive_dist"][2], s), 1),
            "Act_Accel(%)": round(_safe_pct(ep_d["drive_dist"][3], s), 1),
            "Act_Full(%)": round(_safe_pct(ep_d["drive_dist"][4], s), 1),
            "Accident_Info": " | ".join(ep_d["collision_details"]) or "없음",
        })

    # 전체 에피소드 완료 후 CSV 저장 및 요약 출력
    df = pd.DataFrame(results)
    df.to_csv(result_file, index=False)

    n = len(df)
    logger.info(f"\n{'='*70}")
    logger.info(f" 평가 요약 [{DATE_PREFIX} / {intent}] ({n}회)")
    logger.info(f"{'='*70}")
    logger.info(f"  성공률        : {df['Completed'].mean()*100:.1f}%")
    logger.info(f"  사고율        : {_safe_pct(df['Collision'].sum(), n):.1f}%")
    logger.info(f"  타임아웃율    : {_safe_pct(df['Timeout'].sum(), n):.1f}%")
    logger.info(f"  평균 MinTTC   : {df['Min_TTC(s)'].mean():.2f}s")
    logger.info(f"  아차사고      : 총 {df['Near_Misses'].sum()}회")
    logger.info(f"  유령 제동(j1) : 총 {df['Ghost_Brake_J1'].sum()}회 (평균 {df['Ghost_Brake_J1'].mean():.1f}회/ep)")
    logger.info(f"  정상 대기     : 총 {df['Valid_Waits'].sum()}회 (평균 {df['Valid_Waits'].mean():.1f}회/ep)")
    logger.info(f"  평균 j_order  : {df['Avg_Junction_Order'].mean():.2f}")
    logger.info(f"  평균 급제동   : {df['Hard_Brakes'].mean():.1f}회/ep")
    logger.info(f"  정차 시간     : 평균 {df['Stop_Time(s)'].mean():.1f}s")
    logger.info(f"  최대 연속 정차: 평균 {df['Max_Consec_Stop(s)'].mean():.1f}s")
    logger.info(f"  과도 대기(Freeze): 총 {df['Freeze_Events'].sum()}회 (평균 {df['Freeze_Events'].mean():.1f}회/ep)")
    logger.info(f"  기회 상실     : 총 {df['Opportunity_Loss'].sum()}회 (평균 {df['Opportunity_Loss'].mean():.1f}회/ep)")
    logger.info(f"  평균 속도     : {df['Avg_Speed(km/h)'].mean():.1f}km/h")
    logger.info(f"  V2V 탐지      : 총 {df['V2V_Detects'].sum()}회")
    logger.info(f"  긴급이벤트    : 총 {df['Emergencies_Rcvd'].sum()}회")
    logger.info(
        f"\n  행동 분포 (평균):"
        f" AEB={df['Act_AEB(%)'].mean():.1f}%"
        f" 감속={df['Act_Decel(%)'].mean():.1f}%"
        f" 유지={df['Act_Hold(%)'].mean():.1f}%"
        f" 가속={df['Act_Accel(%)'].mean():.1f}%"
        f" 풀가속={df['Act_Full(%)'].mean():.1f}%"
    )
    logger.info(
        f"\n  상태 분포 (평균, 동적 TTC 기준):"
        f" Critical={df['State_Critical(%)'].mean():.1f}%"
        f" Caution={df['State_Caution(%)'].mean():.1f}%"
        f" Safe={df['State_Safe(%)'].mean():.1f}%"
    )
    if df["Collision"].sum() > 0:
        logger.info("\n  사고 패턴:")
        for pattern in df[df["Collision"] == 1]["Accident_Info"]:
            logger.info(f"   {pattern}")

    logger.info(f"\n  결과 CSV : {result_file}")
    logger.info(f"  상세 로그: {log_file}")
    logger.info(f"{'='*70}\n")

    print(f"\n  [{intent}] 완료 -> {result_file}")
    return df


def print_comparison_summary(results: dict):
    """세 방향 평가 결과를 테이블 형태로 콘솔 출력"""
    print(f"\n{'='*95}")
    print(f" 방향별 최종 성능 요약")
    print(f"{'='*95}")
    print(
        f"  {'방향':<10} | {'성공률':>5} | {'충돌률':>5} | {'타임아웃':>6}"
        f" | {'MinTTC':>7} | {'급제동/ep':>9} | {'정차시간':>8}"
        f" | {'유령제동j1':>9} | {'정상대기':>8} | {'Freeze':>6} | {'기회상실':>8}"
    )
    print(f"  {'-'*91}")

    for intent in ["straight", "left", "right"]:
        if intent not in results:
            continue
        df = results[intent]
        n = len(df)
        suc = df["Completed"].mean() * 100
        col = _safe_pct(df["Collision"].sum(), n)
        to = _safe_pct(df["Timeout"].sum(), n)
        ttc = df["Min_TTC(s)"].mean()
        hb = df["Hard_Brakes"].mean()
        stp = df["Stop_Time(s)"].mean()
        gbj = df["Ghost_Brake_J1"].mean()
        vw = df["Valid_Waits"].mean()
        frz = df["Freeze_Events"].mean()
        opl = df["Opportunity_Loss"].mean()

        print(
            f"  {intent:<10} |"
            f" {suc:4.0f}% |"
            f" {col:4.0f}% |"
            f" {to:5.0f}% |"
            f" {ttc:5.2f}s |"
            f" {hb:6.1f}회 |"
            f" {stp:6.1f}s |"
            f" {gbj:7.1f}회 |"
            f" {vw:6.1f}회 |"
            f" {frz:5.1f}회 |"
            f" {opl:7.1f}회"
        )
    print(f"{'='*95}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"체크포인트 평가 [{DATE_PREFIX}]"
    )
    parser.add_argument(
        "--model", type=str,
        default=DEFAULT_MODEL,
        help=f"평가 모델 경로 (기본: {DEFAULT_MODEL})",
    )
    parser.add_argument(
        "--episodes", type=int, default=N_EPISODES,
        help=f"에피소드 수 (기본: {N_EPISODES})",
    )
    parser.add_argument(
        "--nav_intent", type=str, default="all",
        choices=["straight", "left", "right", "all"],
        help="평가 방향. 'all' = 세 방향 순서대로 각각 평가 (기본: all)",
    )
    args = parser.parse_args()

    model_zip = args.model if args.model.endswith(".zip") else args.model + ".zip"
    if not os.path.exists(model_zip):
        print(f"모델 파일 없음: {model_zip}")
        print("  사용 가능한 체크포인트:")
        if os.path.exists(CHECKPOINT_DIR):
            for f in sorted(os.listdir(CHECKPOINT_DIR)):
                if f.endswith(".zip"):
                    print(f"  {CHECKPOINT_DIR}/{f}")
        sys.exit(1)

    intents_to_run = (
        ["straight", "left", "right"]
        if args.nav_intent == "all"
        else [args.nav_intent]
    )

    initial_cleanup()

    print(f">>> 모델 로드: {model_zip}")
    model = PPO.load(model_zip)

    all_results = {}
    env = None

    try:
        for intent in intents_to_run:
            # 방향 전환마다 환경 재생성 + Mininet 정리
            if env is not None:
                try:
                    env.close()
                except Exception:
                    pass
                time.sleep(2.0)
                initial_cleanup()

            env = RealWorldV2XEnv(
                config_file=SUMO_CONFIG_PATH,
                nav_intent=intent,
                randomize_intent=False,
                build_network=True,
            )
            model.set_env(env)

            df = evaluate_single_intent(
                model, env,
                intent=intent,
                n_episodes=args.episodes,
            )
            all_results[intent] = df

        if args.nav_intent == "all":
            print_comparison_summary(all_results)

    except KeyboardInterrupt:
        print("\n>>> 평가 중단됨")

    except Exception as e:
        print(f"평가 오류: {e}")
        import traceback; traceback.print_exc()
        raise

    finally:
        if env is not None:
            try:
                env.close()
            except Exception:
                pass
