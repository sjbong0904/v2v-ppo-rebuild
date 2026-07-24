# 재구성 로드맵

## Stage 0: 기존 프로젝트 진단

목표: 기존 코드를 보존하면서, 기존 학습에서 사고율이 높게 나온 원인을 분리한다.

현재까지 확인한 문제:

- 기존 보상체계에서는 critical 상태에서 제동해도 음수 보상이 들어가는 부분이
  있었다. 즉, 올바른 제동이 명확한 양의 신호로 전달되지 않았다.
- V2V는 관측 feature로 존재했지만, 과제가 V2V 없이는 풀기 어려운 구조로
  명확히 설계되어 있지 않았다.
- SUMO traffic, 교차로 priority, TTC, AoI, PDR, ghost braking, route completion
  등이 한 번에 결합되어 있어 실패 원인을 디버깅하기 어려웠다.

결론:

- 기존 프로젝트 전체를 폐기하기보다는, 작은 환경에서 보상과 관측 구조를 먼저
  검증한 뒤 SUMO로 다시 이식하는 방향이 적절하다.

## Stage 1: 최소 충돌회피 환경

환경:

- ego 차량 1대
- 교차 차량 1대
- SUMO 미사용
- V2V 손실 없음
- 행동 공간: 급제동, 감속, 유지, 가속, 최대가속

완료 기준:

- rule-based TTC 정책이 거의 무사고로 통과해야 한다.
- 단순 고정 정책은 충분히 위험해야 한다.
- PPO가 고정 정책보다 나은 안전정책을 학습해야 한다.

진행 결과:

- `runs/baseline_v3/summary.csv`에서 rule policy는 충돌률 0%, 도착률 100%를
  기록했다.
- `runs/simple_ppo_50k_v3.zip`은 `perfect_v2v` 평가에서 200 episode 기준
  충돌률 0%, 도착률 100%, near miss 0%를 기록했다.
- 초기 보상에서는 PPO가 항상 brake를 선택하는 문제가 있었고, 멈춘 뒤 재출발
  보상을 추가해 해결했다.

## Stage 2: Sensor-only와 Perfect V2V 비교

목표:

- V2V 조기 관측이 실제로 안전정책 학습에 필요한지 확인한다.
- `perfect_v2v`와 `sensor_only`를 각각 따로 학습해 공정하게 비교한다.

관측 모드:

- `perfect_v2v`: 교차 차량 정보를 처음부터 관측할 수 있다.
- `sensor_only`: 교차 차량이 센서 범위 안에 들어온 뒤에야 관측할 수 있다.

완료 기준:

- sensor-only가 blind-intersection 상황에서 더 높은 사고율을 보여야 한다.
- perfect V2V는 충돌률과 near miss를 줄여야 한다.

진행 결과:

- `runs/stage2_2x2_comparison.csv`에 2x2 비교 결과를 저장했다.
- `perfect_v2v` 학습 + `perfect_v2v` 평가:
  - 충돌률 0%
  - 도착률 100%
  - near miss 0%
- `sensor_only` 학습 + `sensor_only` 평가:
  - 충돌률 45.5%
  - 도착률 54.5%
  - near miss 69.0%

해석:

- 단순 blind-intersection 환경에서는 V2V 조기 관측이 단순한 장식 feature가
  아니라 안전정책 학습 가능성을 크게 바꾼다.
- 이 결과는 발표에서 "V2V와의 연결성이 약하다"는 지적을 보완할 수 있는
  핵심 근거가 된다.

## Stage 3: Lossy V2V 추가

목표:

- perfect V2V를 현실적인 통신 환경으로 확장한다.
- V2V 메시지가 항상 완벽하게 들어오는 것이 아니라, 거리와 통신 품질에 따라
  손실/지연될 수 있음을 반영한다.

추가할 요소:

- 거리 기반 PDR
- 메시지 dropout
- AoI
- `lossy_v2v` 관측 모드

완료 기준:

- `perfect_v2v`, `lossy_v2v`, `sensor_only`를 비교할 수 있어야 한다.
- PDR이 낮아질수록 성능이 자연스럽게 저하되어야 한다.
- 평가 지표에 충돌률, 도착률, near miss, 평균 AoI, V2V 탐지 횟수를 포함한다.

진행 결과:

- `lossy_v2v` 관측 모드를 추가했다.
- 거리 기반 PDR, 메시지 dropout, BSM AoI, V2V 수신 횟수 지표를 추가했다.
- `runs/simple_ppo_lossy_50k_stage3.zip` 모델을 5만 step 학습했다.
- 기본 PDR scale 1.0에서 `lossy_v2v` 평가 결과:
  - 충돌률 0%
  - 도착률 100%
  - near miss 0%
  - episode당 평균 V2V 수신 46.685회
- PDR scale sweep 결과:
  - scale 0.5: 충돌률 0%, 평균 V2V 수신 23.035회
  - scale 0.25: 충돌률 2.5%, 평균 V2V 수신 10.325회
  - scale 0.1: 충돌률 17.0%, 평균 V2V 수신 3.405회

해석:

- 통신 품질이 충분하면 `lossy_v2v`도 `perfect_v2v`에 가까운 안전성을 보인다.
- PDR이 낮아질수록 V2V 수신 횟수는 줄고 AoI는 증가하며, 사고율도 함께 증가한다.
- sensor-only의 사고율 45.5%와 비교하면, 낮은 PDR 조건에서도 V2V는 일정 수준의
  안전성 이득을 제공한다.

## Stage 4: SUMO 환경으로 이식

목표:

- 작은 환경에서 검증한 관측/보상 구조를 SUMO 기반 환경으로 옮긴다.
- 기존 `v2x_env_0512.py`(Mininet, 배경 flow, 31차원 관측)는 재사용하지 않고,
  Stage 1-3과 동일한 최소 문제만 TraCI로 이식한다.

환경:

- `sumo_data/`: ego `S_in → N_out`, target `E_in → W_out` 십자 교차로
- `sumo_intersection_env.py`: Discrete(5), 6차원 관측, 보상은 Stage 3와 동일 계열
- 관측 모드: `perfect_v2v`, `sensor_only`, `lossy_v2v` (PDR/AoI는 시뮬, Mininet 없음)
- ego/target 모두 SpeedMode=0으로 SUMO 양보를 끄고 속도를 직접 제어

완료 기준:

- rule-based 정책이 perfect V2V에서 거의 무사고로 통과해야 한다.
- hold/full 고정 정책은 충분히 위험해야 한다.
- SUMO에서도 sensor-only / perfect V2V / lossy V2V ablation이 가능해야 한다.
- 최종 발표에서 V2V가 성능 향상에 필수적이었다는 점을 수치로 설명할 수
  있어야 한다.

진행 결과 (1차 sanity check):

- `python sumo_data/build_net.py`로 `map.net.xml` 생성
- `runs/sumo_baseline/summary.csv` (50 episode)
  - `perfect_v2v` + rule: 충돌률 0%, 도착률 100%
  - `perfect_v2v` + hold: 충돌률 16%, full: 충돌률 38%
  - `sensor_only` + rule: 충돌률 4%, near miss 16%
  - `lossy_v2v` + rule: 충돌률 0%, 도착률 100%

진행 결과 (2차 PPO):

- 모델: `runs/sumo_ppo_50k_{perfect,sensor,lossy}.zip` (각 5만 step)
- 비교: `runs/sumo_stage4_3x3_comparison.csv` (100 episode)
  - perfect 학습 + perfect 평가: 충돌률 0%, 도착률 100%
  - lossy 학습 + lossy 평가: 충돌률 0%, 도착률 100%
  - sensor 학습 + sensor 평가: 충돌률 35%, 도착률 65%, near miss 60%
  - perfect 학습 + sensor 평가: 충돌률 40%

해석:

- SUMO에서도 V2V 조기 관측이 안전정책 학습에 필수적이다.
- Stage 3 kinematic 결과와 같은 방향의 재현되었다.

진행 결과 (3차 PDR sweep):

- `runs/sumo_pdr_sweep_summary.csv`, 그림 `presentation/sumo_pdr_*.png`
  - PDR 1.0: 충돌 0%, V2V 수신 52.9
  - PDR 0.5: 충돌 0%, V2V 수신 26.1
  - PDR 0.25: 충돌 3%, V2V 수신 12.2
  - PDR 0.1: 충돌 6%, V2V 수신 4.7
- Stage 3와 같이 PDR↓ → 수신↓ → AoI↑ → 충돌↑ 패턴이 SUMO에서도 재현됨

진행 결과 (4차 기상·NLOS):

- 기상 proxy (`runs/sumo_weather_sensor_range_sweep.csv`)
  - perfect/lossy: sensor_range 10~35m 모두 충돌 0%
  - sensor-only: 35m 35% → 10m 42% (사거리 축소 시 악화)
- NLOS (`runs/sumo_nlos_comparison.csv`)
  - perfect/lossy + blocker: 충돌 0%
  - sensor-only: LOS 35% → NLOS 39%
  - occluded step fraction ~42–58%

진행 결과 (5차 중간 multi-NPC):

- ego(학습) + target(E→W) + npc(W→E, rule 고정속도)
- primary threat = 충돌점 도착 TTC가 더 급한 차량
- `runs/sumo_multi_npc_comparison.csv`
  - perfect: 충돌 0%, 도착 100%
  - lossy: 충돌 0%, 도착 100%
  - sensor: 충돌 15%, 도착 85%

Stage 4 종합 해석:

- kinematic에서 검증한 “V2V가 센서 한계를 보완한다”는 주장이 SUMO에서도 재현된다.
- 기본 lossy(`pdr_scale=1.0`)는 이 난이도에서 perfect에 가까운 안전성을 보이지만,
  PDR을 낮추면 충돌률이 상승해 통신 품질 민감도가 확인된다.
- 기상·NLOS·중간 다중 장면으로 센서 실패 모드를 넓혀도 V2V 우위가 유지된다.
- 완전 다중에이전트 RL(상대도 학습)은 향후 궁극 목표로 분리한다.

## Stage 5 (향후): 완전 다중에이전트 RL

목표 후보:

- 주변 차량도 학습 정책으로 두어 non-stationary multi-agent로 확장
- V2V를 통한 협력/양보 학습

전제:

- Stage 4까지의 단일 ego + rule NPC 중간 단계가 안정적일 것
- 관측/보상/통신 ablation을 유지한 채 에이전트 수만 늘릴 것

