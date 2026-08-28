# 실험 결과 요약

이 문서는 `v2v_rebuild`에서 지금까지 진행한 실험 결과를 정리한다.

## 실험 목적

기존 프로젝트는 V2V 기반 PPO 자율주행을 목표로 했지만, 발표 과정에서
"V2V와의 연결성이 약하다"는 지적을 받았다. 또한 실제 학습 결과에서 사고율이
높아 보상체계와 환경 설계가 적절한지 판단하기 어려웠다.

따라서 본 재구성 실험의 목적은 다음과 같다.

1. 단순한 교차로 충돌회피 문제가 PPO로 학습 가능한지 확인한다.
2. V2V 조기 관측이 sensor-only 관측보다 안전정책 학습에 유리한지 확인한다.
3. lossy V2V(PDR/AoI)와 SUMO 이식으로 결론을 재현·확장한다.
4. 기상·NLOS·중간 다중 장면에서도 같은 주장이 유지되는지 확인한다.

## 환경 개요

현재 환경은 `simple_intersection_env.py`에 구현되어 있다.

- ego 차량은 남쪽에서 북쪽으로 주행한다.
- target 차량은 동쪽에서 서쪽으로 교차한다.
- 충돌 예상 지점은 원점 `(0, 0)`이다.
- 에이전트는 ego 차량의 종방향 가감속만 제어한다.
- 행동은 5개다.
  - 0: 급제동
  - 1: 감속
  - 2: 유지
  - 3: 가속
  - 4: 최대가속

관측 모드는 두 가지다.

- `perfect_v2v`: target 차량 정보를 조기에 관측한다.
- `sensor_only`: target 차량이 센서 범위 안에 들어온 뒤에야 관측한다.

## Stage 1 결과

목표:

- 환경이 학습 가능한 문제인지 확인한다.
- 고정 정책은 위험하고, rule-based 정책은 안전해야 한다.

기준선 결과:

- `runs/baseline_v3/summary.csv`

주요 결과:

| 관측 모드 | 정책 | 충돌률 | 도착률 | near miss |
| --- | --- | ---: | ---: | ---: |
| perfect_v2v | hold | 29.5% | 70.5% | 48.5% |
| perfect_v2v | full | 36.5% | 63.5% | 62.0% |
| perfect_v2v | rule | 0.0% | 100.0% | 0.0% |
| sensor_only | hold | 29.5% | 70.5% | 48.5% |
| sensor_only | full | 36.5% | 63.5% | 62.0% |
| sensor_only | rule | 0.0% | 100.0% | 8.5% |

해석:

- 단순히 유지하거나 최대가속하는 정책은 사고율이 높다.
- rule-based 정책은 충돌을 회피할 수 있다.
- 따라서 이 환경은 "풀 수 있는 충돌회피 문제"로 볼 수 있다.

## Stage 1 PPO 디버깅

초기 PPO 학습에서는 다음 문제가 나타났다.

- 첫 번째 PPO는 사고율은 낮췄지만, brake와 full accel을 번갈아 쓰는 거친
  정책을 보였다.
- 두 번째 보상 수정에서는 항상 brake를 선택하는 정책으로 붕괴했다.
- 이후 "멈춘 뒤 재출발" 보상을 추가해 통과 정책을 학습하도록 수정했다.

최종 Stage 1 모델:

- `runs/simple_ppo_50k_v3.zip`

평가 결과:

| 학습 모드 | 평가 모드 | 충돌률 | 도착률 | near miss | 평균 보상 |
| --- | --- | ---: | ---: | ---: | ---: |
| perfect_v2v | perfect_v2v | 0.0% | 100.0% | 0.0% | 55.815 |
| perfect_v2v | sensor_only | 47.0% | 53.0% | 69.5% | -18.627 |

해석:

- V2V 조기 관측을 전제로 학습한 정책은 같은 조건에서는 매우 안정적이다.
- 하지만 sensor-only 조건에서는 target을 늦게 보게 되어 사고율이 크게 증가한다.

## Stage 2 결과

목표:

- `perfect_v2v`와 `sensor_only`를 각각 따로 학습해 공정하게 비교한다.

모델:

- `perfect_v2v` 학습 모델: `runs/simple_ppo_50k_v3.zip`
- `sensor_only` 학습 모델: `runs/simple_ppo_sensor_50k_v3.zip`

결과 파일:

- `runs/stage2_2x2_comparison.csv`

핵심 비교:

| 학습 모드 | 평가 모드 | 충돌률 | 도착률 | near miss | 평균 보상 |
| --- | --- | ---: | ---: | ---: | ---: |
| perfect_v2v | perfect_v2v | 0.0% | 100.0% | 0.0% | 55.815 |
| sensor_only | sensor_only | 45.5% | 54.5% | 69.0% | -13.998 |

해석:

- 같은 5만 step PPO 학습 조건에서 V2V 조기 관측 모델은 안정적으로 통과했다.
- sensor-only 모델은 따로 학습해도 사고율이 45.5%로 높게 남았다.
- 현재 단순 blind-intersection 환경에서는 V2V 조기 관측이 안전정책 학습에
  필수적인 정보로 작동한다.

## 현재 결론

현재까지의 재구성 결과는 다음 주장으로 정리할 수 있다.

> 교차로 blind-spot 상황에서 PPO 에이전트는 센서-only 관측만으로는 안정적인
> 충돌회피 정책을 학습하기 어렵다. 반면 V2V를 통해 교차 차량 정보를 조기에
> 관측하면 같은 PPO 구조에서도 충돌률을 크게 낮출 수 있다.

이 결론은 기존 프로젝트의 약점이었던 "V2V와의 연결성 부족"을 보완하는 핵심
방향이다.

## Stage 3 결과

목표:

- 완벽한 V2V가 아니라, 메시지 손실과 AoI가 있는 현실적인 V2V 조건을 추가한다.
- 통신 품질이 나빠질수록 성능이 어떻게 변하는지 확인한다.

추가한 요소:

- `lossy_v2v` 관측 모드
- 거리 기반 PDR
- 메시지 dropout
- BSM AoI
- episode당 V2V 수신 횟수

모델:

- `runs/simple_ppo_lossy_50k_stage3.zip`

3x3 비교 결과:

- `runs/stage3_3x3_comparison.csv`

핵심 결과:

| 학습 모드 | 평가 모드 | 충돌률 | 도착률 | near miss | 평균 V2V 수신 |
| --- | --- | ---: | ---: | ---: | ---: |
| perfect_v2v | perfect_v2v | 0.0% | 100.0% | 0.0% | 0.000 |
| lossy_v2v | lossy_v2v | 0.0% | 100.0% | 0.0% | 46.685 |
| sensor_only | sensor_only | 45.5% | 54.5% | 69.0% | 0.000 |

PDR scale sweep:

| PDR scale | 평가 모드 | 충돌률 | 도착률 | near miss | 평균 AoI | 평균 V2V 수신 |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 1.0 | lossy_v2v | 0.0% | 100.0% | 0.0% | 0.106 | 46.685 |
| 0.5 | lossy_v2v | 0.0% | 100.0% | 0.0% | 0.305 | 23.035 |
| 0.25 | lossy_v2v | 2.5% | 97.5% | 5.0% | 0.578 | 10.325 |
| 0.1 | lossy_v2v | 17.0% | 83.0% | 33.5% | 1.315 | 3.405 |

해석:

- PDR scale이 낮아질수록 평균 V2V 수신 횟수가 감소한다.
- V2V 수신이 줄면 AoI가 증가하고, 충돌률과 near miss가 함께 증가한다.
- 그러나 PDR scale 0.1의 극단적인 조건에서도 sensor-only의 45.5% 사고율보다는
  낮은 17.0% 사고율을 보였다.

현재까지의 결론:

> V2V 조기 관측은 단순 교차로 blind-spot 상황에서 PPO 에이전트의 안전성을 크게
> 향상시킨다. 또한 통신 품질이 낮아질수록 성능이 점진적으로 저하되어, PDR/AoI가
> 자율주행 안전성에 영향을 주는 요소임을 확인할 수 있다.

## Stage 4 결과 (SUMO)

목표:

- Stage 3에서 검증한 관측/보상/V2V 모드를 TraCI-only SUMO 최소 교차로로 이식한다.
- Mininet과 기존 31차원 환경은 사용하지 않는다.

환경:

- `sumo_intersection_env.py` + `sumo_data/`
- ego: `S_in → N_out`, target: `E_in → W_out`
- 관측/행동/보상은 Stage 3와 동일 계열

Sanity check (`runs/sumo_baseline`, 50 episode):

| 관측 모드 | 정책 | 충돌률 | 도착률 |
| --- | --- | ---: | ---: |
| perfect_v2v | rule | 0.0% | 100.0% |
| perfect_v2v | hold | 16.0% | 84.0% |
| perfect_v2v | full | 38.0% | 62.0% |
| sensor_only | rule | 4.0% | 96.0% |

PPO 3x3 (`runs/sumo_stage4_3x3_comparison.csv`, 100 episode):

| 학습 모드 | 평가 모드 | 충돌률 | 도착률 | near miss |
| --- | --- | ---: | ---: | ---: |
| perfect_v2v | perfect_v2v | 0.0% | 100.0% | 0.0% |
| lossy_v2v | lossy_v2v | 0.0% | 100.0% | 0.0% |
| sensor_only | sensor_only | 35.0% | 65.0% | 60.0% |
| perfect_v2v | sensor_only | 40.0% | 60.0% | 62.0% |

해석:

- SUMO에서도 V2V 조기 관측 모델은 무사고 통과가 가능하다.
- sensor-only는 같은 5만 step PPO로도 충돌률 35%가 남는다.
- Stage 3 kinematic 결론이 SUMO에서도 같은 방향으로 재현되었다.

## Stage 4 확장 결과

### SUMO PDR sweep

| PDR scale | 충돌률 | 도착률 | near miss | 평균 AoI | 평균 V2V 수신 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1.0 | 0.0% | 100.0% | 0.0% | 0.098 | 52.85 |
| 0.5 | 0.0% | 100.0% | 0.0% | 0.280 | 26.12 |
| 0.25 | 3.0% | 97.0% | 3.0% | 0.725 | 12.17 |
| 0.1 | 6.0% | 94.0% | 13.0% | 1.156 | 4.68 |

### 기상 proxy (sensor_range 축소)

| 모델 | 35m | 25m | 15m | 10m |
| --- | ---: | ---: | ---: | ---: |
| perfect | 0% | 0% | 0% | 0% |
| lossy | 0% | 0% | 0% | 0% |
| sensor | 35% | 41% | 40% | 42% |

### NLOS 폐색

| 모델 | LOS 충돌 | NLOS 충돌 |
| --- | ---: | ---: |
| perfect | 0% | 0% |
| lossy | 0% | 0% |
| sensor | 35% | 39% |

### 중간 multi-NPC (rule 주변차)

| 모델 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect | 0.0% | 100.0% | 0.0% |
| lossy | 0.0% | 100.0% | 0.0% |
| sensor | 15.0% | 85.0% | 47.0% |

해석:

- SUMO에서도 PDR 저하·기상(사거리 축소)·NLOS 폐색·중간 다중 장면에서
  V2V가 sensor-only보다 안정적이다.
- 센서 실패 모드와 장면 복잡도를 늘려도 “V2V가 센서 한계를 보완한다”는
  주장이 유지된다.
- 기본 `pdr_scale=1.0` lossy가 perfect와 같이 0%인 것은, lossy =
  (센서 OR fresh BSM)이고 조기 수신이 몇 번만 있어도 양보가 가능하기
  때문이다. PDR sweep에서 scale을 낮추면 충돌률이 함께 올라가므로
  lossy가 perfect의 단순 복제는 아니다.

현재까지의 최종 결론:

> 교차로 blind-spot에서 센서만으로는 충돌회피 정책 학습이 어렵지만, V2V 조기
> 정보로 그 한계를 극복할 수 있다. 이 결과는 kinematic·SUMO, 그리고
> 통신저하·기상·NLOS·중간 다중 장면에서도 같은 방향으로 나타난다.

## 다음 실험

현재 작품의 최종 검증은 새로운 simulator나 perception 모델을 추가하는 대신,
동일한 학습 조건의 반복과 재현성 정리에 집중한다.

### 최종 검증 프로토콜

- 관측 모드: `sensor_only`, `perfect_v2v`, `lossy_v2v`
- 학습 알고리즘: 모든 조건에서 동일한 Stable-Baselines3 PPO
- 기본 학습량: mode별 50,000 steps
- 기본 평가량: seed별 100 episodes
- 권장 seed: `0`, `1`, `2`
- 핵심 지표: collision rate, arrival rate, near-miss rate
- 통신 지표: mean final AoI, mean V2V receptions
- 결과 파일: `runs/multiseed/{simple,sumo}/raw_results.csv`,
  `runs/multiseed/{simple,sumo}/summary.csv`

반복 실행은 다음 명령으로 수행한다.

```powershell
python run_multiseed.py --environment simple --timesteps 50000 --episodes 100 --seeds 0 1 2
python run_multiseed.py --environment sumo --timesteps 50000 --episodes 100 --seeds 0 1 2
```

`summary.csv`의 `*_mean`, `*_std`, `*_ci95_half_width`를 최종 발표용 통계로
사용한다. 기존 단일 seed 결과와 반복 결과가 다르면 seed·episode·모델 경로를
함께 기록하고, 어느 결과를 사용했는지 발표자료에 명시한다.

### 다중 seed 반복 결과

seed 0, 1, 2에서 각 mode를 50,000 step 학습하고, seed별 100 episode를
학습한 mode에서 평가했다.

- kinematic:
  - `perfect_v2v`: 충돌률 `0.0% ± 0.0%`, 도착률 `100.0% ± 0.0%`
  - `lossy_v2v`: 충돌률 `0.0% ± 0.0%`, 도착률 `100.0% ± 0.0%`
  - `sensor_only`: 충돌률 `43.3% ± 3.1%`, 도착률 `56.7% ± 3.1%`,
    near miss `67.7% ± 7.6%`
- SUMO:
  - `perfect_v2v`: 충돌률 `0.0% ± 0.0%`, 도착률 `100.0% ± 0.0%`
  - `lossy_v2v`: 충돌률 `0.0% ± 0.0%`, 도착률 `100.0% ± 0.0%`
  - `sensor_only`: 충돌률 `16.0% ± 15.9%`, 도착률 `84.0% ± 15.9%`,
    near miss `34.0% ± 22.9%`

위 `±` 값은 seed 간 표준편차이며, 원본 값과 95% CI half-width는 각각
`runs/multiseed/simple/summary.csv`와 `runs/multiseed/sumo/summary.csv`에
저장되어 있다. SUMO sensor-only의 seed 간 편차가 크므로 “항상 16%”가 아니라
“평균 16%, seed에 따른 변동이 있음”으로 해석한다.

### 최종 범위 밖

- CARLA/OpenCDA 및 대규모 cooperative perception
- Veins/ns-3 기반 실제 무선 통신
- V2V-GoT/LLaVA 기반 MLLM
- 완전 다중에이전트 RL

위 항목은 Stage 5 이후의 후속 연구로 분리한다.

## 다음 대화에서 진행할 실험

기존 프로젝트의 목표였던 다중 배경교통을 현재의 검증 구조에 되돌린다.
학습 에이전트는 ego 1대로 유지하고, target을 포함한 rule-based 배경차량을
3~4대로 늘린다. 모든 관측 모드에서 같은 차량·경로·속도·seed를 사용하고,
`sensor_only`, `perfect_v2v`, `lossy_v2v`의 정보 채널만 비교한다.

이 실험의 위치는 다음과 같다.

- 핵심 인과 실험: 1 ego + 1 target
- 보조 robustness 실험: 1 ego + 3~4 rule-based background vehicles
- 이번 범위에 포함하지 않음: 여러 학습 차량, MAPPO, CARLA, 실제 무선통신

초기 구현에서는 기존 6차원 observation을 유지하고, TTC가 가장 급한 차량을
`primary_threat`로 선택한다. 대신 ego뿐 아니라 전체 차량의 충돌·완주 여부를
별도로 기록한다. 여러 상대차량의 정보를 모두 observation에 넣는 것은
primary-threat 실험 결과를 확인한 뒤 결정한다.
