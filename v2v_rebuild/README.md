# V2V 기반 PPO 자율주행 에이전트 재구성

이 디렉토리는 기존 졸업작품 코드를 바로 폐기하지 않고, V2V 기반 PPO
자율주행 문제를 작은 단위부터 다시 검증하기 위해 만든 재구성 작업공간이다.

**한 줄 주장:** 교차로 blind-spot에서 센서만으로는 충돌회피 정책 학습이 어렵지만,
V2V 조기 정보로 그 한계를 극복할 수 있다.

기존 프로젝트는 SUMO, PPO, TTC, V2V feature, PDR, AoI, 교차로 판단 등이 한
환경 안에 모두 들어가 있어 사고율이 높게 나왔을 때 원인을 분리하기 어려웠다.
따라서 이 재구성 버전에서는 다음 순서로 문제를 다시 쌓는다.

1. 단순 교차로 충돌회피 문제가 PPO로 학습 가능한지 확인한다.
2. 센서-only 관측과 V2V 조기 관측을 비교한다.
3. PDR, 메시지 손실, AoI를 추가해 lossy V2V 환경으로 확장한다.
4. 검증된 관측/보상 구조를 SUMO로 이식하고, 센서 실패 모드·중간 다중 장면까지 확장한다.

## 최종 지표 (canonical)

발표/보고서용 최종 수치는 아래 폴더를 사용한다.

- [`presentation/테스트지표2/`](presentation/테스트지표2/)
- 요약: [`presentation/테스트지표2/FINAL_지표요약.md`](presentation/테스트지표2/FINAL_지표요약.md)

| 시나리오 | Collision | Success | V2V early | 최초인지거리 | 최초인지 TTC |
| --- | ---: | ---: | ---: | ---: | ---: |
| sensor | 39.0% | 61.0% | 0% | 17.0 m | 0.49 s |
| multi_sensor | 54.0% | 46.0% | 0% | 16.0 m | 0.52 s |
| lossy | 0.0% | 100% | 100% | 108.0 m | 6.07 s |
| multi_lossy | 0.0% | 100% | 100% | 107.2 m | 5.88 s |
| lossy_pdr03 | 0.5% | 99.5% | 100% | 96.8 m | 5.15 s |
| multi_lossy_pdr03 | 4.0% | 96.0% | 100% | 96.0 m | 4.85 s |

## Quickstart

```powershell
pip install -r requirements.txt
python sumo_data/build_net.py   # 네트워크가 없을 때
python evaluate_sumo.py --model runs/sumo_ppo_50k_lossy.zip --mode lossy_v2v --episodes 50
python run_metric_eval.py --smoke 5   # 지표 로깅 스모크 테스트
```

GUI 데모: [`demo_package/README.md`](demo_package/README.md)

## 현재 단계

**Stage 4 확장까지 완료**한 상태다. 현재는 새로운 시뮬레이터를 추가하지 않고,
졸업작품 최종 검증과 발표 산출물을 정리하는 단계다.

- Stage 1–3: kinematic blind-intersection에서 V2V 필요성·PDR 민감도 검증
- Stage 4: TraCI-only SUMO 이식 + PPO 3모드 비교
- Stage 4 확장: SUMO PDR sweep, 기상(sensor_range), NLOS 폐색, 중간 multi-NPC

## 단계별 진행 과정과 의미

### Stage 0 — 기존 프로젝트 진단

**진행 과정**

- 기존 SUMO + PPO + Mininet-WiFi 환경의 학습 및 평가 결과를 확인
- 사고율이 높을 때 보상, 관측, 통신, SUMO 교통 중 무엇이 원인인지 분리
- 기존 환경에 교통량, TTC, PDR, AoI, priority, 보상 shaping이 동시에 결합되어
  있음을 확인

**진단 결과**

- V2V feature는 있었지만 V2V가 없으면 풀기 어려운 문제가 명확하지 않았음
- critical 상황에서 올바른 제동을 유도하는 보상 신호가 충분히 명확하지 않았음
- 배경 교통과 실제 통신을 한 번에 사용해 실패 원인 분석이 어려웠음

**의미**

기존 환경을 바로 확장하지 않고, 먼저 작고 통제 가능한 환경에서 “문제가
학습 가능한가”와 “V2V가 실제로 필요한가”를 순서대로 검증하기로 했다.

### Stage 1 — 최소 kinematic 환경과 PPO sanity check

**환경**

- SUMO를 사용하지 않는 2차원 kinematic 환경
- 학습 ego 차량 1대와 교차하는 target 차량 1대
- ego는 종방향 가감속만 제어하며 행동은 5개
  (`급제동`, `감속`, `유지`, `가속`, `최대가속`)
- 관측은 ego 위치·속도와 target 위치·속도·충돌점 도착시간 차이로 구성

**진행 과정**

1. `hold`, `full acceleration`, `rule-based TTC` 고정 정책을 실행
2. rule-based 정책으로 실제로 충돌회피가 가능한지 확인
3. PPO를 `perfect_v2v` 조건에서 학습
4. 초기 보상에서 발생한 항상 제동하는 정책을 재출발 보상으로 수정

**결과**

- hold: 충돌률 29.5%
- full acceleration: 충돌률 36.5%
- rule-based TTC: 충돌률 0%, 도착률 100%
- PPO (`perfect_v2v`, 50,000 steps): 충돌률 0%, 도착률 100%

**의미**

이 단계는 최종 V2V 비교가 아니라 환경 sanity check다. 고정 정책은 위험하고
rule-based 정책은 통과하므로, 환경이 물리적으로 풀 수 있는 문제라는 것을
확인했다. 따라서 이후 PPO 실패를 환경 자체가 불가능해서라고 설명할 수 없다.

관련 파일: `simple_intersection_env.py`, `check_env.py`,
`train_simple_ppo.py`, `evaluate_simple.py`

### Stage 2 — sensor-only와 perfect V2V의 공정 비교

**목표**

센서만으로 늦게 관측하는 경우와 V2V로 target을 조기에 관측하는 경우의
안전정책 학습 차이를 확인한다.

**진행 과정**

1. `perfect_v2v`에서 target 정보를 항상 제공
2. `sensor_only`에서는 sensor range 안에 들어온 뒤에만 target 정보를 제공
3. 두 모드에서 PPO를 각각 50,000 steps씩 따로 학습
4. 각 모델을 자신이 학습한 동일한 관측 모드에서 평가

**결과**

- `perfect_v2v` 학습·평가: 충돌률 0.0%, 도착률 100.0%, near miss 0.0%
- `sensor_only` 학습·평가: 충돌률 45.5%, 도착률 54.5%,
  near miss 69.0%

참고로 `perfect_v2v` 모델을 `sensor_only`에서 평가하면 충돌률 47.0%가
발생한다. 하지만 이 결과만으로 결론을 내리지 않고, sensor-only도 별도로
학습해 비교했기 때문에 “V2V로 학습한 모델을 센서 환경에 넣어서 생긴
문제”라는 반론을 줄였다.

**의미**

두 조건의 차이는 feature의 종류나 PPO 구조가 아니라 target 정보를 사용할 수
있는 시점이다. 같은 학습량으로 각자 학습했는데도 sensor-only의 충돌률이
높게 남아, blind-intersection에서는 조기 V2V 정보가 단순한 장식 feature가
아니라 학습 난이도를 바꾸는 정보 채널임을 보였다.

관련 파일: `simple_intersection_env.py`, `train_simple_ppo.py`,
`compare_models.py`, `runs/stage2_2x2_comparison.csv`

### Stage 3 — lossy V2V, PDR, AoI

**진행 과정**

- perfect V2V를 실제 통신에 가까운 `lossy_v2v`로 확장
- 거리 기반 PDR과 Bernoulli packet dropout 적용
- 마지막 성공 BSM의 freshness를 AoI로 관리
- PDR scale을 1.0, 0.5, 0.25, 0.1로 낮추며 sweep

**결과**

- PDR 1.0: 충돌률 0.0%, 평균 V2V 수신 46.685회
- PDR 0.5: 충돌률 0.0%, 평균 V2V 수신 23.035회
- PDR 0.25: 충돌률 2.5%, 평균 AoI 0.578초
- PDR 0.1: 충돌률 17.0%, 평균 AoI 1.315초

**의미**

V2V가 항상 완벽하다고 가정하지 않아도 기본 통신 품질에서는 안전성이
유지되지만, 통신 품질이 낮아지면 수신 횟수 감소와 AoI 증가를 거쳐 충돌률이
증가한다. 따라서 V2V의 효과와 통신 품질의 한계를 동시에 확인했다.

관련 파일: `simple_intersection_env.py`, `sweep_*`, `plot_stage3_results.py`,
`runs/stage3_3x3_comparison.csv`

### Stage 4 — SUMO/TraCI 이식과 재현

**진행 과정**

- Stage 1–3의 최소 관측·보상·행동 구조를 SUMO로 이식
- ego는 `S_in → N_out`, target은 `E_in → W_out`
- TraCI로 차량 속도를 직접 제어하고 SUMO의 자동 양보 개입을 비활성화
- `perfect_v2v`, `lossy_v2v`, `sensor_only`를 같은 방식으로 비교

**대표 결과**

- `perfect_v2v`: 충돌률 0%, 도착률 100%
- `lossy_v2v`: 충돌률 0%, 도착률 100%
- `sensor_only`: 충돌률 35%, 도착률 65%

**확장 결과**

- PDR 0.1: lossy 충돌률 6%
- sensor range 10m: sensor-only 충돌률 42%
- NLOS blocker: sensor-only 충돌률 39%
- rule-based NPC 1대를 추가한 multi-NPC: sensor-only 충돌률 15%

**다중 seed 최종 검증**

seed 0, 1, 2에서 각 mode를 50,000 steps 학습하고, seed별 100 episode를
평가했다.

- kinematic sensor-only: 충돌률 43.3% ± 3.1%
- SUMO sensor-only: 충돌률 16.0% ± 15.9%
- 두 환경 모두 perfect/lossy V2V의 평균 충돌률은 0%

`±`는 seed 간 표준편차다. SUMO sensor-only는 seed별 편차가 크므로 평균을
고정 성능으로 해석하지 않는다.

**의미**

V2V 효과가 단순한 Python kinematic 환경의 결과가 아니라 SUMO에서도 같은
방향으로 나타나는지 확인했다. Stage 4의 기상 proxy, NLOS, multi-NPC는 핵심
인과 실험을 대체하지 않고, 복잡도가 증가해도 결론이 유지되는지 확인하는
보조 검증이다.

관련 파일: `sumo_intersection_env.py`, `train_sumo_ppo.py`,
`compare_sumo_models.py`, `run_multiseed.py`, `replay_sumo.py`

### 현재 단계의 결론

현재까지의 실험은 다음 구조를 가진다.

```text
Stage 0  기존 환경 진단
   ↓
Stage 1  PPO 학습 가능성 확인
   ↓
Stage 2  sensor-only vs perfect V2V
   ↓
Stage 3  lossy V2V + PDR/AoI
   ↓
Stage 4  SUMO 재현 + 센서 실패·중간 다중 장면
```

핵심 인과 실험은 1 ego + 1 target에서 수행하고, Stage 4 multi-NPC는
1 ego + rule-based 배경차량을 둔 robustness 검증으로 해석한다. 여러 차량을
학습시키는 완전 다중에이전트 RL은 현재 졸업작품 범위가 아닌 후속 연구다.

## 졸업작품 최종 범위

최종 연구 질문은 다음으로 고정한다.

> 동일한 PPO 조건에서 sensor-only와 V2V 조기 관측의 안전성 차이가 나타나는가?

핵심 비교는 `sensor_only`, `perfect_v2v`, `lossy_v2v`이며, `lossy_v2v`는
PDR/AoI sweep으로 통신 품질과 안전성의 관계를 확인한다. kinematic 환경에서
원인을 분리한 뒤 SUMO에서 같은 관측 구조를 재현한다.

다음 프로젝트는 이 저장소에 직접 통합하지 않는다.

- CARLA/OpenCDA 및 대규모 cooperative perception
- Veins/ns-3 또는 실제 무선 통신 스택
- LLaVA/V2V-GoT 기반 MLLM
- 완전 다중에이전트 RL

이 항목들은 현재 결론을 확장하는 후속 연구로 남긴다.

## 참고한 오픈소스 및 관련 프로젝트

직접 실행에 사용한 오픈소스와 연구 방향을 정할 때 참고한 프로젝트를
구분한다. 아래 관련 프로젝트의 코드를 현재 환경에 그대로 통합한 것은 아니다.

### 직접 사용한 오픈소스

- [Eclipse SUMO](https://github.com/eclipse-sumo/sumo): SUMO 교통 시뮬레이터
- [TraCI tools](https://github.com/eclipse-sumo/sumo/tree/main/tools/traci):
  Python에서 SUMO 차량 상태를 읽고 제어하는 인터페이스
- [Gymnasium](https://github.com/Farama-Foundation/Gymnasium): 사용자 정의
  reinforcement learning 환경 API
- [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3): PPO 학습
  및 모델 저장·로드

### 설계와 비교 기준을 위해 참고한 프로젝트

- [V2V Cooperative Intersection RL](https://huggingface.co/amin-amiri/v2v-intersection-rl):
  교차로 V2V 충돌회피와 관측 조건 비교 방향
- [SUMO-V2X Communication Research, Platooning and CIM](https://github.com/sraddon/SUMO-V2X-Communication-Research-Platooning-and-CIM):
  SUMO 기반 V2X 통신·플래투닝 시나리오
- [V2V-GoT](https://github.com/eddyhkchiu/V2V-GoT):
  다중 차량 정보 융합과 V2V 인지 확장 사례

현재 프로젝트에서 직접 구현한 핵심은 `sensor_only`, `perfect_v2v`,
`lossy_v2v`, PDR/AoI, blind-intersection PPO 비교이며, 위 관련 프로젝트의
코드를 복사하거나 직접 통합하지 않았다. 실제 무선 통신, MLLM, 대규모
cooperative perception, 완전 다중에이전트 RL은 후속 연구 범위다.

핵심 주장:

> 교차로 blind-spot에서 센서만으로는 충돌회피 정책 학습이 어렵지만,  
> V2V 조기 정보로 그 한계를 극복할 수 있다.

### Stage 3 (kinematic)

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V (PDR 1.0) | 0.0% | 100.0% | 0.0% |
| lossy V2V (PDR 0.1) | 17.0% | 83.0% | 33.5% |
| sensor-only | 45.5% | 54.5% | 69.0% |

### Stage 4 (SUMO PPO, 학습=평가)

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V (PDR 1.0) | 0.0% | 100.0% | 0.0% |
| sensor-only | 35.0% | 65.0% | 60.0% |

### Stage 4 확장 요약

| 실험 | V2V (perfect/lossy) | sensor-only |
| --- | --- | --- |
| SUMO PDR 0.1 | lossy 충돌 6% | (기본 35%) |
| 기상 sensor 10m | 충돌 0% | 충돌 42% |
| NLOS 폐색 | 충돌 0% | 충돌 39% |
| 중간 multi-NPC | 충돌 0% | 충돌 15% |

참고: 기본 `pdr_scale=1.0`의 lossy는 perfect와 같이 0%인 경우가 많다.
이는 lossy가 perfect의 복제라서가 아니라, 이 난이도에서는 조기 수신이
몇 번만 있어도 충분하기 때문이다. PDR을 낮추면 충돌률이 함께 상승한다.

## 파일 구성

환경/학습

- `simple_intersection_env.py`: kinematic 환경 (Stage 1–3)
- `sumo_intersection_env.py`: SUMO/TraCI 환경 (Stage 4, NLOS/multi-NPC 옵션)
- `sumo_data/`: 네트워크·라우트·sumocfg (`build_net.py`)
- `train_simple_ppo.py` / `train_sumo_ppo.py`
- `evaluate_simple.py` / `evaluate_sumo.py`
- `compare_models.py` / `compare_sumo_models.py`
- `check_env.py` / `check_sumo_env.py`
- `run_multiseed.py`: 동일 학습 조건의 seed 반복 및 평균/표준편차/95% CI 계산
- `plot_multiseed_results.py`: 다중 seed 안전성 요약 그래프 생성
- `replay_sumo.py`: 한 episode의 SUMO GUI replay와 관측·행동 trace 저장

Stage 4 확장 스크립트

- `sweep_sumo_pdr.py` / `plot_sumo_pdr_results.py`
- `sweep_sumo_weather.py` / `plot_sumo_weather_results.py`
- `eval_sumo_nlos.py` / `plot_sumo_nlos_results.py`
- `run_sumo_multi_npc.py` / `plot_sumo_multi_npc_results.py`

문서

- `ROADMAP.md`, `EXPERIMENT_SUMMARY.md`
- `PRESENTATION_NOTES.md` (발표용 그림/표)
- `SERVER_COMPATIBILITY.md`
- `requirements.txt`: 이 재구성 프로젝트만 실행하는 독립 의존성 목록

## 실행 예시

### Kinematic

```powershell
cd C:\Users\sjbon\Desktop\졸작\코드\v2v_rebuild
python -m pip install -r requirements.txt
python check_env.py --episodes 200 --out-dir runs/baseline_v3
python train_simple_ppo.py --timesteps 50000 --mode perfect_v2v --out runs/simple_ppo_50k_v3.zip
```

### SUMO 기본

```powershell
pip install eclipse-sumo
$env:SUMO_HOME = (python -c "import sumo; print(sumo.SUMO_HOME)")
python sumo_data/build_net.py
python check_sumo_env.py --episodes 50 --out-dir runs/sumo_baseline

python train_sumo_ppo.py --timesteps 50000 --mode perfect_v2v --out runs/sumo_ppo_50k_perfect.zip
python train_sumo_ppo.py --timesteps 50000 --mode sensor_only --out runs/sumo_ppo_50k_sensor.zip
python train_sumo_ppo.py --timesteps 50000 --mode lossy_v2v --out runs/sumo_ppo_50k_lossy.zip

python compare_sumo_models.py `
  --model perfect_v2v runs/sumo_ppo_50k_perfect.zip `
  --model sensor_only runs/sumo_ppo_50k_sensor.zip `
  --model lossy_v2v runs/sumo_ppo_50k_lossy.zip `
  --episodes 100 `
  --out runs/sumo_stage4_3x3_comparison.csv
```

### Stage 4 확장

```powershell
python sweep_sumo_pdr.py --episodes 100
python plot_sumo_pdr_results.py

python sweep_sumo_weather.py --episodes 100
python plot_sumo_weather_results.py

python eval_sumo_nlos.py --episodes 100
python plot_sumo_nlos_results.py

python run_sumo_multi_npc.py --timesteps 50000 --episodes 100
python plot_sumo_multi_npc_results.py
```

### 최종 반복 실험

아래 명령은 각 관측 모드별로 모델을 seed마다 따로 학습하고, 학습한 모드에서
동일한 episode 수로 평가한다. 기존 모델이 있으면 재학습하지 않고 결과를 다시
집계한다.

```powershell
python run_multiseed.py `
  --environment simple `
  --timesteps 50000 `
  --episodes 100 `
  --seeds 0 1 2 `
  --out-dir runs/multiseed/simple

python run_multiseed.py `
  --environment sumo `
  --timesteps 50000 `
  --episodes 100 `
  --seeds 0 1 2 `
  --out-dir runs/multiseed/sumo

python plot_multiseed_results.py `
  --summary runs/multiseed/simple/summary.csv

python plot_multiseed_results.py `
  --summary runs/multiseed/sumo/summary.csv
```

결과는 각 디렉토리의 `raw_results.csv`와 `summary.csv`에 저장된다. `summary.csv`
에는 mode별 평균, 표준편차, 95% confidence interval half-width가 포함된다.
시간이 부족하면 `--timesteps 10000 --episodes 30`으로 smoke test를 먼저 실행할
수 있다.

현재 완료된 3-seed 결과 요약:

- kinematic: `perfect_v2v`/`lossy_v2v` 충돌률 0.0%, `sensor_only` 충돌률
  43.3% ± 3.1%
- SUMO: `perfect_v2v`/`lossy_v2v` 충돌률 0.0%, `sensor_only` 충돌률
  16.0% ± 15.9%

위 수치는 seed 간 평균 ± 표준편차이며, seed별 원자료와 95% CI는 각
`summary.csv`에 있다.

### SUMO replay

`--gui`를 사용하면 동일 episode seed에서 차량 움직임과 PPO 행동을 SUMO-GUI로
확인할 수 있다. 실행 후 trace는 CSV로 저장된다.

```powershell
python replay_sumo.py `
  --model runs/multiseed/sumo/ppo_seed0_perfect_v2v.zip `
  --mode perfect_v2v `
  --episode-seed 7 `
  --gui `
  --delay 0.05 `
  --out runs/replay/sumo_seed0_perfect.csv
```

sensor-only와 V2V를 같은 `--episode-seed`로 각각 replay하면, `visible`,
`sensor_visible`, `occluded`, `aoi` 열을 이용해 관측 시점 차이를 비교할 수 있다.
현재 seed 7 예시 trace는 `runs/replay/sumo_seed0_perfect.csv`와
`runs/replay/sumo_seed0_sensor.csv`에 있다.

## 보관 중인 결과 산출물

`runs/`에는 최종 실험만 둔다.

- kinematic: `baseline_v3/`, `simple_ppo_*_v3.zip`, `simple_ppo_lossy_50k_stage3.zip`, stage2/3 CSV
- SUMO: `sumo_baseline/`, `sumo_ppo_50k_*.zip`, multi-NPC 모델, stage4/확장 CSV
- 최종 반복: `multiseed/simple/`, `multiseed/sumo/`, `replay/`
- 중간 버전(`*_v2`, 구 baseline)과 `episodes.csv`는 제거했다.

`presentation/`에는 Stage 3·4 발표용 그래프를 둔다.
`PRESENTATION_DRAFT.md`는 로컬 전용(`.gitignore`)이다.
