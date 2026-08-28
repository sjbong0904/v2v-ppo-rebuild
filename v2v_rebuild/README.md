# V2V 기반 PPO 자율주행 에이전트 재구성

이 디렉토리는 기존 졸업작품 코드를 바로 폐기하지 않고, V2V 기반 PPO
자율주행 문제를 작은 단위부터 다시 검증하기 위해 만든 재구성 작업공간이다.

기존 프로젝트는 SUMO, PPO, TTC, V2V feature, PDR, AoI, 교차로 판단 등이 한
환경 안에 모두 들어가 있어 사고율이 높게 나왔을 때 원인을 분리하기 어려웠다.
따라서 이 재구성 버전에서는 다음 순서로 문제를 다시 쌓는다.

1. 단순 교차로 충돌회피 문제가 PPO로 학습 가능한지 확인한다.
2. 센서-only 관측과 V2V 조기 관측을 비교한다.
3. PDR, 메시지 손실, AoI를 추가해 lossy V2V 환경으로 확장한다.
4. 검증된 관측/보상 구조를 SUMO로 이식하고, 센서 실패 모드·중간 다중 장면까지 확장한다.

## 현재 단계

**Stage 4 확장까지 완료**한 상태다. 현재는 새로운 시뮬레이터를 추가하지 않고,
졸업작품 최종 검증과 발표 산출물을 정리하는 단계다.

- Stage 1–3: kinematic blind-intersection에서 V2V 필요성·PDR 민감도 검증
- Stage 4: TraCI-only SUMO 이식 + PPO 3모드 비교
- Stage 4 확장: SUMO PDR sweep, 기상(sensor_range), NLOS 폐색, 중간 multi-NPC

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
