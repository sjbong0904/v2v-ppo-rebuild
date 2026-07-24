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

**Stage 4 확장까지 완료**한 상태다.

- Stage 1–3: kinematic blind-intersection에서 V2V 필요성·PDR 민감도 검증
- Stage 4: TraCI-only SUMO 이식 + PPO 3모드 비교
- Stage 4 확장: SUMO PDR sweep, 기상(sensor_range), NLOS 폐색, 중간 multi-NPC

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

Stage 4 확장 스크립트

- `sweep_sumo_pdr.py` / `plot_sumo_pdr_results.py`
- `sweep_sumo_weather.py` / `plot_sumo_weather_results.py`
- `eval_sumo_nlos.py` / `plot_sumo_nlos_results.py`
- `run_sumo_multi_npc.py` / `plot_sumo_multi_npc_results.py`

문서

- `ROADMAP.md`, `EXPERIMENT_SUMMARY.md`
- `PRESENTATION_NOTES.md` (발표용 그림/표)
- `SERVER_COMPATIBILITY.md`
- `requirements.txt` → 상위 `aws_requirements.txt` 참조

## 실행 예시

### Kinematic

```powershell
cd C:\Users\sjbon\Desktop\졸작\코드\v2v_rebuild
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

## 보관 중인 결과 산출물

`runs/`에는 최종 실험만 둔다.

- kinematic: `baseline_v3/`, `simple_ppo_*_v3.zip`, `simple_ppo_lossy_50k_stage3.zip`, stage2/3 CSV
- SUMO: `sumo_baseline/`, `sumo_ppo_50k_*.zip`, multi-NPC 모델, stage4/확장 CSV
- 중간 버전(`*_v2`, 구 baseline)과 `episodes.csv`는 제거했다.

`presentation/`에는 Stage 3·4 발표용 그래프를 둔다.
`PRESENTATION_DRAFT.md`는 로컬 전용(`.gitignore`)이다.
