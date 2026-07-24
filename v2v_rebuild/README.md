# V2V 기반 PPO 자율주행 에이전트 재구성

이 디렉토리는 기존 졸업작품 코드를 바로 폐기하지 않고, V2V 기반 PPO
자율주행 문제를 작은 단위부터 다시 검증하기 위해 만든 재구성 작업공간이다.

기존 프로젝트는 SUMO, PPO, TTC, V2V feature, PDR, AoI, 교차로 판단 등이 한
환경 안에 모두 들어가 있어 사고율이 높게 나왔을 때 원인을 분리하기 어려웠다.
따라서 이 재구성 버전에서는 다음 순서로 문제를 다시 쌓는다.

1. 단순 교차로 충돌회피 문제가 PPO로 학습 가능한지 확인한다.
2. 센서-only 관측과 V2V 조기 관측을 비교한다.
3. PDR, 메시지 손실, AoI를 추가해 lossy V2V 환경으로 확장한다.
4. 검증된 관측/보상 구조를 SUMO 환경으로 다시 이식한다.

## 현재 단계

현재는 Stage 4(SUMO 최소 이식 + PPO)까지 진행했다.

- Stage 1-3: 단순 kinematic 환경에서 V2V / sensor-only / lossy V2V를 검증했다.
- Stage 4: 동일 관측·보상·행동 공간을 TraCI-only SUMO 교차로로 옮기고,
  3모드 PPO 학습·비교까지 완료했다. Mininet은 사용하지 않는다.

Stage 3 핵심 결과:

- `perfect_v2v`: 충돌률 0%, 도착률 100%
- `sensor_only`: 충돌률 45.5%, 도착률 54.5%
- `lossy_v2v` (PDR 1.0): 충돌률 0%; PDR 0.1에서는 충돌률 17.0%

Stage 4 SUMO PPO (`runs/sumo_stage4_3x3_comparison.csv`, 100 episode):

- perfect 학습·평가: 충돌률 0%, 도착률 100%
- lossy 학습·평가: 충돌률 0%, 도착률 100%
- sensor 학습·평가: 충돌률 35%, 도착률 65%

## 파일 구성

- `simple_intersection_env.py`: 단순 교차로 Gymnasium 환경 (Stage 1-3)
- `sumo_intersection_env.py`: SUMO/TraCI 최소 교차로 환경 (Stage 4)
- `sumo_data/`: SUMO 네트워크·라우트·sumocfg (`build_net.py`로 net 생성)
- `check_env.py`: kinematic 환경 sanity check
- `check_sumo_env.py`: SUMO 환경 sanity check
- `train_simple_ppo.py`: kinematic PPO 학습
- `train_sumo_ppo.py`: SUMO PPO 학습
- `evaluate_simple.py` / `evaluate_sumo.py`: 단일 모델 평가
- `compare_models.py` / `compare_sumo_models.py`: 관측 모드별 비교
- `ROADMAP.md`: 단계별 진행 계획과 완료 기준
- `EXPERIMENT_SUMMARY.md`: 지금까지의 실험 결과 요약
- `PRESENTATION_NOTES.md`: Stage 3 발표용 그림/표/해석 메모
- `SERVER_COMPATIBILITY.md`: AWS 서버 환경 호환성 메모
- `requirements.txt`: 상위 `aws_requirements.txt`를 참조하는 requirements 파일

## 실행 예시

기준선 평가:

```powershell
cd C:\Users\sjbon\Desktop\졸작\코드\v2v_rebuild
python check_env.py --episodes 200 --out-dir runs/baseline_v3
```

PPO 학습:

```powershell
python train_simple_ppo.py --timesteps 50000 --mode perfect_v2v --out runs/simple_ppo_50k_v3.zip
python train_simple_ppo.py --timesteps 50000 --mode sensor_only --out runs/simple_ppo_sensor_50k_v3.zip
```

2x2 비교 평가:

```powershell
python compare_models.py `
  --model perfect_v2v runs/simple_ppo_50k_v3.zip `
  --model sensor_only runs/simple_ppo_sensor_50k_v3.zip `
  --episodes 200 `
  --out runs/stage2_2x2_comparison.csv
```

## SUMO 선행조건 (Stage 4)

1. SUMO 바이너리가 필요하다. 로컬에서는 예를 들어:

```powershell
pip install eclipse-sumo
$env:SUMO_HOME = (python -c "import sumo; print(sumo.SUMO_HOME)")
```

2. 네트워크 빌드:

```powershell
python sumo_data/build_net.py
```

3. sanity check:

```powershell
python check_sumo_env.py --episodes 50 --out-dir runs/sumo_baseline
```

4. SUMO PPO 학습 및 3x3 비교:

```powershell
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

## 다음 목표

- 필요 시 SUMO PDR sweep / 여러 seed 반복 평가
- 여유 시 AoI 관측 확장
