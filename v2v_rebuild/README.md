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

현재는 Stage 3까지 진행했다.

- Stage 1: 단순 교차로 환경에서 PPO가 안전하게 충돌을 회피하고 통과할 수
  있는지 확인했다.
- Stage 2: `perfect_v2v`와 `sensor_only` 관측 모드를 각각 학습/평가하여
  V2V 조기 관측의 필요성을 비교했다.
- Stage 3: `lossy_v2v` 관측 모드를 추가하고, PDR scale에 따른 성능 저하를
  확인했다.

핵심 결과는 다음과 같다.

- `perfect_v2v` 학습 및 평가: 충돌률 0%, 도착률 100%
- `sensor_only` 학습 및 평가: 충돌률 45.5%, 도착률 54.5%
- `lossy_v2v` 학습 및 평가: 기본 PDR scale 1.0에서 충돌률 0%, 도착률 100%
- PDR scale 0.1에서는 `lossy_v2v` 충돌률이 17.0%까지 증가했다.

즉, 현재 단순 blind-intersection 환경에서는 V2V 조기 관측이 단순한 부가
feature가 아니라 안전정책 학습에 직접적인 영향을 주는 것으로 나타났다.

## 파일 구성

- `simple_intersection_env.py`: 단순 교차로 Gymnasium 환경
- `check_env.py`: 랜덤/고정/rule-based 정책 sanity check
- `train_simple_ppo.py`: PPO 학습 스크립트
- `evaluate_simple.py`: 단일 모델 평가 스크립트
- `compare_models.py`: 여러 모델을 관측 모드별로 비교하는 스크립트
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

## 다음 목표

다음 단계는 Stage 3 결과를 그래프/반복 seed로 더 다듬고, 이후 SUMO 이식을
준비하는 것이다.

- PDR scale별 성능 그래프 생성
- 여러 seed 반복 평가
- AoI를 관측에 포함하는 확장 실험
- SUMO 환경 이식 설계
