# 서버 환경 호환성 메모

이 재구성 디렉토리는 기존 AWS 서버 환경과 충돌하지 않도록 상위
`aws_requirements.txt`를 그대로 사용한다.

설치 명령:

```powershell
pip install -r v2v_rebuild/requirements.txt
```

`v2v_rebuild/requirements.txt`는 다음 내용만 가진다.

```text
-r ../aws_requirements.txt
```

필요한 패키지가 추가되면 `v2v_rebuild/requirements.txt`가 아니라 상위
`aws_requirements.txt`를 함께 갱신한다.

## 현재 사용 중인 주요 패키지

Stage 1–3 (kinematic)은 기존 서버 requirements의 다음 패키지를 사용한다.

- `gymnasium==1.2.3`
- `stable_baselines3==2.8.0`
- `numpy==1.26.4`
- `torch==2.9.1+cpu`

Stage 4 (SUMO)는 추가로 TraCI / SUMO 바이너리가 필요하다.

- `traci`, `sumolib` (보통 eclipse-sumo / SUMO 설치와 함께)
- 로컬 예: `pip install eclipse-sumo` 후 `SUMO_HOME` 설정
- 그래프 스크립트: `matplotlib`

## 의도적인 분리

- Stage 1–3: SUMO/TraCI/Mininet을 import하지 않음
- Stage 4: TraCI-only SUMO, Mininet-WiFi / 기존 `v2x_env_0512.py` 미사용
- V2V 손실(PDR/AoI)은 코드 안 시뮬레이션

이유:

- 보상·관측·통신 품질 효과를 PPO 학습 문제와 분리하기 위해서다.
- kinematic sanity check는 SUMO 없이도 계속 실행 가능해야 한다.

## 현재 import 경계

Kinematic

- `simple_intersection_env.py`: `gymnasium`, `numpy`
- `train_simple_ppo.py` / `evaluate_simple.py` / `compare_models.py` / `check_env.py`

SUMO

- `sumo_intersection_env.py`: `gymnasium`, `numpy`, `traci`
- `train_sumo_ppo.py` / `evaluate_sumo.py` / `compare_sumo_models.py` / `check_sumo_env.py`
- 확장: `sweep_sumo_pdr.py`, `sweep_sumo_weather.py`, `eval_sumo_nlos.py`,
  `run_sumo_multi_npc.py` 및 대응 `plot_sumo_*.py`

SUMO 바이너리:

- PATH의 `sumo`, 또는 `SUMO_HOME/bin`, 또는 `pip install eclipse-sumo`
- 네트워크: `python sumo_data/build_net.py`
