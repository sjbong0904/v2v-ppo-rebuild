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

따라서 새 디렉토리 안에서 별도의 패키지 버전을 고정하지 않는다. 필요한 패키지가
추가되면 `v2v_rebuild/requirements.txt`가 아니라 상위 `aws_requirements.txt`를
함께 갱신하는 방식으로 관리한다.

## 현재 사용 중인 주요 패키지

현재 Stage 1-2 코드는 기존 서버 requirements에 포함된 다음 패키지만 사용한다.

- `gymnasium==1.2.3`
- `stable_baselines3==2.8.0`
- `numpy==1.26.4`
- `torch==2.9.1+cpu`

## 의도적인 분리

Stage 1-2에서는 SUMO, TraCI, Mininet, Mininet-WiFi를 import하지 않는다.

이유:

- 작은 환경에서 보상과 관측 구조를 먼저 검증하기 위해서다.
- SUMO나 네트워크 스택 문제가 PPO 학습 문제와 섞이지 않게 하기 위해서다.
- 나중에 SUMO로 옮길 때도 작은 환경 sanity check는 계속 독립적으로 실행할 수
  있어야 한다.

## 현재 import 경계

- `simple_intersection_env.py`: `gymnasium`, `numpy`
- `check_env.py`: `simple_intersection_env`
- `train_simple_ppo.py`: `stable_baselines3`, `simple_intersection_env`
- `evaluate_simple.py`: `stable_baselines3`, `simple_intersection_env`
- `compare_models.py`: `evaluate_simple`

SUMO 환경을 다시 도입할 때는 별도 파일로 분리한다. 예를 들어
`sumo_v2v_env.py`처럼 만들고, 기존 단순 환경 파일을 덮어쓰지 않는다.

