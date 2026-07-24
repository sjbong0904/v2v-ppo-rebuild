# V2V 기반 PPO 자율주행 에이전트

이 저장소는 졸업작품으로 진행 중인 V2V 기반 PPO 자율주행 에이전트 프로젝트다.

초기 구현은 SUMO 기반 교차로 환경에서 PPO 에이전트를 학습시키는 방향으로
진행되었지만, 발표 과정에서 V2V와의 연결성이 약하다는 피드백을 받았다. 또한
학습 결과에서 사고율이 높아, 보상체계와 환경 설계를 작은 단위부터 다시 검증할
필요가 있었다.

현재는 기존 코드를 보존하면서 `v2v_rebuild` 디렉토리에서 재구성 실험을 진행하고
있다.

## 현재 핵심 결과

`v2v_rebuild`에서 단순 blind-intersection 환경을 만들고, 다음 세 관측 조건을
비교했다.

- `sensor_only`: 센서 범위 안에 들어온 차량만 관측
- `perfect_v2v`: V2V로 교차 차량 정보를 조기에 관측
- `lossy_v2v`: PDR, 메시지 손실, AoI가 있는 V2V 관측

Stage 3 (kinematic) 주요 결과:

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V, PDR scale 1.0 | 0.0% | 100.0% | 0.0% |
| lossy V2V, PDR scale 0.1 | 17.0% | 83.0% | 33.5% |
| sensor-only | 45.5% | 54.5% | 69.0% |

Stage 4 (SUMO PPO, 학습=평가) 주요 결과:

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V | 0.0% | 100.0% | 0.0% |
| sensor-only | 35.0% | 65.0% | 60.0% |

현재 결론은 다음과 같다.

> 교차로 blind-spot에서 센서만으로는 충돌회피 정책 학습이 어렵지만, V2V 조기
> 정보로 그 한계를 극복할 수 있다. 이 결과는 kinematic 환경과 SUMO 최소
> 환경에서 같은 방향으로 재현되었다.

## 디렉토리 구성

- `v2v_rebuild/`: 작은 환경부터 다시 쌓는 재구성 실험
- `v2x_env_0512.py`: 기존 SUMO/V2X 환경 코드
- `train_ppo_0512.py`: 기존 PPO 학습 코드
- `evaluate_0512.py`: 기존 평가 코드
- `map.*.xml`, `multi_routes_easy.rou.xml`: 기존 SUMO 맵/route 파일
- `aws_requirements.txt`: 서버 환경 requirements

## 주요 문서

- `v2v_rebuild/README.md`: 재구성 실험 개요
- `v2v_rebuild/ROADMAP.md`: 단계별 로드맵
- `v2v_rebuild/EXPERIMENT_SUMMARY.md`: 실험 결과 요약
- `v2v_rebuild/PRESENTATION_NOTES.md`: 발표용 그림/표/해석 정리
- `v2v_rebuild/SERVER_COMPATIBILITY.md`: 서버 환경 호환성 메모

## 다음 계획

1. SUMO에서도 PDR scale sweep을 수행한다.
2. 필요 시 센서 한계가 다른 시나리오(사거리 축소/폐색)를 1~2개 추가한다.
3. 여러 seed 반복 평가로 결과 신뢰도를 보강한다.

