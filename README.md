# V2V 기반 PPO 자율주행 에이전트

이 저장소는 졸업작품으로 진행 중인 V2V 기반 PPO 자율주행 에이전트 프로젝트다.

초기 구현은 SUMO 기반 교차로 환경에서 PPO 에이전트를 학습시키는 방향으로
진행되었지만, 발표 과정에서 V2V와의 연결성이 약하다는 피드백을 받았다. 또한
학습 결과에서 사고율이 높아, 보상체계와 환경 설계를 작은 단위부터 다시 검증할
필요가 있었다.

현재는 기존 코드를 보존하면서 `v2v_rebuild` 디렉토리에서 재구성 실험을 진행하고
있다.

## 현재 핵심 결과

`v2v_rebuild`에서 단순 blind-intersection을 만들고 다음을 비교했다.

- `sensor_only`: 센서 범위 안 차량만 관측
- `perfect_v2v`: V2V로 교차 차량을 조기 관측
- `lossy_v2v`: PDR/메시지 손실/AoI가 있는 V2V

### Stage 3 (kinematic)

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V, PDR 1.0 | 0.0% | 100.0% | 0.0% |
| lossy V2V, PDR 0.1 | 17.0% | 83.0% | 33.5% |
| sensor-only | 45.5% | 54.5% | 69.0% |

### Stage 4 (SUMO PPO)

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V, PDR 1.0 | 0.0% | 100.0% | 0.0% |
| sensor-only | 35.0% | 65.0% | 60.0% |

### Stage 4 확장

| 실험 | V2V | sensor |
| --- | --- | --- |
| SUMO PDR 0.1 | lossy 충돌 6% | (기본 35%) |
| 기상 (sensor 10m) | 0% | 42% |
| NLOS 폐색 | 0% | 39% |
| 중간 multi-NPC | 0% | 15% |

현재 결론:

> 교차로 blind-spot에서 센서만으로는 충돌회피 정책 학습이 어렵지만, V2V 조기
> 정보로 그 한계를 극복할 수 있다. kinematic과 SUMO, 그리고 통신저하·기상·
> NLOS·중간 다중 장면에서도 같은 방향이 재현되었다.

## 디렉토리 구성

- `v2v_rebuild/`: 작은 환경부터 다시 쌓는 재구성 실험 (현재 메인)
- `v2x_env_0512.py`: 기존 SUMO/V2X 환경 코드
- `train_ppo_0512.py`: 기존 PPO 학습 코드
- `evaluate_0512.py`: 기존 평가 코드
- `map.*.xml`, `multi_routes_easy.rou.xml`: 기존 SUMO 맵/route 파일
- `aws_requirements.txt`: 서버 환경 requirements

## 주요 문서

- `v2v_rebuild/README.md`: 재구성 실험 개요·실행법
- `v2v_rebuild/ROADMAP.md`: 단계별 로드맵
- `v2v_rebuild/EXPERIMENT_SUMMARY.md`: 실험 결과 요약
- `v2v_rebuild/PRESENTATION_NOTES.md`: 발표용 그림/표/해석
- `v2v_rebuild/SERVER_COMPATIBILITY.md`: 서버 환경 호환성 메모

## 다음 계획

1. 발표자료 정리 및 데모 준비
2. 필요 시 multi-seed로 통계 보강
3. 장기: 완전 다중에이전트 RL (상대도 학습하는 V2V 협력)
