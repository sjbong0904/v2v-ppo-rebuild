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

## 단계별 진행 과정과 의미

### Stage 0 — 기존 프로젝트 진단

기존 환경에는 SUMO, PPO, TTC, V2V feature, PDR, AoI, 배경 교통, 보상
shaping이 한 번에 결합되어 있었다. 사고가 발생해도 관측 부족인지, 통신
손실인지, 보상 또는 교통 상황 때문인지 분리하기 어려웠다.

이 문제를 해결하기 위해 기존 코드를 보존하고, `v2v_rebuild`에서 환경과
실험 변수를 작은 단위로 분리해 다시 검증하는 방향으로 전환했다.

### Stage 1 — 최소 환경에서 PPO 학습 가능성 확인

**진행**

- SUMO 없이 ego 차량 1대와 target 차량 1대만 있는 kinematic blind-intersection 구성
- `hold`, `full acceleration`, `rule-based TTC` 기준 정책 실행
- `perfect_v2v` 조건에서 PPO 학습
- 항상 제동하는 초기 정책 문제를 재출발 보상으로 수정

**결과**

- hold 충돌률: 29.5%
- full acceleration 충돌률: 36.5%
- rule-based TTC 충돌률: 0%, 도착률: 100%
- PPO 충돌률: 0%, 도착률: 100%

**의미**

환경 자체가 물리적으로 해결 가능한 문제인지 먼저 확인했다. 고정 정책은
충돌하지만 rule-based와 PPO는 통과하므로, 이후 PPO가 학습하지 못하는 원인을
환경 불가능성으로 설명할 수 없게 되었다.

### Stage 2 — sensor-only와 perfect V2V 비교

**진행**

- `sensor_only`: target이 센서 범위 안에 들어온 뒤에만 관측
- `perfect_v2v`: V2V로 target을 조기에 관측
- 두 조건에서 PPO를 각각 50,000 steps 학습
- 각 모델을 자신이 학습한 동일한 관측 조건에서 평가

**결과**

- `perfect_v2v`: 충돌률 0.0%, 도착률 100.0%, near miss 0.0%
- `sensor_only`: 충돌률 45.5%, 도착률 54.5%, near miss 69.0%

**의미**

두 조건은 PPO 알고리즘과 학습량을 동일하게 유지하고 target 정보를 얻는
시점만 다르게 했다. 따라서 blind-intersection에서 V2V 조기 정보가 단순한
추가 feature가 아니라, 센서 사각지대 문제를 줄여 안전정책 학습을 가능하게
하는 핵심 정보 채널이라는 점을 확인했다.

### Stage 3 — lossy V2V와 통신 품질 검증

`lossy_v2v`에 PDR, packet dropout, AoI를 추가해 통신이 항상 성공하지 않는
상황을 구성했다. PDR이 1.0일 때 충돌률은 0%였지만, PDR 0.1에서는 충돌률이
17.0%로 증가했다. 이를 통해 V2V의 효과뿐 아니라 통신 품질 저하가 안전성에
미치는 영향도 확인했다.

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

### 다중 seed 최종 검증

seed 0, 1, 2에서 각 관측 모드를 50,000 steps씩 학습하고, 각 seed마다
100 episode를 평가했다.

- kinematic `sensor_only`: 충돌률 43.3% ± 3.1%
- SUMO `sensor_only`: 충돌률 16.0% ± 15.9%
- 두 환경의 `perfect_v2v`/`lossy_v2v`: 평균 충돌률 0%

`±`는 seed 간 표준편차다. SUMO sensor-only의 편차가 크므로 평균을 절대적인
고정 성능으로 해석하지 않고, seed별 원자료와 95% CI를 함께 확인한다.

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

- `v2v_rebuild/README.md`: 재구성 실험의 상세 과정·실행법·재현 방법
- `v2v_rebuild/ROADMAP.md`: 단계별 로드맵
- `v2v_rebuild/EXPERIMENT_SUMMARY.md`: 실험 결과 요약
- `v2v_rebuild/PRESENTATION_NOTES.md`: 발표용 그림/표/해석
- `v2v_rebuild/SERVER_COMPATIBILITY.md`: 서버 환경 호환성 메모

## 현재 범위와 다음 단계

현재 핵심 실험과 다중 seed 검증은 완료되었으며, SUMO GUI replay와 발표자료를
통해 결과를 설명할 수 있는 상태다.

현재 졸업작품 범위는 1대의 학습 ego 차량, 교차 target 차량, 그리고
rule-based 배경차량 검증까지다. 완전 다중에이전트 RL, 실제 무선 통신,
CARLA/OpenCDA, V2V-GoT 기반 MLLM은 후속 연구로 남긴다.

자세한 실행 명령과 결과 파일은 [`v2v_rebuild/README.md`](v2v_rebuild/README.md)를
참조한다.
