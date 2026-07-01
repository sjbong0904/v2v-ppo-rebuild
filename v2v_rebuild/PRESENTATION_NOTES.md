# Stage 3 발표용 정리

이 문서는 Stage 3 결과를 발표 슬라이드에 바로 옮기기 위해 정리한 메모다.

## 핵심 메시지

> V2V 조기 관측은 blind-intersection 상황에서 PPO 에이전트의 사고율을 크게
> 낮춘다. 또한 V2V 통신 품질이 낮아질수록 수신 메시지 수가 감소하고 AoI가
> 증가하며, 그 결과 충돌률과 near miss가 함께 증가한다.

## 사용하면 좋은 그림

### 1. 관측 모드별 안전성 비교

파일:

- `presentation/stage3_safety_comparison.png`

슬라이드 제목 예시:

- `Sensor-only 대비 V2V 관측의 충돌회피 성능`

말할 내용:

- sensor-only는 target 차량을 늦게 인지하기 때문에 사고율과 near miss가 높다.
- perfect V2V와 lossy V2V는 target 정보를 더 일찍 활용해 충돌을 회피한다.
- Stage 3 기본 조건에서 lossy V2V도 perfect V2V와 유사한 무사고 성능을 보였다.

### 2. PDR scale에 따른 충돌률 변화

파일:

- `presentation/stage3_pdr_collision_rate.png`

슬라이드 제목 예시:

- `V2V 통신 품질 저하에 따른 충돌률 증가`

말할 내용:

- PDR scale이 1.0 또는 0.5일 때는 충돌률이 0%였다.
- PDR scale 0.25에서는 충돌률이 2.5%로 증가했다.
- PDR scale 0.1에서는 충돌률이 17.0%까지 증가했다.
- 즉, V2V가 있더라도 통신 품질이 낮아지면 안전성이 점진적으로 저하된다.

### 3. PDR scale에 따른 near miss 변화

파일:

- `presentation/stage3_pdr_near_miss_rate.png`

슬라이드 제목 예시:

- `V2V 손실 증가에 따른 near miss 증가`

말할 내용:

- PDR scale 0.1에서는 near miss가 33.5%까지 증가했다.
- 충돌이 발생하지 않은 episode에서도 안전 여유가 줄어드는 현상을 보여준다.

### 4. PDR scale에 따른 AoI 변화

파일:

- `presentation/stage3_pdr_aoi.png`

슬라이드 제목 예시:

- `메시지 손실과 정보 신선도(AoI)의 관계`

말할 내용:

- PDR scale이 낮아지면 메시지를 덜 받기 때문에 AoI가 증가한다.
- AoI 증가는 에이전트가 오래된 target 정보를 기반으로 판단할 가능성이 커진다는
  의미다.

### 5. PDR scale에 따른 V2V 수신 횟수 변화

파일:

- `presentation/stage3_pdr_v2v_rx.png`

슬라이드 제목 예시:

- `PDR 저하에 따른 V2V 메시지 수신 감소`

말할 내용:

- PDR scale 1.0에서는 episode당 평균 46.685회 수신했다.
- PDR scale 0.1에서는 episode당 평균 3.405회로 감소했다.
- 수신 횟수 감소가 AoI 증가와 충돌률 증가로 연결된다.

## 발표용 표

### 관측 모드 비교

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V, PDR scale 1.0 | 0.0% | 100.0% | 0.0% |
| sensor-only | 45.5% | 54.5% | 69.0% |

### PDR scale sweep

| PDR scale | 충돌률 | 도착률 | near miss | 평균 AoI | 평균 V2V 수신 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1.0 | 0.0% | 100.0% | 0.0% | 0.106 | 46.685 |
| 0.5 | 0.0% | 100.0% | 0.0% | 0.305 | 23.035 |
| 0.25 | 2.5% | 97.5% | 5.0% | 0.578 | 10.325 |
| 0.1 | 17.0% | 83.0% | 33.5% | 1.315 | 3.405 |

## 한 문장 결론

> 본 실험은 V2V가 단순한 추가 feature가 아니라, blind-intersection 상황에서
> PPO 자율주행 에이전트의 안전정책 학습과 충돌회피 성능을 좌우하는 핵심 정보
> 채널임을 보여준다.

