# 발표용 정리 (Stage 3 + Stage 4)

이 문서는 발표 슬라이드에 바로 옮기기 위한 그림/표/해석 메모다.

## 핵심 메시지

> 교차로 blind-spot에서 센서만으로는 충돌회피 정책 학습이 어렵지만,  
> V2V 조기 정보로 그 한계를 극복할 수 있다.  
> 통신 품질이 낮아질수록 수신↓·AoI↑·충돌↑가 함께 나타나며,  
> 기상·NLOS·중간 다중 장면에서도 V2V 우위가 유지된다.

---

## A. Stage 3 (kinematic) 그림

### 1. 관측 모드별 안전성 비교

- 파일: `presentation/stage3_safety_comparison.png`
- 말할 내용: sensor는 늦게 인지해 사고/near miss가 높고, perfect·lossy는 조기 관측으로 회피

### 2–5. PDR sweep

| 그림 | 파일 |
| --- | --- |
| 충돌률 | `presentation/stage3_pdr_collision_rate.png` |
| near miss | `presentation/stage3_pdr_near_miss_rate.png` |
| AoI | `presentation/stage3_pdr_aoi.png` |
| V2V 수신 | `presentation/stage3_pdr_v2v_rx.png` |

Stage 3 PDR 표:

| PDR scale | 충돌률 | near miss | 평균 AoI | 평균 V2V 수신 |
| ---: | ---: | ---: | ---: | ---: |
| 1.0 | 0.0% | 0.0% | 0.106 | 46.685 |
| 0.5 | 0.0% | 0.0% | 0.305 | 23.035 |
| 0.25 | 2.5% | 5.0% | 0.578 | 10.325 |
| 0.1 | 17.0% | 33.5% | 1.315 | 3.405 |

---

## B. Stage 4 (SUMO) 그림

### 1. SUMO 관측 모드 비교

- 파일: `presentation/sumo_stage4_safety_comparison.png`

| 조건 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect V2V | 0.0% | 100.0% | 0.0% |
| lossy V2V (PDR 1.0) | 0.0% | 100.0% | 0.0% |
| sensor-only | 35.0% | 65.0% | 60.0% |

### 2. SUMO PDR sweep

| 그림 | 파일 |
| --- | --- |
| 충돌률 | `presentation/sumo_pdr_collision_rate.png` |
| near miss | `presentation/sumo_pdr_near_miss_rate.png` |
| AoI | `presentation/sumo_pdr_aoi.png` |
| V2V 수신 | `presentation/sumo_pdr_v2v_rx.png` |

| PDR scale | 충돌률 | near miss | 평균 AoI | 평균 V2V 수신 |
| ---: | ---: | ---: | ---: | ---: |
| 1.0 | 0.0% | 0.0% | 0.098 | 52.85 |
| 0.5 | 0.0% | 0.0% | 0.280 | 26.12 |
| 0.25 | 3.0% | 3.0% | 0.725 | 12.17 |
| 0.1 | 6.0% | 13.0% | 1.156 | 4.68 |

말할 내용:

- 기본 lossy가 0%인 것은 “lossy=perfect”가 아니라, 조기 수신이 충분하면
  이 난이도에서는 양보가 가능하기 때문이다.
- PDR을 낮추면 수신↓·AoI↑·충돌↑로 통신 민감도가 드러난다.

### 3. 기상 proxy

- 파일: `presentation/sumo_weather_collision_rate.png`,
  `presentation/sumo_weather_near_miss_rate.png`

| sensor_range | perfect | lossy | sensor |
| ---: | ---: | ---: | ---: |
| 35m | 0% | 0% | 35% |
| 10m | 0% | 0% | 42% |

### 4. NLOS 폐색

- 파일: `presentation/sumo_nlos_collision_comparison.png`,
  `presentation/sumo_nlos_visibility.png`

| 모델 | LOS | NLOS |
| --- | ---: | ---: |
| perfect / lossy | 0% | 0% |
| sensor | 35% | 39% |

### 5. 중간 multi-NPC

- 파일: `presentation/sumo_multi_npc_safety.png`

| 모델 | 충돌률 | 도착률 | near miss |
| --- | ---: | ---: | ---: |
| perfect | 0.0% | 100.0% | 0.0% |
| lossy | 0.0% | 100.0% | 0.0% |
| sensor | 15.0% | 85.0% | 47.0% |

말할 내용:

- 주변차는 rule/고정속도 NPC이며, ego만 학습한다.
- primary threat(더 급한 TTC)를 관측 슬롯에 매핑해 obs 차원을 유지했다.
- 완전 다중에이전트 RL은 향후 목표다.

---

## C. 예상 질문 짧은 답

**Q. lossy가 왜 perfect처럼 0%인가?**  
A. 기본 PDR에서는 조기 수신이 충분하다. PDR을 낮추면 충돌이 올라간다 (SUMO 0.1 → 6%).

**Q. V2V feature가 적지 않나?**  
A. 이번 주장은 feature 개수가 아니라 정보 시점이다. 같은 6D에서 visibility만
바꿔도 충돌률이 크게 갈린다.

**Q. 시나리오가 하나뿐인가?**  
A. 기본 교차 + 기상 + NLOS + 중간 다중으로 센서 실패 모드를 나눴다.

## 한 문장 결론

> V2V는 부가 feature가 아니라, blind-intersection에서 PPO 안전정책 학습과
> 충돌회피를 좌우하는 핵심 정보 채널이다.
