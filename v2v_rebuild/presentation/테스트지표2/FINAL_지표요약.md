# 최종 지표 (canonical)

**폴더:** `presentation/테스트지표2/`  
**프로토콜:** 6시나리오 × 각 200 episodes (perfect 제외, PDR 0.3 포함)  
**동일 seed:** 0~199

---

## 메인 결과표

| 시나리오 | Collision (95% CI) | Success | V2V early | 최초인지거리 | 최초인지 TTC | V2V lead | min distance |
|---|---:|---:|---:|---:|---:|---:|---:|
| sensor | **39.0%** (32.5–45.9) | 61.0% | 0% | 17.0 m | 0.49 s | — | 7.5 m |
| multi_sensor | **54.0%** (47.1–60.8) | 46.0% | 0% | 16.0 m | 0.52 s | — | 5.4 m |
| lossy | **0.0%** (0.0–1.9) | 100% | **100%** | **108.0 m** | **6.07 s** | **6.2 s / 74 m** | 25.9 m |
| multi_lossy | **0.0%** (0.0–1.9) | 100% | **100%** | **107.2 m** | **5.88 s** | **5.3 s / 66 m** | 18.7 m |
| lossy_pdr03 | **0.5%** (0.1–2.8) | 99.5% | **100%** | **96.8 m** | **5.15 s** | **5.3 s / 65 m** | 23.4 m |
| multi_lossy_pdr03 | **4.0%** (2.0–7.7) | 96.0% | **100%** | **96.0 m** | **4.85 s** | **4.7 s / 63 m** | 15.5 m |

상세 CSV: `final_presentation_numbers.csv`, `summary_final.csv`

---

## 메인 지표

1. collision rate
2. successful passage rate
3. first valid perception distance / TTC
4. v2v_early_before_sensor rate
5. v2v_lead_before_sensor (s, m)
6. PDR 0.3 ablation (RX / AoI / collision)

---

