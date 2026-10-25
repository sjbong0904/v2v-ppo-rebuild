# v2v_rebuild

교차로 **blind-spot**에서 센서만으로는 늦은 인지로 충돌회피가 어렵고, **lossy V2V**로 조기 인지하면 안전정책 학습이 가능하다는 것을 SUMO + PPO로 검증한 메인 코드입니다.

공개 랜딩(데모 GIF·요약 표): 저장소 루트 [`../README.md`](../README.md)

## 한 줄 주장

센서-only는 약 **17 m / 0.5 s**에야 위협을 보지만, lossy V2V는 약 **108 m / 6 s**에 먼저 보고 충돌을 크게 줄인다.

## 최종 지표

Canonical 결과: [`presentation/테스트지표2/`](presentation/테스트지표2/)  
요약: [`presentation/테스트지표2/FINAL_지표요약.md`](presentation/테스트지표2/FINAL_지표요약.md)

| 시나리오 | Collision | Success | V2V early | 최초인지거리 | 최초인지 TTC |
| --- | ---: | ---: | ---: | ---: | ---: |
| sensor | 39.0% | 61.0% | 0% | 17.0 m | 0.49 s |
| multi_sensor | 54.0% | 46.0% | 0% | 16.0 m | 0.52 s |
| lossy | 0.0% | 100% | 100% | 108.0 m | 6.07 s |
| multi_lossy | 0.0% | 100% | 100% | 107.2 m | 5.88 s |
| lossy_pdr03 | 0.5% | 99.5% | 100% | 96.8 m | 5.15 s |
| multi_lossy_pdr03 | 4.0% | 96.0% | 100% | 96.0 m | 4.85 s |

프로토콜: 시나리오당 200 episodes, seed 0–199.

## Quickstart

```powershell
pip install -r requirements.txt
python sumo_data/build_net.py   # 네트워크가 없을 때

# 평가 예시
python evaluate_sumo.py --model runs/sumo_ppo_50k_lossy.zip --mode lossy_v2v --episodes 50

# GUI 데모
cd demo_package
.\install.bat
.\run_demo.bat
```

기본 모델 zip은 `models/` · `demo_package/models/`에 포함되어 자동 로드됩니다.  
촬영 가이드: [`DEMO_RECORDING.md`](DEMO_RECORDING.md)

## 구성

| 경로 | 역할 |
| --- | --- |
| `sumo_intersection_env.py` | SUMO/TraCI 환경 (sensor / lossy V2V, NLOS, multi-NPC) |
| `simple_intersection_env.py` | kinematic 선행 검증 환경 |
| `train_sumo_ppo.py` / `evaluate_sumo.py` | 학습·평가 |
| `run_metric_eval.py` | 최종 지표 배치 평가 |
| `demo_package/` | 독립 실행 Compare 데모 |
| `presentation/` | 그래프 · `테스트지표2/` · `media/demo.gif` |
| `runs/` | 학습된 PPO zip · 실험 CSV |

단계별 실험 서술·중간 결과는 [`EXPERIMENT_SUMMARY.md`](EXPERIMENT_SUMMARY.md)를 참고하세요.

## License

MIT — 저장소 루트 [`../LICENSE`](../LICENSE)
