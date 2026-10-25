# Legacy (초기 통합 환경)

이 폴더는 졸업작품 **초기 버전**(2025-05 전후) 코드를 보관한다.

당시 환경은 SUMO + PPO + (계획상) Mininet-WiFi + 배경 교통 + V2V feature가
한 번에 결합되어 있어, 사고 원인 분리가 어려웠다.

## 포함 파일

| 파일 | 역할 |
| --- | --- |
| `v2x_env_0512.py` | 초기 SUMO/TraCI 환경 |
| `train_ppo_0512.py` | 초기 PPO 학습 스크립트 |
| `evaluate_0512.py` | 초기 평가 스크립트 |
| `map.*.xml`, `multi_routes_easy.rou.xml` | 초기 맵/경로 |
| `aws_requirements.txt` | 당시 의존성 메모 |

## 현재 검증 코드

공개·재현·발표용 실험은 모두 **`../v2v_rebuild/`** 를 사용한다.
이 `legacy/` 디렉토리는 참고용이며, 기본 Quickstart 경로가 아니다.
