# 데모 영상 촬영 가이드 (sensor vs lossy)

목표: **30–60초** 안에 “센서는 늦게 인지해 위험하고, V2V는 먼저 인지해 안전하다”를 보여 준다.

## 준비

1. `v2v_rebuild/demo_package/`에서 `install.bat` → `run_demo.bat`
2. 모델 추가 (권장):
   - `runs/sumo_ppo_50k_sensor.zip` (또는 파일명에 `sensor`)
   - `runs/sumo_ppo_50k_lossy.zip` (또는 파일명에 `lossy`)
3. GUI 옵션:
   - **RANGE RINGS** ON (센서 35 m / V2V 120 m)
   - **NLOS** ON (blind intersection이 잘 보임)
   - delay: 화면이 읽히도록 약간 느리게 (예: 30–50 ms)

## 촬영 시나리오 (총 ~45초)

### Clip A — Sensor only (~15–20초)

| 시각 | 화면 / 자막 |
| --- | --- |
| 0–2 s | 자막: `Sensor only` |
| 2–8 s | ego 접근, 센서 링만 보임, V2V RX 없음 |
| 8–12 s | 상대가 센서 범위에 들어올 때 늦게 인지 (거리·TTC가 짧음) |
| 12–18 s | 충돌 또는 급제동 실패 → 자막: `Late perception → high collision risk` |

Seed를 몇 번 바꿔 충돌/아슬아슬한 컷을 고른다.

### Clip B — Lossy V2V (~15–20초)

| 시각 | 화면 / 자막 |
| --- | --- |
| 0–2 s | 자막: `Lossy V2V (PDR < 1)` |
| 2–6 s | 먼 거리에서 **V2V RX / HOLD** 점등 |
| 6–12 s | 센서보다 먼저 제동 (early lead) |
| 12–18 s | 안전 통과 → 자막: `~100 m / ~6 s earlier → collision ≈ 0%` |

동일 맵·비슷한 seed로 대비가 보이면 좋다.

### Clip C — 한 줄 결론 (~5초, 선택)

자막만:

> Sensor ~17 m / 0.5 s → collision 39–54%  
> Lossy V2V ~108 m / 6 s → collision 0%

## 녹화 방법 (Windows)

1. **Win + G** (Xbox Game Bar) → 녹화, 또는 OBS
2. GUI 창만 캡처 (전체 바탕화면 X)
3. 해상도 1280×720 이상, 소리 불필요

## 편집·내보내기

1. Clip A + B (+ C)를 이어 붙임 (총 30–60초)
2. **GIF** (README용): 10–15초로 줄여도 됨  
   - [ezgif.com](https://ezgif.com/video-to-gif) 또는 ffmpeg  
   - 권장: ≤ 8–12 MB, 짧은 loop
3. 저장 위치:
   - `v2v_rebuild/presentation/media/demo.gif` ← README embed
   - `v2v_rebuild/presentation/media/demo.mp4` ← 원본 (선택)

### ffmpeg 예시

```powershell
# mp4 → README용 gif (폭 720, fps 10)
ffmpeg -i demo.mp4 -vf "fps=10,scale=720:-1:flags=lanczos" -loop 0 v2v_rebuild/presentation/media/demo.gif
```

## README에 넣기

파일을 넣은 뒤 루트 `README.md` Demo 섹션의 placeholder를 실제 경로로 유지하면 된다 (이미 `demo.gif` 경로로 작성됨).

선택: YouTube/Drive에 mp4를 올리고 README에 링크를 추가한다. GitHub README는 YouTube iframe을 지원하지 않으므로 **링크 + gif** 조합이 가장 무난하다.

## GitHub About (복붙용)

- **Description:** Blind-intersection V2V vs sensor-only PPO ablation with SUMO (TraCI)
- **Topics:** `sumo` `ppo` `v2v` `reinforcement-learning` `gymnasium` `autonomous-driving`
