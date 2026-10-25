# V2V Demo GUI 사용 안내

교차로 V2V / 센서 관측 비교용 데모 뷰어입니다.  
**모델(`.zip`)은 이 패키지에 포함되어 있지 않습니다.** 모델은 따로 받은 뒤 GUI에 넣으면 됩니다.

---

## 가장 빠른 시작 (Windows 올인원)

1. 이 폴더(또는 zip 압축 해제 폴더)를 연다.
2. **`install.bat`** 더블클릭 → 한 번만 실행 (몇 분 소요)
3. **`run_demo.bat`** 더블클릭 → GUI 실행
4. 따로 받은 모델 `.zip`을 GUI에서 **Add** / 드래그앤드롭

`install.bat`이 하는 일:
- `.venv` 가상환경 생성
- Python 패키지 설치
- **SUMO 본체(`eclipse-sumo`)까지 pip로 설치** → 별도 SUMO 설치 프로그램 없이도 동작

이미 시스템에 SUMO가 있어도 괜찮습니다. 패키지 안 가상환경을 우선 사용합니다.

---

## 1. 폴더 구성

```
demo_package/
  install.bat                 ← 올인원 설치 (최초 1회)
  run_demo.bat                ← GUI 실행
  demo_app.py
  sumo_intersection_env.py
  view_sumo_custom.py
  requirements.txt
  README.md
  models/                     ← (선택) 모델 zip 보관
  sumo_data/                  ← SUMO 맵 (필수, 삭제 금지)
    map.sumocfg
    map.net.xml
    routes.rou.xml
    map.poly.xml
```

---

## 2. 설치 (수동 / macOS·Linux)

### 필요 조건
- Python 3.10 이상 권장
- (선택) 시스템 SUMO — 없어도 `eclipse-sumo` pip 패키지로 대체 가능

### 명령어

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -U pip
pip install torch --index-url https://download.pytorch.org/whl/cpu   # 권장
pip install -r requirements.txt
python demo_app.py
```

---

## 3. 사용법

### 3-1. 모델 추가
1. 받은 PPO `.zip`을 `models/` 또는 아무 폴더에 둡니다.
2. GUI 왼쪽 **MODELS**에 **Add** 또는 드래그앤드롭 (최대 10개).
3. 목록에서 모델을 고른 뒤 **Start**.
4. 모델을 바꾼 뒤에는 **Restart**.

모델 없이 `(rule-based)`만 선택해도 실행됩니다.

### 3-2. 파일명 → 자동 모드

| 파일명 키워드 | 모드 |
|---------------|------|
| `lossy` | lossy V2V |
| `perfect` | perfect V2V |
| `sensor` | 센서만 |
| `multi` | 다중 NPC |

예: `sumo_ppo_50k_lossy.zip` → lossy_v2v

### 3-3. 주요 조작

| 조작 | 동작 |
|------|------|
| Start | 에피소드 시작 |
| Pause / Resume | 일시정지 / 재개 |
| Restart | 다시 시작 |
| 마우스 휠 | 줌 |
| seed / delay | 시드, 프레임 딜레이 |

### 3-4. 옵션
- **NLOS**: 센서 폐색 (건물·트럭). V2V 전파는 막지 않음
- **East / West block**: 동·서 폐색 개별 ON/OFF
- **RANGE RINGS**: 센서(35m) / V2V(120m) 범위 표시
- **pdr_scale** (lossy만): 0.1 나쁨 ↔ 1.0 거의 완벽, 즉시 반영

### 3-5. 화면 표시
- **SENSOR**: 센서 감지
- **V2V RX**: 이번 스텝 패킷 수신
- **V2V HOLD**: AoI 캐시 유지 (수신은 아님)

---

## 4. 문제 해결

| 증상 | 해결 |
|------|------|
| `Python not found` | Python 3.10+ 설치, PATH에 추가 후 `install.bat` 재실행 |
| pip / torch 실패 | [pytorch.org](https://pytorch.org) CPU 휠 설치 후 `install.bat` 다시 |
| `SUMO binary not found` | `.venv\Scripts\pip install eclipse-sumo` |
| `SUMO config not found` | `sumo_data/` 네 파일이 패키지 안에 있는지 확인 |
| 드래그앤드롭 안 됨 | **Add** 버튼 사용 (`tkinterdnd2` 확인) |
| 모델 모드가 이상함 | 파일명에 `lossy`/`perfect`/`sensor`/`multi` 포함 여부 확인 |

---

## 5. 모델

이 패키지에는 학습 가중치가 없습니다.  
팀에서 따로 보낸 `.zip`을 GUI에 넣어 사용하세요.
