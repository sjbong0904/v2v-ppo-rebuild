# Demo media

README 상단 데모용 파일을 여기에 둡니다.

| 경로 | 용도 |
| --- | --- |
| `demo.gif` | GitHub README embed (현재 ~1.3 MB, 960×540 @12fps) |
| `demo.mp4` | 원본/고화질 (선택) |
| `frames/<timestamp>/` | Demo Viewer **Record frames** PNG 시퀀스 |
| `screenshot_*.png` | 단축키 `C` 스크린샷 |

촬영: [`../../DEMO_RECORDING.md`](../../DEMO_RECORDING.md)

```powershell
python tools/frames_to_gif.py presentation/media/frames/<timestamp> --out presentation/media/demo.gif
```
