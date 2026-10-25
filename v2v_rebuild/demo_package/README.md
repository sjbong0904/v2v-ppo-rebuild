# V2V Demo GUI

Intersection demo for comparing **sensor-only** vs **lossy V2V** policies under NLOS occlusion.

Default PPO checkpoints are already in `models/` and auto-load on start.

Capture tips: [`../DEMO_RECORDING.md`](../DEMO_RECORDING.md)

## Features

- **VIEW MODE**: `Single` | `Compare` (50/50 split)
- Compare: drag from MODELS, or use each pane’s **Add** button
- Auto play: loop episode / cycle scenarios / random·conflict seeds
- Shortcuts: `Space` pause · `R` restart · `S` start · `←` `→` seed
- Screen recording: Win+G or OBS

## Quick start (Windows)

1. Open this folder
2. Run **`install.bat`** once
3. Run **`run_demo.bat`**
4. Pick models (preloaded) → **Start**  
   Compare: assign left/right via drag or **Add**

`install.bat` creates `.venv`, installs deps, and installs **eclipse-sumo** via pip (system SUMO optional).

## Bundled models

| File | Profile |
| --- | --- |
| `sumo_ppo_50k_sensor.zip` | sensor-only |
| `sumo_ppo_50k_lossy.zip` | lossy V2V |
| `sumo_ppo_50k_multi_sensor.zip` | sensor-only, multi-NPC |
| `sumo_ppo_50k_multi_lossy.zip` | lossy V2V, multi-NPC |

You can still **Add** / drop extra `.zip` files (max 10).

## Layout

```
demo_package/
  install.bat
  run_demo.bat
  demo_app.py
  models/            # bundled checkpoints (auto-load)
  sumo_data/         # map files
  requirements.txt
  README.md
```

## Manual install (macOS / Linux)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt
python demo_app.py
```

## Usage notes

| Keyword in filename | Mode |
| --- | --- |
| `lossy` | lossy V2V |
| `perfect` | perfect V2V |
| `sensor` | sensor only |
| `multi` | multi NPC |

| Control | Action |
| --- | --- |
| Start / Restart | Run episode |
| Pause / Resume | Pause |
| NLOS / East / West | Sensor occlusion (V2V unaffected) |
| pdr_scale | Lossy only: 0.1 sparse ↔ 1.0 frequent RX |

`(rule-based)` runs without a trained zip.

## Troubleshooting

| Issue | Fix |
| --- | --- |
| `Python not found` | Install Python 3.10+, re-run `install.bat` |
| `SUMO binary not found` | `.venv/bin/pip install eclipse-sumo` |
| `SUMO config not found` | Keep files under `sumo_data/` |
| Drag-and-drop fails | Use pane **Add** |
