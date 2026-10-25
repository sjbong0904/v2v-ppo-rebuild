# V2V Demo GUI

Intersection demo for comparing sensor-only vs V2V observation policies.  
**Model `.zip` files are not bundled** — add them in the GUI after you receive them.

For capture tips see [`../DEMO_RECORDING.md`](../DEMO_RECORDING.md).

## Features

- **VIEW MODE**: `Single` | `Compare` (50/50 split)
- Single: pick a model → Start
- Compare: Add to MODELS → drag list items onto left/right panes → Start
- Top mode chip / bottom perception tip on each pane
- Shortcuts: Space / R / S / ← →
- Screen recording: use Win+G or OBS (no built-in recorder)

---

## Quick start (Windows)

1. Open this folder
2. Run **`install.bat`** once
3. Run **`run_demo.bat`**
4. **Add** / drop model `.zip` files into MODELS

`install.bat` creates `.venv`, installs Python deps, and installs **eclipse-sumo** via pip so a separate SUMO installer is optional.

---

## Layout

```
demo_package/
  install.bat
  run_demo.bat
  demo_app.py
  sumo_intersection_env.py
  view_sumo_custom.py
  requirements.txt
  README.md
  models/          # optional model zips
  sumo_data/       # required map files
```

---

## Manual install (macOS / Linux)

- Python 3.10+
- Optional system SUMO (or use `eclipse-sumo` from pip)

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt
python demo_app.py
```

---

## Usage

### Models

1. Put PPO `.zip` files in `models/` or elsewhere
2. **Add** or drop into MODELS (max 10)
3. Single: select → **Start**
4. Compare: drag from MODELS onto panes → **Start**
5. After changing models, **Restart**

`(rule-based)` works without a trained zip.

### Filename → mode

| Keyword in filename | Mode |
| --- | --- |
| `lossy` | lossy V2V |
| `perfect` | perfect V2V |
| `sensor` | sensor only |
| `multi` | multi NPC |

Example: `sumo_ppo_50k_lossy.zip` → `lossy_v2v`

### Controls

| Control | Action |
| --- | --- |
| Start | Start episode |
| Pause / Resume | Pause / resume |
| Restart | Restart |
| Mouse wheel | Zoom |
| seed / delay | Episode seed, frame delay |

### Options

- **NLOS**: sensor occlusion (buildings/trucks); V2V not blocked
- **East / West block**: per-side occlusion
- **RANGE RINGS**: sensor (35 m) / V2V (120 m)
- **pdr_scale** (lossy only): 0.1 sparse ↔ 1.0 frequent RX

### Overlays

- **SENSOR**: sensor detection beam
- **V2V RX**: packet received this step
- **V2V HOLD**: AoI cache hold

---

## Troubleshooting

| Issue | Fix |
| --- | --- |
| `Python not found` | Install Python 3.10+, re-run `install.bat` |
| pip / torch fail | Install CPU torch from [pytorch.org](https://pytorch.org), then `install.bat` |
| `SUMO binary not found` | `.venv\Scripts\pip install eclipse-sumo` |
| `SUMO config not found` | Keep all four files under `sumo_data/` |
| Drag-and-drop fails | Use **Add** (`tkinterdnd2`) |
| Wrong mode | Ensure filename contains `lossy` / `perfect` / `sensor` / `multi` |

---

## Models

This package does not ship trained weights.  
Load the `.zip` files provided by your team into the GUI.
