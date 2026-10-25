# Demo recording guide (sensor vs lossy)

## VIEW MODE

| Mode | Behavior |
| --- | --- |
| **Single** | Select a model → Start |
| **Compare** | Add models to MODELS → drag onto left/right panes → Start |

Compare uses a **50/50** split view, the same seed on both sides, and **LEFT/RIGHT metrics on the right panel** (scroll each pane to see KEY).

## Compare capture

1. Add sensor / lossy zips to MODELS
2. VIEW MODE → **Compare**
3. Drag each model onto left and right panes
4. Enable NLOS / RANGE RINGS, fix seed
5. **Start**
6. Record the window with **Win+G** or OBS

## Auto play (unattended / looping capture)

| Option | Behavior |
| --- | --- |
| **Loop episode** | Same settings; after each finish waits ~2s and restarts |
| **Cycle scenarios** | Playlist: NLOS both → east → LOS clear → sparse/full PDR |
| **Random seed** | ON: new seed each episode · OFF: seed fixed at **7** |
| **Prefer conflict** | Biases random seeds toward tight ego/target arrival (more drama) |
| **Next scenario** | Jump forward in the playlist immediately |

`Space` during the 2s dwell cancels auto-continue.

Shortcuts: `Space` pause · `R` restart · `S` start · `←` `→` seed
