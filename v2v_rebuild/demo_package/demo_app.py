from __future__ import annotations

import math
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox

import sumolib
import traci
from stable_baselines3 import PPO
from tkinterdnd2 import DND_FILES, TkinterDnD

from sumo_intersection_env import (
    EGO_ID,
    NLOS_BUILDING_EAST_AABB,
    NLOS_BUILDING_WEST_AABB,
    NPC_WE_ID,
    PARKED_TRUCK_ID,
    PARKED_TRUCK_WEST_ID,
    TARGET_ID,
    V2V_MAX_RANGE,
    SumoIntersectionEnv,
    _segment_intersects_aabb,
)
from view_sumo_custom import (
    ACTION_NAME,
    BG,
    BUILDING,
    CONFLICT,
    DEFAULT_NET,
    DEFAULT_ZOOM,
    EGO,
    LOS_BLOCK,
    LOS_NONE,
    LOS_OK,
    NPC,
    PAD,
    PANEL_ACCENT,
    PANEL_BG,
    PANEL_FG,
    PANEL_MUTED,
    ROAD,
    TARGET,
    TRUCK,
    VIEW_H,
    VIEW_W,
    ZOOM_MAX,
    ZOOM_MIN,
    ZOOM_STEP,
    detection_channel,
    heading_label,
)

MAX_MODELS = 10
LEFT_W = 280
PANEL_W = 320
RULE_LABEL = "(rule-based)"
BRAKE_LIGHT = "#ff2a2a"
BRAKE_GLOW = "#ff5555"
SENSOR_FX = "#2ecc71"
V2V_FX = "#3498db"
V2V_HOLD_FX = "#1a5276"
BOTH_FX = "#1abc9c"
SECONDARY_FX = "#9b59b6"
PRIMARY_TAG = "#f39c12"


def infer_model_profile(path: Path | None) -> tuple[str, bool]:
    """Return (observation_mode, multi_npc) from model filename."""
    if path is None:
        return "sensor_only", False
    name = path.stem.lower()
    multi = "multi" in name
    if "perfect" in name:
        return "perfect_v2v", multi
    if "lossy" in name:
        return "lossy_v2v", multi
    if "sensor" in name:
        return "sensor_only", multi
    return "sensor_only", multi


class DemoApp:
    def __init__(self) -> None:
        self.root = TkinterDnD.Tk()
        self.root.title("V2V Demo Viewer")
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.minsize(1280, 760)
        try:
            self.root.state("zoomed")  # maximized (Windows)
        except tk.TclError:
            try:
                self.root.attributes("-zoomed", True)  # some Linux WMs
            except tk.TclError:
                self.root.geometry("1600x900")

        self.zoom = DEFAULT_ZOOM
        self.paused = True
        self.episode_done = False
        self.closing = False
        self.step_i = 0
        self.last_action = 2
        self.last_info: dict = {}
        self.obs = None
        self.env: SumoIntersectionEnv | None = None
        self.model: PPO | None = None
        self.model_paths: list[Path] = []
        self.selected_model: Path | None = None
        self.active_mode = "sensor_only"
        self.active_multi = False

        self.net = sumolib.net.readNet(str(DEFAULT_NET))
        self.lane_shapes = self._load_lane_shapes()

        # NLOS master + per-side occluders (east=SE / west=SW).
        self.nlos_var = tk.BooleanVar(value=True)
        self.nlos_east_var = tk.BooleanVar(value=True)
        self.nlos_west_var = tk.BooleanVar(value=True)
        self.show_sensor_range_var = tk.BooleanVar(value=True)
        self.show_v2v_range_var = tk.BooleanVar(value=True)
        self.pdr_scale_var = tk.DoubleVar(value=1.0)
        self.pdr_label_var = tk.StringVar(value="pdr_scale  1.00")
        self.seed_var = tk.IntVar(value=7)
        self.delay_var = tk.DoubleVar(value=0.08)
        self.profile_var = tk.StringVar(value="mode=sensor_only  multi=0")
        self.status_var = tk.StringVar(value="Add models, select one, then Start")
        self.sensor_range_m = 35.0

        self._build_ui()
        self._refresh_model_list()
        self._sync_profile_from_selection()
        self.root.after(50, self._tick)

    def _load_lane_shapes(self) -> list[list[tuple[float, float]]]:
        shapes: list[list[tuple[float, float]]] = []
        for edge in self.net.getEdges():
            for lane in edge.getLanes():
                shape = [(float(x), float(y)) for x, y in lane.getShape()]
                if len(shape) >= 2:
                    shapes.append(shape)
        return shapes

    def _build_ui(self) -> None:
        outer = tk.Frame(self.root, bg=BG)
        outer.pack(fill=tk.BOTH, expand=True)

        left = tk.Frame(outer, width=LEFT_W, bg=PANEL_BG)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(PAD, 0), pady=PAD)
        left.pack_propagate(False)

        tk.Label(
            left,
            text="MODELS",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(10, 4))

        tk.Label(
            left,
            text=f"Drop .zip here (max {MAX_MODELS})\nor use Add",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 9),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 6))

        self.model_list = tk.Listbox(
            left,
            bg="#0d1014",
            fg=PANEL_FG,
            selectbackground="#2c3e50",
            selectforeground=PANEL_FG,
            activestyle="none",
            font=("Consolas", 10),
            highlightthickness=1,
            highlightbackground="#2a3038",
            relief=tk.FLAT,
            exportselection=False,
        )
        self.model_list.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 6))
        self.model_list.bind("<<ListboxSelect>>", self._on_model_select)
        self.model_list.drop_target_register(DND_FILES)
        self.model_list.dnd_bind("<<Drop>>", self._on_drop)

        btn_row = tk.Frame(left, bg=PANEL_BG)
        btn_row.pack(fill=tk.X, padx=10, pady=(0, 8))
        self._btn(btn_row, "Add", self._add_models).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        self._btn(btn_row, "Remove", self._remove_selected).pack(side=tk.LEFT, expand=True, fill=tk.X)

        tk.Label(
            left,
            text="ACTIVE PROFILE",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(8, 4))
        tk.Label(
            left,
            text="mode / multi follow the selected model name",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 4))
        tk.Label(
            left,
            textvariable=self.profile_var,
            bg=PANEL_BG,
            fg=PANEL_FG,
            font=("Consolas", 10),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 8))

        tk.Label(
            left,
            text="MAP SETTING",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(4, 4))
        tk.Checkbutton(
            left,
            text="NLOS occlusion (sensor)",
            variable=self.nlos_var,
            command=self._on_nlos_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 2))

        self.nlos_side_frame = tk.Frame(left, bg=PANEL_BG)
        self.nlos_side_frame.pack(fill=tk.X, padx=18, pady=(0, 2))
        self.nlos_east_cb = tk.Checkbutton(
            self.nlos_side_frame,
            text="East block (EW target)",
            variable=self.nlos_east_var,
            command=self._on_nlos_side_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 9),
            anchor="w",
        )
        self.nlos_east_cb.pack(fill=tk.X)
        self.nlos_west_cb = tk.Checkbutton(
            self.nlos_side_frame,
            text="West block (WE npc)",
            variable=self.nlos_west_var,
            command=self._on_nlos_side_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 9),
            anchor="w",
        )
        self.nlos_west_cb.pack(fill=tk.X)
        tk.Label(
            left,
            text="East=SE building+truck\nWest=same layout rotated 90 CW",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 6))
        self._sync_nlos_side_controls()

        self.pdr_frame = tk.Frame(left, bg=PANEL_BG)
        self.pdr_frame.pack(fill=tk.X, padx=10, pady=(0, 8))
        tk.Label(
            self.pdr_frame,
            text="LOSSY V2V",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Label(
            self.pdr_frame,
            textvariable=self.pdr_label_var,
            bg=PANEL_BG,
            fg=PANEL_FG,
            font=("Consolas", 9),
            anchor="w",
        ).pack(fill=tk.X, pady=(2, 0))
        self.pdr_scale = tk.Scale(
            self.pdr_frame,
            from_=0.1,
            to=1.0,
            resolution=0.05,
            orient=tk.HORIZONTAL,
            variable=self.pdr_scale_var,
            command=self._on_pdr_scale,
            bg=PANEL_BG,
            fg=PANEL_FG,
            troughcolor="#2a3038",
            highlightthickness=0,
            showvalue=False,
            length=220,
        )
        self.pdr_scale.pack(fill=tk.X, pady=(2, 0))
        tk.Label(
            self.pdr_frame,
            text="0.1 sparse RX  ←→  1.0 frequent RX\n(AoI still holds ~1s after each packet)",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X)
        self._sync_pdr_controls()

        tk.Label(
            left,
            text="RANGE RINGS",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(4, 4))
        tk.Checkbutton(
            left,
            text=f"sensor range ({35.0:.0f} m)",
            variable=self.show_sensor_range_var,
            command=self._on_overlay_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X, padx=10)
        tk.Checkbutton(
            left,
            text=f"V2V range ({V2V_MAX_RANGE:.0f} m)",
            variable=self.show_v2v_range_var,
            command=self._on_overlay_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 8))

        seed_row = tk.Frame(left, bg=PANEL_BG)
        seed_row.pack(fill=tk.X, padx=10, pady=(0, 0))
        tk.Label(seed_row, text="seed", bg=PANEL_BG, fg=PANEL_MUTED, font=("Consolas", 10)).pack(
            side=tk.LEFT
        )
        tk.Spinbox(
            seed_row,
            from_=0,
            to=9999,
            textvariable=self.seed_var,
            width=8,
            font=("Consolas", 10),
            bg="#0d1014",
            fg=PANEL_FG,
            buttonbackground=PANEL_BG,
        ).pack(side=tk.RIGHT)

        delay_row = tk.Frame(left, bg=PANEL_BG)
        delay_row.pack(fill=tk.X, padx=10, pady=(4, 8))
        tk.Label(delay_row, text="delay(s)", bg=PANEL_BG, fg=PANEL_MUTED, font=("Consolas", 10)).pack(
            side=tk.LEFT
        )
        tk.Spinbox(
            delay_row,
            from_=0.02,
            to=1.0,
            increment=0.02,
            textvariable=self.delay_var,
            width=8,
            font=("Consolas", 10),
            bg="#0d1014",
            fg=PANEL_FG,
            buttonbackground=PANEL_BG,
        ).pack(side=tk.RIGHT)

        ctrl = tk.Frame(left, bg=PANEL_BG)
        ctrl.pack(fill=tk.X, padx=10, pady=(4, 10))
        self._btn(ctrl, "Start", self._start).pack(fill=tk.X, pady=2)
        self._btn(ctrl, "Pause / Resume", self._toggle_pause).pack(fill=tk.X, pady=2)
        self._btn(ctrl, "Restart", self._restart).pack(fill=tk.X, pady=2)

        tk.Label(
            left,
            textvariable=self.status_var,
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 9),
            wraplength=LEFT_W - 24,
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 10))

        center = tk.Frame(outer, bg=BG)
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(PAD, 0), pady=PAD)
        self.canvas = tk.Canvas(
            center, width=VIEW_W, height=VIEW_H, bg=BG, highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<MouseWheel>", self._on_wheel)
        self.canvas.bind("<Button-4>", lambda e: self._zoom_by(ZOOM_STEP))
        self.canvas.bind("<Button-5>", lambda e: self._zoom_by(1.0 / ZOOM_STEP))
        self.canvas.bind("<Configure>", self._on_canvas_resize)

        right = tk.Frame(outer, width=PANEL_W, bg=PANEL_BG)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=PAD, pady=PAD)
        right.pack_propagate(False)
        self.hud = tk.Text(
            right,
            bg=PANEL_BG,
            fg=PANEL_FG,
            font=("Consolas", 11),
            relief=tk.FLAT,
            wrap=tk.WORD,
            state=tk.DISABLED,
            padx=10,
            pady=10,
        )
        self.hud.pack(fill=tk.BOTH, expand=True)
        self.hud.tag_configure("title", foreground=PANEL_ACCENT, font=("Consolas", 12, "bold"))
        self.hud.tag_configure("muted", foreground=PANEL_MUTED)
        self.hud.tag_configure("warn", foreground="#e67e22")
        self.hud.tag_configure("ok", foreground="#2ecc71")
        self.hud.tag_configure("bad", foreground="#e74c3c")

        self.root.bind("<space>", lambda e: self._toggle_pause())
        self._draw_placeholder()
        self._update_hud()

    def _btn(self, parent: tk.Misc, text: str, command) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg="#2a3038",
            fg=PANEL_FG,
            activebackground="#3a4250",
            activeforeground=PANEL_FG,
            relief=tk.FLAT,
            font=("Consolas", 10),
            padx=8,
            pady=4,
        )

    def _view_size(self) -> tuple[int, int]:
        w = max(200, int(self.canvas.winfo_width() or VIEW_W))
        h = max(200, int(self.canvas.winfo_height() or VIEW_H))
        return w, h

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        if self.env is None:
            self._draw_placeholder()
        else:
            self._draw_frame()

    def _parse_drop_paths(self, data: str) -> list[Path]:
        # tkdnd may wrap paths with braces when spaces exist.
        parts: list[str] = []
        buf = ""
        in_brace = False
        for ch in data.strip():
            if ch == "{":
                in_brace = True
                buf = ""
            elif ch == "}":
                in_brace = False
                parts.append(buf)
                buf = ""
            elif ch == " " and not in_brace:
                if buf:
                    parts.append(buf)
                    buf = ""
            else:
                buf += ch
        if buf:
            parts.append(buf)
        return [Path(p) for p in parts]

    def _on_drop(self, event) -> None:
        paths = self._parse_drop_paths(event.data)
        self._ingest_paths(paths)

    def _add_models(self) -> None:
        files = filedialog.askopenfilenames(
            title="Select PPO model zip files",
            filetypes=[("PPO zip", "*.zip"), ("All files", "*.*")],
        )
        if files:
            self._ingest_paths([Path(f) for f in files])

    def _ingest_paths(self, paths: list[Path]) -> None:
        added = 0
        skipped = 0
        for path in paths:
            path = path.resolve()
            if path.suffix.lower() != ".zip":
                skipped += 1
                continue
            if not path.is_file():
                skipped += 1
                continue
            if path in self.model_paths:
                skipped += 1
                continue
            if len(self.model_paths) >= MAX_MODELS:
                messagebox.showwarning(
                    "Model limit",
                    f"Maximum {MAX_MODELS} models. Remove one before adding more.",
                )
                break
            self.model_paths.append(path)
            added += 1
        self._refresh_model_list()
        self.status_var.set(f"Added {added}, skipped {skipped}. Models: {len(self.model_paths)}/{MAX_MODELS}")

    def _remove_selected(self) -> None:
        idxs = self.model_list.curselection()
        if not idxs:
            return
        idx = int(idxs[0])
        if idx == 0:
            self.status_var.set("Rule-based entry cannot be removed")
            return
        path_idx = idx - 1
        removed = self.model_paths.pop(path_idx)
        if self.selected_model == removed:
            self.selected_model = None
            self.model = None
        self._refresh_model_list()
        self.model_list.selection_set(0)
        self.status_var.set(f"Removed {removed.name}")

    def _refresh_model_list(self) -> None:
        self.model_list.delete(0, tk.END)
        self.model_list.insert(tk.END, RULE_LABEL)
        for path in self.model_paths:
            mode, multi = infer_model_profile(path)
            tag = mode.replace("_v2v", "").replace("_only", "")
            if multi:
                tag = f"multi/{tag}"
            self.model_list.insert(tk.END, f"{path.name}  [{tag}]")
        self.model_list.selection_clear(0, tk.END)
        if self.selected_model is None:
            self.model_list.selection_set(0)
        else:
            try:
                i = self.model_paths.index(self.selected_model) + 1
                self.model_list.selection_set(i)
            except ValueError:
                self.selected_model = None
                self.model_list.selection_set(0)
        self._sync_profile_from_selection()

    def _sync_profile_from_selection(self) -> None:
        self.active_mode, self.active_multi = infer_model_profile(self.selected_model)
        self.profile_var.set(
            f"mode={self.active_mode}\nmulti_npc={int(self.active_multi)}"
        )
        self._sync_pdr_controls()

    def _on_model_select(self, _event=None) -> None:
        idxs = self.model_list.curselection()
        if not idxs:
            return
        idx = int(idxs[0])
        if idx == 0:
            self.selected_model = None
            self.model = None
            self.status_var.set("Selected rule-based policy")
        else:
            self.selected_model = self.model_paths[idx - 1]
            self.status_var.set(f"Selected {self.selected_model.name} (Restart to apply)")
        self._sync_profile_from_selection()

    def _load_selected_model(self) -> None:
        if self.selected_model is None:
            self.model = None
            return
        try:
            self.model = PPO.load(str(self.selected_model))
        except Exception as exc:
            messagebox.showerror("Load failed", f"{self.selected_model.name}\n{exc}")
            self.model = None
            self.selected_model = None
            self._refresh_model_list()
            self.model_list.selection_set(0)

    def _close_env(self) -> None:
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass
            self.env = None

    def _start(self) -> None:
        self._restart(autoplay=True)

    def _restart(self, autoplay: bool = True) -> None:
        self._close_env()
        self._load_selected_model()
        self._sync_profile_from_selection()
        seed = int(self.seed_var.get())
        self.env = SumoIntersectionEnv(
            observation_mode=self.active_mode,
            pdr_scale=float(self.pdr_scale_var.get()) if self.active_mode == "lossy_v2v" else 1.0,
            sensor_range=35.0,
            nlos_blocker=bool(self.nlos_var.get()),
            nlos_east=bool(self.nlos_east_var.get()),
            nlos_west=bool(self.nlos_west_var.get()),
            multi_npc=bool(self.active_multi),
            seed=seed,
            sumo_binary="sumo",
        )
        self.sensor_range_m = float(self.env.sensor_range)
        self.obs, _ = self.env.reset(seed=seed)
        self.step_i = 0
        self.last_action = 2
        self.last_info = {}
        self.episode_done = False
        self.paused = not autoplay
        label = RULE_LABEL if self.model is None else self.selected_model.name
        self.status_var.set(f"Running: {label}" if autoplay else f"Ready: {label}")
        self._draw_frame()
        self._update_hud()

    def _sync_pdr_controls(self) -> None:
        lossy = self.active_mode == "lossy_v2v"
        state = tk.NORMAL if lossy else tk.DISABLED
        self.pdr_scale.configure(state=state)
        value = float(self.pdr_scale_var.get())
        if lossy:
            self.pdr_label_var.set(f"pdr_scale  {value:.2f}")
        else:
            self.pdr_label_var.set("pdr_scale  (lossy model only)")

    def _on_pdr_scale(self, _value: str | None = None) -> None:
        value = float(self.pdr_scale_var.get())
        if self.active_mode != "lossy_v2v":
            self._sync_pdr_controls()
            return
        self.pdr_label_var.set(f"pdr_scale  {value:.2f}")
        if self.env is not None:
            self.env.pdr_scale = value
        self.status_var.set(f"pdr_scale={value:.2f} (applies immediately)")

    def _sync_nlos_side_controls(self) -> None:
        state = tk.NORMAL if self.nlos_var.get() else tk.DISABLED
        self.nlos_east_cb.configure(state=state)
        self.nlos_west_cb.configure(state=state)

    def _apply_nlos_to_env(self) -> None:
        if self.env is None:
            return
        self.env.nlos_blocker = bool(self.nlos_var.get())
        self.env.nlos_east = bool(self.nlos_east_var.get())
        self.env.nlos_west = bool(self.nlos_west_var.get())

    def _on_nlos_toggle(self) -> None:
        self._sync_nlos_side_controls()
        self._apply_nlos_to_env()
        self._on_overlay_toggle()
        if not self.nlos_var.get():
            self.status_var.set("NLOS OFF: obstacles hidden")
        else:
            sides = []
            if self.nlos_east_var.get():
                sides.append("east")
            if self.nlos_west_var.get():
                sides.append("west")
            self.status_var.set(
                f"NLOS ON: {','.join(sides) if sides else 'no side selected'}"
            )

    def _on_nlos_side_toggle(self) -> None:
        self._apply_nlos_to_env()
        self._on_overlay_toggle()
        if self.nlos_var.get():
            sides = []
            if self.nlos_east_var.get():
                sides.append("east")
            if self.nlos_west_var.get():
                sides.append("west")
            self.status_var.set(
                f"NLOS ON: {','.join(sides) if sides else 'no side selected'}"
            )

    def _on_overlay_toggle(self) -> None:
        if self.env is None:
            self._draw_placeholder()
        else:
            self._draw_frame()

    def _toggle_pause(self) -> None:
        if self.env is None:
            self.status_var.set("Start an episode first")
            return
        if self.episode_done:
            self.status_var.set("Episode finished. Press Restart")
            return
        self.paused = not self.paused
        self.status_var.set("Paused" if self.paused else "Running")

    def _on_wheel(self, event: tk.Event) -> None:
        if event.delta > 0:
            self._zoom_by(ZOOM_STEP)
        else:
            self._zoom_by(1.0 / ZOOM_STEP)

    def _zoom_by(self, factor: float) -> None:
        self.zoom = float(min(ZOOM_MAX, max(ZOOM_MIN, self.zoom * factor)))
        if self.env is None:
            self._draw_placeholder()
        else:
            self._draw_frame()
        self._update_hud()

    def _on_close(self) -> None:
        self.closing = True
        self._close_env()
        self.root.destroy()

    def _ego_pose(self) -> tuple[float, float, float]:
        if self.env is None:
            return 0.0, -80.0, 0.0
        if EGO_ID in traci.vehicle.getIDList():
            x, y = traci.vehicle.getPosition(EGO_ID)
            return float(x), float(y), float(traci.vehicle.getAngle(EGO_ID))
        return 0.0, float(self.env.ego_y), 0.0

    def _world_to_screen(
        self, wx: float, wy: float, cam_x: float, cam_y: float
    ) -> tuple[int, int]:
        vw, vh = self._view_size()
        sx = vw * 0.5 + (wx - cam_x) * self.zoom
        sy = vh * 0.5 - (wy - cam_y) * self.zoom
        return int(sx), int(sy)

    def _draw_placeholder(self) -> None:
        self.canvas.delete("all")
        vw, vh = self._view_size()
        self.canvas.create_text(
            vw // 2,
            vh // 2,
            text="Press Start to begin demo\ncamera locked on ego",
            fill=PANEL_MUTED,
            font=("Consolas", 14),
            justify=tk.CENTER,
        )

    def _draw_frame(self) -> None:
        if self.env is None:
            self._draw_placeholder()
            return
        self.canvas.delete("all")
        cam_x, cam_y, _ = self._ego_pose()

        for shape in self.lane_shapes:
            pts: list[int] = []
            for wx, wy in shape:
                sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
                pts.extend([sx, sy])
            if len(pts) >= 4:
                self.canvas.create_line(
                    *pts, fill=ROAD, width=max(2, int(3.2 * self.zoom / DEFAULT_ZOOM))
                )

        if self.nlos_var.get() and self.nlos_east_var.get():
            xmin, ymin, xmax, ymax = NLOS_BUILDING_EAST_AABB
            bpts: list[int] = []
            for wx, wy in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
                sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
                bpts.extend([sx, sy])
            self.canvas.create_polygon(*bpts, fill=BUILDING, outline="#4b5563")
            self.canvas.create_text(
                *self._world_to_screen((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, cam_x, cam_y),
                text="E",
                fill="#dddddd",
                font=("Consolas", 10, "bold"),
            )
        if self.nlos_var.get() and self.nlos_west_var.get():
            xmin, ymin, xmax, ymax = NLOS_BUILDING_WEST_AABB
            bpts = []
            for wx, wy in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
                sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
                bpts.extend([sx, sy])
            self.canvas.create_polygon(*bpts, fill=BUILDING, outline="#4b5563")
            self.canvas.create_text(
                *self._world_to_screen((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, cam_x, cam_y),
                text="W",
                fill="#dddddd",
                font=("Consolas", 10, "bold"),
            )

        cx, cy = self._world_to_screen(0.0, 0.0, cam_x, cam_y)
        r = max(3, int(0.8 * self.zoom))
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline=CONFLICT, width=2)
        self.canvas.create_line(cx - 2 * r, cy, cx + 2 * r, cy, fill=CONFLICT)
        self.canvas.create_line(cx, cy - 2 * r, cx, cy + 2 * r, fill=CONFLICT)

        threat_id = str(self.last_info.get("primary_threat") or "")
        ids = set(traci.vehicle.getIDList())
        if EGO_ID in ids:
            ex, ey = traci.vehicle.getPosition(EGO_ID)
            for vid in self._traffic_ids():
                if vid not in ids:
                    continue
                tx, ty = traci.vehicle.getPosition(vid)
                role = "primary" if (threat_id and vid == threat_id) else "secondary"
                self._draw_detection_effects(
                    float(ex), float(ey), float(tx), float(ty), cam_x, cam_y,
                    role=role,
                    vid=vid,
                )

        for vid in traci.vehicle.getIDList():
            if vid == PARKED_TRUCK_ID and not (
                self.nlos_var.get() and self.nlos_east_var.get()
            ):
                continue
            if vid == PARKED_TRUCK_WEST_ID and not (
                self.nlos_var.get() and self.nlos_west_var.get()
            ):
                continue
            self._draw_vehicle(vid, cam_x, cam_y)

        self._draw_range_rings(cam_x, cam_y)

        ex, ey, _ = self._ego_pose()
        sx, sy = self._world_to_screen(ex, ey, cam_x, cam_y)
        ring = self._ego_ring_color()
        self.canvas.create_oval(sx - 7, sy - 7, sx + 7, sy + 7, outline=ring, width=2)

        self._draw_legend()

    def _traffic_ids(self) -> list[str]:
        """Non-ego traffic used in the crossing scenario (exclude parked trucks)."""
        skip = {EGO_ID, PARKED_TRUCK_ID, PARKED_TRUCK_WEST_ID}
        return [vid for vid in traci.vehicle.getIDList() if vid not in skip]

    def _los_occluded_to(self, ex: float, ey: float, tx: float, ty: float) -> bool:
        if not self.nlos_var.get():
            return False
        if self.env is not None:
            return self.env._los_occluded_between(ex, ey, tx, ty)
        if self.nlos_east_var.get() and _segment_intersects_aabb(
            ex, ey, tx, ty, NLOS_BUILDING_EAST_AABB
        ):
            return True
        if self.nlos_west_var.get() and _segment_intersects_aabb(
            ex, ey, tx, ty, NLOS_BUILDING_WEST_AABB
        ):
            return True
        return False

    def _per_vehicle_detect(
        self, vid: str, ex: float, ey: float, tx: float, ty: float, is_primary: bool
    ) -> tuple[bool, bool, bool]:
        """Return (sensor_on, v2v_on, occluded) for drawing. Primary uses env info."""
        sens, v2v_state, occ = self._per_vehicle_link_state(vid, ex, ey, tx, ty, is_primary)
        return sens, v2v_state != "none", occ

    def _per_vehicle_link_state(
        self, vid: str, ex: float, ey: float, tx: float, ty: float, is_primary: bool
    ) -> tuple[bool, str, bool]:
        """Return (sensor_on, v2v_state, occluded).

        v2v_state: none | rx | hold | perfect
        rx = packet received this step, hold = AoI cache only.
        """
        dist = math.hypot(tx - ex, ty - ey)
        occ = self._los_occluded_to(ex, ey, tx, ty)
        sensor_m = float(self.env.sensor_range) if self.env is not None else self.sensor_range_m
        sens_geo = dist <= sensor_m and not occ
        mode = self.active_mode

        if mode == "sensor_only":
            sens = (
                bool(self.last_info.get("sensor_visible", sens_geo))
                if is_primary and self.last_info
                else sens_geo
            )
            return sens, "none", (
                bool(self.last_info.get("occluded", occ)) if is_primary and self.last_info else occ
            )

        if mode == "perfect_v2v":
            if is_primary and self.last_info:
                sens = bool(self.last_info.get("sensor_visible", sens_geo))
                occ = bool(self.last_info.get("occluded", occ))
                vis = bool(self.last_info.get("visible", False))
                return sens, ("perfect" if vis else "none"), occ
            return sens_geo, ("perfect" if dist <= V2V_MAX_RANGE else "none"), occ

        # lossy_v2v
        if is_primary and self.last_info:
            sens = bool(self.last_info.get("sensor_visible", sens_geo))
            occ = bool(self.last_info.get("occluded", occ))
            if bool(self.last_info.get("v2v_rx_this_step", False)):
                return sens, "rx", occ
            if self.env is not None and self.env._fresh_bsm_available():
                bsm = self.env.last_bsm or {}
                bsm_vid = bsm.get("vid")
                if bsm_vid is None or bsm_vid == vid:
                    return sens, "hold", occ
            return sens, "none", occ

        # Secondary: only mirror env BSM for that vehicle (no fake always-on radio).
        if self.env is not None and self.env._fresh_bsm_available():
            bsm = self.env.last_bsm or {}
            if bsm.get("vid") == vid:
                if bool(self.last_info.get("v2v_rx_this_step", False)):
                    return sens_geo, "rx", occ
                return sens_geo, "hold", occ
        return sens_geo, "none", occ

    def _draw_range_rings(self, cam_x: float, cam_y: float) -> None:
        """Dashed circles around ego for sensor / V2V max range."""
        ex, ey, _ = self._ego_pose()
        sx, sy = self._world_to_screen(ex, ey, cam_x, cam_y)
        sensor_m = (
            float(self.env.sensor_range)
            if self.env is not None
            else self.sensor_range_m
        )
        if self.show_sensor_range_var.get():
            r = max(2, int(sensor_m * self.zoom))
            self.canvas.create_oval(
                sx - r, sy - r, sx + r, sy + r,
                outline=SENSOR_FX, dash=(6, 5), width=2,
            )
            self.canvas.create_text(
                sx,
                sy - r - 10,
                text=f"sensor {sensor_m:.0f}m",
                fill=SENSOR_FX,
                font=("Consolas", 9),
            )
        if self.show_v2v_range_var.get():
            r = max(2, int(V2V_MAX_RANGE * self.zoom))
            self.canvas.create_oval(
                sx - r, sy - r, sx + r, sy + r,
                outline=V2V_FX, dash=(10, 6), width=2,
            )
            self.canvas.create_text(
                sx,
                sy - r - 10,
                text=f"V2V {V2V_MAX_RANGE:.0f}m",
                fill=V2V_FX,
                font=("Consolas", 9),
            )

    def _detection_flags(self) -> tuple[bool, bool, bool]:
        """Return (sensor_active, v2v_active, occluded)."""
        info = self.last_info
        sens = bool(info.get("sensor_visible", False))
        vis = bool(info.get("visible", False))
        occ = bool(info.get("occluded", False))
        mode = self.active_mode
        v2v_fresh = (
            bool(self.env._fresh_bsm_available())
            if self.env is not None and mode == "lossy_v2v"
            else mode == "perfect_v2v"
        )
        channel = detection_channel(mode, sens, vis, v2v_fresh)
        sensor_on = "sensor" in channel
        v2v_on = "v2v" in channel
        return sensor_on, v2v_on, occ

    def _ego_ring_color(self) -> str:
        sensor_on, v2v_on, _ = self._detection_flags()
        if sensor_on and v2v_on:
            return BOTH_FX
        if sensor_on:
            return SENSOR_FX
        if v2v_on:
            return V2V_FX
        return "#ffffff"

    def _draw_legend(self) -> None:
        x0, y0 = 12, 12
        items = [
            (SENSOR_FX, "SENSOR  solid beam"),
            (V2V_FX, "V2V RX  bright pulse"),
            (V2V_HOLD_FX, "V2V HOLD  AoI cache"),
            (SECONDARY_FX, "SECONDARY  other traffic"),
            (PRIMARY_TAG, "PRIMARY  policy threat"),
            (BRAKE_LIGHT, "BRAKE  rear lights"),
        ]
        for i, (color, label) in enumerate(items):
            y = y0 + i * 18
            self.canvas.create_rectangle(x0, y, x0 + 14, y + 10, fill=color, outline="")
            self.canvas.create_text(
                x0 + 20,
                y + 5,
                text=label,
                fill=PANEL_MUTED,
                font=("Consolas", 9),
                anchor="w",
            )

    def _draw_detection_effects(
        self,
        ex: float,
        ey: float,
        tx: float,
        ty: float,
        cam_x: float,
        cam_y: float,
        role: str = "primary",
        vid: str = TARGET_ID,
    ) -> None:
        """Draw ego-vehicle links only when sensor or V2V detection is active."""
        is_primary = role == "primary"
        sensor_on, v2v_state, _occ = self._per_vehicle_link_state(
            vid, ex, ey, tx, ty, is_primary
        )
        if not sensor_on and v2v_state == "none":
            return

        x0, y0 = self._world_to_screen(ex, ey, cam_x, cam_y)
        x1, y1 = self._world_to_screen(tx, ty, cam_x, cam_y)
        mx, my = (x0 + x1) // 2, (y0 + y1) // 2
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy) or 1.0
        px, py = -dy / length, dx / length

        beam_w = 3 if is_primary else 2
        label_suffix = "" if is_primary else " (sec)"

        if sensor_on:
            color = SENSOR_FX if is_primary else SECONDARY_FX
            self.canvas.create_line(x0, y0, x1, y1, fill=color, width=beam_w)
            tr = max(4, int((1.2 if is_primary else 0.9) * self.zoom))
            self.canvas.create_oval(
                x1 - tr, y1 - tr, x1 + tr, y1 + tr, outline=color, width=2 if is_primary else 1
            )
            if is_primary:
                self.canvas.create_line(x1 - tr - 2, y1, x1 + tr + 2, y1, fill=SENSOR_FX)
                self.canvas.create_line(x1, y1 - tr - 2, x1, y1 + tr + 2, fill=SENSOR_FX)
            self.canvas.create_text(
                mx + int(10 * px),
                my + int(10 * py) - (12 if v2v_state != "none" else 0),
                text=f"SENSOR{label_suffix}",
                fill=color,
                font=("Consolas", 9, "bold"),
            )

        if v2v_state != "none":
            if v2v_state == "perfect":
                color = V2V_FX if is_primary else SECONDARY_FX
                width = beam_w
                dash = (8, 5)
                label = f"V2V{label_suffix}"
            elif v2v_state == "rx":
                color = V2V_FX if is_primary else SECONDARY_FX
                width = beam_w + 1
                dash = ()
                label = f"V2V RX{label_suffix}"
            else:
                color = V2V_HOLD_FX if is_primary else SECONDARY_FX
                width = max(1, beam_w - 1)
                dash = (3, 6)
                label = f"V2V HOLD{label_suffix}"

            off = 8 if sensor_on else 0
            if not is_primary:
                off = -10 if sensor_on else -6
            ax0, ay0 = x0 + int(off * px), y0 + int(off * py)
            ax1, ay1 = x1 + int(off * px), y1 + int(off * py)
            line_kw: dict = {"fill": color, "width": width}
            if dash:
                line_kw["dash"] = dash
            self.canvas.create_line(ax0, ay0, ax1, ay1, **line_kw)
            if is_primary and v2v_state in ("rx", "perfect"):
                phase = (self.step_i % 12) / 12.0
                for k in range(3):
                    rad = int((8 + 10 * k + 6 * phase) * max(0.7, self.zoom / DEFAULT_ZOOM))
                    self.canvas.create_oval(
                        x0 - rad, y0 - rad, x0 + rad, y0 + rad,
                        outline=color, width=1,
                    )
                for t in (0.25, 0.5, 0.75):
                    dx_i = int(ax0 + t * (ax1 - ax0))
                    dy_i = int(ay0 + t * (ay1 - ay0))
                    d = 4
                    self.canvas.create_polygon(
                        dx_i, dy_i - d,
                        dx_i + d, dy_i,
                        dx_i, dy_i + d,
                        dx_i - d, dy_i,
                        fill=color, outline="",
                    )
            self.canvas.create_text(
                mx + int(10 * px),
                my + int(10 * py) + (12 if sensor_on else 0),
                text=label,
                fill=color,
                font=("Consolas", 9, "bold"),
            )

    def _draw_vehicle(self, vid: str, cam_x: float, cam_y: float) -> None:
        x, y = traci.vehicle.getPosition(vid)
        ang = math.radians(float(traci.vehicle.getAngle(vid)))
        length = float(traci.vehicle.getLength(vid))
        width = float(traci.vehicle.getWidth(vid))
        fx, fy = math.sin(ang), math.cos(ang)
        lx, ly = -fy, fx
        hl, hw = 0.5 * length, 0.5 * width
        ordered = [
            (x + hl * fx + hw * lx, y + hl * fy + hw * ly),
            (x + hl * fx - hw * lx, y + hl * fy - hw * ly),
            (x - hl * fx - hw * lx, y - hl * fy - hw * ly),
            (x - hl * fx + hw * lx, y - hl * fy + hw * ly),
        ]
        pts: list[int] = []
        for wx, wy in ordered:
            sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
            pts.extend([sx, sy])
        primary = str(self.last_info.get("primary_threat") or "")
        if vid == EGO_ID:
            color = EGO
        elif vid == TARGET_ID:
            color = TARGET
        elif vid == PARKED_TRUCK_ID or vid == PARKED_TRUCK_WEST_ID:
            color = TRUCK
        elif vid == NPC_WE_ID:
            color = NPC
        else:
            color = NPC
        is_primary = bool(primary) and vid == primary
        outline = PRIMARY_TAG if is_primary else "#111111"
        outline_w = 3 if is_primary else 1
        self.canvas.create_polygon(*pts, fill=color, outline=outline, width=outline_w)
        nx, ny = self._world_to_screen(x + hl * fx, y + hl * fy, cam_x, cam_y)
        cx, cy = self._world_to_screen(float(x), float(y), cam_x, cam_y)
        self.canvas.create_line(cx, cy, nx, ny, fill="#ffffff", width=2)

        if vid == EGO_ID and self.last_action in (0, 1):
            self._draw_brake_effect(x, y, fx, fy, lx, ly, hl, hw, cam_x, cam_y)

        if vid != EGO_ID and vid not in {PARKED_TRUCK_ID, PARKED_TRUCK_WEST_ID}:
            role = "PRIMARY" if is_primary else "OTHER"
            speed = float(traci.vehicle.getSpeed(vid))
            tag_color = PRIMARY_TAG if is_primary else SECONDARY_FX
            self.canvas.create_text(
                cx,
                cy - max(14, int(2.2 * self.zoom)),
                text=f"{vid} [{role}] {speed:.1f}m/s",
                fill=tag_color,
                font=("Consolas", 9, "bold"),
            )

    def _draw_brake_effect(
        self,
        x: float,
        y: float,
        fx: float,
        fy: float,
        lx: float,
        ly: float,
        hl: float,
        hw: float,
        cam_x: float,
        cam_y: float,
    ) -> None:
        """Rear brake lights + short glow when ego is braking."""
        hard = self.last_action == 0
        light_r = max(3, int((1.1 if hard else 0.8) * self.zoom))
        glow_r = max(6, int((2.4 if hard else 1.6) * self.zoom))
        # Rear center slightly behind the body.
        rear_x = x - (hl + 0.4) * fx
        rear_y = y - (hl + 0.4) * fy
        gx, gy = self._world_to_screen(rear_x, rear_y, cam_x, cam_y)
        self.canvas.create_oval(
            gx - glow_r, gy - glow_r, gx + glow_r, gy + glow_r,
            outline=BRAKE_GLOW, width=2 if hard else 1,
        )
        # Left / right brake lamps
        for side in (-1.0, 1.0):
            lx_w = x - hl * fx + side * 0.7 * hw * lx
            ly_w = y - hl * fy + side * 0.7 * hw * ly
            sx, sy = self._world_to_screen(lx_w, ly_w, cam_x, cam_y)
            self.canvas.create_oval(
                sx - light_r, sy - light_r, sx + light_r, sy + light_r,
                fill=BRAKE_LIGHT, outline="#ffdddd",
            )

    def _vehicle_speed_heading(self, vid: str) -> tuple[float, str]:
        if self.env is None or vid not in traci.vehicle.getIDList():
            return 0.0, "n/a"
        return float(traci.vehicle.getSpeed(vid)), heading_label(float(traci.vehicle.getAngle(vid)))

    def _update_hud(self) -> None:
        mode = self.active_mode
        info = self.last_info
        model_name = RULE_LABEL if self.selected_model is None else self.selected_model.name

        if self.env is None:
            lines: list[tuple[str, str]] = [
                ("title", "DEMO\n"),
                ("muted", "No episode running.\n"),
                ("", f"model         {model_name}\n"),
                ("", f"mode          {mode}\n"),
                ("", f"multi_npc     {int(self.active_multi)}\n"),
                ("", f"nlos          {int(self.nlos_var.get())} "
                 f"E={int(self.nlos_east_var.get())} W={int(self.nlos_west_var.get())}\n"),
                ("", f"pdr_scale     {float(self.pdr_scale_var.get()):.2f}\n"),
                ("muted", f"\nzoom          {self.zoom:.2f} px/m\n"),
            ]
        else:
            sens = bool(info.get("sensor_visible", False))
            vis = bool(info.get("visible", False))
            occ = bool(info.get("occluded", False))
            v2v_fresh = (
                bool(self.env._fresh_bsm_available())
                if mode == "lossy_v2v"
                else mode == "perfect_v2v"
            )
            channel = detection_channel(mode, sens, vis, v2v_fresh)
            ego_v, ego_h = self._vehicle_speed_heading(EGO_ID)
            if EGO_ID not in traci.vehicle.getIDList():
                ego_v = float(self.env.ego_speed)
                ego_h = heading_label(0.0)
            threat_id = str(info.get("primary_threat") or "") or "(none)"
            braking = self.last_action in (0, 1)
            ego_t = (0.0 - self.env.ego_y) / max(self.env.ego_speed, 0.1)
            thr_t = self.env.target_x / max(self.env.target_speed, 0.1) if self.env.target_speed > 0 else 10.0
            gap = float(info.get("ttc", abs(ego_t - thr_t)))

            traffic_lines: list[tuple[str, str]] = [("title", "TRAFFIC\n")]
            if EGO_ID in traci.vehicle.getIDList():
                ex, ey = traci.vehicle.getPosition(EGO_ID)
            else:
                ex, ey = 0.0, float(self.env.ego_y)
            for vid in self._traffic_ids():
                speed, head = self._vehicle_speed_heading(vid)
                tx, ty = traci.vehicle.getPosition(vid)
                dist = math.hypot(float(tx) - float(ex), float(ty) - float(ey))
                is_primary = bool(info.get("primary_threat")) and vid == info.get("primary_threat")
                if abs(float(tx)) >= abs(float(ty)):
                    eta = abs(float(tx)) / max(speed, 0.1)
                else:
                    eta = abs(float(ty)) / max(speed, 0.1)
                sens_on, v2v_state, occ = self._per_vehicle_link_state(
                    vid, float(ex), float(ey), float(tx), float(ty), is_primary
                )
                role = "PRIMARY" if is_primary else "OTHER"
                tag = "warn" if is_primary else ""
                traffic_lines.append((tag, f"--- {vid} [{role}] ---\n"))
                traffic_lines.append(("", f"speed         {speed:5.2f} m/s\n"))
                traffic_lines.append(("", f"heading       {head}\n"))
                traffic_lines.append(("", f"distance      {dist:6.2f} m\n"))
                traffic_lines.append(("", f"eta_junction  {eta:6.2f} s\n"))
                traffic_lines.append(("", f"sensor        {sens_on}\n"))
                traffic_lines.append(("", f"v2v           {v2v_state}\n"))
                traffic_lines.append(("warn" if occ else "", f"occluded      {occ}\n"))
            if len(traffic_lines) == 1:
                traffic_lines.append(("muted", "(no other vehicles)\n"))
            traffic_lines.append(("", "\n"))

            lines = [
                ("title", "DEMO\n"),
                ("", f"model         {model_name}\n"),
                ("", f"mode          {mode}\n"),
                ("", f"multi_npc     {int(self.active_multi)}\n"),
                ("", f"nlos          {int(self.nlos_var.get())} "
                 f"E={int(self.nlos_east_var.get())} W={int(self.nlos_west_var.get())}\n"),
                ("", f"pdr_scale     {float(self.pdr_scale_var.get()):.2f}\n"),
                ("", f"pdr           {float(info.get('pdr', 0.0)):.3f}\n"),
                ("", f"seed          {int(self.seed_var.get())}\n"),
                ("", f"step          {self.step_i}  t={self.step_i * 0.1:5.1f}s\n"),
                ("muted", f"paused        {self.paused}\n\n"),
                ("title", "EGO\n"),
                ("", f"speed         {ego_v:5.2f} m/s\n"),
                ("", f"heading       {ego_h}\n"),
                ("", f"position      y={self.env.ego_y:7.1f} m\n"),
                ("", f"action        {ACTION_NAME.get(self.last_action, self.last_action)}\n"),
                ("warn" if braking else "", f"braking       {braking}\n\n"),
                *traffic_lines,
                ("title", "POLICY FOCUS\n"),
                ("", f"primary       {threat_id}\n"),
                ("", f"approach      {self.env.target_x:7.1f} m\n"),
                ("ok" if vis else "bad", f"policy_visible {vis}\n"),
                ("", f"channel       {channel}\n"),
                (
                    "ok" if info.get("v2v_rx_this_step") else "",
                    f"v2v_link      {'RX' if info.get('v2v_rx_this_step') else ('HOLD' if v2v_fresh else 'DROP')}\n",
                ),
                ("", f"aoi           {float(info.get('aoi', 0.0)):4.2f} s\n"),
                ("", f"v2v_rx        {int(info.get('v2v_rx_count', 0))}\n\n"),
                ("title", "CONFLICT\n"),
                ("", "point         (0.0, 0.0) m  junction\n"),
                ("", f"ego_eta       {ego_t:6.2f} s\n"),
                ("", f"threat_eta    {thr_t:6.2f} s\n"),
                ("", f"ttc_gap       {gap:6.2f} s\n"),
                ("", f"distance      {float(info.get('min_distance', 0.0)):6.2f} m\n\n"),
                ("title", "OUTCOME\n"),
                ("bad" if info.get("collision") else "", f"collision     {bool(info.get('collision'))}\n"),
                ("ok" if info.get("arrived") else "", f"arrived       {bool(info.get('arrived'))}\n"),
                ("warn" if info.get("near_miss") else "", f"near_miss     {bool(info.get('near_miss'))}\n"),
                ("muted", f"\nzoom          {self.zoom:.2f} px/m\n"),
            ]

        self.hud.configure(state=tk.NORMAL)
        self.hud.delete("1.0", tk.END)
        for tag, text in lines:
            if tag:
                self.hud.insert(tk.END, text, tag)
            else:
                self.hud.insert(tk.END, text)
        self.hud.configure(state=tk.DISABLED)

    def _tick(self) -> None:
        if self.closing:
            return
        delay_ms = max(10, int(float(self.delay_var.get()) * 1000))

        if self.env is not None and not self.paused and not self.episode_done:
            if self.model is None:
                action = self.env.rule_based_action()
            else:
                action = int(self.model.predict(self.obs, deterministic=True)[0])
            self.last_action = int(action)
            self.obs, _reward, terminated, truncated, info = self.env.step(self.last_action)
            self.last_info = info
            self.step_i += 1
            if terminated or truncated:
                self.paused = True
                self.episode_done = True
                outcome = "collision" if info.get("collision") else (
                    "arrived" if info.get("arrived") else "timeout"
                )
                self.status_var.set(f"Episode done: {outcome}. Restart to replay.")

        if self.env is not None:
            self._draw_frame()
        self._update_hud()
        self.root.after(delay_ms, self._tick)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    DemoApp().run()


if __name__ == "__main__":
    main()
