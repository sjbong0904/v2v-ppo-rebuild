from __future__ import annotations

import math
import random
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
import sumolib
import traci
from stable_baselines3 import PPO
from tkinterdnd2 import DND_FILES, TkinterDnD

# Sentinel for dragging "(rule-based)" from the MODELS list.
RULE_DRAG_TOKEN = "__RULE_BASED__"

from sumo_intersection_env import (
    EGO_ID,
    EGO_START_OFFSET,
    NEAR_MISS_RADIUS,
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
LEFT_W = 300
PANEL_W = 340
RULE_LABEL = "(rule-based)"
BRAKE_LIGHT = "#ff2a2a"
BRAKE_GLOW = "#ff5555"
SENSOR_FX = "#2ecc71"
V2V_FX = "#3498db"
V2V_HOLD_FX = "#1a5276"
BOTH_FX = "#1abc9c"
SECONDARY_FX = "#9b59b6"
PRIMARY_TAG = "#f39c12"
BANNER_HOLD_S = 3.0
COMPARE_PAUSE_S = 1.2
AUTO_CONTINUE_S = 2.0  # dwell on result before auto restart / next scenario
# Demo NEAR MISS = late panic hard-brake only (not controlled V2V yield).
# action 0 = hard brake (-8 m/s²); soft brake (action 1) does not count.
NEAR_MISS_HARD_BRAKE_TTC_S = 2.0
# Prefer seeds whose sampled arrival times nearly collide (demo drama).
CONFLICT_GAP_ACCEPT_S = 1.0  # |target_eta - ego_eta| below this → accept
CONFLICT_SEED_TRIES = 48
HERE = Path(__file__).resolve().parent
MODELS_DIR = HERE / "models"

# Demo playlist: NLOS geometry + seed (+ optional sparse/full PDR).
DEMO_SCENARIOS: list[dict] = [
    {"name": "NLOS both", "nlos": True, "east": True, "west": True, "seed": 7},
    {"name": "NLOS east", "nlos": True, "east": True, "west": False, "seed": 7},
    {"name": "LOS clear", "nlos": False, "east": False, "west": False, "seed": 7},
    {
        "name": "sparse PDR",
        "nlos": True,
        "east": True,
        "west": True,
        "seed": 21,
        "pdr": 0.30,
    },
    {
        "name": "full PDR",
        "nlos": True,
        "east": True,
        "west": True,
        "seed": 21,
        "pdr": 1.00,
    },
]


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

        # Episode perception tracking / banners / compare
        self._reset_episode_metrics()
        self.banner_text = ""
        self.banner_color = PANEL_ACCENT
        self.banner_until = 0.0
        self.split_compare = False
        self.compare_results: dict[str, dict] = {}
        self.view_mode_var = tk.StringVar(value="Single")
        self.phase_label_var = tk.StringVar(value="")
        self.pane_a_label_var = tk.StringVar(value="Left: (drag from MODELS)")
        self.pane_b_label_var = tk.StringVar(value="Right: (drag from MODELS)")
        # Right pane (split compare)
        self.env_b: SumoIntersectionEnv | None = None
        self.model_b: PPO | None = None
        self.selected_model_b: Path | None = None
        self.obs_b = None
        self.last_info_b: dict = {}
        self.last_action_b = 2
        self.episode_done_b = False
        self.active_mode_b = "lossy_v2v"
        self.active_multi_b = False
        self.metrics_b: dict = {}
        self.banner_text_b = ""
        self.banner_color_b = PANEL_ACCENT
        self.banner_until_b = 0.0
        # Internal drag: MODELS list → compare panes
        self._drag_armed = False
        self._dragging = False
        self._drag_start_xy: tuple[int, int] | None = None
        self._drag_payload: str | None = None  # abs path or RULE_DRAG_TOKEN

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
        self.loop_var = tk.BooleanVar(value=False)
        self.auto_scenario_var = tk.BooleanVar(value=False)
        self.random_seed_var = tk.BooleanVar(value=False)
        self.prefer_conflict_var = tk.BooleanVar(value=True)
        self.scenario_idx = 0
        self.scenario_label_var = tk.StringVar(value="scenario: —")
        self._auto_after_id: object | None = None
        self.DEFAULT_SEED = 7
        self._last_conflict_gap_s: float | None = None
        self.profile_var = tk.StringVar(value="mode=sensor_only  multi=0")
        self.status_var = tk.StringVar(value="Add models, select one, then Start")
        self.hud_summary_var = tk.StringVar(value="Ready")
        self.sensor_range_m = 35.0

        self._build_ui()
        self._autoload_models_dir()
        self._refresh_model_list()
        self._sync_profile_from_selection()
        self._bind_shortcuts()
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
            text="VIEW MODE",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(10, 4))
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(
            "Demo.TCombobox",
            fieldbackground="#0d1014",
            background="#2a3038",
            foreground=PANEL_FG,
            arrowcolor=PANEL_FG,
        )
        self.view_mode_box = ttk.Combobox(
            left,
            textvariable=self.view_mode_var,
            values=("Single", "Compare"),
            state="readonly",
            font=("Consolas", 11),
            style="Demo.TCombobox",
        )
        self.view_mode_box.pack(fill=tk.X, padx=10, pady=(0, 4))
        self.view_mode_box.bind("<<ComboboxSelected>>", self._on_view_mode_change)
        tk.Label(
            left,
            text="Single = one view  ·  Compare = 50/50 split\nCompare: drag MODELS or use Add on each pane",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 8))

        tk.Label(
            left,
            text="MODELS",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(4, 4))

        tk.Label(
            left,
            text=f"Add / drop .zip into list (max {MAX_MODELS})\nCompare: drag onto panes, or pane Add buttons",
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
        self.model_list.bind("<ButtonPress-1>", self._on_models_press, add="+")
        self.model_list.bind("<B1-Motion>", self._on_models_drag_motion, add="+")
        self.root.bind_all("<ButtonRelease-1>", self._on_global_drag_release, add="+")

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
        ).pack(fill=tk.X, padx=10, pady=(4, 4))
        tk.Label(
            left,
            textvariable=self.profile_var,
            bg=PANEL_BG,
            fg=PANEL_FG,
            font=("Consolas", 10),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 2))
        tk.Label(
            left,
            textvariable=self.phase_label_var,
            bg=PANEL_BG,
            fg=PRIMARY_TAG,
            font=("Consolas", 9, "bold"),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 2))
        tk.Label(
            left,
            textvariable=self.pane_a_label_var,
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10)
        tk.Label(
            left,
            textvariable=self.pane_b_label_var,
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 6))

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

        auto = tk.Frame(left, bg=PANEL_BG)
        auto.pack(fill=tk.X, padx=10, pady=(0, 4))
        tk.Label(
            auto,
            text="AUTO PLAY",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, pady=(4, 4))
        tk.Checkbutton(
            auto,
            text="Loop episode (same settings)",
            variable=self.loop_var,
            command=self._on_auto_play_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Checkbutton(
            auto,
            text="Cycle scenarios (playlist)",
            variable=self.auto_scenario_var,
            command=self._on_auto_play_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Checkbutton(
            auto,
            text="Random seed each episode",
            variable=self.random_seed_var,
            command=self._on_random_seed_toggle,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Checkbutton(
            auto,
            text="Prefer conflict seeds (tight arrival)",
            variable=self.prefer_conflict_var,
            bg=PANEL_BG,
            fg=PANEL_FG,
            selectcolor="#1a1d23",
            activebackground=PANEL_BG,
            font=("Consolas", 10),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Label(
            auto,
            textvariable=self.scenario_label_var,
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 9),
            anchor="w",
        ).pack(fill=tk.X, pady=(2, 0))
        tk.Label(
            auto,
            text="After each episode waits ~2s, then restarts.\n"
            "Scenario cycle wraps: NLOS / LOS / PDR.\n"
            "Random OFF → seed 7. Conflict ON → pick near-miss timing.",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            justify=tk.LEFT,
            anchor="w",
        ).pack(fill=tk.X, pady=(0, 4))

        ctrl = tk.Frame(left, bg=PANEL_BG)
        ctrl.pack(fill=tk.X, padx=10, pady=(4, 10))
        self._btn(ctrl, "Start", self._start).pack(fill=tk.X, pady=2)
        self._btn(ctrl, "Pause / Resume", self._toggle_pause).pack(fill=tk.X, pady=2)
        self._btn(ctrl, "Restart", self._restart).pack(fill=tk.X, pady=2)
        self._btn(ctrl, "Next scenario", self._manual_next_scenario).pack(fill=tk.X, pady=2)

        # Main body: views on top; compare metrics strip at bottom
        self.body = tk.Frame(outer, bg=BG)
        self.body.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(PAD, 0), pady=PAD)

        self.upper = tk.Frame(self.body, bg=BG)
        self.upper.pack(fill=tk.BOTH, expand=True)

        self.center = tk.Frame(self.upper, bg=BG)
        self.center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.split_host = tk.Frame(self.center, bg=BG)
        self.pane_left = tk.Frame(self.split_host, bg=BG)
        self.pane_right = tk.Frame(self.split_host, bg=BG)
        self.pane_bar_a = self._build_pane_assign_bar(self.pane_left, "LEFT", "a")
        self.pane_bar_b = self._build_pane_assign_bar(self.pane_right, "RIGHT", "b")
        self.canvas = tk.Canvas(
            self.pane_left, width=VIEW_W, height=VIEW_H, bg=BG, highlightthickness=0
        )
        self.canvas_b = tk.Canvas(
            self.pane_right, width=VIEW_W // 2, height=VIEW_H, bg=BG, highlightthickness=0
        )
        for c in (self.canvas, self.canvas_b):
            c.bind("<MouseWheel>", self._on_wheel)
            c.bind("<Button-4>", lambda e: self._zoom_by(ZOOM_STEP))
            c.bind("<Button-5>", lambda e: self._zoom_by(1.0 / ZOOM_STEP))
            c.bind("<Configure>", self._on_canvas_resize)
        self.canvas.drop_target_register(DND_FILES)
        self.canvas.dnd_bind("<<Drop>>", lambda e: self._on_pane_drop(e, "a"))
        self.canvas_b.drop_target_register(DND_FILES)
        self.canvas_b.dnd_bind("<<Drop>>", lambda e: self._on_pane_drop(e, "b"))
        self.canvas.bind("<ButtonRelease-1>", lambda e: self._on_pane_click_drop(e, "a"), add="+")
        self.canvas_b.bind("<ButtonRelease-1>", lambda e: self._on_pane_click_drop(e, "b"), add="+")

        self.right = tk.Frame(self.upper, width=PANEL_W, bg=PANEL_BG)
        self.right.pack(side=tk.RIGHT, fill=tk.Y, padx=(PAD, 0))
        self.right.pack_propagate(False)

        # Single mode: one live summary panel
        self.single_summary = tk.Frame(self.right, bg=PANEL_BG)
        tk.Label(
            self.single_summary,
            text="LIVE SUMMARY",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(10, 4))
        self.hud = self._make_scroll_text(
            self.single_summary,
            bg=PANEL_BG,
            fg=PANEL_FG,
            font=("Consolas", 10),
            padx=10,
            pady=8,
        )
        self.hud.master.pack(fill=tk.BOTH, expand=True, padx=(0, 4), pady=(0, 8))
        self._style_metrics_text(self.hud)

        # Compare mode: LEFT / RIGHT stacked on the side (scrollable → KEY visible)
        self.compare_summary = tk.Frame(self.right, bg=PANEL_BG)
        tk.Label(
            self.compare_summary,
            text="COMPARE METRICS",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(10, 2))
        tk.Label(
            self.compare_summary,
            text="Scroll each pane to see KEY metrics",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            anchor="w",
        ).pack(fill=tk.X, padx=10, pady=(0, 4))

        left_m = tk.Frame(self.compare_summary, bg=PANEL_BG)
        right_m = tk.Frame(self.compare_summary, bg=PANEL_BG)
        left_m.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 4))
        right_m.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        tk.Label(
            left_m,
            text="LEFT",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 11, "bold"),
            anchor="w",
        ).pack(fill=tk.X)
        tk.Label(
            right_m,
            text="RIGHT",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 11, "bold"),
            anchor="w",
        ).pack(fill=tk.X)
        self.metrics_left = self._make_scroll_text(
            left_m,
            bg="#0d1014",
            fg=PANEL_FG,
            font=("Consolas", 9),
            padx=8,
            pady=6,
        )
        self.metrics_right = self._make_scroll_text(
            right_m,
            bg="#0d1014",
            fg=PANEL_FG,
            font=("Consolas", 9),
            padx=8,
            pady=6,
        )
        self.metrics_left.master.pack(fill=tk.BOTH, expand=True)
        self.metrics_right.master.pack(fill=tk.BOTH, expand=True)
        self._style_metrics_text(self.metrics_left)
        self._style_metrics_text(self.metrics_right)

        self._layout_single_view()
        self._refresh_pane_labels()
        self._apply_summary_layout()
        self._draw_placeholder()
        self._update_hud()

    def _make_scroll_text(self, parent: tk.Misc, **text_kwargs) -> tk.Text:
        """Text widget with a vertical scrollbar (KEY metrics stay reachable)."""
        wrap = tk.Frame(parent, bg=PANEL_BG)
        sb = tk.Scrollbar(wrap, orient=tk.VERTICAL)
        text = tk.Text(
            wrap,
            relief=tk.FLAT,
            wrap=tk.WORD,
            state=tk.DISABLED,
            yscrollcommand=sb.set,
            highlightthickness=0,
            **text_kwargs,
        )
        sb.config(command=text.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        def _on_wheel(event, widget=text):
            # Windows / macOS
            if getattr(event, "delta", 0):
                widget.yview_scroll(int(-event.delta / 120), "units")
            return "break"

        def _on_linux_up(_event, widget=text):
            widget.yview_scroll(-1, "units")
            return "break"

        def _on_linux_down(_event, widget=text):
            widget.yview_scroll(1, "units")
            return "break"

        for w in (text, wrap):
            w.bind("<MouseWheel>", _on_wheel)
            w.bind("<Button-4>", _on_linux_up)
            w.bind("<Button-5>", _on_linux_down)
        return text

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

    def _style_metrics_text(self, widget: tk.Text) -> None:
        widget.tag_configure("title", foreground=PANEL_ACCENT, font=("Consolas", 10, "bold"))
        widget.tag_configure("muted", foreground=PANEL_MUTED)
        widget.tag_configure("warn", foreground="#e67e22")
        widget.tag_configure("ok", foreground="#2ecc71")
        widget.tag_configure("bad", foreground="#e74c3c")

    def _apply_summary_layout(self) -> None:
        """Single: one live panel. Compare: LEFT/RIGHT metrics on the right (scrollable)."""
        if not self.right.winfo_ismapped():
            self.right.pack(side=tk.RIGHT, fill=tk.Y, padx=(PAD, 0))
        if self.split_compare:
            self.single_summary.pack_forget()
            if not self.compare_summary.winfo_ismapped():
                self.compare_summary.pack(fill=tk.BOTH, expand=True)
        else:
            self.compare_summary.pack_forget()
            if not self.single_summary.winfo_ismapped():
                self.single_summary.pack(fill=tk.BOTH, expand=True)

    def _bind_shortcuts(self) -> None:
        self.root.bind("<space>", lambda _e: self._toggle_pause())
        self.root.bind("<Key-r>", lambda _e: self._restart(autoplay=True))
        self.root.bind("<Key-R>", lambda _e: self._restart(autoplay=True))
        self.root.bind("<Key-s>", lambda _e: self._start())
        self.root.bind("<Key-S>", lambda _e: self._start())
        self.root.bind("<Left>", lambda _e: self._nudge_seed(-1))
        self.root.bind("<Right>", lambda _e: self._nudge_seed(1))

    def _build_pane_assign_bar(self, parent: tk.Misc, title: str, pane: str) -> tk.Frame:
        """Top bar on each Compare pane: label + Add (picker) button."""
        bar = tk.Frame(parent, bg=PANEL_BG, height=34)
        bar.pack_propagate(False)
        tk.Label(
            bar,
            text=title,
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 11, "bold"),
            anchor="w",
        ).pack(side=tk.LEFT, padx=(10, 6), pady=4)
        self._btn(bar, "Add", lambda p=pane: self._pick_model_for_pane(p)).pack(
            side=tk.RIGHT, padx=(4, 8), pady=4
        )
        hint = tk.Label(
            bar,
            text="drag from MODELS or Add →",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 8),
            anchor="w",
        )
        hint.pack(side=tk.LEFT, padx=(0, 4))
        return bar

    def _layout_single_view(self) -> None:
        # Keep panes under split_host (their real parent). Never pack into center.
        self.pane_left.grid_forget()
        self.pane_right.grid_forget()
        self.pane_right.pack_forget()
        self.canvas_b.pack_forget()
        self.pane_bar_a.pack_forget()
        self.pane_bar_b.pack_forget()
        self.pane_left.pack_forget()
        self.canvas.pack_forget()
        self.split_host.pack(in_=self.center, fill=tk.BOTH, expand=True)
        self.pane_left.pack(in_=self.split_host, fill=tk.BOTH, expand=True)
        self.canvas.pack(in_=self.pane_left, fill=tk.BOTH, expand=True)

    def _layout_split_view(self) -> None:
        self.pane_left.pack_forget()
        self.pane_right.pack_forget()
        self.canvas.pack_forget()
        self.canvas_b.pack_forget()
        self.pane_bar_a.pack_forget()
        self.pane_bar_b.pack_forget()
        self.pane_left.grid_forget()
        self.pane_right.grid_forget()
        self.split_host.pack(in_=self.center, fill=tk.BOTH, expand=True)
        # Equal 50/50 columns
        self.split_host.columnconfigure(0, weight=1, uniform="compare_panes")
        self.split_host.columnconfigure(1, weight=1, uniform="compare_panes")
        self.split_host.rowconfigure(0, weight=1)
        self.pane_left.grid(row=0, column=0, sticky="nsew", padx=(0, 2))
        self.pane_right.grid(row=0, column=1, sticky="nsew", padx=(2, 0))
        self.pane_bar_a.pack(in_=self.pane_left, fill=tk.X, side=tk.TOP)
        self.pane_bar_b.pack(in_=self.pane_right, fill=tk.X, side=tk.TOP)
        self.canvas.pack(in_=self.pane_left, fill=tk.BOTH, expand=True)
        self.canvas_b.pack(in_=self.pane_right, fill=tk.BOTH, expand=True)

    def _refresh_pane_labels(self) -> None:
        def fmt(side: str, path: Path | None) -> str:
            if path is None:
                return f"{side}: (Add / drag from MODELS)"
            mode, multi = infer_model_profile(path)
            tag = mode.replace("_v2v", "").replace("_only", "")
            if multi:
                tag = f"multi/{tag}"
            return f"{side}: {path.name}  [{tag}]"

        self.pane_a_label_var.set(fmt("Left", self.selected_model))
        self.pane_b_label_var.set(fmt("Right", self.selected_model_b))
        if self.split_compare:
            self.phase_label_var.set("Compare · Add or drag onto panes, then Start")
        else:
            self.phase_label_var.set("Single mode")

    def _pick_model_for_pane(self, pane: str) -> None:
        """Dialog: assign a MODELS entry (or rule-based) to left/right Compare pane."""
        if not self.split_compare:
            self.view_mode_var.set("Compare")
            self._on_view_mode_change()

        side = "Right" if pane == "b" else "Left"
        win = tk.Toplevel(self.root)
        win.title(f"Add model — {side}")
        win.configure(bg=PANEL_BG)
        win.transient(self.root)
        win.grab_set()
        win.geometry("420x360")

        tk.Label(
            win,
            text=f"Choose a model for {side} pane",
            bg=PANEL_BG,
            fg=PANEL_ACCENT,
            font=("Consolas", 12, "bold"),
            anchor="w",
        ).pack(fill=tk.X, padx=12, pady=(12, 6))
        tk.Label(
            win,
            text="Items come from the MODELS list (Add zips there first).",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 9),
            anchor="w",
        ).pack(fill=tk.X, padx=12, pady=(0, 8))

        lb = tk.Listbox(
            win,
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
        lb.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 8))
        lb.insert(tk.END, RULE_LABEL)
        for path in self.model_paths:
            mode, multi = infer_model_profile(path)
            tag = mode.replace("_v2v", "").replace("_only", "")
            if multi:
                tag = f"multi/{tag}"
            lb.insert(tk.END, f"{path.name}  [{tag}]")
        lb.selection_set(0)

        current = self.selected_model_b if pane == "b" else self.selected_model
        if current is not None:
            for i, path in enumerate(self.model_paths):
                if path.resolve() == current.resolve():
                    lb.selection_clear(0, tk.END)
                    lb.selection_set(i + 1)
                    lb.see(i + 1)
                    break

        def _apply() -> None:
            idxs = lb.curselection()
            if not idxs:
                return
            idx = int(idxs[0])
            if idx == 0:
                self._assign_rule_to_pane(pane)
            else:
                self._assign_model_to_pane(self.model_paths[idx - 1], pane)
            win.destroy()

        btn_row = tk.Frame(win, bg=PANEL_BG)
        btn_row.pack(fill=tk.X, padx=12, pady=(0, 12))
        self._btn(btn_row, "Cancel", win.destroy).pack(side=tk.RIGHT, padx=(6, 0))
        self._btn(btn_row, "Assign", _apply).pack(side=tk.RIGHT)
        lb.bind("<Double-Button-1>", lambda _e: _apply())
        win.bind("<Return>", lambda _e: _apply())
        win.bind("<Escape>", lambda _e: win.destroy())
        lb.focus_set()

    def _on_view_mode_change(self, _event=None) -> None:
        mode = self.view_mode_var.get()
        want_split = mode == "Compare"
        if want_split == self.split_compare:
            return
        self._close_env()
        self._close_env_b()
        self.episode_done = False
        self.episode_done_b = False
        self.paused = True
        self.compare_results = {}
        self.split_compare = want_split
        if want_split:
            self._layout_split_view()
            self.status_var.set("Compare: use Add on each pane, or drag from MODELS")
        else:
            self._layout_single_view()
            self.status_var.set("Single: select a model, then Start")
        self._apply_summary_layout()
        self._refresh_pane_labels()
        self._update_hud()
        self._draw_all()

    def _on_models_press(self, event) -> None:
        idx = self.model_list.nearest(event.y)
        if idx >= 0:
            self.model_list.selection_clear(0, tk.END)
            self.model_list.selection_set(idx)
            self.model_list.activate(idx)
        self._drag_armed = True
        self._dragging = False
        self._drag_start_xy = (event.x_root, event.y_root)
        self._drag_payload = None

    def _payload_from_list_selection(self) -> str | None:
        idxs = self.model_list.curselection()
        if not idxs:
            return None
        idx = int(idxs[0])
        if idx == 0:
            return RULE_DRAG_TOKEN
        if idx - 1 >= len(self.model_paths):
            return None
        return str(self.model_paths[idx - 1].resolve())

    def _on_models_drag_motion(self, event) -> None:
        if not self._drag_armed or self._drag_start_xy is None:
            return
        dx = abs(event.x_root - self._drag_start_xy[0])
        dy = abs(event.y_root - self._drag_start_xy[1])
        if dx + dy < 10:
            return
        if not self._dragging:
            payload = self._payload_from_list_selection()
            if payload is None:
                return
            self._dragging = True
            self._drag_payload = payload
            try:
                self.root.configure(cursor="hand2")
            except tk.TclError:
                pass
            name = (
                RULE_LABEL
                if payload == RULE_DRAG_TOKEN
                else Path(payload).name
            )
            self.status_var.set(f"Dragging: {name} → drop on left/right pane")

    def _clear_list_drag(self) -> None:
        self._drag_armed = False
        self._dragging = False
        self._drag_start_xy = None
        self._drag_payload = None
        try:
            self.root.configure(cursor="")
        except tk.TclError:
            pass

    def _pane_under_pointer(self, x_root: int, y_root: int) -> str | None:
        try:
            w = self.root.winfo_containing(x_root, y_root)
        except tk.TclError:
            return None
        while w is not None:
            if w == self.canvas:
                return "a"
            if w == self.canvas_b:
                return "b"
            w = w.master if hasattr(w, "master") else None
        return None

    def _apply_drag_payload_to_pane(self, payload: str, pane: str) -> None:
        if not self.split_compare and pane == "b":
            self.view_mode_var.set("Compare")
            self._on_view_mode_change()
        if payload == RULE_DRAG_TOKEN:
            self._assign_rule_to_pane(pane)
            return
        path = Path(payload)
        # Must already be in MODELS list (Add first).
        resolved = {p.resolve() for p in self.model_paths}
        if path.resolve() not in resolved:
            messagebox.showinfo(
                "MODELS",
                "Add the model to MODELS first,\nthen drag it from the list onto a pane.",
            )
            return
        self._assign_model_to_pane(path, pane)

    def _on_global_drag_release(self, event) -> None:
        if not self._dragging or self._drag_payload is None:
            self._clear_list_drag()
            return
        payload = self._drag_payload
        pane = self._pane_under_pointer(event.x_root, event.y_root)
        self._clear_list_drag()
        if pane is None:
            self.status_var.set("Drop on a pane (MODELS → left/right view)")
            return
        if not self.split_compare:
            self.view_mode_var.set("Compare")
            self._on_view_mode_change()
        self._apply_drag_payload_to_pane(payload, pane)

    def _on_pane_click_drop(self, _event, pane: str) -> None:
        # Backup if release lands on canvas binding first.
        if self._dragging and self._drag_payload is not None:
            payload = self._drag_payload
            self._clear_list_drag()
            if not self.split_compare:
                self.view_mode_var.set("Compare")
                self._on_view_mode_change()
            self._apply_drag_payload_to_pane(payload, pane)

    def _assign_rule_to_pane(self, pane: str) -> None:
        if pane == "b":
            self.selected_model_b = None
            self.model_b = None
            self.active_mode_b, self.active_multi_b = "sensor_only", False
            self._close_env_b()
            self.episode_done_b = False
        else:
            self.selected_model = None
            self.model = None
            self.active_mode, self.active_multi = "sensor_only", False
            self._close_env()
            self.episode_done = False
        self._refresh_pane_labels()
        self.status_var.set(f"{'Right' if pane == 'b' else 'Left'} ← rule-based")
        self._draw_all()

    def _assign_model_to_pane(self, path: Path, pane: str) -> None:
        path = path.resolve()
        if path.suffix.lower() != ".zip" or not path.is_file():
            messagebox.showwarning("Model", f"Not a model zip:\n{path}")
            return
        if path not in self.model_paths and path.resolve() not in {p.resolve() for p in self.model_paths}:
            messagebox.showinfo(
                "MODELS",
                "Model is not in MODELS. Add it first, then drag.",
            )
            return
        if pane == "b":
            self.selected_model_b = path
            self.active_mode_b, self.active_multi_b = infer_model_profile(path)
            self.model_b = None
            self._close_env_b()
            self.episode_done_b = False
        else:
            self.selected_model = path
            self.active_mode, self.active_multi = infer_model_profile(path)
            self.model = None
            self._close_env()
            self.episode_done = False
            if not self.split_compare:
                self._refresh_model_list()
                self._sync_profile_from_selection()
        self._refresh_pane_labels()
        self.profile_var.set(
            f"L={infer_model_profile(self.selected_model)[0]}\n"
            f"R={infer_model_profile(self.selected_model_b)[0]}"
            if self.split_compare
            else f"mode={self.active_mode}\nmulti_npc={int(self.active_multi)}"
        )
        self.status_var.set(
            f"{'Right' if pane == 'b' else 'Left'} ← {path.name}  (Start to run)"
        )
        self._draw_all()

    def _on_pane_drop(self, event, pane: str) -> None:
        """OS file drop on pane: only if already in MODELS (Add first)."""
        paths = self._parse_drop_paths(event.data)
        zips = [p for p in paths if p.suffix.lower() == ".zip"]
        if not zips:
            self.status_var.set("Drag from the MODELS list")
            return
        path = zips[0].resolve()
        known = {p.resolve() for p in self.model_paths}
        if path not in known:
            messagebox.showinfo(
                "MODELS",
                "Add the zip to MODELS first (Add / drop on list),\n"
                "then drag the list item onto a pane.",
            )
            return
        if not self.split_compare and pane == "b":
            self.view_mode_var.set("Compare")
            self._on_view_mode_change()
        self._assign_model_to_pane(path, pane)

    def _autoload_models_dir(self) -> None:
        search_dirs = [MODELS_DIR]
        if HERE.name != "demo_package":
            search_dirs.append(HERE / "demo_package" / "models")
            search_dirs.append(HERE / "runs")
        found: list[Path] = []
        for folder in search_dirs:
            if not folder.is_dir():
                continue
            found.extend(sorted(folder.glob("*.zip")))
        # Prefer distinct names; keep first occurrence (models/ before runs/).
        unique: list[Path] = []
        seen: set[str] = set()
        for path in found:
            key = path.name.lower()
            if key in seen:
                continue
            seen.add(key)
            unique.append(path)

        def priority(path: Path) -> tuple[int, str]:
            n = path.stem.lower()
            if "sumo_ppo_50k_sensor" in n and "multi" not in n:
                return (0, n)
            if "sumo_ppo_50k_lossy" in n and "multi" not in n:
                return (1, n)
            if "sumo_ppo_50k_multi_sensor" in n:
                return (2, n)
            if "sumo_ppo_50k_multi_lossy" in n:
                return (3, n)
            if "sensor" in n:
                return (10, n)
            if "lossy" in n:
                return (11, n)
            return (50, n)

        unique.sort(key=priority)
        if unique:
            self._ingest_paths(unique[:MAX_MODELS])
            self.status_var.set(f"Autoloaded {min(len(unique), MAX_MODELS)} model(s)")

    def _empty_metrics(self) -> dict:
        return {
            "first_sensor_step": None,
            "first_sensor_dist": None,
            "first_sensor_ttc": None,
            "first_v2v_step": None,
            "first_v2v_dist": None,
            "first_v2v_ttc": None,
            "v2v_early": False,
            "emergency_brake": False,
            "closest_dist": float("inf"),
            "speed_at_closest": 0.0,
            "hard_brake_ttc": None,
            "first_brake_ttc": None,
            "first_brake_action": None,
            "outcome": "",
        }

    def _reset_episode_metrics(self) -> None:
        m = self._empty_metrics()
        self.first_sensor_step = m["first_sensor_step"]
        self.first_sensor_dist = m["first_sensor_dist"]
        self.first_sensor_ttc = m["first_sensor_ttc"]
        self.first_v2v_step = m["first_v2v_step"]
        self.first_v2v_dist = m["first_v2v_dist"]
        self.first_v2v_ttc = m["first_v2v_ttc"]
        self.v2v_early = False
        self.emergency_brake = False
        self.closest_dist = float("inf")
        self.speed_at_closest = 0.0
        self.hard_brake_ttc = None
        self.first_brake_ttc = None
        self.first_brake_action = None
        self.episode_outcome = ""
        self.metrics_b = self._empty_metrics()

    def _nudge_seed(self, delta: int) -> None:
        self.seed_var.set(max(0, int(self.seed_var.get()) + delta))
        self.status_var.set(f"seed={int(self.seed_var.get())}")

    def _find_model(self, *, want_lossy: bool, want_multi: bool) -> Path | None:
        candidates: list[Path] = []
        for path in self.model_paths:
            mode, multi = infer_model_profile(path)
            if multi != want_multi:
                continue
            is_lossy = mode == "lossy_v2v"
            is_sensor = mode == "sensor_only"
            if want_lossy and is_lossy:
                candidates.append(path)
            elif not want_lossy and is_sensor:
                candidates.append(path)
        if not candidates:
            return None
        # Prefer filenames that match the common sumo_ppo_50k_* pattern.
        candidates.sort(key=lambda p: (0 if "50k" in p.stem.lower() else 1, p.name.lower()))
        return candidates[0]

    def _select_model_path(self, path: Path | None) -> None:
        self.selected_model = path
        self.model = None
        self._refresh_model_list()
        self._sync_profile_from_selection()

    def _set_banner(self, text: str, color: str, hold_s: float = BANNER_HOLD_S, *, pane: str = "a") -> None:
        if pane == "b":
            self.banner_text_b = text
            self.banner_color_b = color
            self.banner_until_b = time.monotonic() + hold_s
        else:
            self.banner_text = text
            self.banner_color = color
            self.banner_until = time.monotonic() + hold_s

    def _draw_banner_on(
        self,
        canvas: tk.Canvas,
        *,
        env: SumoIntersectionEnv | None,
        episode_done: bool,
        mode: str,
        v2v_early: bool,
        first_v2v_step: int | None,
        banner_text: str,
        banner_color: str,
        banner_until: float,
    ) -> None:
        show_live = False
        text, color = banner_text, banner_color
        if env is not None and not episode_done:
            if mode == "sensor_only":
                text, color = "SENSOR ONLY", SENSOR_FX
                show_live = True
            elif v2v_early:
                text, color = "V2V EARLY", V2V_FX
                show_live = True
            elif first_v2v_step is not None:
                text, color = "V2V ACTIVE", V2V_FX
                show_live = True
        timed = bool(banner_text) and time.monotonic() <= banner_until
        if timed:
            text, color = banner_text, banner_color
        if not (show_live or timed or (episode_done and banner_text)):
            return
        if not text:
            return
        vw, vh = self._view_size(canvas)
        is_result = bool(timed or episode_done)
        if is_result:
            # Bottom-left result badge (COLLISION / NEAR MISS / SAFE PASS / TIMEOUT)
            font_size = 16
            pad = 12
            box_h = 34
            box_w = max(140, 18 + len(text) * (font_size // 2 + 3))
            x0 = pad
            y1 = vh - pad
            y0 = y1 - box_h
            x1 = x0 + box_w
            canvas.create_rectangle(x0, y0, x1, y1, fill="#0a0c10", outline=color, width=2)
            canvas.create_text(
                x0 + 10,
                (y0 + y1) // 2,
                text=text,
                fill=color,
                font=("Consolas", font_size, "bold"),
                anchor="w",
            )
        else:
            # Live mid-episode cue stays near the top
            canvas.create_text(
                vw // 2,
                52,
                text=text,
                fill=color,
                font=("Consolas", 16, "bold"),
                justify=tk.CENTER,
            )

    def _track_metrics_into(
        self,
        store: dict,
        info: dict,
        *,
        env: SumoIntersectionEnv | None,
        mode: str,
        step_i: int,
        action: int | None = None,
    ) -> None:
        dist = float(info.get("distance", 0.0) or 0.0)
        ego_ttc = float(info.get("ego_ttc", 0.0) or 0.0)
        ego_speed = float(info.get("ego_speed", 0.0) or 0.0)
        sens = bool(info.get("sensor_visible", False))
        v2v_fresh = (
            bool(env._fresh_bsm_available())
            if env is not None and mode == "lossy_v2v"
            else mode == "perfect_v2v"
        )
        channel = detection_channel(mode, sens, bool(info.get("visible", False)), v2v_fresh)
        if sens and store["first_sensor_step"] is None:
            store["first_sensor_step"] = step_i
            store["first_sensor_dist"] = dist
            store["first_sensor_ttc"] = ego_ttc
        if "v2v" in channel and store["first_v2v_step"] is None:
            store["first_v2v_step"] = step_i
            store["first_v2v_dist"] = dist
            store["first_v2v_ttc"] = ego_ttc
        if store["first_v2v_step"] is not None and (
            store["first_sensor_step"] is None or store["first_v2v_step"] < store["first_sensor_step"]
        ):
            store["v2v_early"] = True

        # Closest approach (HUD only; not used for outcome)
        closest = float(store.get("closest_dist", float("inf")))
        if dist < closest:
            store["closest_dist"] = dist
            store["speed_at_closest"] = ego_speed

        # Late panic: hard brake while TTC is already short.
        # Soft brake / coast after early V2V must not count.
        if action == 0 and 0.0 < ego_ttc < NEAR_MISS_HARD_BRAKE_TTC_S:
            store["emergency_brake"] = True
            if store.get("hard_brake_ttc") is None:
                store["hard_brake_ttc"] = ego_ttc
        if action in (0, 1) and store.get("first_brake_ttc") is None:
            store["first_brake_ttc"] = ego_ttc
            store["first_brake_action"] = action

    def _track_episode_metrics(self, info: dict, action: int) -> None:
        store = {
            "first_sensor_step": self.first_sensor_step,
            "first_sensor_dist": self.first_sensor_dist,
            "first_sensor_ttc": self.first_sensor_ttc,
            "first_v2v_step": self.first_v2v_step,
            "first_v2v_dist": self.first_v2v_dist,
            "first_v2v_ttc": self.first_v2v_ttc,
            "v2v_early": self.v2v_early,
            "emergency_brake": self.emergency_brake,
            "closest_dist": self.closest_dist,
            "speed_at_closest": self.speed_at_closest,
            "hard_brake_ttc": self.hard_brake_ttc,
            "first_brake_ttc": self.first_brake_ttc,
            "first_brake_action": self.first_brake_action,
        }
        self._track_metrics_into(
            store,
            info,
            env=self.env,
            mode=self.active_mode,
            step_i=self.step_i,
            action=action,
        )
        self.first_sensor_step = store["first_sensor_step"]
        self.first_sensor_dist = store["first_sensor_dist"]
        self.first_sensor_ttc = store["first_sensor_ttc"]
        self.first_v2v_step = store["first_v2v_step"]
        self.first_v2v_dist = store["first_v2v_dist"]
        self.first_v2v_ttc = store["first_v2v_ttc"]
        self.v2v_early = bool(store["v2v_early"])
        self.emergency_brake = bool(store["emergency_brake"])
        self.closest_dist = float(store["closest_dist"])
        self.speed_at_closest = float(store["speed_at_closest"])
        self.hard_brake_ttc = store["hard_brake_ttc"]
        self.first_brake_ttc = store["first_brake_ttc"]
        self.first_brake_action = store["first_brake_action"]

    def _classify_outcome(self, info: dict, metrics: dict | None = None) -> str:
        """collision > near_miss > arrived > timeout.

        NEAR MISS = arrived + late panic hard-brake (action 0, short TTC)
        and NOT an early V2V yield. Controlled decelerate-and-yield → SAFE PASS.
        """
        if info.get("collision"):
            return "collision"
        if info.get("arrived"):
            m = metrics or {}
            # Early V2V awareness + yield is the intended "safe" demo outcome.
            if m.get("v2v_early"):
                return "arrived"
            if m.get("emergency_brake"):
                return "near_miss"
            return "arrived"
        if metrics and metrics.get("outcome"):
            return str(metrics["outcome"])
        return "timeout"

    def _outcome_banner(self, outcome: str) -> tuple[str, str]:
        """Return (banner text, color) for end-of-episode badge."""
        if outcome == "collision":
            return "COLLISION", "#e74c3c"
        if outcome == "near_miss":
            return "NEAR MISS", "#e67e22"
        if outcome == "arrived":
            return "SAFE PASS", "#2ecc71"
        return "TIMEOUT", "#e67e22"

    def _snapshot_from(
        self,
        *,
        info: dict,
        mode: str,
        multi: bool,
        model_path: Path | None,
        metrics: dict,
        steps: int,
    ) -> dict:
        outcome = metrics.get("outcome") or self._classify_outcome(info, metrics)
        return {
            "mode": mode,
            "multi": multi,
            "model": RULE_LABEL if model_path is None else model_path.name,
            "seed": int(self.seed_var.get()),
            "outcome": outcome,
            "collision": bool(info.get("collision")),
            "arrived": bool(info.get("arrived")),
            "near_miss": outcome == "near_miss",
            "geo_close": bool(info.get("near_miss")),
            "emergency_brake": bool(metrics.get("emergency_brake")),
            "min_distance": float(info.get("min_distance", 0.0) or 0.0),
            "first_sensor_dist": metrics.get("first_sensor_dist"),
            "first_sensor_ttc": metrics.get("first_sensor_ttc"),
            "first_v2v_dist": metrics.get("first_v2v_dist"),
            "first_v2v_ttc": metrics.get("first_v2v_ttc"),
            "v2v_early": bool(metrics.get("v2v_early")),
            "steps": steps,
        }

    def _snapshot_episode(self) -> dict:
        return self._snapshot_from(
            info=self.last_info,
            mode=self.active_mode,
            multi=self.active_multi,
            model_path=self.selected_model,
            metrics=self._left_metrics_dict(),
            steps=self.step_i,
        )

    def _close_env_b(self) -> None:
        if self.env_b is not None:
            try:
                self.env_b.close()
            except Exception:
                pass
            self.env_b = None


    def _fmt_opt(self, value: float | None, fmt: str = ".1f") -> str:
        if value is None:
            return "—"
        return format(value, fmt)

    def _v2v_lead(
        self,
        *,
        first_sensor_step: int | None,
        first_v2v_step: int | None,
        first_sensor_dist: float | None,
        first_v2v_dist: float | None,
        v2v_early: bool,
    ) -> tuple[float | None, float | None]:
        if not v2v_early or first_v2v_step is None or first_sensor_step is None:
            return None, None
        lead_s = (first_sensor_step - first_v2v_step) * 0.1
        lead_m = None
        if first_v2v_dist is not None and first_sensor_dist is not None:
            lead_m = float(first_v2v_dist) - float(first_sensor_dist)
        return lead_s, lead_m

    def _key_metric_lines(
        self,
        *,
        model_path: Path | None,
        mode: str,
        info: dict,
        env: SumoIntersectionEnv | None,
        episode_done: bool,
        step_i: int,
        last_action: int,
        metrics: dict,
        idle_hint: str = "Assign a model, then Start",
    ) -> list[tuple[str, str]]:
        """Compact live + key metrics (aligned with presentation final metrics)."""
        model_name = RULE_LABEL if model_path is None else model_path.name
        if env is None:
            return [
                ("title", "LIVE\n"),
                ("muted", f"{idle_hint}\n"),
                ("", f"model   {model_name}\n"),
                ("", f"mode    {mode}\n"),
            ]

        sens = bool(info.get("sensor_visible", False))
        vis = bool(info.get("visible", False))
        v2v_fresh = (
            bool(env._fresh_bsm_available())
            if mode == "lossy_v2v"
            else mode == "perfect_v2v"
        )
        channel = detection_channel(mode, sens, vis, v2v_fresh)
        dist = float(info.get("distance", 0.0) or 0.0)
        ego_ttc = float(info.get("ego_ttc", 0.0) or 0.0)
        min_d = float(info.get("min_distance", 0.0) or 0.0)
        collision = bool(info.get("collision"))
        arrived = bool(info.get("arrived"))
        geo_close = bool(info.get("near_miss"))
        emergency_brake = bool(metrics.get("emergency_brake"))
        v2v_early = bool(metrics.get("v2v_early"))
        if episode_done:
            classified = metrics.get("outcome") or self._classify_outcome(info, metrics)
            if classified == "collision":
                outcome, otag = "collision", "bad"
            elif classified == "near_miss":
                outcome, otag = "near_miss", "warn"
            elif classified == "arrived":
                outcome, otag = "success", "ok"
            else:
                outcome, otag = classified, "warn"
        else:
            outcome, otag = "running", ""
        if outcome == "near_miss":
            hb = metrics.get("hard_brake_ttc")
            risk_txt = f"panic hard_brake@{self._fmt_opt(hb)}s"
        elif v2v_early and arrived:
            risk_txt = "V2V early yield → safe"
        elif emergency_brake and v2v_early:
            risk_txt = "hard_brake but V2V early → safe"
        else:
            risk_txt = "none"

        fs_d = metrics.get("first_sensor_dist")
        fs_t = metrics.get("first_sensor_ttc")
        fv_d = metrics.get("first_v2v_dist")
        fv_t = metrics.get("first_v2v_ttc")
        lead_s, lead_m = self._v2v_lead(
            first_sensor_step=metrics.get("first_sensor_step"),
            first_v2v_step=metrics.get("first_v2v_step"),
            first_sensor_dist=fs_d,
            first_v2v_dist=fv_d,
            v2v_early=v2v_early,
        )
        # First valid perception = earlier of sensor / v2v
        first_d, first_t = None, None
        if fs_d is not None and fv_d is not None:
            if metrics.get("first_v2v_step") is not None and (
                metrics.get("first_sensor_step") is None
                or metrics["first_v2v_step"] <= metrics["first_sensor_step"]
            ):
                first_d, first_t = fv_d, fv_t
            else:
                first_d, first_t = fs_d, fs_t
        elif fv_d is not None:
            first_d, first_t = fv_d, fv_t
        elif fs_d is not None:
            first_d, first_t = fs_d, fs_t

        braking = last_action in (0, 1)
        lines: list[tuple[str, str]] = [
            ("title", "LIVE\n"),
            ("", f"model      {model_name}\n"),
            ("", f"mode       {mode}\n"),
            ("", f"t          {step_i * 0.1:5.1f}s  step={step_i}\n"),
            ("", f"channel    {channel}\n"),
            ("", f"distance   {dist:6.1f} m\n"),
            ("", f"ego_ttc    {ego_ttc:6.2f} s\n"),
            ("", f"min_dist   {min_d:6.1f} m\n"),
            ("warn" if braking else "", f"action     {ACTION_NAME.get(last_action, last_action)}\n"),
            (otag, f"outcome    {outcome}\n"),
            ("title", "\nKEY\n"),
            (
                "",
                f"first_perc {self._fmt_opt(first_d)} m / {self._fmt_opt(first_t)} s\n",
            ),
            (
                "",
                f"1st_sensor {self._fmt_opt(fs_d)} m / {self._fmt_opt(fs_t)} s\n",
            ),
            (
                "",
                f"1st_v2v    {self._fmt_opt(fv_d)} m / {self._fmt_opt(fv_t)} s\n",
            ),
            ("ok" if v2v_early else "muted", f"v2v_early  {v2v_early}\n"),
            (
                "",
                f"v2v_lead   {self._fmt_opt(lead_s)} s / {self._fmt_opt(lead_m)} m\n",
            ),
            ("bad" if collision else "", f"collision  {collision}\n"),
            (
                "warn" if outcome == "near_miss" else "muted",
                f"near_miss  {outcome == 'near_miss'}  [{risk_txt}]\n",
            ),
            (
                "muted",
                f"geo_close  {geo_close}  (<{NEAR_MISS_RADIUS:.0f} m)\n",
            ),
            (
                "ok" if outcome == "success" else ("warn" if arrived else ""),
                f"success    {arrived}\n",
            ),
        ]
        return lines

    def _set_text_lines(self, widget: tk.Text, lines: list[tuple[str, str]]) -> None:
        widget.configure(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        for tag, text in lines:
            if tag:
                widget.insert(tk.END, text, tag)
            else:
                widget.insert(tk.END, text)
        widget.configure(state=tk.DISABLED)

    def _view_size(self, canvas: tk.Canvas | None = None) -> tuple[int, int]:
        c = canvas or self.canvas
        w = max(200, int(c.winfo_width() or VIEW_W))
        h = max(200, int(c.winfo_height() or VIEW_H))
        return w, h

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        if self.env is None and not (self.split_compare and self.env_b is not None):
            self._draw_placeholder()
        else:
            self._draw_all()

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

    def _make_env(self, mode: str, multi: bool, label: str, seed: int) -> SumoIntersectionEnv:
        return SumoIntersectionEnv(
            observation_mode=mode,
            pdr_scale=float(self.pdr_scale_var.get()) if mode == "lossy_v2v" else 1.0,
            sensor_range=35.0,
            nlos_blocker=bool(self.nlos_var.get()),
            nlos_east=bool(self.nlos_east_var.get()),
            nlos_west=bool(self.nlos_west_var.get()),
            multi_npc=bool(multi),
            seed=seed,
            sumo_binary="sumo",
            traci_label=label,
        )

    def _on_auto_play_toggle(self) -> None:
        if self.auto_scenario_var.get():
            self._refresh_scenario_label()
            # Apply current playlist entry so UI matches what will run next.
            self._apply_scenario(DEMO_SCENARIOS[self.scenario_idx % len(DEMO_SCENARIOS)])
        else:
            self.scenario_label_var.set("scenario: —")
        if not self.loop_var.get() and not self.auto_scenario_var.get():
            self._cancel_auto_continue()

    def _on_random_seed_toggle(self) -> None:
        if self.random_seed_var.get():
            seed = self._roll_seed()
            gap = self._last_conflict_gap_s
            gap_txt = f", |arrival_gap|={gap:.2f}s" if gap is not None else ""
            self.status_var.set(f"Random seed ON → seed={seed}{gap_txt}")
        else:
            self.seed_var.set(self.DEFAULT_SEED)
            self._last_conflict_gap_s = self._arrival_gap_for_seed(self.DEFAULT_SEED)
            self.status_var.set(
                f"Random seed OFF → seed fixed at {self.DEFAULT_SEED}"
            )

    @staticmethod
    def _arrival_gap_for_seed(seed: int) -> float:
        """Match env._sample_scenario RNG order; return target_eta - ego_eta (s)."""
        rng = np.random.default_rng(int(seed))
        ego_speed = float(rng.uniform(10.0, 14.0))
        target_speed = float(rng.uniform(9.0, 13.5))
        target_offset = float(rng.uniform(55.0, 75.0))
        ego_eta = EGO_START_OFFSET / max(ego_speed, 0.1)
        target_eta = target_offset / max(target_speed, 0.1)
        return target_eta - ego_eta

    def _roll_seed(self) -> int:
        """Pick a seed; if Prefer conflict is on, bias toward tight arrival sync."""
        if self.prefer_conflict_var.get():
            best_seed = int(random.randint(0, 9999))
            best_gap = abs(self._arrival_gap_for_seed(best_seed))
            for _ in range(CONFLICT_SEED_TRIES - 1):
                cand = int(random.randint(0, 9999))
                gap = abs(self._arrival_gap_for_seed(cand))
                if gap < best_gap:
                    best_seed, best_gap = cand, gap
                if best_gap <= CONFLICT_GAP_ACCEPT_S:
                    break
            self._last_conflict_gap_s = best_gap
            self.seed_var.set(best_seed)
            return best_seed

        seed = int(random.randint(0, 9999))
        self._last_conflict_gap_s = abs(self._arrival_gap_for_seed(seed))
        self.seed_var.set(seed)
        return seed

    def _prepare_episode_seed(self) -> int:
        """Resolve seed for the upcoming episode (random/conflict or fixed 7)."""
        if self.random_seed_var.get():
            return self._roll_seed()
        self.seed_var.set(self.DEFAULT_SEED)
        self._last_conflict_gap_s = abs(self._arrival_gap_for_seed(self.DEFAULT_SEED))
        return self.DEFAULT_SEED

    def _refresh_scenario_label(self) -> None:
        sc = DEMO_SCENARIOS[self.scenario_idx % len(DEMO_SCENARIOS)]
        self.scenario_label_var.set(
            f"scenario {self.scenario_idx % len(DEMO_SCENARIOS) + 1}/{len(DEMO_SCENARIOS)}: {sc['name']}"
        )

    def _apply_scenario(self, sc: dict) -> None:
        self.nlos_var.set(bool(sc.get("nlos", True)))
        self.nlos_east_var.set(bool(sc.get("east", True)))
        self.nlos_west_var.set(bool(sc.get("west", True)))
        # Seed is chosen in _restart (fixed 7 or random); playlist seeds ignored.
        if "pdr" in sc:
            self.pdr_scale_var.set(float(sc["pdr"]))
            self._on_pdr_scale()
        self._sync_nlos_side_controls()
        self._refresh_scenario_label()

    def _advance_scenario(self) -> None:
        self.scenario_idx = (self.scenario_idx + 1) % len(DEMO_SCENARIOS)
        self._apply_scenario(DEMO_SCENARIOS[self.scenario_idx])

    def _manual_next_scenario(self) -> None:
        """Jump to next playlist entry and restart (or arm for next Start)."""
        self._cancel_auto_continue()
        self.auto_scenario_var.set(True)
        self._advance_scenario()
        if self.env is not None or (self.split_compare and self.env_b is not None):
            self._restart(autoplay=True)
        else:
            self.status_var.set(
                f"Scenario ready: {DEMO_SCENARIOS[self.scenario_idx]['name']} — press Start"
            )

    def _cancel_auto_continue(self) -> None:
        if self._auto_after_id is not None:
            try:
                self.root.after_cancel(self._auto_after_id)
            except Exception:
                pass
            self._auto_after_id = None

    def _auto_play_enabled(self) -> bool:
        return bool(self.loop_var.get() or self.auto_scenario_var.get())

    def _schedule_auto_continue(self, outcome: str) -> None:
        if not self._auto_play_enabled():
            return
        self.paused = True
        kind = "next scenario" if self.auto_scenario_var.get() else "replay"
        self.status_var.set(
            f"Episode done: {outcome}. Auto {kind} in {AUTO_CONTINUE_S:.0f}s…"
        )
        self._cancel_auto_continue()
        self._auto_after_id = self.root.after(
            int(AUTO_CONTINUE_S * 1000), self._do_auto_continue
        )

    def _do_auto_continue(self) -> None:
        self._auto_after_id = None
        if self.closing or not self._auto_play_enabled():
            return
        if self.auto_scenario_var.get():
            self._advance_scenario()
        self._restart(autoplay=True)

    def _restart(self, autoplay: bool = True) -> None:
        self._cancel_auto_continue()
        self._close_env()
        self._close_env_b()
        self._reset_episode_metrics()
        seed = self._prepare_episode_seed()

        if self.split_compare:
            left_path = self.selected_model
            right_path = self.selected_model_b
            if left_path is None and right_path is None:
                messagebox.showwarning(
                    "Compare mode",
                    "Assign a model to both left and right panes\n"
                    "(drag from MODELS onto each pane).",
                )
                return
            if left_path is None or right_path is None:
                messagebox.showwarning(
                    "Compare mode",
                    "Both panes need a model.\n"
                    f"Left={left_path.name if left_path else 'none'}\n"
                    f"Right={right_path.name if right_path else 'none'}",
                )
                return
            self.active_mode, self.active_multi = infer_model_profile(left_path)
            self.active_mode_b, self.active_multi_b = infer_model_profile(right_path)
            try:
                self.model = PPO.load(str(left_path))
                self.model_b = PPO.load(str(right_path))
            except Exception as exc:
                messagebox.showerror("Load failed", str(exc))
                return
            self.env = self._make_env(self.active_mode, self.active_multi, "pane_left", seed)
            self.env_b = self._make_env(self.active_mode_b, self.active_multi_b, "pane_right", seed)
            self.obs, _ = self.env.reset(seed=seed)
            self.obs_b, _ = self.env_b.reset(seed=seed)
            self.sensor_range_m = float(self.env.sensor_range)
            self.last_info_b = {}
            self.last_action_b = 2
            self.episode_done_b = False
            self.banner_text_b = ""
            self.compare_results = {}
            self._refresh_pane_labels()
            self.status_var.set(
                f"Compare running seed={seed} | L={left_path.name} | R={right_path.name}"
            )
        else:
            self._load_selected_model()
            self._sync_profile_from_selection()
            self.env = self._make_env(
                self.active_mode, self.active_multi, "pane_main", seed
            )
            self.sensor_range_m = float(self.env.sensor_range)
            self.obs, _ = self.env.reset(seed=seed)
            label = RULE_LABEL if self.model is None else (
                self.selected_model.name if self.selected_model else RULE_LABEL
            )
            self.status_var.set(f"Running: {label}" if autoplay else f"Ready: {label}")

        self.step_i = 0
        self.last_action = 2
        self.last_info = {}
        self.episode_done = False
        self.paused = not autoplay
        self._draw_all()
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
        if self.env is None and not (self.split_compare and self.env_b is not None):
            self._draw_placeholder()
        else:
            self._draw_all()

    def _toggle_pause(self) -> None:
        if self.env is None and not (self.split_compare and self.env_b is not None):
            self.status_var.set("Start an episode first")
            return
        both_done = self.episode_done and (
            (not self.split_compare) or self.episode_done_b
        )
        if both_done:
            # Cancel pending auto-continue if user intervenes.
            if self._auto_after_id is not None:
                self._cancel_auto_continue()
                self.status_var.set("Auto-continue cancelled. Press Restart")
            else:
                self.status_var.set("Episode finished. Press Restart")
            return
        self._cancel_auto_continue()
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
        self._cancel_auto_continue()
        self._close_env()
        self._close_env_b()
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
        vw, vh = self._view_size(self.canvas)
        sx = vw * 0.5 + (wx - cam_x) * self.zoom
        sy = vh * 0.5 - (wy - cam_y) * self.zoom
        return int(sx), int(sy)

    def _pane_title(self, path: Path | None, fallback: str) -> str:
        if path is None:
            return fallback
        mode, multi = infer_model_profile(path)
        short = mode.replace("_v2v", "").replace("_only", "").upper()
        if multi:
            short = "MULTI/" + short
        return short

    def _draw_drop_placeholder(self, canvas: tk.Canvas, side: str, path: Path | None) -> None:
        canvas.delete("all")
        vw, vh = self._view_size(canvas)
        if path is None:
            canvas.create_text(
                vw // 2,
                vh // 2 - 16,
                text=f"{side} · empty",
                fill=PANEL_ACCENT,
                font=("Consolas", 14, "bold"),
                justify=tk.CENTER,
            )
            canvas.create_text(
                vw // 2,
                vh // 2 + 16,
                text="Use Add above, or drag from MODELS",
                fill=PANEL_MUTED,
                font=("Consolas", 10),
                justify=tk.CENTER,
            )
        else:
            canvas.create_text(
                vw // 2,
                vh // 2 - 16,
                text=f"{side}: {path.name}",
                fill=PANEL_FG,
                font=("Consolas", 12, "bold"),
                justify=tk.CENTER,
            )
            canvas.create_text(
                vw // 2,
                vh // 2 + 16,
                text="Press Start to run this pane",
                fill=PANEL_MUTED,
                font=("Consolas", 10),
                justify=tk.CENTER,
            )

    def _draw_placeholder(self) -> None:
        if self.split_compare:
            self._draw_drop_placeholder(self.canvas, "LEFT", self.selected_model)
            self._draw_drop_placeholder(self.canvas_b, "RIGHT", self.selected_model_b)
            return
        self.canvas.delete("all")
        vw, vh = self._view_size()
        self.canvas.create_text(
            vw // 2,
            vh // 2 - 20,
            text="Select a model, then Start",
            fill=PANEL_MUTED,
            font=("Consolas", 14),
            justify=tk.CENTER,
        )
        self.canvas.create_text(
            vw // 2,
            vh // 2 + 24,
            text="VIEW MODE → Compare for side-by-side",
            fill=PANEL_MUTED,
            font=("Consolas", 11),
            justify=tk.CENTER,
        )

    def _draw_all(self) -> None:
        if self.split_compare:
            if self.env is None and self.env_b is None:
                self._draw_placeholder()
                return
            if self.env is not None:
                self.env.activate()
                self._draw_frame(
                    title_left=self._pane_title(self.selected_model, "LEFT"),
                )
            else:
                self._draw_drop_placeholder(self.canvas, "LEFT", self.selected_model)
            if self.env_b is not None:
                self._draw_pane_b()
            else:
                self._draw_drop_placeholder(self.canvas_b, "RIGHT", self.selected_model_b)
            return
        if self.env is None:
            self._draw_placeholder()
            return
        self.env.activate()
        self._draw_frame(title_left=None)

    def _draw_pane_b(self) -> None:
        """Temporarily bind right-pane state onto self.* and reuse _draw_frame."""
        saved = {
            "canvas": self.canvas,
            "env": self.env,
            "last_info": self.last_info,
            "last_action": self.last_action,
            "active_mode": self.active_mode,
            "episode_done": self.episode_done,
            "episode_outcome": self.episode_outcome,
            "v2v_early": self.v2v_early,
            "first_v2v_step": self.first_v2v_step,
            "first_sensor_step": self.first_sensor_step,
            "emergency_brake": self.emergency_brake,
            "closest_dist": self.closest_dist,
            "speed_at_closest": self.speed_at_closest,
            "hard_brake_ttc": self.hard_brake_ttc,
            "first_brake_ttc": self.first_brake_ttc,
            "first_brake_action": self.first_brake_action,
            "banner_text": self.banner_text,
            "banner_color": self.banner_color,
            "banner_until": self.banner_until,
        }
        mb = self.metrics_b
        self.canvas = self.canvas_b
        self.env = self.env_b
        self.last_info = self.last_info_b
        self.last_action = self.last_action_b
        self.active_mode = self.active_mode_b
        self.episode_done = self.episode_done_b
        self.episode_outcome = str(mb.get("outcome") or "")
        self.v2v_early = bool(mb.get("v2v_early"))
        self.first_v2v_step = mb.get("first_v2v_step")
        self.first_sensor_step = mb.get("first_sensor_step")
        self.emergency_brake = bool(mb.get("emergency_brake"))
        self.closest_dist = float(mb.get("closest_dist", float("inf")))
        self.speed_at_closest = float(mb.get("speed_at_closest", 0.0))
        self.hard_brake_ttc = mb.get("hard_brake_ttc")
        self.first_brake_ttc = mb.get("first_brake_ttc")
        self.first_brake_action = mb.get("first_brake_action")
        self.banner_text = self.banner_text_b
        self.banner_color = self.banner_color_b
        self.banner_until = self.banner_until_b
        try:
            self.env_b.activate()
            self._draw_frame(
                title_left=self._pane_title(self.selected_model_b, "RIGHT")
            )
        finally:
            self.banner_text_b = self.banner_text
            self.banner_color_b = self.banner_color
            self.banner_until_b = self.banner_until
            for k, v in saved.items():
                setattr(self, k, v)

    def _draw_nlos_building(
        self,
        aabb: tuple[float, float, float, float],
        cam_x: float,
        cam_y: float,
        *,
        side: str,
    ) -> None:
        """Draw NLOS building poly with attribute labels on the footprint."""
        xmin, ymin, xmax, ymax = aabb
        bpts: list[int] = []
        for wx, wy in ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)):
            sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
            bpts.extend([sx, sy])
        self.canvas.create_polygon(*bpts, fill=BUILDING, outline="#9aa0a6", width=2)
        cx, cy = self._world_to_screen((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, cam_x, cam_y)
        fs = int(max(8, min(12, 7 + self.zoom * 0.6)))
        lines = [f"{side} BUILDING", "NLOS occluder"]
        line_h = fs + 2
        y0 = cy - (len(lines) - 1) * line_h * 0.5
        for i, line in enumerate(lines):
            self.canvas.create_text(
                cx,
                y0 + i * line_h,
                text=line,
                fill="#ffffff" if i == 0 else "#f0f0f0",
                font=("Consolas", fs, "bold" if i == 0 else "normal"),
                justify=tk.CENTER,
            )

    def _draw_frame(self, title_left: str | None = None) -> None:
        if self.env is None:
            self._draw_placeholder()
            return
        self.env.activate()
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
            self._draw_nlos_building(
                NLOS_BUILDING_EAST_AABB,
                cam_x,
                cam_y,
                side="EAST",
            )
        if self.nlos_var.get() and self.nlos_west_var.get():
            self._draw_nlos_building(
                NLOS_BUILDING_WEST_AABB,
                cam_x,
                cam_y,
                side="WEST",
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
        self._draw_intuition_chrome(title_left)
        self._draw_banner_on(
            self.canvas,
            env=self.env,
            episode_done=self.episode_done,
            mode=self.active_mode,
            v2v_early=self.v2v_early,
            first_v2v_step=self.first_v2v_step,
            banner_text=self.banner_text,
            banner_color=self.banner_color,
            banner_until=self.banner_until,
        )

    def _perception_tip(self) -> tuple[str, str]:
        """Return (korean tip, color) for current pane state."""
        info = self.last_info
        if self.episode_done:
            banner, color = self._outcome_banner(
                self._classify_outcome(info, self._left_metrics_dict())
            )
            return f"Result: {banner}", color
        sens = bool(info.get("sensor_visible", False))
        mode = self.active_mode
        v2v_fresh = (
            bool(self.env._fresh_bsm_available())
            if self.env is not None and mode == "lossy_v2v"
            else mode == "perfect_v2v"
        )
        channel = detection_channel(
            mode, sens, bool(info.get("visible", False)), v2v_fresh
        )
        if mode == "sensor_only":
            if sens:
                return "Seen by sensor (late)", SENSOR_FX
            return "Outside sensor · unknown (risk)", "#e67e22"
        if "v2v" in channel and not sens:
            return "Seen via V2V first (early)", V2V_FX
        if "v2v" in channel and sens:
            return "Seen via V2V + sensor", BOTH_FX
        if sens:
            return "Sensor only", SENSOR_FX
        return "Threat not yet perceived", PANEL_MUTED

    def _draw_intuition_chrome(self, title_left: str | None) -> None:
        vw, vh = self._view_size(self.canvas)
        mode = self.active_mode
        if title_left:
            title = title_left
        elif mode == "sensor_only":
            title = "SENSOR ONLY"
        elif mode == "lossy_v2v":
            title = "LOSSY V2V"
        else:
            title = "PERFECT V2V"
        title_color = (
            SENSOR_FX if "SENSOR" in title else (V2V_FX if "V2V" in title else PANEL_ACCENT)
        )
        self.canvas.create_rectangle(0, 0, vw, 34, fill="#0a0c10", outline="")
        self.canvas.create_text(
            12, 17, text=title, fill=title_color, font=("Consolas", 13, "bold"), anchor="w"
        )
        if self.last_action in (0, 1):
            self.canvas.create_text(
                vw - 12,
                17,
                text="BRAKING!",
                fill=BRAKE_LIGHT,
                font=("Consolas", 12, "bold"),
                anchor="e",
            )

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
        x0, y0 = 12, 42
        items = [
            (SENSOR_FX, "Green = sensor see"),
            (V2V_FX, "Blue = V2V RX"),
            (V2V_HOLD_FX, "Navy = V2V HOLD (AoI)"),
            (PRIMARY_TAG, "Orange = primary threat"),
            (BRAKE_LIGHT, "Red = ego braking"),
        ]
        for i, (color, label) in enumerate(items):
            y = y0 + i * 17
            self.canvas.create_rectangle(x0, y, x0 + 12, y + 9, fill=color, outline="")
            self.canvas.create_text(
                x0 + 18,
                y + 4,
                text=label,
                fill=PANEL_MUTED,
                font=("Consolas", 8),
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

        if vid == EGO_ID:
            self.canvas.create_text(
                cx,
                cy - max(16, int(2.4 * self.zoom)),
                text="ego",
                fill="#ffffff",
                font=("Consolas", 10, "bold"),
            )
        elif vid == PARKED_TRUCK_ID or vid == PARKED_TRUCK_WEST_ID:
            side = "EAST" if vid == PARKED_TRUCK_ID else "WEST"
            truck_lines = [f"{side} TRUCK", "NLOS occluder"]
            fs = 8
            y0 = cy - max(10, int(1.6 * self.zoom))
            for i, line in enumerate(truck_lines):
                self.canvas.create_text(
                    cx,
                    y0 + i * (fs + 2),
                    text=line,
                    fill="#ffffff" if i == 0 else "#f0f0f0",
                    font=("Consolas", fs, "bold" if i == 0 else "normal"),
                )
        else:
            speed = float(traci.vehicle.getSpeed(vid))
            if is_primary:
                label = f"threat {speed:.0f}m/s"
                tag_color = PRIMARY_TAG
            else:
                label = f"other {speed:.0f}m/s"
                tag_color = SECONDARY_FX
            self.canvas.create_text(
                cx,
                cy - max(14, int(2.2 * self.zoom)),
                text=label,
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

    def _left_metrics_dict(self) -> dict:
        return {
            "first_sensor_step": self.first_sensor_step,
            "first_sensor_dist": self.first_sensor_dist,
            "first_sensor_ttc": self.first_sensor_ttc,
            "first_v2v_step": self.first_v2v_step,
            "first_v2v_dist": self.first_v2v_dist,
            "first_v2v_ttc": self.first_v2v_ttc,
            "v2v_early": self.v2v_early,
            "emergency_brake": self.emergency_brake,
            "closest_dist": self.closest_dist,
            "speed_at_closest": self.speed_at_closest,
            "hard_brake_ttc": self.hard_brake_ttc,
            "first_brake_ttc": self.first_brake_ttc,
            "first_brake_action": self.first_brake_action,
            "outcome": self.episode_outcome,
        }

    def _update_hud(self) -> None:
        left_lines = self._key_metric_lines(
            model_path=self.selected_model,
            mode=self.active_mode,
            info=self.last_info,
            env=self.env,
            episode_done=self.episode_done,
            step_i=self.step_i,
            last_action=self.last_action,
            metrics=self._left_metrics_dict(),
            idle_hint="Select a model, then Start",
        )
        if self.split_compare:
            right_lines = self._key_metric_lines(
                model_path=self.selected_model_b,
                mode=self.active_mode_b,
                info=self.last_info_b,
                env=self.env_b,
                episode_done=self.episode_done_b,
                step_i=self.step_i,
                last_action=self.last_action_b,
                metrics=self.metrics_b if self.metrics_b else self._empty_metrics(),
                idle_hint="Drag a MODELS item here",
            )
            self._set_text_lines(self.metrics_left, left_lines)
            self._set_text_lines(self.metrics_right, right_lines)
        else:
            self._set_text_lines(self.hud, left_lines)

    def _finish_pane(
        self,
        info: dict,
        *,
        pane: str,
        metrics: dict,
    ) -> str:
        outcome = self._classify_outcome(info, metrics)
        banner, color = self._outcome_banner(outcome)
        self._set_banner(banner, color, hold_s=4.0, pane=pane)
        metrics["outcome"] = outcome
        return outcome

    def _tick(self) -> None:
        if self.closing:
            return
        delay_ms = max(10, int(float(self.delay_var.get()) * 1000))

        if not self.paused:
            # Left / single pane
            if self.env is not None and not self.episode_done:
                if self.model is None:
                    action = self.env.rule_based_action()
                else:
                    action = int(self.model.predict(self.obs, deterministic=True)[0])
                self.last_action = int(action)
                self.obs, _reward, terminated, truncated, info = self.env.step(self.last_action)
                self.last_info = info
                self.step_i += 1
                self._track_episode_metrics(info, self.last_action)
                if terminated or truncated:
                    self.episode_done = True
                    outcome = self._finish_pane(
                        info,
                        pane="a",
                        metrics=self._left_metrics_dict(),
                    )
                    self.episode_outcome = outcome
                    if self.split_compare:
                        self.compare_results["A"] = self._snapshot_episode()
                        self._update_hud()
                    if not self.split_compare:
                        if self._auto_play_enabled():
                            self._schedule_auto_continue(outcome)
                        else:
                            self.paused = True
                            self.status_var.set(
                                f"Episode done: {outcome}. Restart to replay."
                            )

            # Right pane (split)
            if self.split_compare and self.env_b is not None and not self.episode_done_b:
                if self.model_b is None:
                    action_b = self.env_b.rule_based_action()
                else:
                    action_b = int(self.model_b.predict(self.obs_b, deterministic=True)[0])
                self.last_action_b = int(action_b)
                self.obs_b, _r, term_b, trunc_b, info_b = self.env_b.step(self.last_action_b)
                self.last_info_b = info_b
                self._track_metrics_into(
                    self.metrics_b,
                    info_b,
                    env=self.env_b,
                    mode=self.active_mode_b,
                    step_i=self.step_i,
                    action=self.last_action_b,
                )
                if term_b or trunc_b:
                    self.episode_done_b = True
                    outcome_b = self._finish_pane(info_b, pane="b", metrics=self.metrics_b)
                    self.compare_results["B"] = self._snapshot_from(
                        info=info_b,
                        mode=self.active_mode_b,
                        multi=self.active_multi_b,
                        model_path=self.selected_model_b,
                        metrics=self.metrics_b,
                        steps=self.step_i,
                    )
                    self._update_hud()
                    self.status_var.set(f"Right done: {outcome_b}")

            if self.split_compare and self.episode_done and self.episode_done_b:
                if self._auto_play_enabled():
                    left_out = (self.compare_results.get("A") or {}).get("outcome", "done")
                    self._schedule_auto_continue(str(left_out))
                else:
                    self.paused = True
                    self.status_var.set("Split compare finished — see result card")

        if self.env is not None or (self.split_compare and self.env_b is not None):
            self._draw_all()
        self._update_hud()
        self.root.after(delay_ms, self._tick)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    DemoApp().run()


if __name__ == "__main__":
    main()
