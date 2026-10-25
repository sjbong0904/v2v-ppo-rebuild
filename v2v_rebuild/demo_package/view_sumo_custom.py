from __future__ import annotations

import argparse
import math
import tkinter as tk
from pathlib import Path

import sumolib
import traci
from stable_baselines3 import PPO

from sumo_intersection_env import (
    EGO_ID,
    NLOS_BUILDING_AABB,
    PARKED_TRUCK_ID,
    TARGET_ID,
    SumoIntersectionEnv,
)

ACTION_NAME = {
    0: "HARD_BRAKE",
    1: "BRAKE",
    2: "HOLD",
    3: "ACCEL",
    4: "FULL_ACCEL",
}

HERE = Path(__file__).resolve().parent
DEFAULT_NET = HERE / "sumo_data" / "map.net.xml"

# Layout
VIEW_W = 900
VIEW_H = 720
PANEL_W = 340
PAD = 12

# Camera
DEFAULT_ZOOM = 5.5  # pixels per meter
ZOOM_MIN = 1.5
ZOOM_MAX = 20.0
ZOOM_STEP = 1.15

# Colors (RGB hex for tk)
BG = "#1a1d23"
ROAD = "#3a3f4b"
LANE = "#5a6170"
BUILDING = "#6b7280"
EGO = "#e74c3c"
TARGET = "#3498db"
TRUCK = "#9aa0a6"
NPC = "#8e44ad"
CONFLICT = "#f1c40f"
LOS_OK = "#2ecc71"
LOS_BLOCK = "#e67e22"
LOS_NONE = "#7f8c8d"
PANEL_BG = "#12151a"
PANEL_FG = "#e8eaed"
PANEL_MUTED = "#9aa0a6"
PANEL_ACCENT = "#5dade2"


def heading_label(angle_deg: float) -> str:
    """SUMO angle: 0=north, 90=east, clockwise."""
    a = angle_deg % 360.0
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    idx = int((a + 22.5) // 45.0) % 8
    return f"{dirs[idx]} ({a:5.1f} deg)"


def detection_channel(mode: str, sensor_visible: bool, visible: bool, v2v_fresh: bool) -> str:
    if not visible:
        return "none"
    if mode == "sensor_only":
        return "sensor" if sensor_visible else "none"
    parts: list[str] = []
    if sensor_visible:
        parts.append("sensor")
    if mode == "perfect_v2v" or (mode == "lossy_v2v" and v2v_fresh):
        parts.append("v2v")
    return "+".join(parts) if parts else "none"


class EgoFollowViewer:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.zoom = DEFAULT_ZOOM
        self.paused = False
        self.done = False
        self.step_i = 0
        self.last_action = 2
        self.last_info: dict = {}
        self.obs = None

        self.env = SumoIntersectionEnv(
            observation_mode=args.mode,
            pdr_scale=args.pdr_scale,
            sensor_range=args.sensor_range,
            nlos_blocker=args.nlos_blocker,
            multi_npc=args.multi_npc,
            seed=args.episode_seed,
            sumo_binary="sumo",
        )
        self.model = PPO.load(args.model) if args.model else None
        self.obs, _ = self.env.reset(seed=args.episode_seed)

        self.net = sumolib.net.readNet(str(args.net))
        self.lane_shapes = self._load_lane_shapes()

        self.root = tk.Tk()
        self.root.title("V2V SUMO Viewer")
        self.root.configure(bg=BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        outer = tk.Frame(self.root, bg=BG)
        outer.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            outer, width=VIEW_W, height=VIEW_H, bg=BG, highlightthickness=0
        )
        self.canvas.pack(side=tk.LEFT, padx=(PAD, 0), pady=PAD)
        self.canvas.bind("<MouseWheel>", self._on_wheel)
        # Linux
        self.canvas.bind("<Button-4>", lambda e: self._zoom_by(ZOOM_STEP))
        self.canvas.bind("<Button-5>", lambda e: self._zoom_by(1.0 / ZOOM_STEP))

        panel = tk.Frame(outer, width=PANEL_W, bg=PANEL_BG)
        panel.pack(side=tk.RIGHT, fill=tk.Y, padx=PAD, pady=PAD)
        panel.pack_propagate(False)

        self.hud = tk.Text(
            panel,
            bg=PANEL_BG,
            fg=PANEL_FG,
            insertbackground=PANEL_FG,
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

        hint = tk.Label(
            panel,
            text="wheel: zoom | space: pause | q: quit\ncamera locked on ego",
            bg=PANEL_BG,
            fg=PANEL_MUTED,
            font=("Consolas", 9),
            justify=tk.LEFT,
        )
        hint.pack(anchor=tk.W, padx=10, pady=(0, 8))

        self.root.bind("<space>", self._toggle_pause)
        self.root.bind("q", lambda e: self._on_close())
        self.root.bind("Q", lambda e: self._on_close())

        self._draw_frame()
        self._update_hud()
        delay_ms = max(10, int(args.delay * 1000))
        self.root.after(delay_ms, self._tick)

    def _load_lane_shapes(self) -> list[list[tuple[float, float]]]:
        shapes: list[list[tuple[float, float]]] = []
        for edge in self.net.getEdges():
            for lane in edge.getLanes():
                shape = [(float(x), float(y)) for x, y in lane.getShape()]
                if len(shape) >= 2:
                    shapes.append(shape)
        return shapes

    def _on_wheel(self, event: tk.Event) -> None:
        if event.delta > 0:
            self._zoom_by(ZOOM_STEP)
        else:
            self._zoom_by(1.0 / ZOOM_STEP)

    def _zoom_by(self, factor: float) -> None:
        self.zoom = float(min(ZOOM_MAX, max(ZOOM_MIN, self.zoom * factor)))
        self._draw_frame()

    def _toggle_pause(self, _event: tk.Event | None = None) -> None:
        self.paused = not self.paused

    def _on_close(self) -> None:
        self.done = True
        try:
            self.env.close()
        except Exception:
            pass
        self.root.destroy()

    def _ego_pose(self) -> tuple[float, float, float]:
        if EGO_ID in traci.vehicle.getIDList():
            x, y = traci.vehicle.getPosition(EGO_ID)
            ang = float(traci.vehicle.getAngle(EGO_ID))
            return float(x), float(y), ang
        return 0.0, float(self.env.ego_y), 0.0

    def _world_to_screen(
        self, wx: float, wy: float, cam_x: float, cam_y: float
    ) -> tuple[int, int]:
        sx = VIEW_W * 0.5 + (wx - cam_x) * self.zoom
        sy = VIEW_H * 0.5 - (wy - cam_y) * self.zoom
        return int(sx), int(sy)

    def _draw_frame(self) -> None:
        self.canvas.delete("all")
        cam_x, cam_y, _ = self._ego_pose()

        # Roads
        for shape in self.lane_shapes:
            pts: list[int] = []
            for wx, wy in shape:
                sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
                pts.extend([sx, sy])
            if len(pts) >= 4:
                self.canvas.create_line(*pts, fill=ROAD, width=max(2, int(3.2 * self.zoom / 5.5)))

        # Building occluder
        xmin, ymin, xmax, ymax = NLOS_BUILDING_AABB
        corners = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
        bpts: list[int] = []
        for wx, wy in corners:
            sx, sy = self._world_to_screen(wx, wy, cam_x, cam_y)
            bpts.extend([sx, sy])
        self.canvas.create_polygon(*bpts, fill=BUILDING, outline="#4b5563")

        # Conflict point (junction center)
        cx, cy = self._world_to_screen(0.0, 0.0, cam_x, cam_y)
        r = max(3, int(0.8 * self.zoom))
        self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline=CONFLICT, width=2)
        self.canvas.create_line(cx - 2 * r, cy, cx + 2 * r, cy, fill=CONFLICT)
        self.canvas.create_line(cx, cy - 2 * r, cx, cy + 2 * r, fill=CONFLICT)

        # LOS ego -> primary threat
        threat_id = str(self.last_info.get("primary_threat", TARGET_ID))
        ids = set(traci.vehicle.getIDList())
        if EGO_ID in ids and threat_id in ids:
            ex, ey = traci.vehicle.getPosition(EGO_ID)
            tx, ty = traci.vehicle.getPosition(threat_id)
            x0, y0 = self._world_to_screen(float(ex), float(ey), cam_x, cam_y)
            x1, y1 = self._world_to_screen(float(tx), float(ty), cam_x, cam_y)
            if self.last_info.get("occluded"):
                color = LOS_BLOCK
            elif self.last_info.get("sensor_visible"):
                color = LOS_OK
            else:
                color = LOS_NONE
            self.canvas.create_line(x0, y0, x1, y1, fill=color, dash=(4, 3), width=2)

        for vid in traci.vehicle.getIDList():
            self._draw_vehicle(vid, cam_x, cam_y)

        # Crosshair at ego (camera lock marker)
        ex, ey, _ = self._ego_pose()
        sx, sy = self._world_to_screen(ex, ey, cam_x, cam_y)
        self.canvas.create_oval(sx - 6, sy - 6, sx + 6, sy + 6, outline="#ffffff", width=1)

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

        if vid == EGO_ID:
            color = EGO
        elif vid == TARGET_ID:
            color = TARGET
        elif vid == PARKED_TRUCK_ID:
            color = TRUCK
        else:
            color = NPC
        self.canvas.create_polygon(*pts, fill=color, outline="#111111")
        # Heading tick
        nose_x = x + hl * fx
        nose_y = y + hl * fy
        nx, ny = self._world_to_screen(nose_x, nose_y, cam_x, cam_y)
        cx, cy = self._world_to_screen(float(x), float(y), cam_x, cam_y)
        self.canvas.create_line(cx, cy, nx, ny, fill="#ffffff", width=2)

    def _vehicle_speed_heading(self, vid: str) -> tuple[float, str]:
        if vid not in traci.vehicle.getIDList():
            return 0.0, "n/a"
        speed = float(traci.vehicle.getSpeed(vid))
        ang = float(traci.vehicle.getAngle(vid))
        return speed, heading_label(ang)

    def _conflict_times(self) -> tuple[float, float, float]:
        """Return (ego_t_to_conflict, threat_t_to_conflict, gap_ttc)."""
        ego_t = (0.0 - self.env.ego_y) / max(self.env.ego_speed, 0.1)
        threat_t = self.env.target_x / max(self.env.target_speed, 0.1)
        gap = float(self.last_info.get("ttc", abs(ego_t - threat_t)))
        return float(ego_t), float(threat_t), gap

    def _update_hud(self) -> None:
        mode = self.args.mode
        info = self.last_info
        sens = bool(info.get("sensor_visible", False))
        vis = bool(info.get("visible", False))
        occ = bool(info.get("occluded", False))
        v2v_fresh = bool(self.env._fresh_bsm_available()) if mode == "lossy_v2v" else mode == "perfect_v2v"
        channel = detection_channel(mode, sens, vis, v2v_fresh)

        ego_v, ego_h = self._vehicle_speed_heading(EGO_ID)
        if EGO_ID not in traci.vehicle.getIDList():
            ego_v = float(self.env.ego_speed)
            ego_h = heading_label(0.0)

        threat_id = str(info.get("primary_threat", TARGET_ID))
        thr_v, thr_h = self._vehicle_speed_heading(threat_id)
        if threat_id not in traci.vehicle.getIDList():
            thr_v = float(self.env.target_speed)
            thr_h = heading_label(270.0 if threat_id == TARGET_ID else 90.0)

        action = self.last_action
        braking = action in (0, 1)
        ego_t, thr_t, gap_ttc = self._conflict_times()

        lines: list[tuple[str, str]] = [
            ("title", "SCENARIO\n"),
            ("", f"mode          {mode}\n"),
            ("", f"nlos_blocker  {int(self.args.nlos_blocker)}\n"),
            ("", f"multi_npc     {int(self.args.multi_npc)}\n"),
            ("", f"seed          {self.args.episode_seed}\n"),
            ("", f"step          {self.step_i}  t={self.step_i * 0.1:5.1f}s\n"),
            ("muted", f"paused        {self.paused}\n\n"),
            ("title", "EGO\n"),
            ("", f"speed         {ego_v:5.2f} m/s\n"),
            ("", f"heading       {ego_h}\n"),
            ("", f"position      y={self.env.ego_y:7.1f} m\n"),
            ("", f"action        {ACTION_NAME.get(action, action)}\n"),
            ("warn" if braking else "", f"braking       {braking}\n\n"),
            ("title", "THREAT\n"),
            ("", f"id            {threat_id}\n"),
            ("", f"speed         {thr_v:5.2f} m/s\n"),
            ("", f"heading       {thr_h}\n"),
            ("", f"approach      {self.env.target_x:7.1f} m\n\n"),
            ("title", "DETECTION\n"),
            ("ok" if vis else "bad", f"policy_visible {vis}\n"),
            ("", f"channel       {channel}\n"),
            ("", f"sensor        {sens}\n"),
            ("warn" if occ else "", f"occluded      {occ}\n"),
            ("", f"aoi           {float(info.get('aoi', 0.0)):4.2f} s\n"),
            ("", f"v2v_rx        {int(info.get('v2v_rx_count', 0))}\n\n"),
            ("title", "CONFLICT\n"),
            ("", "point         (0.0, 0.0) m  junction\n"),
            ("", f"ego_eta       {ego_t:6.2f} s\n"),
            ("", f"threat_eta    {thr_t:6.2f} s\n"),
            ("", f"ttc_gap       {gap_ttc:6.2f} s\n"),
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
        if self.done:
            return
        delay_ms = max(10, int(self.args.delay * 1000))

        if not self.paused and not self.done:
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
                self.done = True

        self._draw_frame()
        self._update_hud()

        if self.done and self.paused:
            # Final frame held; still allow zoom until window close.
            self.root.after(delay_ms, self._tick)
            return
        self.root.after(delay_ms, self._tick)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Custom ego-follow SUMO viewer with HUD.")
    parser.add_argument("--model", default=None)
    parser.add_argument(
        "--mode",
        default="sensor_only",
        choices=["perfect_v2v", "lossy_v2v", "sensor_only"],
    )
    parser.add_argument("--episode-seed", type=int, default=7)
    parser.add_argument("--pdr-scale", type=float, default=1.0)
    parser.add_argument("--sensor-range", type=float, default=35.0)
    parser.add_argument("--nlos-blocker", action="store_true")
    parser.add_argument("--multi-npc", action="store_true")
    parser.add_argument("--delay", type=float, default=0.08, help="Seconds between frames.")
    parser.add_argument("--net", default=str(DEFAULT_NET))
    args = parser.parse_args()

    viewer = EgoFollowViewer(args)
    viewer.run()


if __name__ == "__main__":
    main()
