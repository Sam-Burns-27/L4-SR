"""
L4-SR Quadruped Python Controller
====================================
Connects via Bluetooth SPP (virtual COM port) to the L4-SR ESP32.
Sends commands in the firmware's native format:
  - Individual servo:  s [0-11] [0-180]
  - All servos:        S [0-180]

Timing notes (Bluetooth SPP + PCA9685 I2C):
  - Bluetooth SPP adds ~10-30ms latency per packet
  - PCA9685 I2C setPWM() adds ~1ms (400kHz fast mode) or ~5ms (100kHz default)
  - Firmware readStringUntil() timeout set to 100ms (was 1000ms)
  - Recommended serial_delay: 0.05s (50ms) minimum, tune down carefully

Servo channel map (matches firmware servo_pin[4][3]):
  Leg 0 (BL): ch 0, 4, 8   (hip, knee, ankle)
  Leg 1 (BR): ch 1, 5, 9
  Leg 2 (FR): ch 2, 6, 10
  Leg 3 (FL): ch 3, 7, 11
"""

import tkinter as tk
from tkinter import ttk, messagebox, font
import serial
import serial.tools.list_ports
import threading
import time
import json
import os
import math

# ─── Shared hardware / stance parameters (apply to ALL gaits) ────────────────
SHARED_PARAMS = {
    "stance_hip":    90,
    "stance_knee":   75,
    "stance_ankle":  100,
    "lift_knee":     45,
    "body_tilt":     0,
    "turn_offset":   15,
    "serial_delay":  0.05,
    "pose_delay":    0.05,
}

# ─── Per-gait parameters (each gait has its own stride + timing profile) ──────
GAIT_PARAMS = {
    "trot": {
        "label":           "Trot",
        "desc":            "Diagonal pairs — fast, efficient",
        "swing_hip_fwd":   110,
        "swing_hip_bwd":   70,
        "step_delay":      0.04,
        "steps_per_phase": 8,
        "cycle_pause":     0.02,
    },
    "crawl": {
        "label":           "Crawl",
        "desc":            "One leg at a time — most stable",
        "swing_hip_fwd":   110,
        "swing_hip_bwd":   70,
        "step_delay":      0.06,
        "steps_per_phase": 10,
        "cycle_pause":     0.04,
    },
    "wave": {
        "label":           "Wave",
        "desc":            "Sequential flow FL→FR→BR→BL — smooth",
        "swing_hip_fwd":   112,
        "swing_hip_bwd":   68,
        "step_delay":      0.05,
        "steps_per_phase": 10,
        "cycle_pause":     0.02,
    },
    "pace": {
        "label":           "Pace",
        "desc":            "Same-side pairs FL+BL then FR+BR",
        "swing_hip_fwd":   108,
        "swing_hip_bwd":   72,
        "step_delay":      0.04,
        "steps_per_phase": 8,
        "cycle_pause":     0.03,
    },
    "bound": {
        "label":           "Bound",
        "desc":            "Front pair then back pair — fast gallop",
        "swing_hip_fwd":   120,
        "swing_hip_bwd":   60,
        "step_delay":      0.03,
        "steps_per_phase": 6,
        "cycle_pause":     0.01,
    },
}

ACTIVE_GAIT = "trot"   # currently selected gait key

def gp():
    """Shorthand — returns active gait param dict merged with shared params."""
    d = dict(SHARED_PARAMS)
    d.update(GAIT_PARAMS[ACTIVE_GAIT])
    return d

# ─── Constants ────────────────────────────────────────────────────────────────
BAUD_RATE  = 115200
NUM_SERVOS = 12
POSE_FILE  = "l4sr_poses.json"

# Channels whose angles are mechanically inverted (send 180-angle instead).
# BR knee/ankle = ch 05, 09  |  FL knee/ankle = ch 07, 11
INVERTED_CHANNELS = {5, 9, 7, 11}

def apply_inversion(ch, angle):
    return (180 - angle) if ch in INVERTED_CHANNELS else angle

LEG_NAMES   = ["BL (Back-Left)", "BR (Back-Right)", "FR (Front-Right)", "FL (Front-Left)"]
JOINT_NAMES = ["Hip", "Knee", "Ankle"]

BL, BR, FR, FL = 0, 1, 2, 3

def flat_to_channel(flat_idx):
    leg   = flat_idx // 3
    joint = flat_idx % 3
    return joint * 4 + leg

def servo_ch(leg, joint):
    return joint * 4 + leg

DEFAULT_POSES = {
    "Neutral": [90]*12,
    "Sit":     [90, 45, 45,  90, 135, 135,  90, 45, 45,  90, 135, 135],
    "Stretch": [90,135,160,  90, 45,  20,   90,135,160,  90, 45,  20],
    "Stand":   [90, 60, 120]*4,
}

# ─── Colours ──────────────────────────────────────────────────────────────────
BG        = "#1a1a2e"
PANEL     = "#16213e"
ACCENT    = "#0f3460"
HIGHLIGHT = "#e94560"
TEXT      = "#eaeaea"
GREEN     = "#4ecca3"
YELLOW    = "#f5a623"
ORANGE    = "#ff7b2e"
PURPLE    = "#9b5de5"
SLIDER_BG = "#0d2137"


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                            GAIT ENGINE                                     ║
# ╚══════════════════════════════════════════════════════════════════════════════╝
class GaitEngine:
    """
    Multi-gait engine supporting: trot, crawl, wave, pace, bound.
    All gaits share stance/lift/hardware params from SHARED_PARAMS.
    Stride and timing come from the active gait's GAIT_PARAMS entry.
    """

    # Crawl / wave leg order: FL → FR → BR → BL
    CRAWL_ORDER = [FL, FR, BR, BL]

    def __init__(self, serial_mgr, log_cb, angle_vars):
        self.serial_mgr = serial_mgr
        self.log        = log_cb
        self.angle_vars = angle_vars
        self._running   = False
        self._thread    = None
        self._direction = "forward"
        self._lock      = threading.Lock()

    # ── Public ────────────────────────────────────────────────────────────────
    def start(self, direction="forward"):
        with self._lock:
            if self._running:
                return
            self._direction = direction
            self._running   = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.log(f"🦿 [{GAIT_PARAMS[ACTIVE_GAIT]['label']}] started  [{direction}]")

    def stop(self):
        with self._lock:
            self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        self.log("🛑 Gait stopped")

    @property
    def running(self):
        return self._running

    # ── Main loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        while self._running:
            p   = gp()
            fwd = p["swing_hip_fwd"]
            bwd = p["swing_hip_bwd"]
            off = p["turn_offset"]
            d   = self._direction

            # Compute per-direction fwd/bwd targets
            if d == "forward":
                sf, pb = fwd, bwd
            elif d == "backward":
                sf, pb = bwd, fwd
            elif d == "turn_left":
                sf, pb = fwd - off, bwd + off
            else:  # turn_right
                sf, pb = fwd + off, bwd - off

            gait = ACTIVE_GAIT
            if gait == "trot":
                self._trot_cycle(sf, pb)
            elif gait == "crawl":
                self._crawl_cycle(sf, pb)
            elif gait == "wave":
                self._wave_cycle(sf, pb)
            elif gait == "pace":
                self._pace_cycle(sf, pb)
            elif gait == "bound":
                self._bound_cycle(sf, pb)

            time.sleep(gp()["cycle_pause"])

        self._stance_all()

    # ── Gait implementations ──────────────────────────────────────────────────

    def _trot_cycle(self, sf, pb):
        """Diagonal trot: FL+BR then FR+BL."""
        self._trot_phase([FL, BR], [FR, BL], sf, pb)
        if not self._running: return
        self._trot_phase([FR, BL], [FL, BR], sf, pb)

    def _crawl_cycle(self, sf, pb):
        """Crawl: one leg at a time, other 3 push."""
        for swing_leg in self.CRAWL_ORDER:
            if not self._running: return
            push_legs = [l for l in [FL, FR, BL, BR] if l != swing_leg]
            self._single_phase(swing_leg, push_legs, sf, pb)

    def _wave_cycle(self, sf, pb):
        """Wave: sequential overlapping — like crawl but push starts before plant."""
        for i, swing_leg in enumerate(self.CRAWL_ORDER):
            if not self._running: return
            push_legs = [l for l in [FL, FR, BL, BR] if l != swing_leg]
            self._wave_phase(swing_leg, push_legs, sf, pb)

    def _pace_cycle(self, sf, pb):
        """Pace: same-side pairs FL+BL then FR+BR."""
        self._trot_phase([FL, BL], [FR, BR], sf, pb)
        if not self._running: return
        self._trot_phase([FR, BR], [FL, BL], sf, pb)

    def _bound_cycle(self, sf, pb):
        """Bound: front pair then back pair."""
        self._trot_phase([FL, FR], [BL, BR], sf, pb)
        if not self._running: return
        self._trot_phase([BL, BR], [FL, FR], sf, pb)

    # ── Phase primitives ──────────────────────────────────────────────────────

    def _trot_phase(self, swing_legs, push_legs, swing_fwd, push_bwd):
        """Lift swing legs, sweep all hips simultaneously, plant swing legs."""
        p           = gp()
        steps       = p["steps_per_phase"]
        delay       = p["step_delay"]
        knee_stance = p["stance_knee"] + p["body_tilt"]
        knee_lift   = p["lift_knee"]
        ankle       = p["stance_ankle"]

        # Lift
        for leg in swing_legs:
            self._send(leg, 1, knee_lift)
            self._send(leg, 2, ankle)
        for leg in push_legs:
            self._send(leg, 1, knee_stance)
            self._send(leg, 2, ankle)
        time.sleep(delay * 2)

        # Sweep hips
        swing_start = {leg: self.angle_vars[leg * 3].get() for leg in swing_legs}
        push_start  = {leg: self.angle_vars[leg * 3].get() for leg in push_legs}
        for step in range(1, steps + 1):
            if not self._running: return
            t = step / steps
            for leg in swing_legs:
                self._send(leg, 0, int(swing_start[leg] + (swing_fwd - swing_start[leg]) * t))
            for leg in push_legs:
                self._send(leg, 0, int(push_start[leg]  + (push_bwd  - push_start[leg])  * t))
            time.sleep(delay)

        # Plant
        for leg in swing_legs:
            self._send(leg, 1, knee_stance)
        time.sleep(delay * 2)

    def _single_phase(self, swing_leg, push_legs, swing_fwd, push_bwd):
        """One leg lifts and swings while the other 3 actively push."""
        p           = gp()
        steps       = p["steps_per_phase"]
        delay       = p["step_delay"]
        knee_stance = p["stance_knee"] + p["body_tilt"]
        knee_lift   = p["lift_knee"]
        ankle       = p["stance_ankle"]

        # Lift the one swing leg
        self._send(swing_leg, 1, knee_lift)
        self._send(swing_leg, 2, ankle)
        # Push legs at stance
        for leg in push_legs:
            self._send(leg, 1, knee_stance)
            self._send(leg, 2, ankle)
        time.sleep(delay * 2)

        # Sweep: swing leg goes fwd, push legs go bwd simultaneously
        swing_start = self.angle_vars[swing_leg * 3].get()
        push_starts = {leg: self.angle_vars[leg * 3].get() for leg in push_legs}
        for step in range(1, steps + 1):
            if not self._running: return
            t = step / steps
            self._send(swing_leg, 0, int(swing_start + (swing_fwd - swing_start) * t))
            for leg in push_legs:
                self._send(leg, 0, int(push_starts[leg] + (push_bwd - push_starts[leg]) * t))
            time.sleep(delay)

        # Plant swing leg
        self._send(swing_leg, 1, knee_stance)
        time.sleep(delay * 2)

    def _wave_phase(self, swing_leg, push_legs, swing_fwd, push_bwd):
        """
        Wave variant: push legs begin their sweep as soon as swing starts lifting,
        creating a smoother flowing motion rather than discrete lift-sweep-plant.
        """
        p           = gp()
        steps       = p["steps_per_phase"]
        delay       = p["step_delay"]
        knee_stance = p["stance_knee"] + p["body_tilt"]
        knee_lift   = p["lift_knee"]
        ankle       = p["stance_ankle"]

        # Lift swing leg and immediately begin hip sweep in same loop
        self._send(swing_leg, 1, knee_lift)
        swing_start = self.angle_vars[swing_leg * 3].get()
        push_starts = {leg: self.angle_vars[leg * 3].get() for leg in push_legs}

        for step in range(1, steps + 1):
            if not self._running: return
            t = step / steps
            self._send(swing_leg, 0, int(swing_start + (swing_fwd - swing_start) * t))
            for leg in push_legs:
                self._send(leg, 0, int(push_starts[leg] + (push_bwd - push_starts[leg]) * t))
            time.sleep(delay)

        # Plant
        self._send(swing_leg, 1, knee_stance)
        time.sleep(delay)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _stance_all(self):
        p = gp()
        for leg in range(4):
            self._send(leg, 0, p["stance_hip"])
            self._send(leg, 1, p["stance_knee"] + p["body_tilt"])
            self._send(leg, 2, p["stance_ankle"])
        self.log("✅ Returned to stance")

    def _send(self, leg, joint, angle):
        angle = max(0, min(180, int(angle)))
        ch    = servo_ch(leg, joint)
        flat  = leg * 3 + joint
        self.angle_vars[flat].set(angle)
        self.serial_mgr.send(f"s {ch} {apply_inversion(ch, angle)}")
        time.sleep(SHARED_PARAMS.get("serial_delay", 0.05))


# ─── Serial Manager ───────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self):
        self.ser    = None
        self.lock   = threading.Lock()
        self.log_cb = None

    def connect(self, port):
        try:
            with self.lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            self._log(f"✅ Connected to {port} @ {BAUD_RATE}")
            threading.Thread(target=self._read_loop, daemon=True).start()
            return True
        except Exception as e:
            self._log(f"❌ {e}")
            return False

    def disconnect(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        self._log("🔌 Disconnected")

    def send(self, cmd: str):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd + "\n").encode())
                self._log(f"→ {cmd}")
            else:
                self._log("⚠️  Not connected")

    def _read_loop(self):
        while self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
                if line:
                    self._log(f"← {line}")
            except:
                break

    def _log(self, msg):
        if self.log_cb:
            self.log_cb(msg)

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open


# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║                         MAIN APPLICATION                                   ║
# ╚══════════════════════════════════════════════════════════════════════════════╝
class L4SRController(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("L4-SR Quadruped Controller")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(1000, 720)

        self.serial_mgr        = SerialManager()
        self.serial_mgr.log_cb = self._append_log

        self.angles        = [tk.IntVar(value=90) for _ in range(NUM_SERVOS)]
        self.sliders       = []
        self.poses         = self._load_poses()
        self._pending_send = {}

        self.gait_engine = GaitEngine(self.serial_mgr, self._append_log, self.angles)

        # Tkinter vars mirroring SHARED_PARAMS and per-gait GAIT_PARAMS
        self._shared_vars = {}
        self._gait_vars   = {}   # {gait_key: {param_key: tk.var}}

        self._build_ui()

    # ── Pose persistence ──────────────────────────────────────────────────────
    def _load_poses(self):
        if os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE) as f:
                    data = json.load(f)
                merged = dict(DEFAULT_POSES)
                merged.update(data)
                return merged
            except:
                pass
        return dict(DEFAULT_POSES)

    def _save_poses_file(self):
        with open(POSE_FILE, "w") as f:
            json.dump(self.poses, f, indent=2)

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        tk.Frame(self, bg=HIGHLIGHT, height=4).pack(fill="x")

        title_frame = tk.Frame(self, bg=BG)
        title_frame.pack(fill="x", padx=16, pady=(10, 0))
        tk.Label(title_frame, text="L4-SR  QUADRUPED  CONTROLLER",
                 font=("Courier", 18, "bold"), bg=BG, fg=HIGHLIGHT).pack(side="left")
        self._conn_label = tk.Label(title_frame, text="● OFFLINE",
                                    font=("Courier", 11, "bold"), bg=BG, fg=YELLOW)
        self._conn_label.pack(side="right")

        self._build_connection_bar()

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=6)

        tab_servo = tk.Frame(nb, bg=BG)
        nb.add(tab_servo, text="  🎛  SERVO CONTROLS  ")

        tab_walk = tk.Frame(nb, bg=BG)
        nb.add(tab_walk, text="  🦿  WALK CYCLE  ")

        tab_tune = tk.Frame(nb, bg=BG)
        nb.add(tab_tune, text="  ⚙  TUNING  ")

        self._build_servo_tab(tab_servo)
        self._build_walk_tab(tab_walk)
        self._build_tuning_tab(tab_tune)

    # ── Connection bar ────────────────────────────────────────────────────────
    def _build_connection_bar(self):
        bar = tk.Frame(self, bg=PANEL, pady=6)
        bar.pack(fill="x", padx=8, pady=(4, 0))

        tk.Label(bar, text="Port:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(10, 4))

        self._port_var   = tk.StringVar()
        ports            = self.serial_mgr.list_ports()
        self._port_combo = ttk.Combobox(bar, textvariable=self._port_var,
                                        values=ports, width=14,
                                        font=("Courier", 10))
        if ports:
            self._port_combo.set(ports[0])
        self._port_combo.pack(side="left", padx=4)

        self._style_btn(bar, "⟳ Refresh", self._refresh_ports, ACCENT).pack(side="left", padx=4)
        self._connect_btn = self._style_btn(bar, "Connect", self._toggle_connect, GREEN)
        self._connect_btn.pack(side="left", padx=4)

        tk.Label(bar, text="Quick:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(20, 4))
        for label, cmd in [("All → 90", "S 90"), ("All → 0", "S 0"), ("All → 180", "S 180")]:
            self._style_btn(bar, label,
                            lambda c=cmd: self.serial_mgr.send(c),
                            ACCENT).pack(side="left", padx=3)

    # ══════════════════════════════════════════════════════════════════════════
    #  TAB 1 — Servo Controls
    # ══════════════════════════════════════════════════════════════════════════
    def _build_servo_tab(self, parent):
        paned = tk.PanedWindow(parent, orient="horizontal", bg=BG,
                               sashwidth=6, sashrelief="flat", bd=0)
        paned.pack(fill="both", expand=True)
        left  = tk.Frame(paned, bg=BG)
        right = tk.Frame(paned, bg=BG)
        paned.add(left,  minsize=560)
        paned.add(right, minsize=280)
        self._build_servo_panel(left)
        self._build_right_panel(right)

    def _build_servo_panel(self, parent):
        header = tk.Frame(parent, bg=BG)
        header.pack(fill="x", pady=(4, 2))
        tk.Label(header, text="SERVO CONTROLS",
                 font=("Courier", 12, "bold"), bg=BG, fg=GREEN).pack(side="left", padx=4)
        tk.Label(header, text="(s [ch] [angle]  →  firmware)",
                 font=("Courier", 9), bg=BG, fg="#557799").pack(side="left", padx=8)

        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        scroll = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner  = tk.Frame(canvas, bg=BG)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        self.sliders = []
        for leg in range(4):
            leg_frame = tk.LabelFrame(inner, text=f"  {LEG_NAMES[leg]}  ",
                                      bg=PANEL, fg=YELLOW,
                                      font=("Courier", 10, "bold"),
                                      bd=1, relief="groove", labelanchor="nw")
            leg_frame.pack(fill="x", padx=6, pady=5)

            for joint in range(3):
                flat = leg * 3 + joint
                ch   = flat_to_channel(flat)
                row  = tk.Frame(leg_frame, bg=PANEL)
                row.pack(fill="x", padx=8, pady=3)

                tk.Label(row, text=f"{JOINT_NAMES[joint]:6s}  #{flat} (ch{ch:02d})",
                         width=18, anchor="w",
                         font=("Courier", 9), bg=PANEL, fg=TEXT).pack(side="left")
                tk.Label(row, textvariable=self.angles[flat],
                         width=4, font=("Courier", 10, "bold"),
                         bg=PANEL, fg=HIGHLIGHT).pack(side="right", padx=(0, 6))
                tk.Label(row, text="°", font=("Courier", 10),
                         bg=PANEL, fg=TEXT).pack(side="right")

                slider = tk.Scale(row, variable=self.angles[flat],
                                  from_=0, to=180, orient="horizontal",
                                  showvalue=False, length=320,
                                  bg=SLIDER_BG, fg=TEXT, troughcolor=ACCENT,
                                  highlightthickness=0, bd=0,
                                  activebackground=HIGHLIGHT,
                                  command=lambda val, i=flat: self._on_slider(i, val))
                slider.pack(side="left", fill="x", expand=True, padx=6)
                self.sliders.append(slider)

    def _build_right_panel(self, parent):
        pose_frame = tk.LabelFrame(parent, text="  POSE MANAGER  ",
                                   bg=PANEL, fg=YELLOW,
                                   font=("Courier", 10, "bold"),
                                   bd=1, relief="groove")
        pose_frame.pack(fill="x", padx=6, pady=6)

        self._pose_var   = tk.StringVar(value=list(self.poses.keys())[0])
        self._pose_combo = ttk.Combobox(pose_frame, textvariable=self._pose_var,
                                        values=list(self.poses.keys()),
                                        font=("Courier", 10), width=18)
        self._pose_combo.pack(padx=8, pady=(8, 4))

        btn_grid = tk.Frame(pose_frame, bg=PANEL)
        btn_grid.pack(padx=8, pady=4)
        self._style_btn(btn_grid, "▶ Move To",  self._move_to_pose,     GREEN    ).grid(row=0, column=0, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "💾 Save As", self._save_pose,         YELLOW   ).grid(row=0, column=1, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "🗑 Delete",  self._delete_pose,       HIGHLIGHT).grid(row=1, column=0, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "⟳ Refresh", self._refresh_pose_list, ACCENT   ).grid(row=1, column=1, padx=4, pady=3, sticky="ew")

        spd_row = tk.Frame(pose_frame, bg=PANEL)
        spd_row.pack(padx=8, pady=(2, 8), fill="x")
        tk.Label(spd_row, text="Transition steps:", bg=PANEL, fg=TEXT,
                 font=("Courier", 9)).pack(side="left")
        self._steps_var = tk.IntVar(value=20)
        tk.Spinbox(spd_row, from_=1, to=100, textvariable=self._steps_var,
                   width=5, font=("Courier", 10),
                   bg=ACCENT, fg=TEXT, insertbackground=TEXT).pack(side="right")

        cmd_frame = tk.LabelFrame(parent, text="  MANUAL COMMAND  ",
                                  bg=PANEL, fg=YELLOW,
                                  font=("Courier", 10, "bold"),
                                  bd=1, relief="groove")
        cmd_frame.pack(fill="x", padx=6, pady=6)

        self._cmd_entry = tk.Entry(cmd_frame, bg=SLIDER_BG, fg=TEXT,
                                   insertbackground=TEXT,
                                   font=("Courier", 11), width=22)
        self._cmd_entry.pack(padx=8, pady=(8, 4))
        self._cmd_entry.bind("<Return>", lambda e: self._send_manual())
        self._style_btn(cmd_frame, "Send", self._send_manual, GREEN).pack(pady=(0, 8))
        tk.Label(cmd_frame, text="e.g.  s 4 90   or   S 90",
                 bg=PANEL, fg="#557799", font=("Courier", 8)).pack(pady=(0, 6))

        log_frame = tk.LabelFrame(parent, text="  SERIAL LOG  ",
                                  bg=PANEL, fg=YELLOW,
                                  font=("Courier", 10, "bold"),
                                  bd=1, relief="groove")
        log_frame.pack(fill="both", expand=True, padx=6, pady=6)

        self._log_text = tk.Text(log_frame, bg=SLIDER_BG, fg=GREEN,
                                 font=("Courier", 8), state="disabled",
                                 wrap="word", height=12)
        log_scroll = ttk.Scrollbar(log_frame, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=log_scroll.set)
        log_scroll.pack(side="right", fill="y")
        self._log_text.pack(fill="both", expand=True, padx=4, pady=4)

        self._style_btn(log_frame, "Clear Log",
                        lambda: (self._log_text.configure(state="normal"),
                                 self._log_text.delete("1.0", "end"),
                                 self._log_text.configure(state="disabled")),
                        ACCENT).pack(pady=(0, 4))

    # ══════════════════════════════════════════════════════════════════════════
    #  TAB 2 — Walk Cycle
    # ══════════════════════════════════════════════════════════════════════════
    def _build_walk_tab(self, parent):
        hdr = tk.Frame(parent, bg=BG)
        hdr.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(hdr, text="WALK CYCLE CONTROL",
                 font=("Courier", 14, "bold"), bg=BG, fg=ORANGE).pack(side="left")

        # ── Gait selector ─────────────────────────────────────────────────────
        gait_frame = tk.LabelFrame(parent, text="  GAIT MODE  ",
                                   bg=PANEL, fg=YELLOW,
                                   font=("Courier", 10, "bold"),
                                   bd=1, relief="groove")
        gait_frame.pack(fill="x", padx=12, pady=(4, 8))

        self._gait_var = tk.StringVar(value=ACTIVE_GAIT)
        btn_row = tk.Frame(gait_frame, bg=PANEL)
        btn_row.pack(pady=10, padx=10)

        self._gait_btns = {}
        for key, info in GAIT_PARAMS.items():
            col_frame = tk.Frame(btn_row, bg=PANEL)
            col_frame.pack(side="left", padx=6)
            btn = tk.Button(col_frame, text=info["label"],
                            font=("Courier", 10, "bold"),
                            width=8, relief="flat", cursor="hand2",
                            command=lambda k=key: self._select_gait(k))
            btn.pack()
            tk.Label(col_frame, text=info["desc"],
                     font=("Courier", 7), bg=PANEL, fg="#557799",
                     wraplength=90, justify="center").pack(pady=(2, 0))
            self._gait_btns[key] = btn

        self._select_gait(ACTIVE_GAIT, silent=True)  # set initial highlight

        # ── Status ────────────────────────────────────────────────────────────
        status_frame = tk.Frame(parent, bg=PANEL, pady=8)
        status_frame.pack(fill="x", padx=12, pady=4)
        tk.Label(status_frame, text="Status:", font=("Courier", 10),
                 bg=PANEL, fg=TEXT).pack(side="left", padx=10)
        self._walk_status = tk.Label(status_frame, text="⬛ STOPPED",
                                     font=("Courier", 12, "bold"),
                                     bg=PANEL, fg=YELLOW)
        self._walk_status.pack(side="left", padx=8)
        self._direction_label = tk.Label(status_frame, text="",
                                         font=("Courier", 10), bg=PANEL, fg=GREEN)
        self._direction_label.pack(side="left", padx=8)

        # ── D-pad ─────────────────────────────────────────────────────────────
        dpad_outer = tk.LabelFrame(parent, text="  DIRECTION  ",
                                   bg=PANEL, fg=YELLOW,
                                   font=("Courier", 10, "bold"),
                                   bd=1, relief="groove")
        dpad_outer.pack(padx=12, pady=8, fill="x")

        dpad = tk.Frame(dpad_outer, bg=PANEL)
        dpad.pack(pady=12)

        btn_cfg = dict(font=("Courier", 13, "bold"), width=10, height=2,
                       relief="flat", cursor="hand2", activebackground=TEXT)

        tk.Button(dpad, text="▲  FORWARD",  bg=GREEN,     fg="#0a0a14",
                  command=lambda: self._walk_start("forward"),    **btn_cfg).grid(row=0, column=1, padx=6, pady=4)
        tk.Button(dpad, text="◄  LEFT",     bg=PURPLE,    fg=TEXT,
                  command=lambda: self._walk_start("turn_left"),  **btn_cfg).grid(row=1, column=0, padx=6, pady=4)
        tk.Button(dpad, text="■  STOP",     bg=HIGHLIGHT, fg=TEXT,
                  command=self._walk_stop,                        **btn_cfg).grid(row=1, column=1, padx=6, pady=4)
        tk.Button(dpad, text="RIGHT  ►",    bg=PURPLE,    fg=TEXT,
                  command=lambda: self._walk_start("turn_right"), **btn_cfg).grid(row=1, column=2, padx=6, pady=4)
        tk.Button(dpad, text="▼  BACKWARD", bg=ORANGE,    fg="#0a0a14",
                  command=lambda: self._walk_start("backward"),   **btn_cfg).grid(row=2, column=1, padx=6, pady=4)

        self.bind("<w>",     lambda e: self._walk_start("forward"))
        self.bind("<s>",     lambda e: self._walk_start("backward"))
        self.bind("<a>",     lambda e: self._walk_start("turn_left"))
        self.bind("<d>",     lambda e: self._walk_start("turn_right"))
        self.bind("<space>", lambda e: self._walk_stop())

        tk.Label(dpad_outer, text="Keyboard:  W A S D  =  move   |   SPACE  =  stop",
                 font=("Courier", 8), bg=PANEL, fg="#557799").pack(pady=(0, 8))

        # ── Live timing sliders ───────────────────────────────────────────────
        timing_frame = tk.LabelFrame(parent, text="  LIVE TIMING  ",
                                     bg=PANEL, fg=YELLOW,
                                     font=("Courier", 10, "bold"),
                                     bd=1, relief="groove")
        timing_frame.pack(padx=12, pady=(4, 8), fill="x")

        def _live_slider(parent, label, get_fn, set_fn, lo, hi):
            row = tk.Frame(parent, bg=PANEL)
            row.pack(fill="x", padx=14, pady=4)
            tk.Label(row, text=label, font=("Courier", 9), bg=PANEL,
                     fg=TEXT, width=22, anchor="w").pack(side="left")
            val_lbl = tk.Label(row, text=f"{get_fn():.3f}s",
                               font=("Courier", 9), bg=PANEL, fg=GREEN, width=7)
            val_lbl.pack(side="right")
            def on_change(v):
                rounded = round(float(v), 3)
                set_fn(rounded)
                val_lbl.config(text=f"{rounded:.3f}s")
            tk.Scale(row, from_=lo, to=hi, resolution=0.005, orient="horizontal",
                     command=on_change, bg=PANEL, fg=TEXT, troughcolor=BG,
                     highlightthickness=0, showvalue=False, length=260,
                     ).pack(side="left", padx=8)

        _live_slider(timing_frame, "serial_delay (hardware)",
                     lambda: SHARED_PARAMS["serial_delay"],
                     lambda v: SHARED_PARAMS.update({"serial_delay": v}),
                     0.005, 0.15)
        _live_slider(timing_frame, "step_delay (sweep pace)",
                     lambda: GAIT_PARAMS[ACTIVE_GAIT]["step_delay"],
                     lambda v: GAIT_PARAMS[ACTIVE_GAIT].update({"step_delay": v}),
                     0.005, 0.2)
        _live_slider(timing_frame, "cycle_pause (between cycles)",
                     lambda: GAIT_PARAMS[ACTIVE_GAIT]["cycle_pause"],
                     lambda v: GAIT_PARAMS[ACTIVE_GAIT].update({"cycle_pause": v}),
                     0.0, 0.3)

        tk.Label(timing_frame,
                 text="step_delay and cycle_pause update the active gait only",
                 font=("Courier", 8), bg=PANEL, fg="#557799").pack(pady=(0, 6))

    # ══════════════════════════════════════════════════════════════════════════
    #  TAB 3 — Tuning
    # ══════════════════════════════════════════════════════════════════════════
    def _build_tuning_tab(self, parent):
        hdr = tk.Frame(parent, bg=BG)
        hdr.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(hdr, text="PARAMETER TUNING",
                 font=("Courier", 14, "bold"), bg=BG, fg=ORANGE).pack(side="left")
        tk.Label(hdr, text="— shared + per-gait settings",
                 font=("Courier", 9), bg=BG, fg="#557799").pack(side="left", padx=8)

        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        scroll = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        inner  = tk.Frame(canvas, bg=BG)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        canvas.pack(fill="both", expand=True)

        # ── Shared params ─────────────────────────────────────────────────────
        shared_defs = {
            "stance_hip":    (0,   180, "Hip angle while standing"),
            "stance_knee":   (0,   180, "Knee angle — lower = taller body"),
            "stance_ankle":  (0,   180, "Ankle angle while standing"),
            "lift_knee":     (0,   130, "Knee angle during lift — lower = higher lift"),
            "turn_offset":   (0,    45, "Hip degree offset for turning"),
            "body_tilt":     (-20,  20, "Global knee bias for body height trim"),
            "serial_delay":  (0.005, 0.15, "Delay after each 's' command (hardware — affects all gaits)"),
            "pose_delay":    (0.005, 0.3,  "Delay between steps when moving to a pose"),
        }
        self._build_param_group(inner, "SHARED  (all gaits)", shared_defs,
                                SHARED_PARAMS, self._shared_vars)

        # ── Per-gait params ───────────────────────────────────────────────────
        gait_defs = {
            "swing_hip_fwd":   (0,   180, "Hip angle at end of forward swing"),
            "swing_hip_bwd":   (0,   180, "Hip angle at start of push-off"),
            "step_delay":      (0.005, 0.3,  "Seconds between interpolation steps"),
            "steps_per_phase": (2,    30,  "Interpolation steps per phase"),
            "cycle_pause":     (0.0,  0.5, "Pause between full gait cycles"),
        }
        for key, info in GAIT_PARAMS.items():
            self._gait_vars[key] = {}
            self._build_param_group(inner,
                                    f"{info['label'].upper()}  — {info['desc']}",
                                    gait_defs, info, self._gait_vars[key],
                                    color=PURPLE)

        # Apply / Reset
        btn_row = tk.Frame(inner, bg=BG)
        btn_row.pack(pady=14)
        self._style_btn(btn_row, "✅  Apply All", self._apply_params, GREEN).pack(side="left", padx=8)
        self._style_btn(btn_row, "↺  Reset All", self._reset_params, YELLOW).pack(side="left", padx=8)

    def _build_param_group(self, parent, title, param_defs, data_dict, var_dict, color=YELLOW):
        grp = tk.LabelFrame(parent, text=f"  {title}  ",
                            bg=PANEL, fg=color,
                            font=("Courier", 10, "bold"),
                            bd=1, relief="groove")
        grp.pack(fill="x", padx=10, pady=8)

        for key, (lo, hi, desc) in param_defs.items():
            if key not in data_dict:
                continue
            row = tk.Frame(grp, bg=PANEL)
            row.pack(fill="x", padx=10, pady=5)

            tk.Label(row, text=key, font=("Courier", 10, "bold"),
                     bg=PANEL, fg=GREEN, anchor="w").pack(fill="x", padx=4)
            tk.Label(row, text=desc, font=("Courier", 7),
                     bg=PANEL, fg="#557799", anchor="w",
                     wraplength=500, justify="left").pack(fill="x", padx=4)

            cur = data_dict[key]
            if isinstance(cur, float):
                var = tk.DoubleVar(value=cur)
                entry_row = tk.Frame(row, bg=PANEL)
                entry_row.pack(fill="x", padx=4, pady=2)
                tk.Entry(entry_row, textvariable=var, bg=SLIDER_BG, fg=TEXT,
                         insertbackground=TEXT, font=("Courier", 11), width=8).pack(side="left")
                tk.Label(entry_row, textvariable=var, width=6,
                         font=("Courier", 10, "bold"), bg=PANEL, fg=HIGHLIGHT).pack(side="left", padx=6)
            else:
                var = tk.IntVar(value=int(cur))
                sl_row = tk.Frame(row, bg=PANEL)
                sl_row.pack(fill="x", padx=4, pady=2)
                tk.Label(sl_row, textvariable=var, width=5,
                         font=("Courier", 11, "bold"), bg=PANEL, fg=HIGHLIGHT).pack(side="right", padx=4)
                tk.Scale(sl_row, variable=var, from_=int(lo), to=int(hi),
                         orient="horizontal", showvalue=False, length=300,
                         bg=SLIDER_BG, fg=TEXT, troughcolor=ACCENT,
                         highlightthickness=0, bd=0,
                         activebackground=ORANGE).pack(side="left", fill="x", expand=True)

            var_dict[key] = var

    # ── Gait selection ────────────────────────────────────────────────────────
    def _select_gait(self, key, silent=False):
        global ACTIVE_GAIT
        ACTIVE_GAIT = key
        for k, btn in self._gait_btns.items():
            if k == key:
                btn.configure(bg=ORANGE, fg="#0a0a14")
            else:
                btn.configure(bg=ACCENT, fg=TEXT)
        if not silent:
            info = GAIT_PARAMS[key]
            self._append_log(f"🔄 Gait → {info['label']}  ({info['desc']})")

    # ── Walk engine controls ──────────────────────────────────────────────────
    def _walk_start(self, direction: str):
        if self.gait_engine.running:
            self.gait_engine.stop()
            time.sleep(0.05)
        self._apply_params(silent=True)
        self.gait_engine.start(direction)
        labels = {"forward": "▲ FORWARD", "backward": "▼ BACKWARD",
                  "turn_left": "◄ TURN LEFT", "turn_right": "► TURN RIGHT"}
        self._walk_status.configure(text="🟢 WALKING", fg=GREEN)
        self._direction_label.configure(text=labels.get(direction, direction))

    def _walk_stop(self):
        self.gait_engine.stop()
        self._walk_status.configure(text="⬛ STOPPED", fg=YELLOW)
        self._direction_label.configure(text="")

    # ── Param sync ────────────────────────────────────────────────────────────
    def _apply_params(self, silent=False):
        for key, var in self._shared_vars.items():
            try: SHARED_PARAMS[key] = var.get()
            except: pass
        for gait_key, vars_dict in self._gait_vars.items():
            for key, var in vars_dict.items():
                try: GAIT_PARAMS[gait_key][key] = var.get()
                except: pass
        if not silent:
            self._append_log("✅ All parameters applied")

    def _reset_params(self):
        defaults_shared = {
            "stance_hip": 90, "stance_knee": 75, "stance_ankle": 100,
            "lift_knee": 45, "turn_offset": 15, "body_tilt": 0,
            "serial_delay": 0.05, "pose_delay": 0.05,
        }
        defaults_gaits = {
            "trot":  {"swing_hip_fwd": 110, "swing_hip_bwd": 70, "step_delay": 0.04, "steps_per_phase": 8,  "cycle_pause": 0.02},
            "crawl": {"swing_hip_fwd": 110, "swing_hip_bwd": 70, "step_delay": 0.06, "steps_per_phase": 10, "cycle_pause": 0.04},
            "wave":  {"swing_hip_fwd": 112, "swing_hip_bwd": 68, "step_delay": 0.05, "steps_per_phase": 10, "cycle_pause": 0.02},
            "pace":  {"swing_hip_fwd": 108, "swing_hip_bwd": 72, "step_delay": 0.04, "steps_per_phase": 8,  "cycle_pause": 0.03},
            "bound": {"swing_hip_fwd": 120, "swing_hip_bwd": 60, "step_delay": 0.03, "steps_per_phase": 6,  "cycle_pause": 0.01},
        }
        for k, v in defaults_shared.items():
            SHARED_PARAMS[k] = v
            if k in self._shared_vars: self._shared_vars[k].set(v)
        for gait_key, defs in defaults_gaits.items():
            for k, v in defs.items():
                GAIT_PARAMS[gait_key][k] = v
                if gait_key in self._gait_vars and k in self._gait_vars[gait_key]:
                    self._gait_vars[gait_key][k].set(v)
        self._append_log("↺ All parameters reset to defaults")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _style_btn(self, parent, text, cmd, color):
        return tk.Button(parent, text=text, command=cmd,
                         bg=color, fg="#0a0a14",
                         font=("Courier", 9, "bold"),
                         relief="flat", cursor="hand2",
                         activebackground=TEXT,
                         padx=8, pady=4)

    def _append_log(self, msg):
        def _do():
            self._log_text.configure(state="normal")
            self._log_text.insert("end", msg + "\n")
            self._log_text.see("end")
            self._log_text.configure(state="disabled")
        self.after(0, _do)

    def _refresh_ports(self):
        ports = self.serial_mgr.list_ports()
        self._port_combo["values"] = ports
        if ports: self._port_combo.set(ports[0])

    def _toggle_connect(self):
        if self.serial_mgr.connected:
            self.serial_mgr.disconnect()
            self._connect_btn.configure(text="Connect", bg=GREEN)
            self._conn_label.configure(text="● OFFLINE", fg=YELLOW)
        else:
            port = self._port_var.get()
            if not port:
                messagebox.showwarning("No Port", "Select a COM port first.")
                return
            ok = self.serial_mgr.connect(port)
            if ok:
                self._connect_btn.configure(text="Disconnect", bg=HIGHLIGHT)
                self._conn_label.configure(text=f"● {port}", fg=GREEN)

    def _on_slider(self, flat_idx, val):
        if flat_idx in self._pending_send:
            self.after_cancel(self._pending_send[flat_idx])
        self._pending_send[flat_idx] = self.after(40, lambda: self._send_servo(flat_idx))

    def _send_servo(self, flat_idx):
        angle = self.angles[flat_idx].get()
        ch    = flat_to_channel(flat_idx)
        self.serial_mgr.send(f"s {ch} {apply_inversion(ch, angle)}")

    def _move_to_pose(self):
        name = self._pose_var.get()
        if name not in self.poses:
            messagebox.showerror("Error", f"Pose '{name}' not found.")
            return
        target = self.poses[name]
        steps  = max(1, self._steps_var.get())
        def run():
            starts = [self.angles[i].get() for i in range(NUM_SERVOS)]
            for step in range(1, steps + 1):
                for i in range(NUM_SERVOS):
                    a  = int(starts[i] + (target[i] - starts[i]) * step / steps)
                    self.angles[i].set(a)
                    ch = flat_to_channel(i)
                    self.serial_mgr.send(f"s {ch} {apply_inversion(ch, a)}")
                time.sleep(SHARED_PARAMS.get("pose_delay", 0.05))
            self._append_log(f"✅ Arrived at pose: {name}")
        threading.Thread(target=run, daemon=True).start()

    def _save_pose(self):
        win = tk.Toplevel(self, bg=PANEL)
        win.title("Save Pose")
        win.resizable(False, False)
        tk.Label(win, text="Pose name:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(padx=12, pady=(10, 2))
        entry = tk.Entry(win, bg=SLIDER_BG, fg=TEXT, insertbackground=TEXT,
                         font=("Courier", 11))
        entry.insert(0, self._pose_var.get())
        entry.pack(padx=12, pady=4)
        def confirm():
            n = entry.get().strip()
            if not n: return
            self.poses[n] = [self.angles[i].get() for i in range(NUM_SERVOS)]
            self._save_poses_file()
            self._refresh_pose_list()
            self._pose_var.set(n)
            self._append_log(f"💾 Saved pose: {n}")
            win.destroy()
        self._style_btn(win, "Save", confirm, GREEN).pack(pady=(4, 12))
        entry.bind("<Return>", lambda e: confirm())

    def _delete_pose(self):
        n = self._pose_var.get()
        if n in DEFAULT_POSES:
            messagebox.showwarning("Protected", f"Cannot delete built-in pose '{n}'.")
            return
        if n in self.poses:
            if messagebox.askyesno("Delete", f"Delete pose '{n}'?"):
                del self.poses[n]
                self._save_poses_file()
                self._refresh_pose_list()
                self._append_log(f"🗑 Deleted pose: {n}")

    def _refresh_pose_list(self):
        names = list(self.poses.keys())
        self._pose_combo["values"] = names
        if names: self._pose_var.set(names[0])

    def _send_manual(self):
        cmd = self._cmd_entry.get().strip()
        if cmd:
            self.serial_mgr.send(cmd)
            self._cmd_entry.delete(0, "end")

    def destroy(self):
        if self.gait_engine.running:
            self.gait_engine.stop()
        super().destroy()


# ─── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app   = L4SRController()
    style = ttk.Style(app)
    style.theme_use("clam")
    style.configure("TScrollbar",  background=ACCENT, troughcolor=BG, borderwidth=0)
    style.configure("TCombobox",   fieldbackground=SLIDER_BG, background=ACCENT,
                    foreground=TEXT, selectbackground=ACCENT, arrowcolor=TEXT)
    style.configure("TNotebook",   background=BG, borderwidth=0)
    style.configure("TNotebook.Tab", background=ACCENT, foreground=TEXT,
                    font=("Courier", 10, "bold"), padding=(10, 6))
    style.map("TNotebook.Tab",
              background=[("selected", PANEL)],
              foreground=[("selected", ORANGE)])
    app.mainloop()