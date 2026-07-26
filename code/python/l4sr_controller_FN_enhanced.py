"""
L4-SR Quadruped Python Controller
====================================
Connects via USB Serial to the L4-SR ESP32.
Sends commands in the firmware's native format:
  - Individual servo:  s [0-11] [0-180]
  - All servos:        S [0-180]

The firmware's serial-control mode overrides ESP-NOW for 5 seconds
after each serial command, then reverts to joystick automatically.

Servo channel map (matches L4-SR.ino servo_pin[4][3]):
  Leg 1 (FL): ch 0, 4, 8   (hip, knee, ankle)
  Leg 2 (FR): ch 1, 5, 9
  Leg 3 (BL): ch 2, 6, 10
  Leg 4 (BR): ch 3, 7, 11
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

WALK_PARAMS = {
    # ── Stance (robot standing still between steps) ───────────────────────────
    "stance_hip":    90,    # hip servo angle while standing  (0-180)
    "stance_knee":   75,    # knee angle while standing — lower = taller body
    "stance_ankle":  100,   # ankle angle while standing
 
    # ── Swing phase (foot in the air) ─────────────────────────────────────────
    "lift_ankle":    60,    # ankle angle during lift (raises foot)
    "swing_hip_fwd": 110,   # hip angle at end of forward swing
    "swing_hip_bwd": 70,    # hip angle at start of backward push
 
    # ── Timing ────────────────────────────────────────────────────────────────
    "step_delay":    0.04,  # seconds between each serial command inside a step
    "steps_per_phase": 8,   # interpolation steps per swing/push phase
    "cycle_pause":   0.02,  # pause between full gait cycles (seconds)
 
    # ── Direction offsets applied to hip angles ───────────────────────────────
    # Positive = turn right,  Negative = turn left
    "turn_offset":   15,    # degrees added/subtracted for turning
 
    # ── Body tilt offsets (experimental) ─────────────────────────────────────
    "body_tilt":     0,     # global knee bias for body height trim
}
# ─── Constants ───────────────────────────────────────────────────────────────
BAUD_RATE     = 115200
NUM_SERVOS    = 12
POSE_FILE     = "l4sr_poses.json"

LEG_NAMES     = ["FL (Front-Left)", "FR (Front-Right)", "BL (Back-Left)", "BR (Back-Right)"]
JOINT_NAMES   = ["Hip", "Knee", "Ankle"]

# Leg index constants for readability
FL, FR, BL, BR = 0, 1, 2, 3
 
def flat_to_channel(flat_idx):
    leg   = flat_idx // 3
    joint = flat_idx % 3
    return joint * 4 + leg
 
def servo_ch(leg, joint):
    """Return physical channel for a given leg (0-3) and joint (0=hip,1=knee,2=ankle)."""
    return joint * 4 + leg
 
# channel index = leg*3 + joint  (matches firmware servo_pin layout transposed to flat list)
# servo_pin[leg][joint]:  0,4,8 | 1,5,9 | 2,6,10 | 3,7,11
# flat index = leg*3+joint, but physical channel = joint*4+leg

# Default poses (angles for flat indices 0-11)
DEFAULT_POSES = {
    "Neutral":  [90]*12,
    "Sit":      [90, 45, 45,  90, 135, 135,  90, 45, 45,  90, 135, 135],
    "Stretch":  [90,135,160, 90,45,20, 90,135,160, 90,45,20],
    "Stand":    [90, 60, 120]*4,
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
# ║                          WALK ENGINE                                       ║
# ╚══════════════════════════════════════════════════════════════════════════════╝
class WalkEngine:
    """
    Freenove-style diagonal trot gait.
 
    Gait sequence (matches Freenove quadruped logic):
      Cycle start → stance all legs
      Phase A: FL + BR swing  (lift, swing forward, plant)
               FR + BL push   (hip sweeps backward)
      Phase B: FR + BL swing  (lift, swing forward, plant)
               FL + BR push   (hip sweeps backward)
 
    Direction can be FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT.
    """
 
    def __init__(self, serial_mgr, log_cb, angle_vars):
        self.serial_mgr  = serial_mgr
        self.log         = log_cb
        self.angle_vars  = angle_vars   # list of 12 tk.IntVar for UI sync
 
        self._running    = False
        self._thread     = None
        self._direction  = "forward"
        self._lock       = threading.Lock()
 
    # ── Public control ────────────────────────────────────────────────────────
    def start(self, direction="forward"):
        with self._lock:
            if self._running:
                return
            self._direction = direction
            self._running   = True
        self._thread = threading.Thread(target=self._walk_loop, daemon=True)
        self._thread.start()
        self.log(f"🦿 Walk started  [{direction}]")
 
    def stop(self):
        with self._lock:
            self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.log("🛑 Walk stopped")
 
    @property
    def running(self):
        return self._running
 
    # ── Internal gait loop ────────────────────────────────────────────────────
    def _walk_loop(self):
        p = WALK_PARAMS          # shorthand
 
        # Compute per-leg hip targets based on direction
        # FL=0 FR=1 BL=2 BR=3
        fwd_ang  = p["swing_hip_fwd"]
        bwd_ang  = p["swing_hip_bwd"]
        neutral  = p["stance_hip"]
        offset   = p["turn_offset"]
 
        while self._running:
            direction = self._direction
 
            if direction == "forward":
                # Phase A swing legs: FL + BR  → fwd hip
                # Phase A push legs:  FR + BL  → bwd hip
                self._trot_phase(
                    swing_legs=[FL, BR], push_legs=[FR, BL],
                    swing_fwd=fwd_ang, push_bwd=bwd_ang
                )
                if not self._running:
                    break
                # Phase B swing legs: FR + BL  → fwd hip
                # Phase B push legs:  FL + BR  → bwd hip
                self._trot_phase(
                    swing_legs=[FR, BL], push_legs=[FL, BR],
                    swing_fwd=fwd_ang, push_bwd=bwd_ang
                )
 
            elif direction == "backward":
                # Reverse: fwd/bwd hip angles swapped
                self._trot_phase(
                    swing_legs=[FL, BR], push_legs=[FR, BL],
                    swing_fwd=bwd_ang, push_bwd=fwd_ang
                )
                if not self._running:
                    break
                self._trot_phase(
                    swing_legs=[FR, BL], push_legs=[FL, BR],
                    swing_fwd=bwd_ang, push_bwd=fwd_ang
                )
 
            elif direction == "turn_left":
                # Left legs push harder, right legs swing wider
                self._trot_phase(
                    swing_legs=[FL, BR], push_legs=[FR, BL],
                    swing_fwd=fwd_ang - offset, push_bwd=bwd_ang + offset
                )
                if not self._running:
                    break
                self._trot_phase(
                    swing_legs=[FR, BL], push_legs=[FL, BR],
                    swing_fwd=fwd_ang + offset, push_bwd=bwd_ang - offset
                )
 
            elif direction == "turn_right":
                self._trot_phase(
                    swing_legs=[FL, BR], push_legs=[FR, BL],
                    swing_fwd=fwd_ang + offset, push_bwd=bwd_ang - offset
                )
                if not self._running:
                    break
                self._trot_phase(
                    swing_legs=[FR, BL], push_legs=[FL, BR],
                    swing_fwd=fwd_ang - offset, push_bwd=bwd_ang + offset
                )
 
            time.sleep(WALK_PARAMS["cycle_pause"])
 
        # Return to neutral stance on stop
        self._send_stance_all()
 
    def _trot_phase(self, swing_legs, push_legs, swing_fwd, push_bwd):
        """
        Execute one half-cycle:
          1. Lift swing legs (ankle up)
          2. Interpolate swing hips forward  +  push hips backward  simultaneously
          3. Plant swing legs (ankle down)
        """
        p      = WALK_PARAMS
        steps  = p["steps_per_phase"]
        delay  = p["step_delay"]
        knee   = p["stance_knee"] + p["body_tilt"]
        ankle_lift  = p["lift_ankle"]
        ankle_plant = p["stance_ankle"]
 
        # Step 1 — Lift swing legs (raise ankle, keep knee at stance)
        for leg in swing_legs:
            self._send(leg, 1, knee)         # knee to stance height
            self._send(leg, 2, ankle_lift)   # ankle up
        # Ensure push legs are planted at stance knee
        for leg in push_legs:
            self._send(leg, 1, knee)         # knee to stance height
            self._send(leg, 2, ankle_plant)  # ankle planted
 
        time.sleep(delay * 2)
 
        # Step 2 — Sweep hips (swing fwd, push bwd) simultaneously
        # Read current hip angles as starting points
        swing_start = {leg: self.angle_vars[leg * 3].get() for leg in swing_legs}
        push_start  = {leg: self.angle_vars[leg * 3].get() for leg in push_legs}
 
        for step in range(1, steps + 1):
            if not self._running:
                return
            t = step / steps  # 0→1
            for leg in swing_legs:
                ang = int(swing_start[leg] + (swing_fwd - swing_start[leg]) * t)
                self._send(leg, 0, ang)   # hip
            for leg in push_legs:
                ang = int(push_start[leg] + (push_bwd - push_start[leg]) * t)
                self._send(leg, 0, ang)   # hip
            time.sleep(delay)
 
        # Step 3 — Plant swing legs (ankle down, knee to stance)
        for leg in swing_legs:
            self._send(leg, 1, knee)         # knee back to stance
            self._send(leg, 2, ankle_plant)  # ankle down
 
        time.sleep(delay * 2)
 
    def _send_stance_all(self):
        """Return all legs to neutral standing position."""
        p = WALK_PARAMS
        for leg in range(4):
            self._send(leg, 0, p["stance_hip"])
            self._send(leg, 1, p["stance_knee"] + p["body_tilt"])
            self._send(leg, 2, p["stance_ankle"])
        self.log("✅ Returned to stance")
 
    def _send(self, leg, joint, angle):
        """Send one servo command and update UI angle variable."""
        angle = max(0, min(180, angle))
        ch    = servo_ch(leg, joint)
        flat  = leg * 3 + joint
        self.angle_vars[flat].set(angle)
        self.serial_mgr.send(f"s {ch} {angle}")
 
 
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
 
 
# ─── Main Application ─────────────────────────────────────────────────────────
class L4SRController(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("L4-SR Quadruped Controller  —  Walk Cycle Edition")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(1000, 720)
 
        self.serial_mgr        = SerialManager()
        self.serial_mgr.log_cb = self._append_log
 
        self.angles            = [tk.IntVar(value=90) for _ in range(NUM_SERVOS)]
        self.sliders           = []
        self.poses             = self._load_poses()
        self._pending_send     = {}
 
        # Walk engine
        self.walk_engine = WalkEngine(self.serial_mgr, self._append_log, self.angles)
 
        # Walk param tk vars (mirrors WALK_PARAMS for live editing)
        self._wp_vars = {}
 
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
 
    # ── UI Construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        # Top accent bar
        tk.Frame(self, bg=HIGHLIGHT, height=4).pack(fill="x")
 
        # Title
        title_frame = tk.Frame(self, bg=BG)
        title_frame.pack(fill="x", padx=16, pady=(10, 0))
        tk.Label(title_frame, text="L4-SR  QUADRUPED  CONTROLLER",
                 font=("Courier", 18, "bold"), bg=BG, fg=HIGHLIGHT).pack(side="left")
        self._conn_label = tk.Label(title_frame, text="● OFFLINE",
                                    font=("Courier", 11, "bold"), bg=BG, fg=YELLOW)
        self._conn_label.pack(side="right")
 
        self._build_connection_bar()
 
        # Notebook — tabs
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=6)
 
        # Tab 1: Servo sliders + pose manager
        tab_servo = tk.Frame(nb, bg=BG)
        nb.add(tab_servo, text="  🎛  SERVO CONTROLS  ")
 
        # Tab 2: Walk cycle
        tab_walk = tk.Frame(nb, bg=BG)
        nb.add(tab_walk, text="  🦿  WALK CYCLE  ")
 
        # Tab 3: Walk tuning
        tab_tune = tk.Frame(nb, bg=BG)
        nb.add(tab_tune, text="  ⚙  WALK TUNING  ")
 
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
 
        self._style_btn(bar, "⟳ Refresh", self._refresh_ports,
                        ACCENT).pack(side="left", padx=4)
        self._connect_btn = self._style_btn(bar, "Connect",
                                            self._toggle_connect, GREEN)
        self._connect_btn.pack(side="left", padx=4)
 
        tk.Label(bar, text="Quick:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(20, 4))
        for label, cmd in [("All → 90", "S 90"), ("All → 0", "S 0"),
                            ("All → 180", "S 180")]:
            self._style_btn(bar, label,
                            lambda c=cmd: self.serial_mgr.send(c),
                            ACCENT).pack(side="left", padx=3)
 
    # ══════════════════════════════════════════════════════════════════════════
    #  TAB 1 — Servo sliders
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
        tk.Label(header, text="(s [num] [angle]  →  firmware)",
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
        # Pose manager
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
        self._style_btn(btn_grid, "▶ Move To",  self._move_to_pose,      GREEN    ).grid(row=0, column=0, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "💾 Save As", self._save_pose,          YELLOW   ).grid(row=0, column=1, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "🗑 Delete",  self._delete_pose,        HIGHLIGHT).grid(row=1, column=0, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "⟳ Refresh", self._refresh_pose_list,  ACCENT   ).grid(row=1, column=1, padx=4, pady=3, sticky="ew")
 
        spd_row = tk.Frame(pose_frame, bg=PANEL)
        spd_row.pack(padx=8, pady=(2, 8), fill="x")
        tk.Label(spd_row, text="Transition steps:", bg=PANEL, fg=TEXT,
                 font=("Courier", 9)).pack(side="left")
        self._steps_var = tk.IntVar(value=20)
        tk.Spinbox(spd_row, from_=1, to=100, textvariable=self._steps_var,
                   width=5, font=("Courier", 10),
                   bg=ACCENT, fg=TEXT, insertbackground=TEXT).pack(side="right")
 
        # Manual command
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
 
        # Serial log
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
    #  TAB 2 — Walk Cycle Controls
    # ══════════════════════════════════════════════════════════════════════════
    def _build_walk_tab(self, parent):
        # Header
        hdr = tk.Frame(parent, bg=BG)
        hdr.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(hdr, text="WALK CYCLE CONTROL",
                 font=("Courier", 14, "bold"), bg=BG, fg=ORANGE).pack(side="left")
        tk.Label(hdr, text="— Freenove diagonal trot gait",
                 font=("Courier", 9), bg=BG, fg="#557799").pack(side="left", padx=8)
 
        # Status indicator
        status_frame = tk.Frame(parent, bg=PANEL, pady=8)
        status_frame.pack(fill="x", padx=12, pady=4)
 
        tk.Label(status_frame, text="Gait Status:",
                 font=("Courier", 10), bg=PANEL, fg=TEXT).pack(side="left", padx=10)
        self._walk_status = tk.Label(status_frame, text="⬛ STOPPED",
                                     font=("Courier", 12, "bold"),
                                     bg=PANEL, fg=YELLOW)
        self._walk_status.pack(side="left", padx=8)
 
        self._direction_label = tk.Label(status_frame, text="",
                                         font=("Courier", 10), bg=PANEL, fg=GREEN)
        self._direction_label.pack(side="left", padx=8)
 
        # ── Direction Buttons (big D-pad style) ───────────────────────────────
        dpad_outer = tk.LabelFrame(parent, text="  DIRECTION  ",
                                   bg=PANEL, fg=YELLOW,
                                   font=("Courier", 10, "bold"),
                                   bd=1, relief="groove")
        dpad_outer.pack(padx=12, pady=8, fill="x")
 
        dpad = tk.Frame(dpad_outer, bg=PANEL)
        dpad.pack(pady=12)
 
        btn_cfg = dict(font=("Courier", 13, "bold"), width=10, height=2,
                       relief="flat", cursor="hand2", activebackground=TEXT)
 
        # Forward
        tk.Button(dpad, text="▲  FORWARD",
                  bg=GREEN, fg="#0a0a14",
                  command=lambda: self._walk_start("forward"),
                  **btn_cfg).grid(row=0, column=1, padx=6, pady=4)
 
        # Left / Stop / Right
        tk.Button(dpad, text="◄  LEFT",
                  bg=PURPLE, fg=TEXT,
                  command=lambda: self._walk_start("turn_left"),
                  **btn_cfg).grid(row=1, column=0, padx=6, pady=4)
 
        self._stop_btn = tk.Button(dpad, text="■  STOP",
                                   bg=HIGHLIGHT, fg=TEXT,
                                   command=self._walk_stop,
                                   **btn_cfg)
        self._stop_btn.grid(row=1, column=1, padx=6, pady=4)
 
        tk.Button(dpad, text="RIGHT  ►",
                  bg=PURPLE, fg=TEXT,
                  command=lambda: self._walk_start("turn_right"),
                  **btn_cfg).grid(row=1, column=2, padx=6, pady=4)
 
        # Backward
        tk.Button(dpad, text="▼  BACKWARD",
                  bg=ORANGE, fg="#0a0a14",
                  command=lambda: self._walk_start("backward"),
                  **btn_cfg).grid(row=2, column=1, padx=6, pady=4)
 
        # Keyboard bindings
        self.bind("<w>",      lambda e: self._walk_start("forward"))
        self.bind("<s>",      lambda e: self._walk_start("backward"))
        self.bind("<a>",      lambda e: self._walk_start("turn_left"))
        self.bind("<d>",      lambda e: self._walk_start("turn_right"))
        self.bind("<space>",  lambda e: self._walk_stop())
        self.bind("<Return>", lambda e: self._walk_stop())
 
        tk.Label(dpad_outer, text="Keyboard:  W / A / S / D  =  move   |   SPACE / ENTER  =  stop",
                 font=("Courier", 8), bg=PANEL, fg="#557799").pack(pady=(0, 8))
 
        # ── Gait phase diagram ────────────────────────────────────────────────
        phase_frame = tk.LabelFrame(parent, text="  GAIT PHASE DIAGRAM  ",
                                    bg=PANEL, fg=YELLOW,
                                    font=("Courier", 10, "bold"),
                                    bd=1, relief="groove")
        phase_frame.pack(padx=12, pady=8, fill="x")
 
        diagram = tk.Frame(phase_frame, bg=PANEL)
        diagram.pack(padx=12, pady=10)
 
        # Column headers
        for col, txt in enumerate(["LEG", "Phase A (swing)", "Phase B (push)"]):
            tk.Label(diagram, text=txt, font=("Courier", 9, "bold"),
                     bg=PANEL, fg=YELLOW, width=18).grid(row=0, column=col, padx=4, pady=2)
 
        gait_rows = [
            ("FL (Front-Left)",  "↑ SWING → fwd",   "↓ push ← bwd"),
            ("FR (Front-Right)", "↓ push ← bwd",    "↑ SWING → fwd"),
            ("BL (Back-Left)",   "↓ push ← bwd",    "↑ SWING → fwd"),
            ("BR (Back-Right)",  "↑ SWING → fwd",   "↓ push ← bwd"),
        ]
        for row_i, (leg, phA, phB) in enumerate(gait_rows, start=1):
            fg_a = GREEN   if "SWING" in phA else "#557799"
            fg_b = GREEN   if "SWING" in phB else "#557799"
            tk.Label(diagram, text=leg,  font=("Courier", 9), bg=PANEL,
                     fg=TEXT,  width=18).grid(row=row_i, column=0, padx=4, pady=1)
            tk.Label(diagram, text=phA,  font=("Courier", 9), bg=PANEL,
                     fg=fg_a, width=18).grid(row=row_i, column=1, padx=4, pady=1)
            tk.Label(diagram, text=phB,  font=("Courier", 9), bg=PANEL,
                     fg=fg_b, width=18).grid(row=row_i, column=2, padx=4, pady=1)
 
        tk.Label(phase_frame,
                 text="Diagonal pairs move together  |  FL+BR  ↔  FR+BL",
                 font=("Courier", 8), bg=PANEL, fg="#557799").pack(pady=(0, 8))
 
    # ══════════════════════════════════════════════════════════════════════════
    #  TAB 3 — Walk Tuning
    # ══════════════════════════════════════════════════════════════════════════
    def _build_tuning_tab(self, parent):
        hdr = tk.Frame(parent, bg=BG)
        hdr.pack(fill="x", padx=12, pady=(10, 4))
        tk.Label(hdr, text="WALK PARAMETER TUNING",
                 font=("Courier", 14, "bold"), bg=BG, fg=ORANGE).pack(side="left")
        tk.Label(hdr, text="— changes apply to next walk cycle",
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
 
        # ── Parameter groups ──────────────────────────────────────────────────
        groups = {
            "STANCE ANGLES  (robot at rest)": {
                "stance_hip":   (0,  180, "Hip angle while standing"),
                "stance_knee":  (0,  180, "Knee angle — lower value = taller body"),
                "stance_ankle": (0,  180, "Ankle angle while standing"),
            },
            "SWING ANGLES  (foot in air)": {
                "lift_ankle":    (0,  180, "Ankle angle during lift  (raises foot)"),
                "swing_hip_fwd": (0,  180, "Hip angle at END of forward swing"),
                "swing_hip_bwd": (0,  180, "Hip angle at START of push-off"),
            },
            "TIMING": {
                "step_delay":      (0.005, 0.2,  "Seconds between serial commands (float)"),
                "steps_per_phase": (2,     30,   "Interpolation steps per swing phase"),
                "cycle_pause":     (0.0,   0.5,  "Pause between full gait cycles (float)"),
            },
            "DIRECTION & TRIM": {
                "turn_offset": (0,  45,  "Hip degree offset for turning"),
                "body_tilt":   (-20, 20, "Knee bias for body height trim"),
            },
        }
 
        for group_name, params in groups.items():
            grp = tk.LabelFrame(inner, text=f"  {group_name}  ",
                                bg=PANEL, fg=YELLOW,
                                font=("Courier", 10, "bold"),
                                bd=1, relief="groove")
            grp.pack(fill="x", padx=10, pady=8)
 
            for key, (lo, hi, desc) in params.items():
                row = tk.Frame(grp, bg=PANEL)
                row.pack(fill="x", padx=10, pady=5)
 
                # Label + description
                lbl_frame = tk.Frame(row, bg=PANEL, width=260)
                lbl_frame.pack(side="left")
                lbl_frame.pack_propagate(False)
                tk.Label(lbl_frame, text=key, font=("Courier", 10, "bold"),
                         bg=PANEL, fg=GREEN, anchor="w").pack(fill="x")
                tk.Label(lbl_frame, text=desc, font=("Courier", 7),
                         bg=PANEL, fg="#557799", anchor="w").pack(fill="x")
 
                cur_val = WALK_PARAMS[key]
 
                # Float params: use Entry
                if isinstance(cur_val, float):
                    var = tk.DoubleVar(value=cur_val)
                    entry = tk.Entry(row, textvariable=var,
                                     bg=SLIDER_BG, fg=TEXT,
                                     insertbackground=TEXT,
                                     font=("Courier", 11), width=8)
                    entry.pack(side="left", padx=10)
                    val_display = tk.Label(row, textvariable=var,
                                           width=6, font=("Courier", 10, "bold"),
                                           bg=PANEL, fg=HIGHLIGHT)
                    val_display.pack(side="left")
                else:
                    var = tk.IntVar(value=int(cur_val))
                    val_lbl = tk.Label(row, textvariable=var,
                                       width=5, font=("Courier", 11, "bold"),
                                       bg=PANEL, fg=HIGHLIGHT)
                    val_lbl.pack(side="right", padx=(0, 6))
                    tk.Label(row, text="°" if "angle" in key or "hip" in key
                                           or "knee" in key or "ankle" in key
                                           or "offset" in key or "tilt" in key
                                       else "",
                             font=("Courier", 10), bg=PANEL, fg=TEXT).pack(side="right")
 
                    slider = tk.Scale(row, variable=var,
                                      from_=int(lo), to=int(hi),
                                      orient="horizontal", showvalue=False,
                                      length=300,
                                      bg=SLIDER_BG, fg=TEXT, troughcolor=ACCENT,
                                      highlightthickness=0, bd=0,
                                      activebackground=ORANGE)
                    slider.pack(side="left", fill="x", expand=True, padx=6)
 
                self._wp_vars[key] = var
 
        # Apply / Reset buttons
        btn_row = tk.Frame(inner, bg=BG)
        btn_row.pack(pady=14)
 
        self._style_btn(btn_row, "✅  Apply Parameters",
                        self._apply_walk_params, GREEN).pack(side="left", padx=8)
        self._style_btn(btn_row, "↺  Reset to Defaults",
                        self._reset_walk_params, YELLOW).pack(side="left", padx=8)
 
        tk.Label(inner,
                 text="Tip: stop the walk cycle before applying parameters "
                      "if you want changes to take effect immediately.",
                 font=("Courier", 8), bg=BG, fg="#557799",
                 wraplength=600, justify="left").pack(padx=10, pady=(0, 12))
 
    # ── Walk engine controls ──────────────────────────────────────────────────
    def _walk_start(self, direction: str):
        if self.walk_engine.running:
            self.walk_engine.stop()
            time.sleep(0.05)
 
        self._apply_walk_params(silent=True)   # sync UI tuning vars first
        self.walk_engine.start(direction)
 
        labels = {
            "forward":    "▲ FORWARD",
            "backward":   "▼ BACKWARD",
            "turn_left":  "◄ TURN LEFT",
            "turn_right": "► TURN RIGHT",
        }
        self._walk_status.configure(text="🟢 WALKING", fg=GREEN)
        self._direction_label.configure(text=labels.get(direction, direction))
 
    def _walk_stop(self):
        self.walk_engine.stop()
        self._walk_status.configure(text="⬛ STOPPED", fg=YELLOW)
        self._direction_label.configure(text="")
 
    # ── Tuning param sync ─────────────────────────────────────────────────────
    def _apply_walk_params(self, silent=False):
        """Push UI values → WALK_PARAMS dict (read by WalkEngine)."""
        for key, var in self._wp_vars.items():
            try:
                WALK_PARAMS[key] = var.get()
            except Exception:
                pass
        if not silent:
            self._append_log("✅ Walk parameters applied")
 
    def _reset_walk_params(self):
        defaults = {
            "stance_hip": 90, "stance_knee": 75, "stance_ankle": 100,
            "lift_ankle": 60, "swing_hip_fwd": 110, "swing_hip_bwd": 70,
            "step_delay": 0.04, "steps_per_phase": 8, "cycle_pause": 0.02,
            "turn_offset": 15, "body_tilt": 0,
        }
        for key, val in defaults.items():
            WALK_PARAMS[key] = val
            if key in self._wp_vars:
                self._wp_vars[key].set(val)
        self._append_log("↺ Walk parameters reset to defaults")
 
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
 
    # ── Serial actions ────────────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = self.serial_mgr.list_ports()
        self._port_combo["values"] = ports
        if ports:
            self._port_combo.set(ports[0])
 
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
 
    # ── Slider debounce ───────────────────────────────────────────────────────
    def _on_slider(self, flat_idx, val):
        if flat_idx in self._pending_send:
            self.after_cancel(self._pending_send[flat_idx])
        self._pending_send[flat_idx] = self.after(
            40, lambda: self._send_servo(flat_idx))
 
    def _send_servo(self, flat_idx):
        angle = self.angles[flat_idx].get()
        ch    = flat_to_channel(flat_idx)
        self.serial_mgr.send(f"s {ch} {angle}")
 
    # ── Pose manager ──────────────────────────────────────────────────────────
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
                    a = int(starts[i] + (target[i] - starts[i]) * step / steps)
                    self.angles[i].set(a)
                    self.serial_mgr.send(f"s {flat_to_channel(i)} {a}")
                time.sleep(0.05)
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
            if not n:
                return
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
        if names:
            self._pose_var.set(names[0])
 
    def _send_manual(self):
        cmd = self._cmd_entry.get().strip()
        if cmd:
            self.serial_mgr.send(cmd)
            self._cmd_entry.delete(0, "end")
 
    def destroy(self):
        """Clean shutdown — stop walk cycle before exit."""
        if self.walk_engine.running:
            self.walk_engine.stop()
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