"""
L4-SR Quadruped Controller  —  Simplified / Readable Version
=============================================================
This file controls the L4-SR robot over Bluetooth.
It sends short text commands to the ESP32 firmware, which moves the servos.

HOW COMMANDS WORK:
  s 4 90      = move servo channel 4 to 90 degrees
  S 90        = move ALL servos to 90 degrees
  F           = free all servos (go limp — support robot first!)
  R           = resume all servos (return to 90)
  IMU_ON      = start streaming gyro data from GY-521
  IMU_OFF     = stop gyro stream

SERVO CHANNEL MAP (which PCA9685 output = which joint):
  Hips   (joint 0): BL=ch0   BR=ch1   FR=ch2   FL=ch3
  Knees  (joint 1): BL=ch4   BR=ch5   FR=ch6   FL=ch7
  Ankles (joint 2): BL=ch8   BR=ch9   FR=ch10  FL=ch11

  Formula:  channel = joint * 4 + leg
  Example:  FR knee = 1 * 4 + 2 = channel 6
"""

# ── Python standard libraries ─────────────────────────────────────────────────
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import json
import os
import queue


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 1 — ROBOT SETTINGS
#  These are the numbers you'll tune most often.
#  Change them here and they affect everything — gaits, poses, sliders.
# ═════════════════════════════════════════════════════════════════════════════

# --- Shared stance settings (used by ALL gaits and balance mode) ---
SHARED_PARAMS = {
    "stance_hip":    90,    # hip angle when standing still (90 = centered)
    "stance_knee":   88,    # knee angle when standing still
    "stance_ankle":  93,    # ankle angle when standing still
    "lift_knee":     45,    # knee angle when a leg is lifted off the ground
    "body_tilt":     0,     # IMU correction offset added to knee (auto-set by IMU tab)
    "turn_offset":   15,    # how many degrees hips shift when turning
    "serial_delay":  0.05,  # seconds between servo commands (too low = Bluetooth drops)
    "pose_delay":    0.05,  # seconds between steps when moving to a pose
}

# --- Per-gait settings (each gait has its own speed and stride) ---
# swing_hip_fwd = where the hip goes when a leg swings forward
# swing_hip_bwd = where the hip goes when a leg pushes backward
GAIT_PARAMS = {
    "trot": {
        "label":           "Trot",
        "desc":            "Diagonal pairs — fast, efficient",
        "swing_hip_fwd":   110,
        "swing_hip_bwd":   70,
        "step_delay":      0.04,   # seconds per step within a phase
        "steps_per_phase": 8,      # how many steps to interpolate across
        "cycle_pause":     0.02,   # pause between full cycles
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
        "desc":            "Sequential FL→FR→BR→BL — smooth",
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

ACTIVE_GAIT = "trot"  # which gait is currently selected

def get_params():
    """Returns a merged dict of shared + active gait params. Used everywhere."""
    p = dict(SHARED_PARAMS)
    p.update(GAIT_PARAMS[ACTIVE_GAIT])
    return p


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 2 — HARDWARE CONSTANTS
#  These describe the physical robot — don't change unless you rewire.
# ═════════════════════════════════════════════════════════════════════════════

BAUD_RATE  = 115200          # Bluetooth serial speed — must match firmware
NUM_SERVOS = 12              # 4 legs × 3 joints
POSE_FILE  = "l4sr_poses.json"  # where saved poses are stored on disk

# Leg and joint name labels (for the UI)
LEG_NAMES   = ["BL (Back-Left)", "BR (Back-Right)", "FR (Front-Right)", "FL (Front-Left)"]
JOINT_NAMES = ["Hip", "Knee", "Ankle"]

# Leg number shortcuts — easier to read than bare numbers in gait code
BL, BR, FR, FL = 0, 1, 2, 3

# --- Servo inversion ---
# After the rebuild, ALL knee and ankle servos are physically flipped.
# We fix this in software: instead of sending angle X, we send (180 - X).
# 90 stays as 90 (180-90=90), but 0 and 180 swap — exactly what we need.
INVERTED_CHANNELS = {4, 5, 6, 7, 8, 9, 10, 11}  # all knees + all ankles

# --- Hip mirroring ---
# Left-side legs (BL, FL) have their hip servo mounted facing the other way.
# So "forward" for BL/FL means the OPPOSITE direction vs BR/FR.
# We mirror those hips so all 4 legs push the same physical direction.
HIP_INVERTED_LEGS = {BL, FL}  # legs 0 and 3

# Default poses — [hip, knee, ankle] × 4 legs, in flat order
DEFAULT_POSES = {
    "Neutral": [90] * 12,
    # Stand — angles observed from stable physical stance on rebuilt robot
    # Hip=90 all legs, Knees/Ankles averaged from left/right pairs
    "Stand":   [90, 88, 93,   # BL hip, knee, ankle
                90, 88, 93,   # BR hip, knee, ankle
                90, 88, 93,   # FR hip, knee, ankle
                90, 88, 93],  # FL hip, knee, ankle
    "Sit":     [90, 45,  45,  90, 135, 135, 90, 45,  45,  90, 135, 135],
    "Stretch": [90, 135, 160, 90, 45,  20,  90, 135, 160, 90, 45,  20],
}


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 3 — ANGLE MATH
#  Three small functions that convert a "logical" angle into the actual
#  number we send to the servo, accounting for flips and mirroring.
# ═════════════════════════════════════════════════════════════════════════════

def servo_ch(leg, joint):
    """
    Convert leg + joint into a PCA9685 channel number.
    Formula: joint * 4 + leg
    Example: FR (leg 2) knee (joint 1) = 1*4+2 = channel 6
    """
    return joint * 4 + leg

def output_angle(leg, joint, angle):
    """
    THE main function for sending a servo command.
    Always call this before sending — it handles all the flipping.

    Step 1 — Hip mirror:
      If this is a hip (joint 0) on a left-side leg (BL or FL),
      flip the angle so the leg moves the same direction as the right-side legs.

    Step 2 — Mechanical inversion:
      If this is a knee or ankle (channels 4-11), flip the angle
      because those servos were physically reversed during the rebuild.
    """
    ch = servo_ch(leg, joint)

    # Step 1: mirror hips on left-side legs
    if joint == 0 and leg in HIP_INVERTED_LEGS:
        angle = 180 - angle

    # Step 2: flip knees and ankles (rebuild changed their direction)
    if ch in INVERTED_CHANNELS:
        angle = 180 - angle

    return angle


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 4 — SERIAL / BLUETOOTH MANAGER
#  Handles the Bluetooth connection and sending commands to the ESP32.
#  Uses a "replace queue" for servo commands — if you drag a slider fast,
#  old positions are thrown away and only the latest one gets sent.
#  This stops the robot from lagging behind after you release a slider.
# ═════════════════════════════════════════════════════════════════════════════

class SerialManager:
    def __init__(self):
        self.ser     = None           # the serial port object
        self.lock    = threading.Lock()
        self.log_cb  = None           # function to call to write to the log panel
        self.imu_cb  = None           # function to call when IMU data arrives

        # Replace queue — one slot per servo channel
        # If channel 6 gets 50 updates while busy, only the last one survives
        self._servo_slots  = {}
        self._slots_lock   = threading.Lock()
        self._slots_event  = threading.Event()
        self._writer_alive = False

    def connect(self, port):
        """Open the Bluetooth COM port and start background read/write threads."""
        try:
            with self.lock:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            self._log(f"✅ Connected to {port}")
            self._writer_alive = True
            threading.Thread(target=self._read_loop,  daemon=True).start()
            threading.Thread(target=self._write_loop, daemon=True).start()
            return True
        except Exception as e:
            self._log(f"❌ {e}")
            return False

    def disconnect(self):
        """Close the Bluetooth connection."""
        self._writer_alive = False
        self._slots_event.set()  # wake the writer thread so it can exit cleanly
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        self._log("🔌 Disconnected")

    def send(self, cmd: str):
        """
        Send a command to the robot.
        - Servo commands (s <ch> <angle>) go into the replace queue
        - Everything else (F, R, S 90, IMU_ON...) sends immediately
        """
        cmd = cmd.strip()
        if cmd.startswith("s "):
            parts = cmd.split()
            if len(parts) == 3:
                ch = parts[1]
                with self._slots_lock:
                    self._servo_slots[ch] = cmd  # overwrite any waiting command
                self._slots_event.set()          # wake the writer thread
                return
        self._write_now(cmd)  # non-servo: bypass queue

    def _write_loop(self):
        """
        Background thread that drains the servo queue.
        Waits until there's something to send, then sends one command at a time
        with serial_delay between each to avoid flooding Bluetooth.
        """
        while self._writer_alive:
            self._slots_event.wait()   # sleep until send() wakes us
            self._slots_event.clear()
            while True:
                with self._slots_lock:
                    if not self._servo_slots:
                        break
                    # grab the next waiting command
                    ch, cmd = next(iter(self._servo_slots.items()))
                    del self._servo_slots[ch]
                self._write_now(cmd)
                time.sleep(SHARED_PARAMS.get("serial_delay", 0.05))
                with self._slots_lock:
                    if not self._servo_slots:
                        break

    def _write_now(self, cmd: str):
        """Actually write bytes to the serial port."""
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write((cmd + "\n").encode())
                    self.ser.flush()
                    self._log(f"→ {cmd}")
                except Exception as e:
                    self._log(f"❌ write error: {e}")
            else:
                self._log("⚠️  Not connected")

    def _read_loop(self):
        """
        Background thread that reads incoming data from the ESP32.
        IMU lines (starting with 'IMU:') are sent to the IMU tab silently.
        Everything else goes to the log panel.
        """
        while self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
                if line:
                    if line.startswith("IMU:") and self.imu_cb:
                        self.imu_cb(line)   # silent — don't flood the log
                    else:
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


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 5 — GAIT ENGINE
#  Runs gait cycles in a background thread so the UI stays responsive.
#  All gaits boil down to two primitives:
#    _trot_phase  — multiple legs lift and sweep simultaneously
#    _single_phase — one leg lifts while others push (crawl/wave)
# ═════════════════════════════════════════════════════════════════════════════

class GaitEngine:

    # The order legs move in crawl and wave gaits
    CRAWL_ORDER = [FL, FR, BR, BL]

    def __init__(self, serial_mgr, log_cb, angle_vars, app=None):
        self.serial_mgr = serial_mgr
        self.log        = log_cb
        self.angle_vars = angle_vars  # shared list of tkinter IntVars for slider display
        self._app       = app         # main tk window — needed for thread-safe UI updates
        self._running   = False
        self._thread    = None
        self._direction = "forward"
        self._lock      = threading.Lock()

    # ── Start / stop ──────────────────────────────────────────────────────────

    def start(self, direction="forward"):
        """Start the selected gait in a background thread."""
        with self._lock:
            if self._running:
                return
            self._direction = direction
            self._running   = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.log(f"🦿 [{GAIT_PARAMS[ACTIVE_GAIT]['label']}] started — {direction}")

    def start_balance(self):
        """Start balance stance mode (holds still, adjusts knees from IMU)."""
        with self._lock:
            if self._running:
                return
            self._running = True
        self._thread = threading.Thread(target=self._balance_loop, daemon=True)
        self._thread.start()
        self.log("⚖️  Balance stance — enable IMU stream + auto tilt in IMU tab")

    def stop(self):
        """Signal the background thread to stop and wait for it."""
        with self._lock:
            self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        self.log("🛑 Stopped")

    @property
    def running(self):
        return self._running

    # ── Main gait loop ────────────────────────────────────────────────────────

    def _loop(self):
        """Runs continuously until stop() is called. Picks the right gait each cycle."""
        while self._running:
            p = get_params()

            # Work out hip targets based on direction
            fwd = p["swing_hip_fwd"]
            bwd = p["swing_hip_bwd"]
            off = p["turn_offset"]
            d   = self._direction

            if   d == "forward":    sf, pb = fwd,       bwd
            elif d == "backward":   sf, pb = bwd,       fwd
            elif d == "turn_left":  sf, pb = fwd - off, bwd + off
            else:                   sf, pb = fwd + off, bwd - off  # turn_right

            # Run one cycle of the selected gait
            if   ACTIVE_GAIT == "trot":  self._trot_cycle(sf, pb)
            elif ACTIVE_GAIT == "crawl": self._crawl_cycle(sf, pb)
            elif ACTIVE_GAIT == "wave":  self._wave_cycle(sf, pb)
            elif ACTIVE_GAIT == "pace":  self._pace_cycle(sf, pb)
            elif ACTIVE_GAIT == "bound": self._bound_cycle(sf, pb)

            time.sleep(get_params()["cycle_pause"])

        self._stance_all()  # return to neutral when stopped

    # ── Gait cycle definitions ────────────────────────────────────────────────
    # Each gait picks which legs swing together and calls the right phase primitive.

    def _trot_cycle(self, sf, pb):
        # Diagonal pairs: FL+BR move together, then FR+BL
        self._trot_phase([FL, BR], [FR, BL], sf, pb)
        if not self._running: return
        self._trot_phase([FR, BL], [FL, BR], sf, pb)

    def _crawl_cycle(self, sf, pb):
        # One leg at a time, other 3 push
        for swing in self.CRAWL_ORDER:
            if not self._running: return
            push = [l for l in [FL, FR, BL, BR] if l != swing]
            self._single_phase(swing, push, sf, pb)

    def _wave_cycle(self, sf, pb):
        # Like crawl but push starts immediately (smoother flow)
        for swing in self.CRAWL_ORDER:
            if not self._running: return
            push = [l for l in [FL, FR, BL, BR] if l != swing]
            self._wave_phase(swing, push, sf, pb)

    def _pace_cycle(self, sf, pb):
        # Same-side pairs: FL+BL then FR+BR
        self._trot_phase([FL, BL], [FR, BR], sf, pb)
        if not self._running: return
        self._trot_phase([FR, BR], [FL, BL], sf, pb)

    def _bound_cycle(self, sf, pb):
        # Front pair then back pair
        self._trot_phase([FL, FR], [BL, BR], sf, pb)
        if not self._running: return
        self._trot_phase([BL, BR], [FL, FR], sf, pb)

    # ── Phase primitives ──────────────────────────────────────────────────────

    def _trot_phase(self, swing_legs, push_legs, swing_fwd, push_bwd):
        """
        One half-cycle: lift swing legs, sweep ALL hips, plant swing legs.
        Interpolates the hip sweep over multiple steps for smooth motion.
        """
        p           = get_params()
        steps       = p["steps_per_phase"]
        delay       = p["step_delay"]
        knee_stance = p["stance_knee"] + p["body_tilt"]
        knee_lift   = p["lift_knee"]
        ankle       = p["stance_ankle"]

        # Lift swing legs, keep push legs on ground
        for leg in swing_legs:
            self._send(leg, 1, knee_lift)
            self._send(leg, 2, ankle)
        for leg in push_legs:
            self._send(leg, 1, knee_stance)
            self._send(leg, 2, ankle)
        time.sleep(delay * 2)

        # Sweep hips — interpolate from current position to target
        swing_start = {leg: self.angle_vars[leg * 3].get() for leg in swing_legs}
        push_start  = {leg: self.angle_vars[leg * 3].get() for leg in push_legs}
        for step in range(1, steps + 1):
            if not self._running: return
            t = step / steps  # 0.0 → 1.0
            for leg in swing_legs:
                self._send(leg, 0, int(swing_start[leg] + (swing_fwd - swing_start[leg]) * t))
            for leg in push_legs:
                self._send(leg, 0, int(push_start[leg]  + (push_bwd  - push_start[leg])  * t))
            time.sleep(delay)

        # Plant swing legs back on ground
        for leg in swing_legs:
            self._send(leg, 1, knee_stance)
        time.sleep(delay * 2)

    def _single_phase(self, swing_leg, push_legs, swing_fwd, push_bwd):
        """One leg lifts while the other 3 actively push. Used in crawl."""
        p           = get_params()
        steps       = p["steps_per_phase"]
        delay       = p["step_delay"]
        knee_stance = p["stance_knee"] + p["body_tilt"]
        knee_lift   = p["lift_knee"]
        ankle       = p["stance_ankle"]

        self._send(swing_leg, 1, knee_lift)
        self._send(swing_leg, 2, ankle)
        for leg in push_legs:
            self._send(leg, 1, knee_stance)
            self._send(leg, 2, ankle)
        time.sleep(delay * 2)

        swing_start = self.angle_vars[swing_leg * 3].get()
        push_starts = {leg: self.angle_vars[leg * 3].get() for leg in push_legs}
        for step in range(1, steps + 1):
            if not self._running: return
            t = step / steps
            self._send(swing_leg, 0, int(swing_start + (swing_fwd - swing_start) * t))
            for leg in push_legs:
                self._send(leg, 0, int(push_starts[leg] + (push_bwd - push_starts[leg]) * t))
            time.sleep(delay)

        self._send(swing_leg, 1, knee_stance)
        time.sleep(delay * 2)

    def _wave_phase(self, swing_leg, push_legs, swing_fwd, push_bwd):
        """Wave variant: push starts as soon as swing lifts — no discrete pause."""
        p           = get_params()
        steps       = p["steps_per_phase"]
        delay       = p["step_delay"]
        knee_stance = p["stance_knee"] + p["body_tilt"]
        knee_lift   = p["lift_knee"]
        ankle       = p["stance_ankle"]

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
        self._send(swing_leg, 1, knee_stance)
        time.sleep(delay)

    # ── Balance stance loop ───────────────────────────────────────────────────

    def _balance_loop(self):
        """
        Holds all legs at stance and adjusts knee angles live from IMU tilt.
        Runs at 10Hz to match the IMU stream rate.
        Requires IMU_ON and auto body_tilt enabled in the IMU tab.
        """
        self._stance_all()
        while self._running:
            p           = get_params()
            knee_target = max(0, min(180, p["stance_knee"] + p["body_tilt"]))
            for leg in range(4):
                self._send(leg, 1, knee_target)
                self._send(leg, 2, p["stance_ankle"])
            time.sleep(0.1)
        self._stance_all()
        self.log("⚖️  Balance stance stopped")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stance_all(self):
        """Move all legs to neutral standing position."""
        p = get_params()
        for leg in range(4):
            self._send(leg, 0, p["stance_hip"])
            self._send(leg, 1, p["stance_knee"] + p["body_tilt"])
            self._send(leg, 2, p["stance_ankle"])
        self.log("✅ Returned to stance")

    def _send(self, leg, joint, angle):
        """
        Send one servo command. Clamps angle to 0-180, queues the serial command.
        NOTE: angle_vars update is posted to the main thread via the app reference
        so tkinter is never touched from a background thread.
        output_angle() handles all the hip mirroring and mechanical inversion.
        """
        angle = max(0, min(180, int(angle)))
        ch    = servo_ch(leg, joint)
        flat  = leg * 3 + joint
        # Post slider update safely to main thread
        if self._app:
            self._app.after(0, lambda f=flat, a=angle: self.angle_vars[f].set(a))
        self.serial_mgr.send(f"s {ch} {output_angle(leg, joint, angle)}")
        time.sleep(SHARED_PARAMS.get("serial_delay", 0.05))


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 6 — UI COLOURS
# ═════════════════════════════════════════════════════════════════════════════

BG        = "#1a1a2e"   # main background (dark navy)
PANEL     = "#16213e"   # panel background (slightly lighter)
ACCENT    = "#0f3460"   # button / highlight background
HIGHLIGHT = "#e94560"   # red — stop / danger actions
TEXT      = "#eaeaea"   # main text colour
GREEN     = "#4ecca3"   # green — go / active / angles
YELLOW    = "#f5a623"   # yellow — labels / warnings
ORANGE    = "#ff7b2e"   # orange — section headers
PURPLE    = "#9b5de5"   # purple — free servo button
SLIDER_BG = "#0d2137"   # slider track background


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 7 — MAIN UI (L4SRController)
#  Everything below is the Tkinter window — tabs, sliders, buttons, log panel.
#  You shouldn't need to change anything here unless you want to add UI elements.
# ═════════════════════════════════════════════════════════════════════════════

class L4SRController(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("L4-SR Quadruped Controller")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(1000, 720)

        # Thread-safe log queue — background threads put messages here,
        # _poll_log() drains it on the main thread every 50ms
        self._log_queue = queue.Queue()

        # --- Core systems ---
        self.serial_mgr        = SerialManager()
        self.serial_mgr.log_cb = self._append_log
        self.serial_mgr.imu_cb = self._on_imu_data

        # Servo angle display (one IntVar per servo, shared with gait engine)
        self.angles = [tk.IntVar(value=90) for _ in range(NUM_SERVOS)]

        # Gait engine gets the serial manager, angle vars, and app reference
        self.gait_engine = GaitEngine(self.serial_mgr, self._append_log, self.angles, app=self)

        # Pending slider debounce timers (prevents flooding on fast drag)
        self._pending_send = {}

        # Pose storage — load from file or use defaults
        self.poses = dict(DEFAULT_POSES)
        self._load_poses()

        # IMU display vars (updated from _on_imu_data)
        self._imu_angle_x   = tk.DoubleVar(value=0.0)
        self._imu_angle_y   = tk.DoubleVar(value=0.0)
        self._imu_gx        = tk.DoubleVar(value=0.0)
        self._imu_gy        = tk.DoubleVar(value=0.0)
        self._imu_gz        = tk.DoubleVar(value=0.0)
        self._imu_ax        = tk.DoubleVar(value=0.0)
        self._imu_ay        = tk.DoubleVar(value=0.0)
        self._imu_az        = tk.DoubleVar(value=0.0)
        self._imu_streaming = False
        self._auto_tilt     = tk.BooleanVar(value=False)

        # Balance test state
        self._bal_test_active = False
        self._bal_test_thread = None
        self._bal_sensitivity = tk.DoubleVar(value=0.4)
        self._bal_deadzone    = tk.DoubleVar(value=2.0)
        self._bal_leg_knees   = [tk.IntVar(value=88) for _ in range(4)]
        self._bal_level_color = None  # set after UI built

        self._build_ui()
        self.after(50, self._poll_log)  # start draining log queue on main thread

    # ── UI Layout ─────────────────────────────────────────────────────────────

    def _build_ui(self):
        """Build the main window: connection bar at top, then 4 tabs."""
        self._build_connection_bar()

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=4, pady=4)

        tab1 = tk.Frame(nb, bg=BG); nb.add(tab1, text="  🎛  SERVO CONTROLS  ")
        tab2 = tk.Frame(nb, bg=BG); nb.add(tab2, text="  🦿  WALK CYCLE  ")
        tab3 = tk.Frame(nb, bg=BG); nb.add(tab3, text="  ⚙  TUNING  ")
        tab4 = tk.Frame(nb, bg=BG); nb.add(tab4, text="  📡  IMU / GY-521  ")

        self._build_servo_tab(tab1)
        self._build_walk_tab(tab2)
        self._build_tuning_tab(tab3)
        self._build_imu_tab(tab4)

    def _build_connection_bar(self):
        """Top bar: port selector, connect button, quick commands, free/resume."""
        bar = tk.Frame(self, bg=PANEL, pady=6)
        bar.pack(fill="x", padx=4, pady=(4, 0))

        tk.Label(bar, text="PORT:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(10, 4))

        self._port_var  = tk.StringVar()
        ports           = self.serial_mgr.list_ports()
        self._port_combo = ttk.Combobox(bar, textvariable=self._port_var,
                                        values=ports, width=14, font=("Courier", 10))
        if ports:
            self._port_combo.set(ports[0])
        self._port_combo.pack(side="left", padx=4)

        self._btn(bar, "⟳",          self._refresh_ports,  ACCENT  ).pack(side="left", padx=2)
        self._connect_btn = self._btn(bar, "Connect", self._toggle_connect, GREEN)
        self._connect_btn.pack(side="left", padx=4)

        self._conn_label = tk.Label(bar, text="● OFFLINE", font=("Courier", 10, "bold"),
                                    bg=PANEL, fg=YELLOW)
        self._conn_label.pack(side="left", padx=8)

        tk.Label(bar, text=" | Quick:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(12, 4))

        for label, cmd in [("All→90", "S 90"), ("All→0", "S 0"), ("All→180", "S 180")]:
            self._btn(bar, label, lambda c=cmd: self.serial_mgr.send(c), ACCENT).pack(side="left", padx=3)

        tk.Label(bar, text=" |", bg=PANEL, fg=TEXT, font=("Courier", 10)).pack(side="left", padx=4)
        self._btn(bar, "⚡ FREE",    lambda: self.serial_mgr.send("F"), PURPLE).pack(side="left", padx=3)
        self._btn(bar, "↺ RESUME",  lambda: self.serial_mgr.send("R"), GREEN ).pack(side="left", padx=3)

    # ── TAB 1: Servo Controls ─────────────────────────────────────────────────

    def _build_servo_tab(self, parent):
        """12 sliders (one per servo) on the left, pose manager + log on the right."""
        paned = tk.PanedWindow(parent, orient="horizontal", bg=BG, sashwidth=6, bd=0)
        paned.pack(fill="both", expand=True)
        left  = tk.Frame(paned, bg=BG)
        right = tk.Frame(paned, bg=BG)
        paned.add(left,  minsize=560)
        paned.add(right, minsize=280)

        # Left: sliders
        tk.Label(left, text="SERVO CONTROLS", font=("Courier", 12, "bold"),
                 bg=BG, fg=GREEN).pack(anchor="w", padx=4, pady=(4, 2))

        canvas = tk.Canvas(left, bg=BG, highlightthickness=0)
        scroll = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        inner  = tk.Frame(canvas, bg=BG)
        inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=scroll.set)
        canvas.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        for leg in range(4):
            frm = tk.LabelFrame(inner, text=f"  {LEG_NAMES[leg]}  ",
                                 bg=PANEL, fg=YELLOW, font=("Courier", 10, "bold"),
                                 bd=1, relief="groove")
            frm.pack(fill="x", padx=6, pady=5)
            for joint in range(3):
                flat = leg * 3 + joint
                ch   = servo_ch(leg, joint)
                row  = tk.Frame(frm, bg=PANEL)
                row.pack(fill="x", padx=8, pady=3)
                tk.Label(row, text=f"{JOINT_NAMES[joint]:6s} #{flat} (ch{ch:02d})",
                         width=18, anchor="w", font=("Courier", 9), bg=PANEL, fg=TEXT).pack(side="left")
                tk.Label(row, textvariable=self.angles[flat], width=4,
                         font=("Courier", 10, "bold"), bg=PANEL, fg=HIGHLIGHT).pack(side="right", padx=(0,6))
                tk.Label(row, text="°", font=("Courier", 10), bg=PANEL, fg=TEXT).pack(side="right")
                tk.Scale(row, variable=self.angles[flat], from_=0, to=180,
                         orient="horizontal", showvalue=False, length=320,
                         bg=SLIDER_BG, fg=TEXT, troughcolor=ACCENT,
                         highlightthickness=0, bd=0, activebackground=HIGHLIGHT,
                         command=lambda val, i=flat: self._on_slider(i, val)
                         ).pack(side="left", fill="x", expand=True, padx=6)

        # Right: pose manager + manual command + log
        self._build_right_panel(right)

    def _build_right_panel(self, parent):
        # Pose manager
        pf = tk.LabelFrame(parent, text="  POSE MANAGER  ", bg=PANEL, fg=YELLOW,
                            font=("Courier", 10, "bold"), bd=1, relief="groove")
        pf.pack(fill="x", padx=6, pady=6)

        self._pose_var   = tk.StringVar(value=list(self.poses.keys())[0])
        self._pose_combo = ttk.Combobox(pf, textvariable=self._pose_var,
                                        values=list(self.poses.keys()),
                                        font=("Courier", 10), width=18)
        self._pose_combo.pack(padx=8, pady=(8, 4))

        bg = tk.Frame(pf, bg=PANEL); bg.pack(padx=8, pady=4)
        self._btn(bg, "▶ Move To",  self._move_to_pose,     GREEN    ).grid(row=0, column=0, padx=4, pady=3, sticky="ew")
        self._btn(bg, "💾 Save As", self._save_pose,         YELLOW   ).grid(row=0, column=1, padx=4, pady=3, sticky="ew")
        self._btn(bg, "🗑 Delete",  self._delete_pose,       HIGHLIGHT).grid(row=1, column=0, padx=4, pady=3, sticky="ew")
        self._btn(bg, "⟳ Refresh", self._refresh_pose_list, ACCENT   ).grid(row=1, column=1, padx=4, pady=3, sticky="ew")

        sr = tk.Frame(pf, bg=PANEL); sr.pack(padx=8, pady=(2, 8), fill="x")
        tk.Label(sr, text="Transition steps:", bg=PANEL, fg=TEXT, font=("Courier", 9)).pack(side="left")
        self._steps_var = tk.IntVar(value=20)
        tk.Spinbox(sr, from_=1, to=100, textvariable=self._steps_var,
                   width=5, font=("Courier", 10), bg=ACCENT, fg=TEXT).pack(side="right")

        # Manual command
        cf = tk.LabelFrame(parent, text="  MANUAL COMMAND  ", bg=PANEL, fg=YELLOW,
                           font=("Courier", 10, "bold"), bd=1, relief="groove")
        cf.pack(fill="x", padx=6, pady=6)
        self._cmd_entry = tk.Entry(cf, bg=SLIDER_BG, fg=TEXT, insertbackground=TEXT,
                                   font=("Courier", 11), width=22)
        self._cmd_entry.pack(padx=8, pady=(8, 4))
        self._cmd_entry.bind("<Return>", lambda e: self._send_manual())
        self._btn(cf, "Send", self._send_manual, GREEN).pack(pady=(0, 8))
        tk.Label(cf, text="e.g.  s 4 90   or   S 90",
                 bg=PANEL, fg="#557799", font=("Courier", 8)).pack(pady=(0, 6))

        # Log
        lf = tk.LabelFrame(parent, text="  SERIAL LOG  ", bg=PANEL, fg=YELLOW,
                            font=("Courier", 10, "bold"), bd=1, relief="groove")
        lf.pack(fill="both", expand=True, padx=6, pady=6)
        self._log_text = tk.Text(lf, bg=SLIDER_BG, fg=GREEN, font=("Courier", 8),
                                 state="disabled", wrap="word", height=12)
        ls = ttk.Scrollbar(lf, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=ls.set)
        ls.pack(side="right", fill="y")
        self._log_text.pack(fill="both", expand=True, padx=4, pady=4)
        self._btn(lf, "Clear Log",
                  lambda: (self._log_text.configure(state="normal"),
                           self._log_text.delete("1.0", "end"),
                           self._log_text.configure(state="disabled")),
                  ACCENT).pack(pady=(0, 4))

    # ── TAB 2: Walk Cycle ─────────────────────────────────────────────────────

    def _build_walk_tab(self, parent):
        tk.Label(parent, text="WALK CYCLE CONTROL", font=("Courier", 14, "bold"),
                 bg=BG, fg=ORANGE).pack(anchor="w", padx=12, pady=(10, 4))

        # Gait selector buttons
        gf = tk.LabelFrame(parent, text="  GAIT MODE  ", bg=PANEL, fg=YELLOW,
                            font=("Courier", 10, "bold"), bd=1, relief="groove")
        gf.pack(fill="x", padx=12, pady=(4, 8))

        self._gait_btns = {}
        row = tk.Frame(gf, bg=PANEL); row.pack(pady=10, padx=10)
        for key, info in GAIT_PARAMS.items():
            col = tk.Frame(row, bg=PANEL); col.pack(side="left", padx=6)
            b = tk.Button(col, text=info["label"], font=("Courier", 10, "bold"),
                          width=8, relief="flat", cursor="hand2",
                          command=lambda k=key: self._select_gait(k))
            b.pack()
            tk.Label(col, text=info["desc"], font=("Courier", 7), bg=PANEL, fg="#557799",
                     wraplength=90, justify="center").pack(pady=(2, 0))
            self._gait_btns[key] = b
        self._select_gait(ACTIVE_GAIT, silent=True)

        # Status
        sf = tk.Frame(parent, bg=PANEL, pady=8); sf.pack(fill="x", padx=12, pady=4)
        tk.Label(sf, text="Status:", font=("Courier", 10), bg=PANEL, fg=TEXT).pack(side="left", padx=10)
        self._walk_status     = tk.Label(sf, text="⬛ STOPPED", font=("Courier", 12, "bold"), bg=PANEL, fg=YELLOW)
        self._direction_label = tk.Label(sf, text="", font=("Courier", 10), bg=PANEL, fg=GREEN)
        self._walk_status.pack(side="left", padx=8)
        self._direction_label.pack(side="left", padx=8)

        # D-pad
        df = tk.LabelFrame(parent, text="  DIRECTION  ", bg=PANEL, fg=YELLOW,
                            font=("Courier", 10, "bold"), bd=1, relief="groove")
        df.pack(padx=12, pady=8, fill="x")
        dpad = tk.Frame(df, bg=PANEL); dpad.pack(pady=12)
        bcfg = dict(font=("Courier", 13, "bold"), width=10, height=2, relief="flat", cursor="hand2")
        tk.Button(dpad, text="▲  FORWARD",  bg=GREEN,     fg="#0a0a14", command=lambda: self._walk_start("forward"),    **bcfg).grid(row=0, column=1, padx=6, pady=4)
        tk.Button(dpad, text="◄  LEFT",     bg=PURPLE,    fg=TEXT,      command=lambda: self._walk_start("turn_left"),  **bcfg).grid(row=1, column=0, padx=6, pady=4)
        tk.Button(dpad, text="■  STOP",     bg=HIGHLIGHT, fg=TEXT,      command=self._walk_stop,                        **bcfg).grid(row=1, column=1, padx=6, pady=4)
        tk.Button(dpad, text="RIGHT  ►",    bg=PURPLE,    fg=TEXT,      command=lambda: self._walk_start("turn_right"), **bcfg).grid(row=1, column=2, padx=6, pady=4)
        tk.Button(dpad, text="▼  BACKWARD", bg=ORANGE,    fg="#0a0a14", command=lambda: self._walk_start("backward"),   **bcfg).grid(row=2, column=1, padx=6, pady=4)

        self.bind("<w>",     lambda e: self._walk_start("forward"))
        self.bind("<s>",     lambda e: self._walk_start("backward"))
        self.bind("<a>",     lambda e: self._walk_start("turn_left"))
        self.bind("<d>",     lambda e: self._walk_start("turn_right"))
        self.bind("<space>", lambda e: self._walk_stop())
        tk.Label(df, text="Keyboard: W A S D = move  |  SPACE = stop",
                 font=("Courier", 8), bg=PANEL, fg="#557799").pack(pady=(0, 8))

        # Live timing sliders
        tf = tk.LabelFrame(parent, text="  LIVE TIMING  ", bg=PANEL, fg=YELLOW,
                           font=("Courier", 10, "bold"), bd=1, relief="groove")
        tf.pack(padx=12, pady=(4, 8), fill="x")
        for label, get_fn, set_fn, lo, hi in [
            ("serial_delay",  lambda: SHARED_PARAMS["serial_delay"],              lambda v: SHARED_PARAMS.update({"serial_delay": v}),                           0.005, 0.15),
            ("step_delay",    lambda: GAIT_PARAMS[ACTIVE_GAIT]["step_delay"],     lambda v: GAIT_PARAMS[ACTIVE_GAIT].update({"step_delay": v}),                  0.005, 0.2),
            ("cycle_pause",   lambda: GAIT_PARAMS[ACTIVE_GAIT]["cycle_pause"],    lambda v: GAIT_PARAMS[ACTIVE_GAIT].update({"cycle_pause": v}),                 0.0,   0.3),
        ]:
            r = tk.Frame(tf, bg=PANEL); r.pack(fill="x", padx=14, pady=4)
            tk.Label(r, text=label, font=("Courier", 9), bg=PANEL, fg=TEXT, width=22, anchor="w").pack(side="left")
            vl = tk.Label(r, text=f"{get_fn():.3f}s", font=("Courier", 9), bg=PANEL, fg=GREEN, width=7)
            vl.pack(side="right")
            def on_change(v, sf=set_fn, vl=vl):
                rv = round(float(v), 3); sf(rv); vl.config(text=f"{rv:.3f}s")
            tk.Scale(r, from_=lo, to=hi, resolution=0.005, orient="horizontal",
                     command=on_change, bg=PANEL, fg=TEXT, troughcolor=BG,
                     highlightthickness=0, showvalue=False, length=260).pack(side="left", padx=8)

        # Balance stance
        bf = tk.LabelFrame(parent, text="  ⚖  BALANCE STANCE  ", bg=PANEL, fg=GREEN,
                            font=("Courier", 10, "bold"), bd=1, relief="groove")
        bf.pack(padx=12, pady=(4, 8), fill="x")
        tk.Label(bf, text="Holds all legs at stance and applies live IMU tilt correction.\n"
                           "Enable IMU stream + auto body_tilt in the 📡 IMU tab first.",
                 font=("Courier", 8), bg=PANEL, fg="#557799", justify="left").pack(anchor="w", padx=12, pady=(10, 4))
        br = tk.Frame(bf, bg=PANEL); br.pack(pady=(4, 10))
        self._bal_status = tk.Label(br, text="⬛ OFF", font=("Courier", 11, "bold"), bg=PANEL, fg=YELLOW)
        self._bal_status.pack(side="left", padx=12)
        self._btn(br, "▶ START BALANCE", self._balance_start, GREEN    ).pack(side="left", padx=4)
        self._btn(br, "■ STOP BALANCE",  self._balance_stop,  HIGHLIGHT).pack(side="left", padx=4)

    # ── TAB 3: Tuning ─────────────────────────────────────────────────────────

    def _build_tuning_tab(self, parent):
        tk.Label(parent, text="PARAMETER TUNING", font=("Courier", 14, "bold"),
                 bg=BG, fg=ORANGE).pack(anchor="w", padx=12, pady=(10, 4))

        # Shared params
        sf = tk.LabelFrame(parent, text="  SHARED PARAMS (all gaits)  ", bg=PANEL, fg=YELLOW,
                           font=("Courier", 10, "bold"), bd=1, relief="groove")
        sf.pack(fill="x", padx=12, pady=(4, 8))
        for key, lo, hi in [
            ("stance_hip", 0, 180), ("stance_knee", 0, 180), ("stance_ankle", 0, 180),
            ("lift_knee",  0, 90),  ("turn_offset", 0, 45),
        ]:
            self._param_slider(sf, key, SHARED_PARAMS, lo, hi)

        # Per-gait params
        gf = tk.LabelFrame(parent, text="  PER-GAIT PARAMS (active gait only)  ", bg=PANEL, fg=YELLOW,
                           font=("Courier", 10, "bold"), bd=1, relief="groove")
        gf.pack(fill="x", padx=12, pady=(4, 8))
        for key, lo, hi in [
            ("swing_hip_fwd", 90, 180), ("swing_hip_bwd", 0, 90),
            ("steps_per_phase", 2, 20),
        ]:
            self._param_slider(gf, key, GAIT_PARAMS[ACTIVE_GAIT], lo, hi, integer=True)

        self._btn(parent, "↺ Reset All to Defaults",
                  self._reset_params, ACCENT).pack(pady=12)

    def _param_slider(self, parent, key, param_dict, lo, hi, integer=False):
        """Helper that builds one labelled slider tied to a param dict entry."""
        row = tk.Frame(parent, bg=PANEL); row.pack(fill="x", padx=14, pady=5)
        tk.Label(row, text=key, font=("Courier", 9), bg=PANEL, fg=TEXT,
                 width=20, anchor="w").pack(side="left")
        vl = tk.Label(row, text=str(param_dict[key]), font=("Courier", 9),
                      bg=PANEL, fg=GREEN, width=6)
        vl.pack(side="right")
        def on_change(v, k=key, pd=param_dict, vl=vl):
            val = int(float(v)) if integer else round(float(v), 3)
            pd[k] = val
            vl.config(text=str(val))
        res = 1 if integer else 0.005
        tk.Scale(row, from_=lo, to=hi, resolution=res, orient="horizontal",
                 command=on_change, bg=PANEL, fg=TEXT, troughcolor=BG,
                 highlightthickness=0, showvalue=False, length=280,
                 ).pack(side="left", padx=8)

    # ── TAB 4: IMU ────────────────────────────────────────────────────────────

    def _build_imu_tab(self, parent):
        # Stream controls
        ctrl = tk.Frame(parent, bg=PANEL, pady=8); ctrl.pack(fill="x", padx=8, pady=(8, 0))
        tk.Label(ctrl, text="GY-521 / MPU-6050", font=("Courier", 13, "bold"),
                 bg=PANEL, fg=HIGHLIGHT).pack(side="left", padx=12)
        self._imu_status = tk.Label(ctrl, text="● STREAM OFF",
                                    font=("Courier", 10, "bold"), bg=PANEL, fg=YELLOW)
        self._imu_status.pack(side="left", padx=16)
        self._btn(ctrl, "▶ START STREAM", self._imu_stream_on,  GREEN    ).pack(side="left", padx=4)
        self._btn(ctrl, "■ STOP STREAM",  self._imu_stream_off, HIGHLIGHT).pack(side="left", padx=4)

        # Big angle readout
        af = tk.Frame(parent, bg=BG); af.pack(fill="x", padx=16, pady=12)
        for label, var, color in [("ROLL  (angleX)", self._imu_angle_x, GREEN),
                                   ("PITCH (angleY)", self._imu_angle_y, ORANGE)]:
            col = tk.Frame(af, bg=PANEL, padx=20, pady=12)
            col.pack(side="left", expand=True, fill="both", padx=8)
            tk.Label(col, text=label, font=("Courier", 10), bg=PANEL, fg=TEXT).pack()
            tk.Label(col, textvariable=var, font=("Courier", 32, "bold"), bg=PANEL, fg=color).pack()
            tk.Label(col, text="degrees", font=("Courier", 9), bg=PANEL, fg=TEXT).pack()

        # Raw gyro / accel values
        rf = tk.Frame(parent, bg=BG); rf.pack(fill="x", padx=16, pady=4)
        for section, items in [
            ("GYRO (°/s)", [("gx", self._imu_gx), ("gy", self._imu_gy), ("gz", self._imu_gz)]),
            ("ACCEL (g)",  [("ax", self._imu_ax), ("ay", self._imu_ay), ("az", self._imu_az)]),
        ]:
            sec = tk.Frame(rf, bg=PANEL, padx=12, pady=10)
            sec.pack(side="left", expand=True, fill="both", padx=8)
            tk.Label(sec, text=section, font=("Courier", 10, "bold"), bg=PANEL, fg=YELLOW).pack(anchor="w")
            for name, var in items:
                r = tk.Frame(sec, bg=PANEL); r.pack(fill="x", pady=2)
                tk.Label(r, text=f"{name}:", width=4, font=("Courier", 10), bg=PANEL, fg=TEXT).pack(side="left")
                tk.Label(r, textvariable=var, font=("Courier", 10, "bold"),
                         bg=PANEL, fg=GREEN, width=10, anchor="e").pack(side="left")

        # Auto body_tilt toggle
        tf = tk.Frame(parent, bg=PANEL, padx=12, pady=10); tf.pack(fill="x", padx=16, pady=8)
        tk.Checkbutton(tf, text="  Auto body_tilt from IMU roll (angleX)",
                       variable=self._auto_tilt, font=("Courier", 10), bg=PANEL, fg=TEXT,
                       selectcolor=ACCENT, activebackground=PANEL,
                       activeforeground=TEXT).pack(side="left")
        tk.Label(tf, text="— maps roll → knee correction (angleX × 0.3, clamped ±20°)",
                 font=("Courier", 9), bg=PANEL, fg=TEXT).pack(side="left", padx=8)

        # ── Balance Test ──────────────────────────────────────────────────────
        # Holds robot in stance and applies per-leg knee correction from IMU.
        # Each leg gets independent correction based on roll AND pitch combined.
        # Roll  tilts left  → BL/FL knees extend,  BR/FR retract
        # Pitch tilts fwd   → FL/FR knees extend,  BL/BR retract
        bf = tk.LabelFrame(parent, text="  ⚖  BALANCE TEST  ",
                           bg=PANEL, fg=GREEN,
                           font=("Courier", 11, "bold"), bd=1, relief="groove")
        bf.pack(fill="x", padx=16, pady=(4, 12))

        # Controls row
        bc = tk.Frame(bf, bg=PANEL); bc.pack(fill="x", padx=12, pady=(10, 4))
        self._bal_engage_btn = self._btn(bc, "▶ ENGAGE", self._bal_engage, GREEN)
        self._bal_engage_btn.pack(side="left", padx=4)
        self._btn(bc, "■ DISENGAGE", self._bal_disengage, HIGHLIGHT).pack(side="left", padx=4)

        self._bal_status_lbl = tk.Label(bc, text="⬛ DISENGAGED",
                                        font=("Courier", 10, "bold"), bg=PANEL, fg=YELLOW)
        self._bal_status_lbl.pack(side="left", padx=12)

        # Level indicator
        self._bal_level_lbl = tk.Label(bc, text="LEVEL: --",
                                       font=("Courier", 10, "bold"), bg=PANEL, fg=TEXT)
        self._bal_level_lbl.pack(side="right", padx=12)

        # Sensitivity + deadzone sliders
        sl = tk.Frame(bf, bg=PANEL); sl.pack(fill="x", padx=12, pady=4)
        for label, var, lo, hi, res in [
            ("Sensitivity",  self._bal_sensitivity, 0.1, 2.0, 0.05),
            ("Dead zone (°)", self._bal_deadzone,   0.5, 10.0, 0.5),
        ]:
            r = tk.Frame(sl, bg=PANEL); r.pack(fill="x", pady=3)
            tk.Label(r, text=label, font=("Courier", 9), bg=PANEL, fg=TEXT,
                     width=16, anchor="w").pack(side="left")
            vl = tk.Label(r, text=f"{var.get():.2f}", font=("Courier", 9),
                          bg=PANEL, fg=GREEN, width=6)
            vl.pack(side="right")
            tk.Scale(r, variable=var, from_=lo, to=hi, resolution=res,
                     orient="horizontal", showvalue=False, length=300,
                     bg=PANEL, troughcolor=BG, highlightthickness=0,
                     command=lambda v, vl=vl: vl.config(text=f"{float(v):.2f}")
                     ).pack(side="left", padx=8)

        # Per-leg knee display
        lf = tk.Frame(bf, bg=PANEL); lf.pack(fill="x", padx=12, pady=(6, 12))
        tk.Label(lf, text="Live knee targets:", font=("Courier", 9),
                 bg=PANEL, fg=YELLOW).pack(anchor="w", pady=(0, 4))
        leg_row = tk.Frame(lf, bg=PANEL); leg_row.pack(fill="x")
        self._bal_leg_displays = []
        for i, name in enumerate(["BL", "BR", "FR", "FL"]):
            col = tk.Frame(leg_row, bg=ACCENT, padx=10, pady=6)
            col.pack(side="left", expand=True, fill="both", padx=4)
            tk.Label(col, text=name, font=("Courier", 9, "bold"),
                     bg=ACCENT, fg=TEXT).pack()
            lbl = tk.Label(col, textvariable=self._bal_leg_knees[i],
                           font=("Courier", 14, "bold"), bg=ACCENT, fg=GREEN)
            lbl.pack()
            tk.Label(col, text="°", font=("Courier", 8), bg=ACCENT, fg=TEXT).pack()
            self._bal_leg_displays.append(lbl)

    # ── Action handlers ───────────────────────────────────────────────────────

    def _on_slider(self, flat_idx, val):
        """Called every time a slider moves. Debounces to avoid flooding."""
        if flat_idx in self._pending_send:
            self.after_cancel(self._pending_send[flat_idx])
        self._pending_send[flat_idx] = self.after(40, lambda: self._send_servo(flat_idx))

    def _send_servo(self, flat_idx):
        """Send the current slider value for one servo."""
        angle = self.angles[flat_idx].get()
        leg   = flat_idx // 3
        joint = flat_idx % 3
        ch    = servo_ch(leg, joint)
        self.serial_mgr.send(f"s {ch} {output_angle(leg, joint, angle)}")

    def _walk_start(self, direction):
        if not self.serial_mgr.connected:
            messagebox.showwarning("Not connected", "Connect to robot first.")
            return
        self.gait_engine.start(direction)
        self._walk_status.configure(text="🟢 RUNNING", fg=GREEN)
        self._direction_label.configure(text=direction.upper().replace("_", " "))

    def _walk_stop(self):
        self.gait_engine.stop()
        self._walk_status.configure(text="⬛ STOPPED", fg=YELLOW)
        self._direction_label.configure(text="")

    def _balance_start(self):
        if not self.serial_mgr.connected:
            messagebox.showwarning("Not connected", "Connect to robot first.")
            return
        if self.gait_engine.running:
            self._walk_stop()
        self.gait_engine.start_balance()
        self._bal_status.configure(text="🟢 BALANCING", fg=GREEN)
        self._walk_status.configure(text="⚖ BALANCE", fg=GREEN)

    def _balance_stop(self):
        self.gait_engine.stop()
        self._bal_status.configure(text="⬛ OFF", fg=YELLOW)
        self._walk_status.configure(text="⬛ STOPPED", fg=YELLOW)

    def _imu_stream_on(self):
        self.serial_mgr.send("IMU_ON")
        self._imu_streaming = True
        self._imu_status.configure(text="● STREAMING", fg=GREEN)

    def _imu_stream_off(self):
        self.serial_mgr.send("IMU_OFF")
        self._imu_streaming = False
        self._imu_status.configure(text="● STREAM OFF", fg=YELLOW)

    def _bal_engage(self):
        """Start balance test — plant stance then begin IMU correction loop."""
        if not self.serial_mgr.connected:
            messagebox.showwarning("Not connected", "Connect to robot first.")
            return
        if not self._imu_streaming:
            messagebox.showwarning("IMU off", "Start IMU stream first.")
            return
        if self._bal_test_active:
            return
        # Stop any running gait first
        if self.gait_engine.running:
            self.gait_engine.stop()
        self._bal_test_active = True
        self._bal_status_lbl.configure(text="🟢 ENGAGED", fg=GREEN)
        self._bal_test_thread = threading.Thread(
            target=self._bal_loop, daemon=True)
        self._bal_test_thread.start()
        self._append_log("⚖️  Balance test engaged")

    def _bal_disengage(self):
        """Stop balance test and return to neutral stance."""
        self._bal_test_active = False
        if self._bal_test_thread:
            self._bal_test_thread.join(timeout=3)
        self._bal_status_lbl.configure(text="⬛ DISENGAGED", fg=YELLOW)
        self._bal_level_lbl.configure(text="LEVEL: --", fg=TEXT)
        self._append_log("⚖️  Balance test disengaged")

    def _bal_loop(self):
        """
        Balance correction loop — runs at 10Hz while engaged.

        Per-leg knee correction based on roll AND pitch:
          BL: base + (-roll) + ( pitch)   back-left
          BR: base + ( roll) + ( pitch)   back-right
          FR: base + ( roll) + (-pitch)   front-right
          FL: base + (-roll) + (-pitch)   front-left

        Roll positive  = robot tilting right  → right legs extend, left retract
        Pitch positive = robot tilting forward → front legs extend, back retract

        Sensitivity scales how many degrees of knee move per degree of tilt.
        Dead zone ignores small tilts to prevent jitter on flat ground.
        """
        # Plant all legs at stance first
        base = SHARED_PARAMS["stance_knee"]
        ankle = SHARED_PARAMS["stance_ankle"]
        hip   = SHARED_PARAMS["stance_hip"]
        for leg in range(4):
            self.gait_engine._send(leg, 0, hip)
            self.gait_engine._send(leg, 1, base)
            self.gait_engine._send(leg, 2, ankle)

        while self._bal_test_active:
            roll  = self._imu_angle_x.get()
            pitch = self._imu_angle_y.get()
            sens  = self._bal_sensitivity.get()
            dz    = self._bal_deadzone.get()

            # Apply dead zone
            roll  = 0.0 if abs(roll)  < dz else roll
            pitch = 0.0 if abs(pitch) < dz else pitch

            # Compute per-leg knee targets
            #         base  roll            pitch
            targets = [
                base + (-roll * sens) + ( pitch * sens),  # BL
                base + ( roll * sens) + ( pitch * sens),  # BR
                base + ( roll * sens) + (-pitch * sens),  # FR
                base + (-roll * sens) + (-pitch * sens),  # FL
            ]
            targets = [int(max(30, min(150, t))) for t in targets]

            # Send knee corrections
            for leg, knee in enumerate(targets):
                self.gait_engine._send(leg, 1, knee)

            # Update UI via queue (thread-safe)
            def _update_ui(t=targets, r=roll, p=pitch, dz=dz):
                for i, v in enumerate(t):
                    self._bal_leg_knees[i].set(v)
                total_tilt = (abs(r) + abs(p)) / 2
                if total_tilt < dz:
                    self._bal_level_lbl.configure(text="LEVEL: ✓ FLAT", fg=GREEN)
                elif total_tilt < 10:
                    self._bal_level_lbl.configure(text=f"LEVEL: {total_tilt:.1f}°", fg=YELLOW)
                else:
                    self._bal_level_lbl.configure(text=f"LEVEL: {total_tilt:.1f}° !", fg=HIGHLIGHT)
            self._log_queue.put(("__imu__", _update_ui))

            time.sleep(0.1)  # 10Hz — matches IMU stream rate

    def _on_imu_data(self, line):
        """
        Called from the serial read thread when an IMU: line arrives.
        Posts UI updates to the main thread via a queue — never touches tkinter directly.
        """
        try:
            angle_x, angle_y, gx, gy, gz, ax, ay, az = [float(x) for x in line[4:].split(",")]
            def _update():
                self._imu_angle_x.set(round(angle_x, 1))
                self._imu_angle_y.set(round(angle_y, 1))
                self._imu_gx.set(round(gx, 1));  self._imu_gy.set(round(gy, 1));  self._imu_gz.set(round(gz, 1))
                self._imu_ax.set(round(ax, 3));   self._imu_ay.set(round(ay, 3));  self._imu_az.set(round(az, 3))
                if self._auto_tilt.get():
                    SHARED_PARAMS["body_tilt"] = int(max(-20, min(20, angle_x * 0.3)))
            # Schedule on main thread
            self._log_queue.put(("__imu__", _update))
        except Exception:
            pass

    def _send_manual(self):
        cmd = self._cmd_entry.get().strip()
        if cmd:
            self.serial_mgr.send(cmd)

    def _select_gait(self, key, silent=False):
        global ACTIVE_GAIT
        ACTIVE_GAIT = key
        for k, b in self._gait_btns.items():
            b.configure(bg=GREEN if k == key else ACCENT,
                        fg="#0a0a14" if k == key else TEXT)
        if not silent:
            self._append_log(f"Gait → {GAIT_PARAMS[key]['label']}")

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
                    a     = int(starts[i] + (target[i] - starts[i]) * step / steps)
                    leg   = i // 3
                    joint = i % 3
                    ch    = servo_ch(leg, joint)
                    self.angles[i].set(a)
                    self.serial_mgr.send(f"s {ch} {output_angle(leg, joint, a)}")
                time.sleep(SHARED_PARAMS.get("pose_delay", 0.05))
            self._append_log(f"✅ Pose: {name}")
        threading.Thread(target=run, daemon=True).start()

    def _save_pose(self):
        win = tk.Toplevel(self, bg=PANEL)
        win.title("Save Pose")
        win.resizable(False, False)
        tk.Label(win, text="Pose name:", bg=PANEL, fg=TEXT, font=("Courier", 10)).pack(padx=12, pady=(10, 2))
        entry = tk.Entry(win, bg=SLIDER_BG, fg=TEXT, insertbackground=TEXT, font=("Courier", 11))
        entry.insert(0, self._pose_var.get())
        entry.pack(padx=12, pady=4)
        def confirm():
            name = entry.get().strip()
            if name:
                self.poses[name] = [self.angles[i].get() for i in range(NUM_SERVOS)]
                self._save_poses_file()
                self._refresh_pose_list()
                self._pose_var.set(name)
                self._append_log(f"💾 Saved pose: {name}")
            win.destroy()
        self._btn(win, "Save", confirm, GREEN).pack(pady=8)
        entry.focus()

    def _delete_pose(self):
        name = self._pose_var.get()
        if name in DEFAULT_POSES:
            messagebox.showwarning("Can't delete", f"'{name}' is a built-in pose.")
            return
        if name in self.poses:
            del self.poses[name]
            self._save_poses_file()
            self._refresh_pose_list()
            self._append_log(f"🗑 Deleted pose: {name}")

    def _refresh_pose_list(self):
        keys = list(self.poses.keys())
        self._pose_combo["values"] = keys
        if keys:
            self._pose_var.set(keys[0])

    def _reset_params(self):
        SHARED_PARAMS.update({"stance_hip": 90, "stance_knee": 88, "stance_ankle": 93,
                               "lift_knee": 45, "body_tilt": 0, "turn_offset": 15,
                               "serial_delay": 0.05, "pose_delay": 0.05})
        self._append_log("↺ Params reset to defaults")

    def _load_poses(self):
        if os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE) as f:
                    self.poses.update(json.load(f))
            except Exception:
                pass

    def _save_poses_file(self):
        try:
            with open(POSE_FILE, "w") as f:
                json.dump({k: v for k, v in self.poses.items() if k not in DEFAULT_POSES}, f, indent=2)
        except Exception:
            pass

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
            if self.serial_mgr.connect(port):
                self._connect_btn.configure(text="Disconnect", bg=HIGHLIGHT)
                self._conn_label.configure(text=f"● {port}", fg=GREEN)

    def _refresh_ports(self):
        ports = self.serial_mgr.list_ports()
        self._port_combo["values"] = ports
        if ports:
            self._port_combo.set(ports[0])

    def _append_log(self, msg):
        """Thread-safe log — any thread can call this."""
        self._log_queue.put(msg)

    def _poll_log(self):
        """Called by tkinter every 50ms to drain the queue on the main thread."""
        try:
            while True:
                item = self._log_queue.get_nowait()
                if isinstance(item, tuple) and item[0] == "__imu__":
                    item[1]()  # call the IMU update function
                else:
                    self._log_text.configure(state="normal")
                    self._log_text.insert("end", str(item) + "\n")
                    self._log_text.see("end")
                    self._log_text.configure(state="disabled")
        except queue.Empty:
            pass
        self.after(50, self._poll_log)

    def _btn(self, parent, text, cmd, color):
        """Shorthand for creating a styled button."""
        return tk.Button(parent, text=text, command=cmd,
                         bg=color, fg="#0a0a14",
                         font=("Courier", 9, "bold"),
                         relief="flat", cursor="hand2",
                         activebackground=TEXT, padx=8, pady=4)


# ═════════════════════════════════════════════════════════════════════════════
#  SECTION 8 — ENTRY POINT
#  This runs when you launch the script.
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    app = L4SRController()
    app.mainloop()
