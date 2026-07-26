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

# ─── Constants ───────────────────────────────────────────────────────────────
BAUD_RATE     = 115200
NUM_SERVOS    = 12
POSE_FILE     = "l4sr_poses.json"

LEG_NAMES     = ["FL (Front-Left)", "FR (Front-Right)", "BL (Back-Left)", "BR (Back-Right)"]
JOINT_NAMES   = ["Hip", "Knee", "Ankle"]

# channel index = leg*3 + joint  (matches firmware servo_pin layout transposed to flat list)
# servo_pin[leg][joint]:  0,4,8 | 1,5,9 | 2,6,10 | 3,7,11
# flat index = leg*3+joint, but physical channel = joint*4+leg
def flat_to_channel(flat_idx):
    leg   = flat_idx // 3
    joint = flat_idx % 3
    return joint * 4 + leg

# Default poses (angles for flat indices 0-11)
DEFAULT_POSES = {
    "Neutral":  [90]*12,
    "Sit":      [0, 45, 90,  0, 45, 90,  0, 45, 90,  0, 45, 90],
    "Stretch":  [180,135,90, 180,135,90, 180,135,90, 180,135,90],
    "Stand":    [90, 60, 120]*4,
}

# ─── Colours ─────────────────────────────────────────────────────────────────
BG         = "#1a1a2e"
PANEL      = "#16213e"
ACCENT     = "#0f3460"
HIGHLIGHT  = "#e94560"
TEXT       = "#eaeaea"
GREEN      = "#4ecca3"
YELLOW     = "#f5a623"
SLIDER_BG  = "#0d2137"

# ─── Serial Manager ──────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self):
        self.ser      = None
        self.lock     = threading.Lock()
        self.log_cb   = None   # callback(str)

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
                self._log("⚠️ Not connected")

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
        self.title("L4-SR Quadruped Controller")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(900, 680)

        self.serial_mgr   = SerialManager()
        self.serial_mgr.log_cb = self._append_log

        self.angles       = [tk.IntVar(value=90) for _ in range(NUM_SERVOS)]
        self.sliders      = []
        self.poses        = self._load_poses()
        self._pending_send = {}   # {servo_idx: after_id}  debounce

        self._build_ui()

    # ── Pose persistence ──────────────────────────────────────────────────────
    def _load_poses(self):
        if os.path.exists(POSE_FILE):
            try:
                with open(POSE_FILE) as f:
                    data = json.load(f)
                # merge with defaults (keep user additions)
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
        # Title bar
        hdr = tk.Frame(self, bg=HIGHLIGHT, height=4)
        hdr.pack(fill="x")

        title_frame = tk.Frame(self, bg=BG)
        title_frame.pack(fill="x", padx=16, pady=(10, 0))
        tk.Label(title_frame, text="L4-SR  QUADRUPED  CONTROLLER",
                 font=("Courier", 18, "bold"), bg=BG, fg=HIGHLIGHT).pack(side="left")
        self._conn_label = tk.Label(title_frame, text="● OFFLINE",
                                    font=("Courier", 11, "bold"), bg=BG, fg=YELLOW)
        self._conn_label.pack(side="right")

        # Top: Connection bar
        self._build_connection_bar()

        # Main paned area
        paned = tk.PanedWindow(self, orient="horizontal", bg=BG, sashwidth=6,
                                sashrelief="flat", bd=0)
        paned.pack(fill="both", expand=True, padx=8, pady=6)

        left  = tk.Frame(paned, bg=BG)
        right = tk.Frame(paned, bg=BG)
        paned.add(left,  minsize=560)
        paned.add(right, minsize=280)

        self._build_servo_panel(left)
        self._build_right_panel(right)

    def _build_connection_bar(self):
        bar = tk.Frame(self, bg=PANEL, pady=6)
        bar.pack(fill="x", padx=8, pady=(4, 0))

        tk.Label(bar, text="Port:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(10, 4))

        self._port_var = tk.StringVar()
        ports = self.serial_mgr.list_ports()
        self._port_combo = ttk.Combobox(bar, textvariable=self._port_var,
                                         values=ports, width=14,
                                         font=("Courier", 10))
        if ports:
            self._port_combo.set(ports[0])
        self._port_combo.pack(side="left", padx=4)

        self._style_btn(bar, "⟳ Refresh", self._refresh_ports,
                        ACCENT).pack(side="left", padx=4)
        self._connect_btn = self._style_btn(bar, "Connect", self._toggle_connect, GREEN)
        self._connect_btn.pack(side="left", padx=4)

        tk.Label(bar, text="Quick:", bg=PANEL, fg=TEXT,
                 font=("Courier", 10)).pack(side="left", padx=(20, 4))

        for label, cmd in [("All → 90", "S 90"), ("All → 0", "S 0"), ("All → 180", "S 180")]:
            self._style_btn(bar, label,
                            lambda c=cmd: self.serial_mgr.send(c),
                            ACCENT).pack(side="left", padx=3)

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
                                       bd=1, relief="groove",
                                       labelanchor="nw")
            leg_frame.pack(fill="x", padx=6, pady=5)

            for joint in range(3):
                flat  = leg * 3 + joint
                ch    = flat_to_channel(flat)
                row   = tk.Frame(leg_frame, bg=PANEL)
                row.pack(fill="x", padx=8, pady=3)

                lbl = tk.Label(row, text=f"{JOINT_NAMES[joint]:6s}  ch{ch:02d}",
                                width=14, anchor="w",
                                font=("Courier", 9), bg=PANEL, fg=TEXT)
                lbl.pack(side="left")

                val_lbl = tk.Label(row, textvariable=self.angles[flat],
                                   width=4, font=("Courier", 10, "bold"),
                                   bg=PANEL, fg=HIGHLIGHT)
                val_lbl.pack(side="right", padx=(0, 6))

                deg_lbl = tk.Label(row, text="°", font=("Courier", 10),
                                   bg=PANEL, fg=TEXT)
                deg_lbl.pack(side="right")

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
        # ── Pose Manager ──
        pose_frame = tk.LabelFrame(parent, text="  POSE MANAGER  ",
                                    bg=PANEL, fg=YELLOW,
                                    font=("Courier", 10, "bold"),
                                    bd=1, relief="groove")
        pose_frame.pack(fill="x", padx=6, pady=6)

        self._pose_var = tk.StringVar(value=list(self.poses.keys())[0])
        self._pose_combo = ttk.Combobox(pose_frame, textvariable=self._pose_var,
                                         values=list(self.poses.keys()),
                                         font=("Courier", 10), width=18)
        self._pose_combo.pack(padx=8, pady=(8, 4))

        btn_grid = tk.Frame(pose_frame, bg=PANEL)
        btn_grid.pack(padx=8, pady=4)

        self._style_btn(btn_grid, "▶ Move To",    self._move_to_pose,   GREEN ).grid(row=0, col=0, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "💾 Save As",   self._save_pose,      YELLOW).grid(row=0, col=1, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "🗑 Delete",    self._delete_pose,    HIGHLIGHT).grid(row=1, col=0, padx=4, pady=3, sticky="ew")
        self._style_btn(btn_grid, "⟳ Refresh",   self._refresh_pose_list, ACCENT).grid(row=1, col=1, padx=4, pady=3, sticky="ew")

        # Speed control
        spd_row = tk.Frame(pose_frame, bg=PANEL)
        spd_row.pack(padx=8, pady=(2, 8), fill="x")
        tk.Label(spd_row, text="Transition steps:", bg=PANEL, fg=TEXT,
                 font=("Courier", 9)).pack(side="left")
        self._steps_var = tk.IntVar(value=20)
        tk.Spinbox(spd_row, from_=1, to=100, textvariable=self._steps_var,
                   width=5, font=("Courier", 10),
                   bg=ACCENT, fg=TEXT, insertbackground=TEXT).pack(side="right")

        # ── Manual command ──
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

        # ── Serial Log ──
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

    # ── Slider handling with debounce ─────────────────────────────────────────
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
                    ch = flat_to_channel(i)
                    self.serial_mgr.send(f"s {ch} {a}")
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


# ─── Entry Point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Apply ttk style
    app = L4SRController()
    style = ttk.Style(app)
    style.theme_use("clam")
    style.configure("TScrollbar", background=ACCENT, troughcolor=BG, borderwidth=0)
    style.configure("TCombobox", fieldbackground=SLIDER_BG, background=ACCENT,
                    foreground=TEXT, selectbackground=ACCENT,
                    arrowcolor=TEXT)
    app.mainloop()
