"""
ESP32-C3 Drone — Controller with Live PID Tuning + Step Response Analysis
Tabs: Flight | Motor Test | PID Tuning | Step Response

Usage:  python3 drone_controller_tuning.py
Deps:   pip install bleak matplotlib
"""

import asyncio, threading, tkinter as tk, time, json, os, csv, math
from tkinter import ttk
from pathlib import Path
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError
import evdev
from evdev import ecodes as e


SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_UUID    = "abcdefab-cdef-abcd-efab-cdefabcdefab"
NOTIFY_UUID  = "abcdefab-cdef-abcd-efab-cdefabcdef00"
DEVICE_NAME  = "ESP32-C3-Drone"
DUTY_MAX     = 1023
SETTINGS_FILE = Path.home() / ".drone_pid_settings.json"

MOTOR_LABELS = ["M1 FL GPIO5 CW","M2 FR GPIO4 CCW",
                "M3 BL GPIO2 CCW","M4 BR GPIO3 CW"]

DEFAULT_GAINS = {
    # LQR K matrix rows — columns are [phi, phi_dot, theta, theta_dot]
    # These match the defaults from lqr_designer.py output
    "R": {"k0": 8.165882, "k1": 0.572471, "k2": 0.0, "k3": 0.0},
    "P": {"k0": 0.0,      "k1": 0.0,      "k2": 8.165882, "k3": 0.572471},
    "Y": {"kP": 2.0},  # yaw uses simple P on rate
}

def load_settings():
    try:
        with open(SETTINGS_FILE) as f:
            return json.load(f)
    except Exception:
        return DEFAULT_GAINS.copy()

def save_settings(gains):
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump(gains, f, indent=2)
    except Exception:
        pass

def compute_motors(T, P_deg, R_deg, Y):
    base = T / 100 * DUTY_MAX
    p = (P_deg / 30) * 150
    r = (R_deg / 30) * 150
    y = (Y / 100) * 100
    fl = max(0, min(DUTY_MAX, base + p + r - y))
    fr = max(0, min(DUTY_MAX, base + p - r + y))
    bl = max(0, min(DUTY_MAX, base - p + r + y))
    br = max(0, min(DUTY_MAX, base - p - r - y))
    return fl, fr, bl, br


# ═══════════════════════════════════════════════════════════════
#  BLE Manager
# ═══════════════════════════════════════════════════════════════
class BLEManager:
    def __init__(self):
        self.client    = None
        self.connected = False
        self.loop      = asyncio.new_event_loop()
        self.throttle  = 0
        self.pitch     = 0.0
        self.roll      = 0.0
        self.yaw       = 0
        self.on_connect    = None
        self.on_disconnect = None
        self.on_log        = None
        self.on_tx         = None
        self.on_notify     = None
        threading.Thread(target=lambda: (
            asyncio.set_event_loop(self.loop), self.loop.run_forever()
        ), daemon=True).start()

    def _log(self, msg, tag="inf"):
        if self.on_log: self.on_log(msg, tag)

    def connect(self):
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)

    def send_raw(self, packet: str):
        asyncio.run_coroutine_threadsafe(self._send_once(packet), self.loop)

    def set_controls(self, t, p, r, y):
        self.throttle=int(t); self.pitch=float(p)
        self.roll=float(r);   self.yaw=int(y)

    async def _send_once(self, packet: str):
        if not self.client or not self.client.is_connected: return
        try:
            await self.client.write_gatt_char(CHAR_UUID, packet.encode(), response=False)
        except Exception as e:
            self._log(f"TX error: {e}", "err")

    async def _connect(self):
        self._log(f"Scanning for '{DEVICE_NAME}'...")
        try:
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
            if device is None:
                self._log("Not found. Is it powered on?", "err")
                if self.on_disconnect: self.on_disconnect()
                return
            self._log(f"Found {device.name} [{device.address}]", "ok")
            last_exc = None
            for attempt in range(1, 4):
                try:
                    self._log(f"Connecting (attempt {attempt}/3)...")
                    self.client = BleakClient(device,
                        disconnected_callback=self._on_disc_cb, timeout=15.0)
                    await self.client.connect(timeout=15.0)
                    await asyncio.sleep(1.5)
                    if self.client.is_connected:
                        last_exc = None; break
                    raise BleakError("Dropped immediately")
                except Exception as e:
                    last_exc = e
                    self._log(f"Attempt {attempt} failed: {e}", "warn")
                    if attempt < 3: await asyncio.sleep(2.0)
            if last_exc: raise last_exc
            self.connected = True
            self._log("Connected!", "ok")
            try:
                await self.client.start_notify(NOTIFY_UUID, self._on_notify_cb)
                self._log("Notification channel open", "ok")
            except Exception:
                self._log("Notify subscribe failed", "warn")
            if self.on_connect: self.on_connect()
            await self._send_loop()
        except BleakError as e:
            self._log(f"BLE error: {e}", "err")
            if self.on_disconnect: self.on_disconnect()
        except Exception as e:
            self._log(f"Error: {e}", "err")
            if self.on_disconnect: self.on_disconnect()

    def _on_notify_cb(self, sender, data):
        msg = data.decode(errors="replace").strip()
        if self.on_notify: self.on_notify(msg)

    async def _disconnect(self):
        self.connected = False
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self._log("Disconnected.", "warn")
        if self.on_disconnect: self.on_disconnect()

    def _on_disc_cb(self, client):
        self.connected = False
        self._log("Disconnected unexpectedly!", "err")
        if self.on_disconnect: self.on_disconnect()

    async def _send_loop(self):
        interval = 1.0 / 20
        while self.connected and self.client and self.client.is_connected:
            pkt = f"T:{self.throttle},P:{int(self.pitch)},R:{int(self.roll)},Y:{self.yaw}"
            try:
                await self.client.write_gatt_char(CHAR_UUID, pkt.encode(), response=False)
                if self.on_tx: self.on_tx(pkt)
            except Exception as e:
                self._log(f"TX error: {e}", "err"); break
            await asyncio.sleep(interval)


# ═══════════════════════════════════════════════════════════════
#  Step response analyser (pure Python, no scipy needed)
# ═══════════════════════════════════════════════════════════════
def analyse_step(t_ms, setpoints, measured):
    """Returns dict with rise_time, overshoot_pct, settling_time, steady_state."""
    if len(t_ms) < 10:
        return None

    t  = [x / 1000.0 for x in t_ms]   # convert to seconds
    sp = setpoints
    m  = measured

    # Find step start: first index where setpoint changes significantly
    step_idx = 0
    for i in range(1, len(sp)):
        if abs(sp[i] - sp[0]) > 1.0:
            step_idx = i
            break

    if step_idx == 0:
        return None

    sp_init  = sp[0]
    sp_final = sp[step_idx]
    step_mag = sp_final - sp_init
    if abs(step_mag) < 0.5:
        return None

    # Steady-state: mean of last 20% of samples
    tail = m[int(len(m)*0.8):]
    steady = sum(tail) / len(tail)

    # Rise time: 10% → 90% of step magnitude
    lo = sp_init + 0.1 * step_mag
    hi = sp_init + 0.9 * step_mag
    t10 = t90 = None
    for i in range(step_idx, len(m)):
        if t10 is None and ((step_mag > 0 and m[i] >= lo) or (step_mag < 0 and m[i] <= lo)):
            t10 = t[i]
        if t90 is None and ((step_mag > 0 and m[i] >= hi) or (step_mag < 0 and m[i] <= hi)):
            t90 = t[i]
            break
    rise_time = (t90 - t10) if (t10 and t90) else None

    # Overshoot
    if step_mag > 0:
        peak = max(m[step_idx:])
    else:
        peak = min(m[step_idx:])
    overshoot_pct = abs((peak - sp_final) / step_mag * 100) if step_mag != 0 else 0

    # Settling time: last time response is outside ±5% band around steady state
    band = abs(step_mag) * 0.05
    settle_idx = len(m) - 1
    for i in range(len(m)-1, step_idx, -1):
        if abs(m[i] - steady) > band:
            settle_idx = i + 1
            break
    settling_time = t[settle_idx] - t[step_idx] if settle_idx < len(t) else None

    return {
        "step_start_s": t[step_idx],
        "sp_init": sp_init,
        "sp_final": sp_final,
        "steady_state": steady,
        "rise_time_s": rise_time,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "t": t, "sp": sp, "m": m,
    }


# ═══════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════
class DroneGUI:
    BG     = "#0a0c10"
    PANEL  = "#111520"
    BORDER = "#1e2a40"
    ACCENT = "#00e5ff"
    DANGER = "#ff3b5c"
    WARN   = "#ffb800"
    OK     = "#00e676"
    DIM    = "#4a5a7a"
    TEXT   = "#c8d8f0"
    MONO   = ("Courier New", 9)
    MONO_LG= ("Courier New", 10, "bold")
    MONO_SM= ("Courier New", 8)

    def __init__(self, root):
        self.root = root
        self.root.title("Drone Controller — Tuning")
        self.root.configure(bg=self.BG)
        self.root.resizable(True, True)
        self.root.minsize(540, 520)
        self.root.geometry("540x820")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.ble = BLEManager()
        self.ble.on_connect    = self._on_ble_connect
        self.ble.on_disconnect = self._on_ble_disconnect
        self.ble.on_log        = self._log
        self.ble.on_tx         = self._on_tx
        self.ble.on_notify     = self._on_notify

        self._tx_count  = 0
        self._tx_window = 0
        self._last_hz   = time.time()
        self._connected = False
        self._test_duty = tk.IntVar(value=20)
        self._gyro_live = False
        self._imu_roll  = tk.StringVar(value="--")
        self._imu_pitch = tk.StringVar(value="--")

        # Gains loaded from disk
        self._saved_gains = load_settings()
        self._gain_vars   = {}   # axis -> {kP, kI, kD} -> DoubleVar

        # Step response data
        self._log_rows    = []   # list of (dt_ms, sp, meas, fl, fr)
        self._receiving_log = False
        self._capture_axis  = tk.StringVar(value="R")

        self._setup_scroll()
        self._build_all(self.inner)
        self._update_motor_viz()
        self._tick_hz()
        self._log("Ready. Connect to drone.", "inf")

        # Game controller support (evdev for Steam Controller)
        self.gamepad_enabled  = False
        self.gamepad_device   = None
        self.deadzone         = 0.10    # adjust if sticks drift at rest
        self.last_gamepad_update = 0
        self.gamepad_debug    = False   # axis confirmed: ABS_HAT2X = right trigger
        self._seen_axes       = set()   # tracks which axis codes we have logged

    # ── Scrollable canvas ────────────────────────────────────
    def _setup_scroll(self):
        outer = tk.Frame(self.root, bg=self.BG)
        outer.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(outer, bg=self.BG, highlightthickness=0)
        sb = tk.Scrollbar(outer, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.inner = tk.Frame(self.canvas, bg=self.BG)
        self._cw = self.canvas.create_window((0,0), window=self.inner, anchor="nw")
        self.inner.bind("<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>",
            lambda e: self.canvas.itemconfig(self._cw, width=e.width))
        for ev, d in [("<MouseWheel>", None), ("<Button-4>", -1), ("<Button-5>", 1)]:
            if d is None:
                self.canvas.bind_all(ev,
                    lambda e: self.canvas.yview_scroll(int(-1*(e.delta/120)), "units"))
            else:
                self.canvas.bind_all(ev,
                    lambda e, dd=d: self.canvas.yview_scroll(dd, "units"))

    # ── Main layout ──────────────────────────────────────────
    def _build_all(self, parent):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Dark.TNotebook", background=self.BG, borderwidth=0)
        style.configure("Dark.TNotebook.Tab",
            background=self.PANEL, foreground=self.DIM,
            font=("Courier New", 8, "bold"), padding=[8, 5])
        style.map("Dark.TNotebook.Tab",
            background=[("selected", self.ACCENT)],
            foreground=[("selected", self.BG)])
        style.configure("Horizontal.TScale",
            background=self.PANEL, troughcolor="#1a2235",
            sliderlength=18, sliderrelief="flat")

        # Header
        hdr = tk.Frame(parent, bg=self.BG)
        hdr.pack(fill="x", padx=12, pady=(10,0))
        tk.Label(hdr, text="ESP32-C3 DRONE", font=self.MONO_LG,
                 fg=self.ACCENT, bg=self.BG).pack(side="left")
        self.status_lbl = tk.Label(hdr, text="● DISCONNECTED",
                                   font=self.MONO, fg=self.DIM, bg=self.BG)
        self.status_lbl.pack(side="right")
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill="x", padx=12, pady=4)

        # Connect + E-Stop — always near top
        btns = tk.Frame(parent, bg=self.BG)
        btns.pack(fill="x", padx=12, pady=(0,4))
        self.connect_btn = tk.Button(btns, text="CONNECT BLE",
            font=("Courier New",11,"bold"), bg=self.ACCENT, fg=self.BG,
            activebackground="#00b8cc", bd=0, pady=12, cursor="hand2",
            command=self._toggle_connect)
        self.connect_btn.pack(side="left", fill="x", expand=True, padx=(0,6))
        tk.Button(btns, text="E-STOP",
            font=("Courier New",11,"bold"), bg=self.DANGER, fg="white",
            activebackground="#cc2040", bd=0, pady=12, cursor="hand2",
            command=self._emergency_stop).pack(side="right", fill="x", expand=True)

        # Tabs
        nb = ttk.Notebook(parent, style="Dark.TNotebook")
        nb.pack(fill="x", padx=12, pady=4)
        tabs = {}
        for name in ("Flight", "Motor Test", "PID Tuning", "Step Response"):
            f = tk.Frame(nb, bg=self.BG)
            nb.add(f, text=f"  {name}  ")
            tabs[name] = f
        self._build_flight_tab(tabs["Flight"])
        self._build_test_tab(tabs["Motor Test"])
        self._build_pid_tab(tabs["PID Tuning"])
        self._build_step_tab(tabs["Step Response"])

        # Log
        lf = tk.Frame(parent, bg=self.PANEL)
        lf.pack(fill="x", padx=12, pady=(4,12))
        tk.Label(lf, text="// LOG", font=self.MONO, fg=self.DIM,
                 bg=self.PANEL, anchor="w").pack(fill="x", padx=10, pady=(6,2))
        self.log_text = tk.Text(lf, height=5, bg=self.BG, fg=self.DIM,
            font=self.MONO_SM, bd=0, state="disabled", wrap="word")
        self.log_text.pack(fill="x", padx=8, pady=(0,8))
        for tag, col in [("ok",self.OK),("err",self.DANGER),
                         ("inf",self.ACCENT),("warn",self.WARN)]:
            self.log_text.tag_config(tag, foreground=col)

    # ── Flight tab ───────────────────────────────────────────
    def _build_flight_tab(self, parent):
        warn = tk.Frame(parent, bg="#1a1400")
        warn.pack(fill="x", pady=(4,0))
        tk.Label(warn, text="  Remove props. P/R = target angle in degrees.",
                 font=self.MONO_SM, fg=self.WARN, bg="#1a1400").pack(padx=6, pady=5)
        ctrl = self._panel(parent, "// FLIGHT CONTROLS")
        self.throttle_var = tk.IntVar(value=0)
        self.pitch_var    = tk.IntVar(value=0)
        self.roll_var     = tk.IntVar(value=0)
        self.yaw_var      = tk.IntVar(value=0)
        self._slider(ctrl, "THROTTLE  T", self.throttle_var,  0,  100, "%")
        self._slider(ctrl, "PITCH     P", self.pitch_var,   -30,   30, "°")
        self._slider(ctrl, "ROLL      R", self.roll_var,    -30,   30, "°")
        self._slider(ctrl, "YAW RATE  Y", self.yaw_var,   -100,  100, "")
        self.hz_lbl = tk.Label(ctrl, text="TX: -- Hz", font=self.MONO,
                               fg=self.DIM, bg=self.PANEL, anchor="e")
        self.hz_lbl.pack(fill="x", padx=10, pady=(0,6))

        mviz = self._panel(parent, "// MOTOR OUTPUTS")
        mgrid = tk.Frame(mviz, bg=self.PANEL)
        mgrid.pack(fill="x", padx=10, pady=(0,8))
        self.motor_bars = {}
        self.motor_lbls = {}

        for tag, row, col, color in [
                ("FL",0,0,self.ACCENT),("FR",0,1,self.ACCENT),
                ("BL",1,0,"#b060ff"), ("BR",1,1,"#b060ff")]:
            cell = tk.Frame(mgrid, bg="#0d1220")
            cell.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
            mgrid.columnconfigure(col, weight=1)
            tk.Label(cell, text=tag, font=self.MONO_LG, fg=color,
                     bg="#0d1220").pack(pady=(5,2))
            bg_bar = tk.Frame(cell, bg=self.BORDER, height=8)
            bg_bar.pack(fill="x", padx=8)
            bg_bar.pack_propagate(False)
            bar = tk.Frame(bg_bar, bg=color, height=8)
            bar.place(x=0, y=0, relheight=1.0, relwidth=0)
            self.motor_bars[tag] = (bar, bg_bar, color)
            lbl = tk.Label(cell, text="0%", font=self.MONO, fg=self.TEXT, bg="#0d1220")
            lbl.pack(pady=(2,5))
            self.motor_lbls[tag] = lbl

        # ── Gamepad panel (below motor outputs) ──────────────
        gp_frame = self._panel(parent, "// GAME CONTROLLER")
        self.gp_status = tk.Label(gp_frame, text="Gamepad: OFF",
                                  font=self.MONO, fg=self.DIM, bg=self.PANEL)
        self.gp_status.pack(fill="x", padx=10, pady=4)
        tk.Label(gp_frame,
                 text="Left stick: Roll/Pitch  |  Right stick X: Yaw  |  Right trigger: Throttle",
                 font=self.MONO_SM, fg=self.DIM, bg=self.PANEL).pack(
            fill="x", padx=10, pady=(0,4))
        btn_frame = tk.Frame(gp_frame, bg=self.PANEL)
        btn_frame.pack(fill="x", padx=10, pady=(0,8))
        tk.Button(btn_frame, text="Enable Gamepad",
                  font=("Courier New",9,"bold"), bg="#0d1a20", fg=self.ACCENT,
                  activebackground=self.ACCENT, activeforeground=self.BG,
                  relief="flat", bd=1, pady=6, cursor="hand2",
                  command=self._toggle_gamepad).pack(
            side="left", fill="x", expand=True, padx=(0,6))
        tk.Button(btn_frame, text="Disable Gamepad",
                  font=("Courier New",9,"bold"), bg="#1a0a0a", fg=self.DANGER,
                  activebackground=self.DANGER, activeforeground="white",
                  relief="flat", bd=1, pady=6, cursor="hand2",
                  command=self._disable_gamepad).pack(
            side="left", fill="x", expand=True)

    # ── Motor test tab ───────────────────────────────────────
    def _build_test_tab(self, parent):
        imu_f = self._panel(parent, "// IMU angles")
        ir = tk.Frame(imu_f, bg=self.PANEL)
        ir.pack(fill="x", padx=10, pady=(0,4))
        for lbl, var in [("Roll:", self._imu_roll), ("Pitch:", self._imu_pitch)]:
            tk.Label(ir, text=lbl, font=self.MONO, fg=self.DIM,
                     bg=self.PANEL, width=7, anchor="w").pack(side="left")
            tk.Label(ir, textvariable=var, font=self.MONO_LG,
                     fg=self.ACCENT, bg=self.PANEL, width=8).pack(side="left")
        ib = tk.Frame(imu_f, bg=self.PANEL)
        ib.pack(fill="x", padx=10, pady=(0,8))
        tk.Button(ib, text="IMU snapshot", font=self.MONO, bg="#0d1220",
                  fg=self.ACCENT, relief="flat", bd=1, cursor="hand2",
                  command=lambda: self.ble.send_raw("IMU")).pack(side="left", padx=(0,8))
        self.gyro_btn = tk.Button(ib, text="Start gyro live", font=self.MONO,
                  bg="#0d1220", fg=self.ACCENT, relief="flat", bd=1, cursor="hand2",
                  command=self._toggle_gyro_live)
        self.gyro_btn.pack(side="left")

        test_f = self._panel(parent, "// Individual motor test  (props OFF!)")
        dr = tk.Frame(test_f, bg=self.PANEL)
        dr.pack(fill="x", padx=10, pady=(0,6))
        tk.Label(dr, text="Test duty:", font=self.MONO, fg=self.DIM,
                 bg=self.PANEL).pack(side="left")
        self._duty_lbl = tk.Label(dr, text="20%", font=self.MONO_LG,
                                  fg=self.ACCENT, bg=self.PANEL, width=5)
        self._duty_lbl.pack(side="right")
        ttk.Scale(dr, from_=5, to=60, orient="horizontal",
                  variable=self._test_duty, length=220).pack(
            side="left", fill="x", expand=True, padx=6)
        self._test_duty.trace_add("write",
            lambda *_: self._duty_lbl.config(text=f"{self._test_duty.get()}%"))

        mgrid = tk.Frame(test_f, bg=self.PANEL)
        mgrid.pack(fill="x", padx=10, pady=(0,6))
        self._motor_btns = []
        for idx, (row, col, color) in enumerate([
                (0,0,self.ACCENT),(0,1,self.ACCENT),(1,0,"#b060ff"),(1,1,"#b060ff")]):
            btn = tk.Button(mgrid, text=MOTOR_LABELS[idx],
                font=("Courier New",8,"bold"), bg="#0d1220", fg=color,
                activebackground=color, activeforeground=self.BG,
                relief="flat", bd=1, pady=10, cursor="hand2",
                command=lambda i=idx: self._test_motor(i))
            btn.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
            mgrid.columnconfigure(col, weight=1)
            self._motor_btns.append((btn, color))

        ar = tk.Frame(test_f, bg=self.PANEL)
        ar.pack(fill="x", padx=10, pady=(0,10))
        tk.Button(ar, text="All ON", font=("Courier New",9,"bold"),
                  bg="#0d1a0d", fg=self.OK, activebackground=self.OK,
                  activeforeground=self.BG, relief="flat", bd=1, pady=8,
                  cursor="hand2", command=self._test_all
                  ).pack(side="left", fill="x", expand=True, padx=(0,6))
        tk.Button(ar, text="All OFF", font=("Courier New",9,"bold"),
                  bg="#1a0a0a", fg=self.DANGER, activebackground=self.DANGER,
                  activeforeground="white", relief="flat", bd=1, pady=8,
                  cursor="hand2", command=self._test_stop
                  ).pack(side="right", fill="x", expand=True)

    # ── LQR tuning tab ───────────────────────────────────────
    def _build_pid_tab(self, parent):
        info = self._panel(parent, "// Live LQR tuning")
        tk.Label(info,
            text="Run lqr_designer.py on your laptop to compute K values.\n"
                 "Enter the K row values here and Send — no reflash needed.\n"
                 "Settings auto-save to ~/.drone_pid_settings.json",
            font=self.MONO_SM, fg=self.DIM, bg=self.PANEL,
            justify="left").pack(padx=12, pady=(0,8))

        # Roll and Pitch rows — 4 K values each
        for axis, label, col_labels in [
                ("R", "Roll  K row  [phi, phi_dot, theta, theta_dot]",
                       ["k0 (phi)",  "k1 (phi_dot)", "k2 (theta)", "k3 (th_dot)"]),
                ("P", "Pitch K row  [phi, phi_dot, theta, theta_dot]",
                       ["k0 (phi)",  "k1 (phi_dot)", "k2 (theta)", "k3 (th_dot)"])]:
            saved = self._saved_gains.get(axis, DEFAULT_GAINS[axis])
            self._gain_vars[axis] = {}
            frame = self._panel(parent, f"// {label}")
            for param, col_label, lo, hi in [
                    ("k0", col_labels[0],  0.0, 50.0),
                    ("k1", col_labels[1],  0.0, 10.0),
                    ("k2", col_labels[2],  0.0, 50.0),
                    ("k3", col_labels[3],  0.0, 10.0)]:
                var = tk.DoubleVar(value=saved.get(param, DEFAULT_GAINS[axis].get(param, 0.0)))
                self._gain_vars[axis][param] = var
                row = tk.Frame(frame, bg=self.PANEL)
                row.pack(fill="x", padx=10, pady=2)
                tk.Label(row, text=col_label, font=self.MONO, fg=self.ACCENT,
                         bg=self.PANEL, width=14, anchor="w").pack(side="left")
                val_lbl = tk.Label(row, text=f"{var.get():.4f}", font=self.MONO,
                                   fg=self.TEXT, bg=self.PANEL, width=8, anchor="e")
                val_lbl.pack(side="right")
                sl = ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                               variable=var, length=220)
                sl.pack(side="left", fill="x", expand=True, padx=6)
                def on_gain(*_, vr=var, vl=val_lbl):
                    vl.config(text=f"{vr.get():.4f}")
                var.trace_add("write", on_gain)

            bf = tk.Frame(frame, bg=self.PANEL)
            bf.pack(fill="x", padx=10, pady=(4,10))
            tk.Button(bf, text=f"Send {axis} row to drone",
                font=("Courier New",9,"bold"), bg="#0d1a20", fg=self.ACCENT,
                activebackground=self.ACCENT, activeforeground=self.BG,
                relief="flat", bd=1, pady=6, cursor="hand2",
                command=lambda ax=axis: self._send_gains(ax)
            ).pack(side="left", fill="x", expand=True, padx=(0,6))
            tk.Button(bf, text="Reset",
                font=("Courier New",9,"bold"), bg="#0d1220", fg=self.DIM,
                relief="flat", bd=1, pady=6, cursor="hand2",
                command=lambda ax=axis: self._reset_gains(ax)
            ).pack(side="right")

        # Yaw — just kP
        saved_y = self._saved_gains.get("Y", DEFAULT_GAINS["Y"])
        self._gain_vars["Y"] = {}
        yframe = self._panel(parent, "// Yaw rate  (simple P on gyro Z)")
        yrow = tk.Frame(yframe, bg=self.PANEL)
        yrow.pack(fill="x", padx=10, pady=4)
        tk.Label(yrow, text="kP", font=self.MONO, fg=self.ACCENT,
                 bg=self.PANEL, width=14, anchor="w").pack(side="left")
        yvar = tk.DoubleVar(value=saved_y.get("kP", 2.0))
        self._gain_vars["Y"]["kP"] = yvar
        y_lbl = tk.Label(yrow, text=f"{yvar.get():.3f}", font=self.MONO,
                         fg=self.TEXT, bg=self.PANEL, width=8, anchor="e")
        y_lbl.pack(side="right")
        ttk.Scale(yrow, from_=0.0, to=10.0, orient="horizontal",
                  variable=yvar, length=220).pack(
            side="left", fill="x", expand=True, padx=6)
        yvar.trace_add("write", lambda *_: y_lbl.config(text=f"{yvar.get():.3f}"))
        ybf = tk.Frame(yframe, bg=self.PANEL)
        ybf.pack(fill="x", padx=10, pady=(0,10))
        tk.Button(ybf, text="Send Yaw kP",
            font=("Courier New",9,"bold"), bg="#0d1a20", fg=self.ACCENT,
            activebackground=self.ACCENT, activeforeground=self.BG,
            relief="flat", bd=1, pady=6, cursor="hand2",
            command=lambda: self._send_gains("Y")
        ).pack(fill="x")

        save_row = tk.Frame(parent, bg=self.BG)
        save_row.pack(fill="x", padx=12, pady=(4,8))
        tk.Button(save_row, text="Save all to disk",
            font=("Courier New",10,"bold"), bg=self.OK, fg=self.BG,
            activebackground="#00b060", bd=0, pady=10, cursor="hand2",
            command=self._save_all_gains
        ).pack(fill="x")

    # ── Step response tab ────────────────────────────────────
    def _build_step_tab(self, parent):
        info = self._panel(parent, "// Step response capture")
        tk.Label(info,
            text="1. Set throttle to hover level in Flight tab.\n"
                 "2. Choose axis, click ARM CAPTURE.\n"
                 "3. Make a sudden step input on that axis slider.\n"
                 "4. Wait ~5 s, then click FETCH DATA.\n"
                 "5. Click PLOT to see the response curve.",
            font=self.MONO_SM, fg=self.TEXT, bg=self.PANEL,
            justify="left").pack(padx=12, pady=(0,8))

        ax_f = self._panel(parent, "// Capture axis")
        ar = tk.Frame(ax_f, bg=self.PANEL)
        ar.pack(fill="x", padx=10, pady=(0,8))
        for val, lbl in [("R","Roll"), ("P","Pitch")]:
            tk.Radiobutton(ar, text=lbl, variable=self._capture_axis,
                value=val, font=self.MONO_LG, fg=self.ACCENT, bg=self.PANEL,
                selectcolor=self.BG, activebackground=self.PANEL,
                activeforeground=self.ACCENT).pack(side="left", padx=8)

        ctrl_f = self._panel(parent, "// Controls")
        cr = tk.Frame(ctrl_f, bg=self.PANEL)
        cr.pack(fill="x", padx=10, pady=(0,10))
        self.arm_btn = tk.Button(cr, text="ARM CAPTURE",
            font=("Courier New",9,"bold"), bg="#0d1a20", fg=self.ACCENT,
            activebackground=self.ACCENT, activeforeground=self.BG,
            relief="flat", bd=1, pady=8, cursor="hand2",
            command=self._arm_capture)
        self.arm_btn.pack(side="left", fill="x", expand=True, padx=(0,4))
        tk.Button(cr, text="FETCH DATA",
            font=("Courier New",9,"bold"), bg="#0d1a20", fg=self.WARN,
            activebackground=self.WARN, activeforeground=self.BG,
            relief="flat", bd=1, pady=8, cursor="hand2",
            command=lambda: self.ble.send_raw("LOGSTREAM")
        ).pack(side="left", fill="x", expand=True, padx=(0,4))
        tk.Button(cr, text="PLOT",
            font=("Courier New",9,"bold"), bg="#0d1a20", fg=self.OK,
            activebackground=self.OK, activeforeground=self.BG,
            relief="flat", bd=1, pady=8, cursor="hand2",
            command=self._plot_step).pack(side="left", fill="x", expand=True, padx=(0,4))
        tk.Button(cr, text="SAVE CSV",
            font=("Courier New",9,"bold"), bg="#0d1220", fg=self.DIM,
            relief="flat", bd=1, pady=8, cursor="hand2",
            command=self._save_csv).pack(side="left", fill="x", expand=True)

        # Results readout
        res_f = self._panel(parent, "// Analysis results")
        self._result_vars = {}
        for key, label in [
                ("rise_time_s",    "Rise time"),
                ("overshoot_pct",  "Overshoot"),
                ("settling_time_s","Settling time"),
                ("steady_state",   "Steady state"),
        ]:
            row = tk.Frame(res_f, bg=self.PANEL)
            row.pack(fill="x", padx=10, pady=2)
            tk.Label(row, text=f"{label}:", font=self.MONO, fg=self.DIM,
                     bg=self.PANEL, width=16, anchor="w").pack(side="left")
            var = tk.StringVar(value="--")
            self._result_vars[key] = var
            tk.Label(row, textvariable=var, font=self.MONO_LG,
                     fg=self.ACCENT, bg=self.PANEL).pack(side="left")

        self._sample_count = tk.StringVar(value="Samples: 0")
        tk.Label(res_f, textvariable=self._sample_count,
                 font=self.MONO_SM, fg=self.DIM, bg=self.PANEL,
                 anchor="w").pack(fill="x", padx=10, pady=(4,8))

    # ── PID actions ──────────────────────────────────────────
    def _send_gains(self, axis):
        v = self._gain_vars[axis]
        if axis == "Y":
            kp = v["kP"].get()
            pkt = f"PID:Y,{kp:.4f},0,0"
            self.ble.send_raw(pkt)
            self._log(f"Sent Yaw kP={kp:.4f}", "inf")
        else:
            k0 = v["k0"].get(); k1 = v["k1"].get()
            k2 = v["k2"].get(); k3 = v["k3"].get()
            pkt = f"LQR:{axis},{k0:.6f},{k1:.6f},{k2:.6f},{k3:.6f}"
            self.ble.send_raw(pkt)
            self._log(f"Sent LQR {axis}: [{k0:.4f}, {k1:.4f}, {k2:.4f}, {k3:.4f}]", "inf")

    def _reset_gains(self, axis):
        defs = DEFAULT_GAINS[axis]
        for param, val in defs.items():
            if param in self._gain_vars.get(axis, {}):
                self._gain_vars[axis][param].set(val)

    def _save_all_gains(self):
        gains = {}
        for axis in ("R","P","Y"):
            gains[axis] = {p: self._gain_vars[axis][p].get()
                           for p in self._gain_vars[axis]}
        save_settings(gains)
        self._log(f"Gains saved to {SETTINGS_FILE}", "ok")

    # ── Step response actions ────────────────────────────────
    def _arm_capture(self):
        axis = self._capture_axis.get()
        self._log_rows = []
        self._receiving_log = False
        cmd = f"CAPTURE:{axis}"
        self.ble.send_raw(cmd)
        self.arm_btn.config(bg=self.WARN, fg=self.BG, text="ARMED...")
        self._log(f"Capture armed on {axis} axis. Make a step input now!", "warn")
        self.root.after(5500, lambda: self.arm_btn.config(
            bg="#0d1a20", fg=self.ACCENT, text="ARM CAPTURE"))

    def _plot_step(self):
        if len(self._log_rows) < 10:
            self._log("Not enough data — fetch first", "err")
            return
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as mpatches
        except ImportError:
            self._log("pip install matplotlib", "err")
            return

        t_ms = [r[0] for r in self._log_rows]
        sp   = [r[1] for r in self._log_rows]
        meas = [r[2] for r in self._log_rows]
        fl   = [r[3]/DUTY_MAX*100 for r in self._log_rows]

        result = analyse_step(t_ms, sp, meas)
        if result:
            self._update_results(result)

        t_s = [x/1000.0 for x in t_ms]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        fig.patch.set_facecolor("#0a0c10")
        for ax in (ax1, ax2):
            ax.set_facecolor("#111520")
            ax.tick_params(colors="#c8d8f0")
            ax.spines[:].set_color("#1e2a40")

        ax1.plot(t_s, sp,   color="#4a5a7a", linewidth=1.5,
                 linestyle="--", label="Setpoint")
        ax1.plot(t_s, meas, color="#00e5ff", linewidth=2,   label="Measured")

        if result:
            ss = result["steady_state"]
            step_s = result["step_start_s"]
            ax1.axhline(ss, color="#ffb800", linewidth=0.8, linestyle=":")

            if result["rise_time_s"]:
                ax1.annotate(
                    f"Rise: {result['rise_time_s']*1000:.0f} ms",
                    xy=(step_s + result["rise_time_s"], ss*0.9),
                    color="#00e676", fontsize=9,
                    arrowprops=dict(arrowstyle="->", color="#00e676"))
            if result["settling_time_s"]:
                ax1.annotate(
                    f"Settle: {result['settling_time_s']*1000:.0f} ms",
                    xy=(step_s + result["settling_time_s"], ss),
                    xytext=(step_s + result["settling_time_s"] + 0.2, ss * 1.2),
                    color="#ffb800", fontsize=9,
                    arrowprops=dict(arrowstyle="->", color="#ffb800"))
            ax1.annotate(
                f"Overshoot: {result['overshoot_pct']:.1f}%",
                xy=(t_s[meas.index(max(meas))], max(meas)),
                xytext=(t_s[meas.index(max(meas))]+0.1, max(meas)*1.05),
                color="#ff3b5c", fontsize=9,
                arrowprops=dict(arrowstyle="->", color="#ff3b5c"))

        ax1.set_ylabel("Angle (°)", color="#c8d8f0")
        ax1.set_title("Step Response", color="#c8d8f0", fontsize=11)
        ax1.legend(facecolor="#111520", edgecolor="#1e2a40",
                   labelcolor="#c8d8f0", fontsize=9)

        ax2.plot(t_s, fl, color="#b060ff", linewidth=1.5, label="FL motor %")
        ax2.set_ylabel("Motor output %", color="#c8d8f0")
        ax2.set_xlabel("Time (s)", color="#c8d8f0")
        ax2.legend(facecolor="#111520", edgecolor="#1e2a40",
                   labelcolor="#c8d8f0", fontsize=9)

        fig.tight_layout()
        plt.show()

    def _update_results(self, r):
        self._result_vars["rise_time_s"].set(
            f"{r['rise_time_s']*1000:.1f} ms" if r["rise_time_s"] else "--")
        self._result_vars["overshoot_pct"].set(f"{r['overshoot_pct']:.1f}%")
        self._result_vars["settling_time_s"].set(
            f"{r['settling_time_s']*1000:.1f} ms" if r["settling_time_s"] else "--")
        self._result_vars["steady_state"].set(f"{r['steady_state']:.2f}°")

    def _save_csv(self):
        if not self._log_rows:
            self._log("No data to save", "err"); return
        path = Path.home() / f"drone_step_{int(time.time())}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_ms","setpoint_deg","measured_deg","motor_FL_duty","motor_FR_duty"])
            w.writerows(self._log_rows)
        self._log(f"Saved {len(self._log_rows)} rows to {path}", "ok")

    # ── Notification handler ─────────────────────────────────
    def _on_notify(self, msg):
        self.root.after(0, lambda: self._process_notify(msg))

    def _process_notify(self, msg):
        # Step log streaming
        if msg == "LOG:START":
            self._log_rows = []
            self._receiving_log = True
            self._log("Receiving step data...", "inf")
            return
        if msg == "LOG:END":
            self._receiving_log = False
            self._sample_count.set(f"Samples: {len(self._log_rows)}")
            self._log(f"Data received: {len(self._log_rows)} samples. Click PLOT.", "ok")
            # Auto-analyse
            if len(self._log_rows) > 10:
                t_ms = [r[0] for r in self._log_rows]
                sp   = [r[1] for r in self._log_rows]
                meas = [r[2] for r in self._log_rows]
                result = analyse_step(t_ms, sp, meas)
                if result:
                    self._update_results(result)
            return
        if msg == "LOG:READY":
            self._log("Capture complete on drone. Click FETCH DATA.", "ok")
            return
        if msg == "LOG:ARMED":
            self._log("Drone armed. Make your step input!", "warn")
            return
        if self._receiving_log and msg.startswith("D:"):
            try:
                parts = msg[2:].split(",")
                # idx, dt_ms, sp, meas, fl, fr
                row = (int(parts[1]), int(parts[2]),
                       int(parts[3]), int(parts[4]), int(parts[5]))
                self._log_rows.append(row)
            except Exception:
                pass
            return

        # PID ack — sync sliders with what drone confirmed
        if msg.startswith("LQR:ACK:") or msg.startswith("PID:ACK:"):
            try:
                rest = msg[8:]           # e.g. "R,8.1659,0.5725,0.0,0.0"
                axis = rest[0]
                vals = [float(x) for x in rest[2:].split(",")]
                if axis in self._gain_vars:
                    params = list(self._gain_vars[axis].keys())
                    for i, p in enumerate(params):
                        if i < len(vals):
                            self._gain_vars[axis][p].set(vals[i])
                self._log(f"Drone confirmed {axis}: {vals}", "ok")
            except Exception:
                pass
            return

        # IMU / gyro angle updates
        if msg.startswith(("IMU:", "GYRO:")):
            try:
                for part in msg.split(":")[1].split(","):
                    k, v = part.split("=")
                    if k == "R": self._imu_roll.set(f"{float(v):+.1f}°")
                    if k == "P": self._imu_pitch.set(f"{float(v):+.1f}°")
            except Exception:
                pass
            return

        self._log(f"<- {msg}", "ok")

    # ── Motor test helpers ───────────────────────────────────
    def _test_motor(self, idx):
        duty = self._test_duty.get()
        self._highlight_btn(idx)
        self.ble.send_raw(f"TEST:{idx},{duty}")
        self._log(f"Testing {MOTOR_LABELS[idx]} at {duty}%", "inf")

    def _test_all(self):
        self._highlight_btn(-1)
        self.ble.send_raw(f"TEST:4,{self._test_duty.get()}")

    def _test_stop(self):
        self._highlight_btn(-1)
        self.ble.send_raw("TEST:4,0")

    def _highlight_btn(self, active):
        for i, (btn, color) in enumerate(self._motor_btns):
            btn.config(bg=color if i==active else "#0d1220",
                       fg=self.BG if i==active else color)

    def _toggle_gyro_live(self):
        self._gyro_live = not self._gyro_live
        if self._gyro_live:
            self.gyro_btn.config(text="Stop gyro live", fg=self.WARN)
            self.ble.send_raw("GYRO")
        else:
            self.gyro_btn.config(text="Start gyro live", fg=self.ACCENT)
            self.ble.send_raw("GYROSTOP")

    # ── Flight helpers ───────────────────────────────────────
    def _push_controls(self):
        self.ble.set_controls(self.throttle_var.get(), self.pitch_var.get(),
                              self.roll_var.get(),     self.yaw_var.get())

    def _update_motor_viz(self):
        try:
            fl, fr, bl, br = compute_motors(
                self.throttle_var.get(), self.pitch_var.get(),
                self.roll_var.get(),     self.yaw_var.get())
            for tag, duty in [("FL",fl),("FR",fr),("BL",bl),("BR",br)]:
                pct = duty / DUTY_MAX
                bar, _, base_color = self.motor_bars[tag]
                bar.place(relwidth=pct)
                bar.config(bg=self.DANGER if pct>0.85 else
                             self.WARN    if pct>0.50 else base_color)
                self.motor_lbls[tag].config(text=f"{int(pct*100)}%")
        except Exception:
            pass
        self.root.after(60, self._update_motor_viz)

    # ── Panel / slider helpers ───────────────────────────────
    def _panel(self, parent, title):
        f = tk.Frame(parent, bg=self.PANEL)
        f.pack(fill="x", pady=3)
        tk.Label(f, text=title, font=self.MONO, fg=self.DIM,
                 bg=self.PANEL, anchor="w").pack(fill="x", padx=10, pady=(7,3))
        return f

    def _slider(self, parent, label, var, lo, hi, unit):
        row = tk.Frame(parent, bg=self.PANEL)
        row.pack(fill="x", padx=10, pady=3)
        tk.Label(row, text=label, font=self.MONO, fg=self.ACCENT,
                 bg=self.PANEL, width=14, anchor="w").pack(side="left")
        val_lbl = tk.Label(row, text=f"0{unit}", font=self.MONO,
                           fg=self.TEXT, bg=self.PANEL, width=6, anchor="e")
        val_lbl.pack(side="right")
        ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                  variable=var, length=240).pack(
            side="left", fill="x", expand=True, padx=6)
        def on(*_):
            val_lbl.config(text=f"{var.get()}{unit}")
            self._push_controls()
        var.trace_add("write", on)

    # ====================== STEAM CONTROLLER via evdev ======================
    def _init_gamepad(self):
        try:
            devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
            for dev in devices:
                if any(x in dev.name.lower() for x in ["steam", "xbox", "360", "controller", "wireless"]):
                    self.gamepad_device = dev
                    self.gamepad_enabled = True
                    self._log(f"Gamepad: {dev.name} ({dev.path})", "ok")

                    # Probe and log all absolute axes so we can see what the trigger reports
                    caps = dev.capabilities(verbose=True)
                    abs_axes = caps.get(("EV_ABS", 3), [])
                    self._throttle_axis  = None   # will be set on first trigger event
                    self._throttle_max   = 255
                    self._log("Absolute axes on this device:", "inf")
                    for name_tuple, info in abs_axes:
                        name = name_tuple[0] if isinstance(name_tuple, tuple) else name_tuple
                        self._log(f"  {name}: min={info.min} max={info.max} cur={info.value}", "inf")
                    return True

            self._log("No gamepad found. Is the Steam Controller turned on and paired?", "warn")
            return False
        except ImportError:
            self._log("evdev not installed. Run: pip install evdev", "err")
            return False
        except Exception as e:
            self._log(f"Gamepad init failed: {e}", "err")
            return False

    # Axis codes that various controllers use for the right trigger
    TRIGGER_AXES = {
        "ABS_RZ":       evdev.ecodes.ABS_RZ,        # Xbox, most gamepads
        "ABS_GAS":      evdev.ecodes.ABS_GAS,        # some Steam Controller modes
        "ABS_HAT2X":    evdev.ecodes.ABS_HAT2X,      # Steam Controller right trigger
        "ABS_BRAKE":    evdev.ecodes.ABS_BRAKE,      # alternate Steam mapping
    }

    def _apply_throttle(self, raw, axis_code):
        """Normalise a raw trigger value to 0-100 throttle using absinfo max."""
        try:
            info = self.gamepad_device.absinfo(axis_code)
            max_val = info.max if info and info.max > 0 else 255
            min_val = info.min if info else 0
        except Exception:
            max_val = 255; min_val = 0
        span = max_val - min_val
        if span == 0:
            return
        throttle = max(0.0, min(1.0, (raw - min_val) / span))
        self.throttle_var.set(int(throttle * 100))

    def _update_from_gamepad(self):
        """Non-blocking gamepad poll — reads all queued events then returns.
        Uses read_one() which returns None when the queue is empty,
        so this never stalls the tkinter main loop."""
        if not self.gamepad_enabled or not self.gamepad_device:
            return

        trigger_codes = set(self.TRIGGER_AXES.values())

        try:
            while True:
                event = self.gamepad_device.read_one()
                if event is None:
                    break
                if event.type != evdev.ecodes.EV_ABS:
                    continue

                code = event.code
                raw  = event.value

                # Debug: log any unrecognised axis the first time we see it
                if self.gamepad_debug and code not in self._seen_axes:
                    self._seen_axes.add(code)
                    name = evdev.ecodes.ABS.get(code, f"ABS_{code}")
                    self._log(f"[GP] New axis: {name} (code={code}) raw={raw}", "inf")

                if code == evdev.ecodes.ABS_X:
                    val = raw / 32768.0
                    if abs(val) < self.deadzone: val = 0.0
                    self.roll_var.set(int(max(-30, min(30, val * 30))))

                elif code == evdev.ecodes.ABS_Y:
                    val = raw / 32768.0
                    if abs(val) < self.deadzone: val = 0.0
                    self.pitch_var.set(int(max(-30, min(30, -val * 30))))

                elif code == evdev.ecodes.ABS_RX:
                    val = raw / 32768.0
                    if abs(val) < self.deadzone: val = 0.0
                    self.yaw_var.set(int(max(-100, min(100, val * 100))))

                elif code in trigger_codes:
                    # Right trigger — whichever axis code this controller uses
                    if self.gamepad_debug:
                        name = evdev.ecodes.ABS.get(code, f"ABS_{code}")
                        self._log(f"[GP] Trigger axis={name} raw={raw}", "inf")
                    self._apply_throttle(raw, code)

        except BlockingIOError:
            pass
        except OSError:
            self._log("Gamepad disconnected", "warn")
            self._disable_gamepad()
        except Exception as ex:
            self._log(f"Gamepad read error: {ex}", "warn")
            self._disable_gamepad()

    def _toggle_gamepad(self):
        if not self.gamepad_enabled:
            if self._init_gamepad():
                self.gp_status.config(text=f"Gamepad: {self.gamepad_device.name}", fg=self.OK)
                self.root.after(10, self._gamepad_tick)
        else:
            self._disable_gamepad()

    def _disable_gamepad(self):
        self.gamepad_enabled = False
        if self.gamepad_device:
            try:
                self.gamepad_device.close()
            except:
                pass
            self.gamepad_device = None
        self.gp_status.config(text="Gamepad: OFF", fg=self.DIM)
        self._log("Gamepad disabled", "inf")

    def _gamepad_tick(self):
        if self.gamepad_enabled:
            self._update_from_gamepad()
            self._push_controls()   # push whatever the gamepad just set
        self.root.after(15, self._gamepad_tick)

    # ── BLE state ────────────────────────────────────────────
    def _on_ble_connect(self):    self.root.after(0, self._gui_connect)
    def _on_ble_disconnect(self): self.root.after(0, self._gui_disconnect)
    def _on_tx(self, pkt):
        self._tx_count += 1; self._tx_window += 1

    def _gui_connect(self):
        self._connected = True
        self.status_lbl.config(text="● CONNECTED", fg=self.OK)
        self.connect_btn.config(text="DISCONNECT", bg=self.WARN, fg=self.BG)
        # Send saved gains on connect
        self.root.after(1000, self._send_all_saved_gains)

    def _send_all_saved_gains(self):
        for axis in ("R","P","Y"):
            self._send_gains(axis)
            time.sleep(0.05)

    def _gui_disconnect(self):
        self._connected = False
        self._gyro_live = False
        self.status_lbl.config(text="● DISCONNECTED", fg=self.DIM)
        self.connect_btn.config(text="CONNECT BLE", bg=self.ACCENT, fg=self.BG)

    def _toggle_connect(self):
        if self._connected:
            self.ble.disconnect()
        else:
            self.status_lbl.config(text="SCANNING...", fg=self.WARN)
            self.connect_btn.config(state="disabled")
            self.root.after(600, lambda: self.connect_btn.config(state="normal"))
            self.ble.connect()

    def _emergency_stop(self):
        for v in [self.throttle_var, self.pitch_var, self.roll_var, self.yaw_var]:
            v.set(0)
        self._push_controls()
        self.ble.send_raw("TEST:4,0")
        self._log("EMERGENCY STOP", "err")

    def _tick_hz(self):
        now = time.time()
        dt  = now - self._last_hz
        if dt >= 1.0:
            hz = self._tx_window / dt
            self._tx_window = 0; self._last_hz = now
            self.hz_lbl.config(text=f"TX: {hz:.1f} Hz  |  total: {self._tx_count}")
        self.root.after(1000, self._tick_hz)

    def _log(self, msg, tag=""):
        ts = time.strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n", tag)
        self.log_text.see("end")
        if int(self.log_text.index("end-1c").split(".")[0]) > 100:
            self.log_text.delete("1.0", "15.0")
        self.log_text.config(state="disabled")

    def _on_close(self):
        if self._connected:
            self.ble.send_raw("TEST:4,0")
            self.ble.disconnect()
        self.root.after(300, self.root.destroy)


if __name__ == "__main__":
    import sys
    try:
        import bleak
    except ImportError:
        print("ERROR: pip install bleak"); sys.exit(1)

    root = tk.Tk()
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("Horizontal.TScale",
        background="#111520", troughcolor="#1a2235",
        sliderlength=18, sliderrelief="flat")
    DroneGUI(root)
    root.mainloop()