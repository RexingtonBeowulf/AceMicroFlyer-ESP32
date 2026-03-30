"""
ESP32-C3 Drone — Python BLE Controller with Motor Test Mode
Layout: scrollable canvas so all controls always visible.

Usage:  python3 drone_controller_test.py
Deps:   pip install bleak
"""

import asyncio, threading, tkinter as tk, time
from tkinter import ttk
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_UUID    = "abcdefab-cdef-abcd-efab-cdefabcdefab"
NOTIFY_UUID  = "abcdefab-cdef-abcd-efab-cdefabcdef00"
DEVICE_NAME  = "ESP32-C3-Drone"
DUTY_MAX     = 800

MOTOR_LABELS = ["M1 FL  GPIO2  CW",
                "M2 FR  GPIO3  CCW",
                "M3 BL  GPIO4  CCW",
                "M4 BR  GPIO5  CW"]

def compute_motors(T, P_deg, R_deg, Y):
    base = T / 100 * DUTY_MAX
    p    = (P_deg / 30) * 150
    r    = (R_deg / 30) * 150
    y    = (Y / 100) * 100
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
        threading.Thread(target=self._run_loop, daemon=True).start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _log(self, msg, tag="inf"):
        if self.on_log: self.on_log(msg, tag)

    def connect(self):
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)

    def send_raw(self, packet: str):
        asyncio.run_coroutine_threadsafe(self._send_once(packet), self.loop)

    def set_controls(self, t, p, r, y):
        self.throttle = int(t)
        self.pitch    = float(p)
        self.roll     = float(r)
        self.yaw      = int(y)

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
                self._log(f"'{DEVICE_NAME}' not found. Is it powered on?", "err")
                if self.on_disconnect: self.on_disconnect()
                return
            self._log(f"Found {device.name} [{device.address}]", "ok")

            # Retry up to 3 times — Linux BlueZ GATT discovery sometimes needs a second attempt
            last_exc = None
            for attempt in range(1, 4):
                try:
                    self._log(f"Connecting (attempt {attempt}/3)...")
                    self.client = BleakClient(
                        device,
                        disconnected_callback=self._on_disc_cb,
                        timeout=15.0)
                    await self.client.connect(timeout=15.0)
                    # Wait for BlueZ to finish GATT service discovery
                    await asyncio.sleep(1.5)
                    if self.client.is_connected:
                        last_exc = None
                        break
                    else:
                        raise BleakError("Connected but then immediately dropped")
                except Exception as e:
                    last_exc = e
                    self._log(f"Attempt {attempt} failed: {e}", "warn")
                    if attempt < 3:
                        await asyncio.sleep(2.0)

            if last_exc:
                raise last_exc

            if not self.client.is_connected:
                raise BleakError("Could not connect after 3 attempts")

            self.connected = True
            self._log("Connected! Sending at 20 Hz...", "ok")

            try:
                await self.client.start_notify(NOTIFY_UUID, self._on_notify_cb)
                self._log("Notification channel open", "ok")
            except Exception:
                self._log("Notify subscribe failed (older firmware?)", "warn")

            if self.on_connect: self.on_connect()
            await self._send_loop()

        except BleakError as e:
            self._log(f"BLE error: {e}", "err")
            if self.on_disconnect: self.on_disconnect()
        except Exception as e:
            self._log(f"Error: {e}", "err")
            if self.on_disconnect: self.on_disconnect()

    def _on_notify_cb(self, sender, data):
        msg = data.decode(errors="replace")
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
            packet = f"T:{self.throttle},P:{int(self.pitch)},R:{int(self.roll)},Y:{self.yaw}"
            try:
                await self.client.write_gatt_char(CHAR_UUID, packet.encode(), response=False)
                if self.on_tx: self.on_tx(packet)
            except Exception as e:
                self._log(f"TX error: {e}", "err")
                break
            await asyncio.sleep(interval)


# ═══════════════════════════════════════════════════════════════
#  GUI — uses a Canvas+Scrollbar so nothing is ever cut off
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
        self.root.title("Drone Controller")
        self.root.configure(bg=self.BG)
        self.root.resizable(True, True)
        self.root.minsize(520, 500)
        self.root.geometry("520x780")
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

        self._setup_scroll()
        self._build_all(self.inner)
        self._update_motor_viz()
        self._tick_hz()
        self._log("Ready. Click CONNECT BLE to connect.", "inf")

    # ── Scrollable canvas wrapper ────────────────────────────
    def _setup_scroll(self):
        outer = tk.Frame(self.root, bg=self.BG)
        outer.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(outer, bg=self.BG, highlightthickness=0)
        sb = tk.Scrollbar(outer, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=sb.set)

        sb.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.inner = tk.Frame(self.canvas, bg=self.BG)
        self._canvas_window = self.canvas.create_window(
            (0, 0), window=self.inner, anchor="nw")

        self.inner.bind("<Configure>", self._on_inner_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        # Mouse wheel scrolling
        self.canvas.bind_all("<MouseWheel>",
            lambda e: self.canvas.yview_scroll(int(-1*(e.delta/120)), "units"))
        self.canvas.bind_all("<Button-4>",
            lambda e: self.canvas.yview_scroll(-1, "units"))
        self.canvas.bind_all("<Button-5>",
            lambda e: self.canvas.yview_scroll(1, "units"))

    def _on_inner_configure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.canvas.itemconfig(self._canvas_window, width=event.width)

    # ── Build everything inside self.inner ──────────────────
    def _build_all(self, parent):
        # Configure ttk styles
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Dark.TNotebook", background=self.BG, borderwidth=0)
        style.configure("Dark.TNotebook.Tab",
                        background=self.PANEL, foreground=self.DIM,
                        font=("Courier New", 9, "bold"), padding=[12, 6])
        style.map("Dark.TNotebook.Tab",
                  background=[("selected", self.ACCENT)],
                  foreground=[("selected", self.BG)])
        style.configure("Horizontal.TScale",
                        background=self.PANEL, troughcolor="#1a2235",
                        sliderlength=20, sliderrelief="flat")

        # ── Header ──
        hdr = tk.Frame(parent, bg=self.BG)
        hdr.pack(fill="x", padx=12, pady=(10, 0))
        tk.Label(hdr, text="ESP32-C3 DRONE", font=self.MONO_LG,
                 fg=self.ACCENT, bg=self.BG).pack(side="left")
        self.status_lbl = tk.Label(hdr, text="● DISCONNECTED",
                                   font=self.MONO, fg=self.DIM, bg=self.BG)
        self.status_lbl.pack(side="right")
        tk.Frame(parent, bg=self.BORDER, height=1).pack(fill="x", padx=12, pady=4)

        # ── Connect + E-Stop buttons ── (near top so always visible)
        btns = tk.Frame(parent, bg=self.BG)
        btns.pack(fill="x", padx=12, pady=(0, 4))
        self.connect_btn = tk.Button(
            btns, text="CONNECT BLE",
            font=("Courier New", 11, "bold"),
            bg=self.ACCENT, fg=self.BG,
            activebackground="#00b8cc", activeforeground=self.BG,
            bd=0, pady=12, cursor="hand2",
            command=self._toggle_connect)
        self.connect_btn.pack(side="left", fill="x", expand=True, padx=(0, 6))
        tk.Button(btns, text="E-STOP",
                  font=("Courier New", 11, "bold"),
                  bg=self.DANGER, fg="white",
                  activebackground="#cc2040", activeforeground="white",
                  bd=0, pady=12, cursor="hand2",
                  command=self._emergency_stop
                  ).pack(side="right", fill="x", expand=True)

        # ── Tabs ──
        nb = ttk.Notebook(parent, style="Dark.TNotebook")
        nb.pack(fill="x", padx=12, pady=4)
        flight_tab = tk.Frame(nb, bg=self.BG)
        test_tab   = tk.Frame(nb, bg=self.BG)
        nb.add(flight_tab, text="  Flight  ")
        nb.add(test_tab,   text="  Motor Test  ")
        self._build_flight_tab(flight_tab)
        self._build_test_tab(test_tab)

        # ── Log ──
        lf = tk.Frame(parent, bg=self.PANEL)
        lf.pack(fill="x", padx=12, pady=(4, 12))
        tk.Label(lf, text="// LOG", font=self.MONO,
                 fg=self.DIM, bg=self.PANEL, anchor="w").pack(
            fill="x", padx=10, pady=(6, 2))
        self.log_text = tk.Text(
            lf, height=5, bg=self.BG, fg=self.DIM,
            font=self.MONO_SM, bd=0, state="disabled", wrap="word")
        self.log_text.pack(fill="x", padx=8, pady=(0, 8))
        for tag, col in [("ok", self.OK), ("err", self.DANGER),
                         ("inf", self.ACCENT), ("warn", self.WARN)]:
            self.log_text.tag_config(tag, foreground=col)

    # ── Flight tab ──────────────────────────────────────────
    def _build_flight_tab(self, parent):
        warn = tk.Frame(parent, bg="#1a1400")
        warn.pack(fill="x", pady=(4, 0))
        tk.Label(warn, text="  Remove props. P/R = target angle in degrees.",
                 font=self.MONO_SM, fg=self.WARN, bg="#1a1400",
                 justify="left").pack(padx=6, pady=5)

        ctrl = self._panel(parent, "// FLIGHT CONTROLS")
        self.throttle_var = tk.IntVar(value=0)
        self.pitch_var    = tk.IntVar(value=0)
        self.roll_var     = tk.IntVar(value=0)
        self.yaw_var      = tk.IntVar(value=0)
        self._slider(ctrl, "THROTTLE  T", self.throttle_var,  0,   100, "%")
        self._slider(ctrl, "PITCH     P", self.pitch_var,   -30,   30,  "°")
        self._slider(ctrl, "ROLL      R", self.roll_var,    -30,   30,  "°")
        self._slider(ctrl, "YAW RATE  Y", self.yaw_var,    -100,  100,  "")
        self.hz_lbl = tk.Label(ctrl, text="TX: -- Hz", font=self.MONO,
                               fg=self.DIM, bg=self.PANEL, anchor="e")
        self.hz_lbl.pack(fill="x", padx=10, pady=(0, 6))

        mviz = self._panel(parent, "// MOTOR OUTPUTS (estimated)")
        mgrid = tk.Frame(mviz, bg=self.PANEL)
        mgrid.pack(fill="x", padx=10, pady=(0, 8))
        self.motor_bars = {}
        self.motor_lbls = {}
        for tag, row, col, color in [
                ("FL", 0, 0, self.ACCENT), ("FR", 0, 1, self.ACCENT),
                ("BL", 1, 0, "#b060ff"),   ("BR", 1, 1, "#b060ff")]:
            cell = tk.Frame(mgrid, bg="#0d1220")
            cell.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
            mgrid.columnconfigure(col, weight=1)
            tk.Label(cell, text=tag, font=self.MONO_LG,
                     fg=color, bg="#0d1220").pack(pady=(5, 2))
            bg_bar = tk.Frame(cell, bg=self.BORDER, height=8)
            bg_bar.pack(fill="x", padx=8)
            bg_bar.pack_propagate(False)
            bar = tk.Frame(bg_bar, bg=color, height=8)
            bar.place(x=0, y=0, relheight=1.0, relwidth=0)
            self.motor_bars[tag] = (bar, bg_bar, color)
            lbl = tk.Label(cell, text="0%", font=self.MONO,
                           fg=self.TEXT, bg="#0d1220")
            lbl.pack(pady=(2, 5))
            self.motor_lbls[tag] = lbl

    # ── Motor test tab ───────────────────────────────────────
    def _build_test_tab(self, parent):
        guide = self._panel(parent, "// MPU-6050 orientation")
        tk.Label(guide,
                 text="+Y = FORWARD (nose)   +X = RIGHT   +Z = UP\n"
                      "Mount chip arrow facing the front arm.",
                 font=self.MONO_SM, fg=self.TEXT, bg=self.PANEL,
                 justify="left").pack(padx=12, pady=(0, 8))

        imu_f = self._panel(parent, "// IMU angles")
        imu_row = tk.Frame(imu_f, bg=self.PANEL)
        imu_row.pack(fill="x", padx=10, pady=(0, 4))
        for lbl, var in [("Roll:", self._imu_roll), ("Pitch:", self._imu_pitch)]:
            tk.Label(imu_row, text=lbl, font=self.MONO, fg=self.DIM,
                     bg=self.PANEL, width=7, anchor="w").pack(side="left")
            tk.Label(imu_row, textvariable=var, font=self.MONO_LG,
                     fg=self.ACCENT, bg=self.PANEL, width=8).pack(side="left")
        imu_btns = tk.Frame(imu_f, bg=self.PANEL)
        imu_btns.pack(fill="x", padx=10, pady=(0, 8))
        tk.Button(imu_btns, text="IMU snapshot",
                  font=self.MONO, bg="#0d1220", fg=self.ACCENT,
                  relief="flat", bd=1, cursor="hand2",
                  command=self._req_imu).pack(side="left", padx=(0, 8))
        self.gyro_btn = tk.Button(imu_btns, text="Start gyro live",
                  font=self.MONO, bg="#0d1220", fg=self.ACCENT,
                  relief="flat", bd=1, cursor="hand2",
                  command=self._toggle_gyro_live)
        self.gyro_btn.pack(side="left")

        test_f = self._panel(parent, "// Individual motor test  (props OFF!)")
        duty_row = tk.Frame(test_f, bg=self.PANEL)
        duty_row.pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(duty_row, text="Test duty:", font=self.MONO,
                 fg=self.DIM, bg=self.PANEL).pack(side="left")
        self._duty_lbl = tk.Label(duty_row, text="20%", font=self.MONO_LG,
                                  fg=self.ACCENT, bg=self.PANEL, width=5)
        self._duty_lbl.pack(side="right")
        ttk.Scale(duty_row, from_=5, to=60, orient="horizontal",
                  variable=self._test_duty, length=220).pack(
            side="left", fill="x", expand=True, padx=6)
        self._test_duty.trace_add("write",
            lambda *_: self._duty_lbl.config(text=f"{self._test_duty.get()}%"))

        mgrid = tk.Frame(test_f, bg=self.PANEL)
        mgrid.pack(fill="x", padx=10, pady=(0, 6))
        self._motor_btns = []
        for idx, (row, col, color) in enumerate([
                (0, 0, self.ACCENT), (0, 1, self.ACCENT),
                (1, 0, "#b060ff"),   (1, 1, "#b060ff")]):
            btn = tk.Button(
                mgrid, text=MOTOR_LABELS[idx],
                font=("Courier New", 8, "bold"),
                bg="#0d1220", fg=color,
                activebackground=color, activeforeground=self.BG,
                relief="flat", bd=1, pady=10, cursor="hand2",
                command=lambda i=idx: self._test_motor(i))
            btn.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
            mgrid.columnconfigure(col, weight=1)
            self._motor_btns.append((btn, color))

        all_row = tk.Frame(test_f, bg=self.PANEL)
        all_row.pack(fill="x", padx=10, pady=(0, 10))
        tk.Button(all_row, text="All motors ON",
                  font=("Courier New", 9, "bold"),
                  bg="#0d1a0d", fg=self.OK,
                  activebackground=self.OK, activeforeground=self.BG,
                  relief="flat", bd=1, pady=8, cursor="hand2",
                  command=self._test_all).pack(
            side="left", fill="x", expand=True, padx=(0, 6))
        tk.Button(all_row, text="All motors OFF",
                  font=("Courier New", 9, "bold"),
                  bg="#1a0a0a", fg=self.DANGER,
                  activebackground=self.DANGER, activeforeground="white",
                  relief="flat", bd=1, pady=8, cursor="hand2",
                  command=self._test_stop).pack(
            side="right", fill="x", expand=True)

        chk = self._panel(parent, "// Assignment checklist")
        for step in [
            "1. Set duty ~20%.  Click CONNECT BLE.",
            "2. Click M1 FL -> front-LEFT arm should spin.",
            "3. Click M2 FR -> front-RIGHT arm should spin.",
            "4. Click M3 BL -> back-LEFT arm should spin.",
            "5. Click M4 BR -> back-RIGHT arm should spin.",
            "6. Tilt drone forward -> Pitch goes negative.",
            "7. Tilt drone right   -> Roll goes positive.",
            "8. Wrong motor? Swap pins in MOTOR_PINS[] array.",
        ]:
            tk.Label(chk, text=step, font=self.MONO_SM,
                     fg=self.DIM, bg=self.PANEL, anchor="w").pack(
                fill="x", padx=12, pady=1)
        tk.Frame(chk, bg=self.PANEL, height=8).pack()

    # ── Helpers ──────────────────────────────────────────────
    def _panel(self, parent, title):
        f = tk.Frame(parent, bg=self.PANEL)
        f.pack(fill="x", pady=3)
        tk.Label(f, text=title, font=self.MONO,
                 fg=self.DIM, bg=self.PANEL, anchor="w").pack(
            fill="x", padx=10, pady=(7, 3))
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

    # ── Test actions ─────────────────────────────────────────
    def _test_motor(self, idx):
        duty = self._test_duty.get()
        self._highlight_btn(idx)
        self._log(f"Testing {MOTOR_LABELS[idx]} at {duty}%", "inf")
        self.ble.send_raw(f"TEST:{idx},{duty}")

    def _test_all(self):
        duty = self._test_duty.get()
        self._highlight_btn(-1)
        self._log(f"All motors at {duty}%", "warn")
        self.ble.send_raw(f"TEST:4,{duty}")

    def _test_stop(self):
        self._highlight_btn(-1)
        self._log("All motors stopped", "err")
        self.ble.send_raw("TEST:4,0")

    def _highlight_btn(self, active):
        for i, (btn, color) in enumerate(self._motor_btns):
            btn.config(bg=color if i == active else "#0d1220",
                       fg=self.BG if i == active else color)

    def _req_imu(self):
        self.ble.send_raw("IMU")
        self._log("IMU snapshot requested", "inf")

    def _toggle_gyro_live(self):
        self._gyro_live = not self._gyro_live
        if self._gyro_live:
            self.gyro_btn.config(text="Stop gyro live", fg=self.WARN)
            self.ble.send_raw("GYRO")
            self._log("Gyro live ON - tilt the drone", "inf")
        else:
            self.gyro_btn.config(text="Start gyro live", fg=self.ACCENT)
            self.ble.send_raw("GYROSTOP")
            self._log("Gyro live OFF", "inf")

    def _push_controls(self):
        self.ble.set_controls(
            self.throttle_var.get(), self.pitch_var.get(),
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
                bar.config(bg=self.DANGER if pct > 0.85 else
                             self.WARN    if pct > 0.50 else base_color)
                self.motor_lbls[tag].config(text=f"{int(pct*100)}%")
        except Exception:
            pass
        self.root.after(60, self._update_motor_viz)

    # ── BLE callbacks ────────────────────────────────────────
    def _on_ble_connect(self):
        self.root.after(0, self._gui_connect)

    def _on_ble_disconnect(self):
        self.root.after(0, self._gui_disconnect)

    def _on_tx(self, packet):
        self._tx_count  += 1
        self._tx_window += 1

    def _on_notify(self, msg):
        self.root.after(0, lambda: self._process_notify(msg))

    def _process_notify(self, msg):
        self._log(f"<- {msg}", "ok")
        if msg.startswith(("IMU:", "GYRO:")):
            try:
                for part in msg.split(":")[1].split(","):
                    k, v = part.split("=")
                    if k == "R": self._imu_roll.set(f"{float(v):+.1f}°")
                    if k == "P": self._imu_pitch.set(f"{float(v):+.1f}°")
            except Exception:
                pass

    def _gui_connect(self):
        self._connected = True
        self.status_lbl.config(text="● CONNECTED", fg=self.OK)
        self.connect_btn.config(text="DISCONNECT",
                                bg=self.WARN, fg=self.BG)

    def _gui_disconnect(self):
        self._connected = False
        self._gyro_live = False
        self.gyro_btn.config(text="Start gyro live", fg=self.ACCENT)
        self.status_lbl.config(text="● DISCONNECTED", fg=self.DIM)
        self.connect_btn.config(text="CONNECT BLE",
                                bg=self.ACCENT, fg=self.BG)
        self._highlight_btn(-1)

    def _toggle_connect(self):
        if self._connected:
            self.ble.disconnect()
        else:
            self.status_lbl.config(text="SCANNING...", fg=self.WARN)
            self.connect_btn.config(state="disabled")
            self.root.after(600, lambda: self.connect_btn.config(state="normal"))
            self.ble.connect()

    def _emergency_stop(self):
        for v in [self.throttle_var, self.pitch_var,
                  self.roll_var, self.yaw_var]:
            v.set(0)
        self._push_controls()
        self.ble.send_raw("TEST:4,0")
        self._highlight_btn(-1)
        self._log("EMERGENCY STOP", "err")

    def _tick_hz(self):
        now = time.time()
        dt  = now - self._last_hz
        if dt >= 1.0:
            hz = self._tx_window / dt
            self._tx_window = 0
            self._last_hz   = now
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
        print("ERROR: pip install bleak")
        sys.exit(1)

    root = tk.Tk()
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("Horizontal.TScale",
                    background="#111520", troughcolor="#1a2235",
                    sliderlength=20, sliderrelief="flat")
    DroneGUI(root)
    root.mainloop()