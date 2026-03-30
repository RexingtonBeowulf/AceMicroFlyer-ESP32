"""
ESP32-C3 Drone — Python BLE Controller (PID version)
Pitch/Roll sliders now send target ANGLES in degrees (±30°).
Yaw sends rate -100..100 mapped to ±90°/s on the drone.

Usage:  python3 drone_controller_pid.py
Deps:   pip install bleak
"""

import asyncio, threading, tkinter as tk, time
from tkinter import ttk
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
CHAR_UUID    = "abcdefab-cdef-abcd-efab-cdefabcdefab"
DEVICE_NAME  = "ESP32-C3-Drone"

DUTY_MAX = 800

# ═══════════════════════════════════════════════════════════════
#  Motor mix preview — mirrors PID firmware mixer
#  (approximate: shows effect without actual PID corrections)
# ═══════════════════════════════════════════════════════════════
def compute_motors(T, P_deg, R_deg, Y):
    base  = T / 100 * DUTY_MAX
    p     = (P_deg / 30) * 150   # scale ±30° to ±150 duty units
    r     = (R_deg / 30) * 150
    y     = (Y / 100) * 100

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
        self.pitch     = 0.0   # degrees
        self.roll      = 0.0   # degrees
        self.yaw       = 0     # -100..100

        self.on_connect    = None
        self.on_disconnect = None
        self.on_log        = None
        self.on_tx         = None

        threading.Thread(target=self._run_loop, daemon=True).start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _log(self, msg, tag="inf"):
        if self.on_log:
            self.on_log(msg, tag)

    def connect(self):
        asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def disconnect(self):
        asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)

    def set_controls(self, t, p, r, y):
        self.throttle = int(t)
        self.pitch    = float(p)
        self.roll     = float(r)
        self.yaw      = int(y)

    async def _connect(self):
        self._log(f"Scanning for '{DEVICE_NAME}'…")
        try:
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=8.0)
            if device is None:
                self._log(f"'{DEVICE_NAME}' not found. Is it on?", "err")
                if self.on_disconnect: self.on_disconnect()
                return

            self._log(f"Found {device.name} [{device.address}]", "ok")
            self.client = BleakClient(device, disconnected_callback=self._on_disc_cb)
            await self.client.connect()
            self.connected = True
            self._log("Connected! Sending at 20 Hz…", "ok")
            if self.on_connect: self.on_connect()
            await self._send_loop()

        except BleakError as e:
            self._log(f"BLE error: {e}", "err")
            if self.on_disconnect: self.on_disconnect()
        except Exception as e:
            self._log(f"Error: {e}", "err")
            if self.on_disconnect: self.on_disconnect()

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
        interval = 1.0 / 20  # 20 Hz
        while self.connected and self.client and self.client.is_connected:
            # P and R are integer degrees; Y is -100..100
            packet = (f"T:{self.throttle},"
                      f"P:{int(self.pitch)},"
                      f"R:{int(self.roll)},"
                      f"Y:{self.yaw}")
            try:
                await self.client.write_gatt_char(
                    CHAR_UUID, packet.encode(), response=False)
                if self.on_tx: self.on_tx(packet)
            except Exception as e:
                self._log(f"TX error: {e}", "err")
                break
            await asyncio.sleep(interval)


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
    MONO_LG= ("Courier New", 11, "bold")

    def __init__(self, root):
        self.root = root
        self.root.title("Drone Controller — PID")
        self.root.configure(bg=self.BG)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.ble = BLEManager()
        self.ble.on_connect    = self._on_ble_connect
        self.ble.on_disconnect = self._on_ble_disconnect
        self.ble.on_log        = self._log
        self.ble.on_tx         = self._on_tx

        self._tx_count  = 0
        self._tx_window = 0
        self._last_hz   = time.time()
        self._connected = False

        self._build_ui()
        self._update_motor_viz()
        self._tick_hz()
        self._log("PID controller ready. Click Connect.", "inf")

    def _build_ui(self):
        root = self.root

        hdr = tk.Frame(root, bg=self.BG)
        hdr.pack(fill="x", padx=12, pady=(12, 0))
        tk.Label(hdr, text="ESP32-C3 DRONE  //  PID", font=self.MONO_LG,
                 fg=self.ACCENT, bg=self.BG).pack(side="left")
        self.status_lbl = tk.Label(hdr, text="● DISCONNECTED",
                                   font=self.MONO, fg=self.DIM, bg=self.BG)
        self.status_lbl.pack(side="right")

        tk.Frame(root, bg=self.BORDER, height=1).pack(fill="x", padx=12, pady=4)

        warn = tk.Frame(root, bg="#1a1400")
        warn.pack(fill="x", padx=12, pady=(0, 6))
        tk.Label(warn,
                 text="⚠  Remove props during testing. Pitch/Roll are now TARGET ANGLES (°).",
                 font=self.MONO, fg=self.WARN, bg="#1a1400",
                 wraplength=420, justify="left").pack(padx=10, pady=6)

        # ── Sliders ──
        ctrl = self._panel(root, "// FLIGHT CONTROLS")

        self.throttle_var = tk.IntVar(value=0)
        self.pitch_var    = tk.IntVar(value=0)
        self.roll_var     = tk.IntVar(value=0)
        self.yaw_var      = tk.IntVar(value=0)

        self._slider(ctrl, "THROTTLE  T", self.throttle_var,  0, 100, "%")
        self._slider(ctrl, "PITCH     P", self.pitch_var,   -30,  30, "°")
        self._slider(ctrl, "ROLL      R", self.roll_var,    -30,  30, "°")
        self._slider(ctrl, "YAW RATE  Y", self.yaw_var,    -100, 100, "")

        self.hz_lbl = tk.Label(ctrl, text="TX: — Hz", font=self.MONO,
                               fg=self.DIM, bg=self.PANEL, anchor="e")
        self.hz_lbl.pack(fill="x", padx=10, pady=(0, 6))

        # ── PID gain display ──
        pg = self._panel(root, "// PID GAINS  (edit firmware to change)")
        gains_text = (
            "Roll / Pitch   kP=1.20  kI=0.004  kD=0.080\n"
            "Yaw rate       kP=2.00  kI=0.010  kD=0.000\n"
            "Filter alpha   CF_ALPHA=0.98   IMU=500 Hz   PID=100 Hz"
        )
        tk.Label(pg, text=gains_text, font=self.MONO,
                 fg=self.DIM, bg=self.PANEL, anchor="w",
                 justify="left").pack(fill="x", padx=10, pady=(0, 8))

        # ── Motor viz ──
        mviz = self._panel(root, "// MOTOR OUTPUTS (open-loop estimate)")
        mgrid = tk.Frame(mviz, bg=self.PANEL)
        mgrid.pack(fill="x", padx=10, pady=(0, 8))
        self.motor_bars = {}
        self.motor_lbls = {}
        for tag, row, col in [("FL",0,0),("FR",0,1),("BL",1,0),("BR",1,1)]:
            cell = tk.Frame(mgrid, bg="#0d1220")
            cell.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
            mgrid.columnconfigure(col, weight=1)
            tk.Label(cell, text=tag, font=self.MONO_LG,
                     fg=self.ACCENT, bg="#0d1220").pack(pady=(6,2))
            bg = tk.Frame(cell, bg=self.BORDER, height=8)
            bg.pack(fill="x", padx=8)
            bg.pack_propagate(False)
            bar = tk.Frame(bg, bg=self.ACCENT, height=8)
            bar.place(x=0, y=0, relheight=1.0, relwidth=0)
            self.motor_bars[tag] = (bar, bg)
            lbl = tk.Label(cell, text="0%", font=self.MONO,
                           fg=self.TEXT, bg="#0d1220")
            lbl.pack(pady=(2,6))
            self.motor_lbls[tag] = lbl

        # ── Buttons ──
        bf = tk.Frame(root, bg=self.BG)
        bf.pack(fill="x", padx=12, pady=6)
        self.connect_btn = tk.Button(
            bf, text="⬡  CONNECT BLE",
            font=("Courier New",10,"bold"),
            bg=self.ACCENT, fg=self.BG,
            activebackground="#00b8cc", activeforeground=self.BG,
            bd=0, pady=10, cursor="hand2", command=self._toggle_connect)
        self.connect_btn.pack(side="left", fill="x", expand=True, padx=(0,6))
        tk.Button(bf, text="⬛  E-STOP",
                  font=("Courier New",10,"bold"),
                  bg=self.DANGER, fg="white",
                  activebackground="#cc2040", activeforeground="white",
                  bd=0, pady=10, cursor="hand2",
                  command=self._emergency_stop).pack(side="right", fill="x", expand=True)

        # ── Log ──
        lf = self._panel(root, "// LOG")
        self.log_text = tk.Text(
            lf, height=6, bg=self.BG, fg=self.DIM,
            font=self.MONO, bd=0, state="disabled", wrap="word")
        self.log_text.pack(fill="x", padx=8, pady=(0,8))
        for tag, col in [("ok",self.OK),("err",self.DANGER),
                         ("inf",self.ACCENT),("warn",self.WARN)]:
            self.log_text.tag_config(tag, foreground=col)

    def _sep(self, p):
        tk.Frame(p, bg=self.BORDER, height=1).pack(fill="x", padx=12, pady=4)

    def _panel(self, parent, title):
        outer = tk.Frame(parent, bg=self.PANEL)
        outer.pack(fill="x", padx=12, pady=4)
        tk.Label(outer, text=title, font=self.MONO,
                 fg=self.DIM, bg=self.PANEL, anchor="w").pack(
            fill="x", padx=10, pady=(8,4))
        return outer

    def _slider(self, parent, label, var, lo, hi, unit):
        row = tk.Frame(parent, bg=self.PANEL)
        row.pack(fill="x", padx=10, pady=3)
        tk.Label(row, text=label, font=self.MONO, fg=self.ACCENT,
                 bg=self.PANEL, width=14, anchor="w").pack(side="left")
        val_lbl = tk.Label(row, text=f"0{unit}", font=self.MONO,
                           fg=self.TEXT, bg=self.PANEL, width=6, anchor="e")
        val_lbl.pack(side="right")
        s = ttk.Scale(row, from_=lo, to=hi, orient="horizontal",
                      variable=var, length=260)
        s.pack(side="left", fill="x", expand=True, padx=6)
        def on(*_):
            val_lbl.config(text=f"{var.get()}{unit}")
            self._push_controls()
            self._update_motor_viz()
        var.trace_add("write", on)

    def _push_controls(self):
        self.ble.set_controls(
            self.throttle_var.get(),
            self.pitch_var.get(),
            self.roll_var.get(),
            self.yaw_var.get())

    def _update_motor_viz(self):
        T = self.throttle_var.get()
        P = self.pitch_var.get()
        R = self.roll_var.get()
        Y = self.yaw_var.get()
        fl, fr, bl, br = compute_motors(T, P, R, Y)
        for tag, duty in [("FL",fl),("FR",fr),("BL",bl),("BR",br)]:
            pct = duty / DUTY_MAX
            bar, _ = self.motor_bars[tag]
            bar.place(relwidth=pct)
            bar.config(bg=self.DANGER if pct > 0.85 else
                         (self.WARN if pct > 0.5 else self.ACCENT))
            self.motor_lbls[tag].config(text=f"{int(pct*100)}%")
        self.root.after(50, self._update_motor_viz)

    def _on_ble_connect(self):
        self.root.after(0, self._gui_on_connect)

    def _on_ble_disconnect(self):
        self.root.after(0, self._gui_on_disconnect)

    def _on_tx(self, packet):
        self._tx_count  += 1
        self._tx_window += 1

    def _gui_on_connect(self):
        self._connected = True
        self.status_lbl.config(text="● CONNECTED", fg=self.OK)
        self.connect_btn.config(text="⬡  DISCONNECT", bg=self.WARN, fg=self.BG)

    def _gui_on_disconnect(self):
        self._connected = False
        self.status_lbl.config(text="● DISCONNECTED", fg=self.DIM)
        self.connect_btn.config(text="⬡  CONNECT BLE", bg=self.ACCENT, fg=self.BG)

    def _toggle_connect(self):
        if self._connected:
            self.ble.disconnect()
        else:
            self.status_lbl.config(text="◌ SCANNING…", fg=self.WARN)
            self.connect_btn.config(state="disabled")
            self.root.after(500, lambda: self.connect_btn.config(state="normal"))
            self.ble.connect()

    def _emergency_stop(self):
        for v in [self.throttle_var, self.pitch_var,
                  self.roll_var, self.yaw_var]:
            v.set(0)
        self._push_controls()
        self._log("⬛ EMERGENCY STOP", "err")

    def _tick_hz(self):
        now = time.time()
        dt  = now - self._last_hz
        if dt >= 1.0:
            hz = self._tx_window / dt
            self._tx_window = 0
            self._last_hz   = now
            self.hz_lbl.config(
                text=f"TX: {hz:.1f} Hz  |  total: {self._tx_count}")
        self.root.after(1000, self._tick_hz)

    def _log(self, msg, tag=""):
        ts = time.strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {msg}\n", tag)
        self.log_text.see("end")
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 80:
            self.log_text.delete("1.0", "10.0")
        self.log_text.config(state="disabled")

    def _on_close(self):
        if self._connected:
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
                    sliderlength=22, sliderrelief="flat")
    DroneGUI(root)
    root.mainloop()