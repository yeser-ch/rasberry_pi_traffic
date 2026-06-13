#!/usr/bin/env python3
"""
RPiZA v2.0 — Raspberry Pi Pedestrian Traffic Light Controller
Compliant: StVO §37 · RiLSA 2015 · RPiZA SRS v3.0

Pin map (BCM):
  FzA  R=5  Y=6  G=13
  FgA1 R=27 G=10
  FgA2 R=21 G=16
  HC-SR04 TRIG=26 ECHO=24
"""

import os
import threading
import time
import tkinter as tk
from tkinter import ttk

# ── Detect hardware ────────────────────────────────────────────────────────────
try:
    import RPi.GPIO as _RPIGPIO
    _MOCK = False
except (ImportError, RuntimeError):
    _MOCK = True
    os.environ['GPIOZERO_PIN_FACTORY'] = 'mock'

from gpiozero import LED  # noqa: E402

# ── Pin assignments (BCM) ──────────────────────────────────────────────────────
PIN_FZA_RED    = 5
PIN_FZA_YELLOW = 6
PIN_FZA_GREEN  = 13
PIN_FGA1_RED   = 27
PIN_FGA1_GREEN = 10
PIN_FGA2_RED   = 21
PIN_FGA2_GREEN = 16
PIN_TRIG       = 26
PIN_ECHO       = 24

# ── Fixed timings — DS-03 / RiLSA ZRA ─────────────────────────────────────────
T_YELLOW    = 1.0   # FA-02: FzA Grün→Rot
T_ROT_GELB  = 1.0   # FA-02: FzA Rot→Grün
T_RAEUMZEIT = 4.0   # FA-05: all-red clearance (4 m ÷ 1.0 m/s)
T_MIN_GREEN = 10.0  # FA-12: minimum FgA green before adaptive cut

# ── Sensor parameters — DS-01, DS-02 ──────────────────────────────────────────
SENSOR_DISTANCE_CM = 50    # anything closer = vehicle present
SENSOR_CONFIRM     = 3     # consecutive readings needed (DS-02)
SENSOR_INTERVAL    = 0.1   # 100 ms polling → detect within 200 ms (NFA-P01)

# ── Config ranges — FA-07, FA-08 ──────────────────────────────────────────────
FZA_GREEN_MIN, FZA_GREEN_MAX = 120, 300  # FA-07 / SR-05: Rot-Rot-Periodendauer (= FzA green phase)
FGA_GRN_MIN,  FGA_GRN_MAX  =  10,  40


# ── Vehicle sensor ─────────────────────────────────────────────────────────────

class VehicleSensor:
    """
    HC-SR04 driver.  Real Pi uses RPi.GPIO; emulator uses a simulated flag.
    DS-01: voltage divider (330Ω+470Ω) on ECHO pin required in hardware.
    DS-02: 3 consecutive readings confirm stable detection.
    """

    def __init__(self, mock: bool):
        self._mock    = mock
        self._sim     = False   # toggled by GUI when mock
        self._confirm = 0
        self._detected = False
        self._lock    = threading.Lock()
        self._stop    = threading.Event()

        if not mock:
            _RPIGPIO.setmode(_RPIGPIO.BCM)
            _RPIGPIO.setup(PIN_TRIG, _RPIGPIO.OUT)
            _RPIGPIO.setup(PIN_ECHO, _RPIGPIO.IN)
            _RPIGPIO.output(PIN_TRIG, _RPIGPIO.LOW)

        threading.Thread(target=self._poll, daemon=True).start()

    def _measure(self):
        """Returns distance in cm, or None on timeout."""
        GPIO = _RPIGPIO
        GPIO.output(PIN_TRIG, GPIO.LOW)
        time.sleep(0.002)
        GPIO.output(PIN_TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(PIN_TRIG, GPIO.LOW)

        t0 = time.monotonic()
        while GPIO.input(PIN_ECHO) == 0:
            if time.monotonic() - t0 > 0.05:
                return None
        t1 = time.monotonic()
        while GPIO.input(PIN_ECHO) == 1:
            if time.monotonic() - t1 > 0.05:
                return None
        return (time.monotonic() - t1) * 17150

    def _poll(self):
        while not self._stop.is_set():
            if self._mock:
                present = self._sim
            else:
                d = self._measure()
                present = d is not None and d < SENSOR_DISTANCE_CM

            with self._lock:
                self._confirm = min(self._confirm + 1, SENSOR_CONFIRM) if present else 0
                self._detected = self._confirm >= SENSOR_CONFIRM

            time.sleep(SENSOR_INTERVAL)

    @property
    def detected(self) -> bool:
        with self._lock:
            return self._detected

    def simulate(self, present: bool):
        """GUI calls this to simulate a vehicle in emulator mode."""
        self._sim = present

    def stop(self):
        self._stop.set()
        if not self._mock:
            _RPIGPIO.cleanup()


# ── State machine ──────────────────────────────────────────────────────────────

class RPiZA:
    """
    Implements FA-01 through FA-13.

    Cycle (always automatic — no pedestrian button):
      FzA Green → FzA Yellow (1s) → FgA Green (adaptive) →
      Räumzeit all-red (4s) → FzA Rot-Gelb (1s) → repeat
    """

    S_CONFIG    = "CONFIG"
    S_FZA_GREEN = "FZA_GREEN"
    S_FZA_YEL   = "FZA_YELLOW"
    S_FGA_GREEN = "FGA_GREEN"
    S_RAEUMZEIT = "RAEUMZEIT"
    S_ROT_GELB  = "FZA_ROT_GELB"
    S_ERROR     = "ERROR"

    def __init__(self, mock: bool):
        self.fza_red    = LED(PIN_FZA_RED)
        self.fza_yellow = LED(PIN_FZA_YELLOW)
        self.fza_green  = LED(PIN_FZA_GREEN)
        self.fga1_red   = LED(PIN_FGA1_RED)
        self.fga1_green = LED(PIN_FGA1_GREEN)
        self.fga2_red   = LED(PIN_FGA2_RED)
        self.fga2_green = LED(PIN_FGA2_GREEN)
        self.sensor     = VehicleSensor(mock=mock)

        self.fza_green_time = 120   # FA-07 / SR-05: Rot-Rot-Periodendauer, operator sets 120–300
        self.fga_green_time = 20    # FA-08: operator sets 10–40 s
        self.speed          = 1.0   # emulator only

        self.state           = self.S_CONFIG
        self.on_state_change = None   # set by GUI
        self._stop           = threading.Event()
        self._phase_start    = time.monotonic()

        self._all_red()   # FA-06: all red on startup

    # ── Light helpers — FA-04: never simultaneous green ───────────────────────

    def _all_red(self):
        self.fza_green.off();  self.fza_yellow.off(); self.fza_red.on()
        self.fga1_green.off(); self.fga2_green.off()
        self.fga1_red.on();    self.fga2_red.on()

    def _fza_green(self):
        # Peds go red before FzA goes green (FA-04)
        self.fga1_green.off(); self.fga2_green.off()
        self.fga1_red.on();    self.fga2_red.on()
        self.fza_yellow.off(); self.fza_red.off(); self.fza_green.on()

    def _fza_yellow(self):
        self.fza_green.off(); self.fza_red.off(); self.fza_yellow.on()

    def _fza_rot_gelb(self):
        self.fza_green.off(); self.fza_red.on(); self.fza_yellow.on()

    def _fga_green(self):
        # FzA goes red before FgA goes green (FA-04)
        self.fza_green.off()
        self.fza_yellow.off(); self.fza_red.on()
        self.fga1_red.off(); self.fga2_red.off()
        self.fga1_green.on(); self.fga2_green.on()

    # ── Timing helpers ─────────────────────────────────────────────────────────

    def _sleep(self, seconds: float) -> bool:
        """Real-time interruptible sleep. Returns False if stopped."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self._stop.is_set():
                return False
            time.sleep(0.02)
        return True

    def _sleep_scaled(self, seconds: float) -> bool:
        """Speed-scaled sleep for configurable phases (emulator convenience)."""
        return self._sleep(seconds / max(self.speed, 0.1))

    def _sleep_fga(self, max_seconds: float):
        """
        FgA green phase with adaptive shortening.
        FA-11: cut short only if vehicle detected AND min-green has elapsed (FA-12).
        Both fga_green_time and min_green are speed-scaled equally so the ratio holds.
        """
        spd     = max(self.speed, 0.1)
        end     = time.monotonic() + max_seconds / spd
        min_end = time.monotonic() + T_MIN_GREEN / spd

        while time.monotonic() < end:
            if self._stop.is_set():
                return
            if time.monotonic() >= min_end and self.sensor.detected:
                return   # adaptive shortening triggered
            time.sleep(0.02)

    # ── State notification ─────────────────────────────────────────────────────

    def _set_state(self, s: str):
        self.state = s
        self._phase_start = time.monotonic()
        if self.on_state_change:
            self.on_state_change()

    @property
    def phase_elapsed(self) -> float:
        return time.monotonic() - self._phase_start

    # ── Main cycle ─────────────────────────────────────────────────────────────

    def run(self):
        """Traffic cycle — call in a daemon thread after config."""
        self._stop.clear()
        try:
            self._cycle()
        except Exception:
            # FA-13 / NFA-S01: any error → safe state within 500 ms
            self._set_state(self.S_ERROR)
            self._all_red()

    def _cycle(self):
        while not self._stop.is_set():

            # 1 — FzA Green  (FA-01, FA-07)
            self._set_state(self.S_FZA_GREEN)
            self._fza_green()
            if not self._sleep_scaled(self.fza_green_time):
                break

            # 2 — FzA Yellow 1 s  (FA-02, DS-03)
            self._set_state(self.S_FZA_YEL)
            self._fza_yellow()
            if not self._sleep(T_YELLOW):
                break

            # 3 — FgA Green with adaptive shortening  (FA-03, FA-08, FA-11, FA-12)
            self._set_state(self.S_FGA_GREEN)
            self._fga_green()
            self._sleep_fga(self.fga_green_time)
            if self._stop.is_set():
                break

            # 4 — Räumzeit 4 s — all red  (FA-05, DS-03)
            self._set_state(self.S_RAEUMZEIT)
            self._all_red()
            if not self._sleep(T_RAEUMZEIT):
                break

            # 5 — FzA Rot-Gelb 1 s  (FA-01, FA-02, DS-03)
            self._set_state(self.S_ROT_GELB)
            self._fza_rot_gelb()
            if not self._sleep(T_ROT_GELB):
                break

        self._all_red()
        self._set_state(self.S_CONFIG)

    def stop(self):
        self._stop.set()

    def trigger_error(self):
        """FA-13: force permanent safe state (all red)."""
        self._stop.set()
        self._set_state(self.S_ERROR)
        self._all_red()

    def cleanup(self):
        self.stop()
        self.sensor.stop()

    @property
    def leds(self) -> dict:
        return {
            'fza_red':    bool(self.fza_red.value),
            'fza_yellow': bool(self.fza_yellow.value),
            'fza_green':  bool(self.fza_green.value),
            'fga1_red':   bool(self.fga1_red.value),
            'fga1_green': bool(self.fga1_green.value),
            'fga2_red':   bool(self.fga2_red.value),
            'fga2_green': bool(self.fga2_green.value),
        }


# ── GUI ────────────────────────────────────────────────────────────────────────

_BG   = "#16213e"
_CAR  = "#1a1a3e"
_PED  = "#1a3e1a"
_OFF  = "#2a2a2a"
_RED  = "#ff3333"
_YEL  = "#ffcc00"
_GRN  = "#33ff66"

_STATE_LABELS = {
    RPiZA.S_CONFIG:    "CONFIG — Parameter eingeben und Start drücken",
    RPiZA.S_FZA_GREEN: "FzA: GRÜN — Fahrzeuge fahren",
    RPiZA.S_FZA_YEL:   "FzA: GELB — Übergang (1 s)",
    RPiZA.S_FGA_GREEN: "FgA: GRÜN — Fußgänger überqueren",
    RPiZA.S_RAEUMZEIT: "RÄUMZEIT — Alle Rot (4 s)",
    RPiZA.S_ROT_GELB:  "FzA: ROT-GELB — Vorbereitung (1 s)",
    RPiZA.S_ERROR:     "FEHLER — Sicherer Zustand (alle Rot)",
}


class _Lens(tk.Canvas):
    """Single circular traffic-light lens."""

    def __init__(self, parent, color: str, size: int = 52, bg: str = "#111122"):
        super().__init__(parent, width=size, height=size, bg=bg, highlightthickness=0)
        m = 5
        self._oval = self.create_oval(m, m, size - m, size - m,
                                      fill=_OFF, outline="#444", width=2)
        self._on_color = color

    def set_on(self, on: bool):
        self.itemconfig(self._oval, fill=self._on_color if on else _OFF)


class RPiZAGui:

    def __init__(self, root: tk.Tk, ctrl: RPiZA, mock: bool):
        self.root = root
        self.ctrl = ctrl
        self.mock = mock
        ctrl.on_state_change = lambda: root.after(0, self._refresh)

        root.title("RPiZA v2.0 — Ampelsteuerung")
        root.configure(bg=_BG)
        root.resizable(False, False)
        self._build()
        self._poll()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build(self):
        r = self.root

        tk.Label(r, text="RPiZA — Fußgänger-Ampelanlage",
                 font=("Helvetica", 16, "bold"), bg=_BG, fg="white").pack(pady=(12, 2))

        self._status_var = tk.StringVar(value=_STATE_LABELS[RPiZA.S_CONFIG])
        tk.Label(r, textvariable=self._status_var, font=("Helvetica", 11),
                 bg=_BG, fg="#aaaaff", width=50).pack(pady=(0, 2))

        self._timer_var = tk.StringVar(value="Phase: 0s")
        tk.Label(r, textvariable=self._timer_var, font=("Courier", 11),
                 bg=_BG, fg="#ffcc44").pack(pady=(0, 8))

        # Traffic lights
        lf = tk.Frame(r, bg=_BG)
        lf.pack(padx=20, pady=4)
        self._fza_r, self._fza_y, self._fza_g = self._car_panel(lf, col=0)
        self._fga1_r, self._fga1_g = self._ped_panel(lf, col=1, label="FgA1\n(Rechts)")
        self._fga2_r, self._fga2_g = self._ped_panel(lf, col=2, label="FgA2\n(Links)")

        ttk.Separator(r).pack(fill="x", padx=12, pady=8)

        # Config panel (FA-06, FA-07, FA-08, FA-09, NFA-U01)
        self._cfg_frame = tk.Frame(r, bg=_BG)
        self._cfg_frame.pack(padx=20, pady=4)
        self._build_config(self._cfg_frame)

        ttk.Separator(r).pack(fill="x", padx=12, pady=8)

        # Control buttons
        cf = tk.Frame(r, bg=_BG)
        cf.pack(pady=4)

        self._btn_start = tk.Button(
            cf, text="▶  Start", font=("Helvetica", 11),
            bg="#1a6b3a", fg="white", width=12, command=self._start)
        self._btn_start.grid(row=0, column=0, padx=6)

        self._btn_stop = tk.Button(
            cf, text="■  Stop", font=("Helvetica", 11),
            bg="#6b1a1a", fg="white", width=12,
            command=self._stop, state="disabled")
        self._btn_stop.grid(row=0, column=1, padx=6)

        self._btn_error = tk.Button(
            cf, text="⚠  Fehler / Fail-safe", font=("Helvetica", 10),
            bg="#7a3a00", fg="white", width=20, command=self._trigger_error)
        self._btn_error.grid(row=0, column=2, padx=6)

        # Emulator extras
        if self.mock:
            ttk.Separator(r).pack(fill="x", padx=12, pady=6)
            ef = tk.Frame(r, bg=_BG)
            ef.pack(pady=4)

            tk.Label(ef, text="[Emulator] Fahrzeugsensor:",
                     bg=_BG, fg="#cccccc", font=("Helvetica", 10)
                     ).grid(row=0, column=0, padx=8)

            self._sensor_var = tk.BooleanVar(value=False)
            tk.Checkbutton(ef, text="Fahrzeug wartet", variable=self._sensor_var,
                           bg=_BG, fg="white", selectcolor="#333355",
                           activebackground=_BG, font=("Helvetica", 10),
                           command=self._toggle_sensor
                           ).grid(row=0, column=1, padx=8)

            self._sensor_lbl = tk.Label(ef, text="Kein Fahrzeug",
                                        bg=_BG, fg="#88aa88", font=("Helvetica", 10))
            self._sensor_lbl.grid(row=0, column=2, padx=8)

            sf = tk.Frame(r, bg=_BG)
            sf.pack(pady=(2, 10))
            tk.Label(sf, text="[Emulator] Geschwindigkeit:",
                     bg=_BG, fg="#cccccc", font=("Helvetica", 10)
                     ).grid(row=0, column=0, padx=8)
            self._speed_var = tk.DoubleVar(value=self.ctrl.speed)
            ttk.Scale(sf, from_=1, to=30, orient="horizontal",
                      variable=self._speed_var, length=180,
                      command=self._set_speed).grid(row=0, column=1, padx=8)
            self._speed_lbl = tk.Label(sf, text="1×", bg=_BG, fg="#aaaaff",
                                       font=("Courier", 9), width=5)
            self._speed_lbl.grid(row=0, column=2)

    def _build_config(self, parent):
        tk.Label(parent, text="Operator-Konfiguration (FA-06)",
                 font=("Helvetica", 12, "bold"), bg=_BG, fg="#ffcc44"
                 ).grid(row=0, column=0, columnspan=3, pady=(0, 8))

        # FzA Rot-Rot-Periodendauer / Grünphase  (FA-07, SR-05)
        tk.Label(parent,
                 text=f"FzA Rot-Rot-Periodendauer ({FZA_GREEN_MIN}–{FZA_GREEN_MAX} s):",
                 bg=_BG, fg="#cccccc", font=("Helvetica", 10),
                 anchor="w", width=32
                 ).grid(row=1, column=0, sticky="w", pady=4)
        self._fza_entry = tk.Entry(parent, width=7, font=("Courier", 11))
        self._fza_entry.insert(0, str(self.ctrl.fza_green_time))
        self._fza_entry.grid(row=1, column=1, padx=6)
        self._fza_err = tk.Label(parent, text="", bg=_BG, fg="#ff6666",
                                 font=("Helvetica", 9), width=32, anchor="w")
        self._fza_err.grid(row=1, column=2, sticky="w")

        # FgA Grünphase  (FA-08)
        tk.Label(parent,
                 text=f"FgA Grünphase ({FGA_GRN_MIN}–{FGA_GRN_MAX} s):",
                 bg=_BG, fg="#cccccc", font=("Helvetica", 10),
                 anchor="w", width=32
                 ).grid(row=2, column=0, sticky="w", pady=4)
        self._fga_entry = tk.Entry(parent, width=7, font=("Courier", 11))
        self._fga_entry.insert(0, str(self.ctrl.fga_green_time))
        self._fga_entry.grid(row=2, column=1, padx=6)
        self._fga_err = tk.Label(parent, text="", bg=_BG, fg="#ff6666",
                                 font=("Helvetica", 9), width=32, anchor="w")
        self._fga_err.grid(row=2, column=2, sticky="w")

    def _car_panel(self, parent, col):
        f = tk.Frame(parent, bg=_CAR, bd=2, relief="ridge")
        f.grid(row=0, column=col, padx=14, pady=4)
        tk.Label(f, text="FzA\n(Fahrzeugampel)", bg=_CAR, fg="white",
                 font=("Helvetica", 9, "bold")).pack(pady=(8, 2))
        r = _Lens(f, _RED, bg=_CAR); r.pack(pady=3)
        y = _Lens(f, _YEL, bg=_CAR); y.pack(pady=3)
        g = _Lens(f, _GRN, bg=_CAR); g.pack(pady=3)
        tk.Frame(f, height=6, bg=_CAR).pack()
        return r, y, g

    def _ped_panel(self, parent, col, label):
        f = tk.Frame(parent, bg=_PED, bd=2, relief="ridge")
        f.grid(row=0, column=col, padx=14, pady=4)
        tk.Label(f, text=f"FgA\n{label}", bg=_PED, fg="white",
                 font=("Helvetica", 9, "bold")).pack(pady=(8, 2))
        r = _Lens(f, _RED, bg=_PED); r.pack(pady=3)
        g = _Lens(f, _GRN, bg=_PED); g.pack(pady=3)
        tk.Frame(f, height=6, bg=_PED).pack()
        return r, g

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _validate(self) -> bool:
        """FA-09: reject invalid input, show entered value and allowed range."""
        ok = True

        try:
            v = int(self._fza_entry.get())
            if FZA_GREEN_MIN <= v <= FZA_GREEN_MAX:
                self._fza_err.config(text="")
                self.ctrl.fza_green_time = v
            else:
                self._fza_err.config(
                    text=f"Wert {v} ungültig — erlaubt [{FZA_GREEN_MIN}, {FZA_GREEN_MAX}]")
                ok = False
        except ValueError:
            self._fza_err.config(text="Ganzzahl erforderlich")
            ok = False

        try:
            v = int(self._fga_entry.get())
            if FGA_GRN_MIN <= v <= FGA_GRN_MAX:
                self._fga_err.config(text="")
                self.ctrl.fga_green_time = v
            else:
                self._fga_err.config(
                    text=f"Wert {v} ungültig — erlaubt [{FGA_GRN_MIN}, {FGA_GRN_MAX}]")
                ok = False
        except ValueError:
            self._fga_err.config(text="Ganzzahl erforderlich")
            ok = False

        return ok

    def _start(self):
        if not self._validate():
            return
        self._fza_entry.config(state="disabled")
        self._fga_entry.config(state="disabled")
        self._btn_start.config(state="disabled")
        self._btn_stop.config(state="normal")
        threading.Thread(target=self.ctrl.run, daemon=True).start()

    def _stop(self):
        self.ctrl.stop()
        self._fza_entry.config(state="normal")
        self._fga_entry.config(state="normal")
        self._btn_start.config(state="normal")
        self._btn_stop.config(state="disabled")

    def _trigger_error(self):
        """FA-13: force permanent safe state — all red, locked until app restart."""
        self.ctrl.trigger_error()
        self._fza_entry.config(state="disabled")
        self._fga_entry.config(state="disabled")
        self._btn_start.config(state="disabled")
        self._btn_stop.config(state="disabled")

    def _toggle_sensor(self):
        present = self._sensor_var.get()
        self.ctrl.sensor.simulate(present)

    def _set_speed(self, v):
        self.ctrl.speed = float(v)
        self._speed_lbl.config(text=f"{float(v):.0f}×")

    # ── Display update ─────────────────────────────────────────────────────────

    def _timer_text(self) -> str:
        c       = self.ctrl
        elapsed = c.phase_elapsed
        spd     = max(c.speed, 0.1)
        state   = c.state

        if state == RPiZA.S_FZA_GREEN:
            total = c.fza_green_time / spd
        elif state == RPiZA.S_FGA_GREEN:
            total = c.fga_green_time / spd
        elif state == RPiZA.S_FZA_YEL:
            total = T_YELLOW
        elif state == RPiZA.S_RAEUMZEIT:
            total = T_RAEUMZEIT
        elif state == RPiZA.S_ROT_GELB:
            total = T_ROT_GELB
        else:
            return f"Elapsed: {elapsed:.0f}s"

        remaining = max(total - elapsed, 0)

        # warn when vehicle can trigger adaptive cut
        if state == RPiZA.S_FGA_GREEN:
            min_left = max(T_MIN_GREEN / spd - elapsed, 0)
            if min_left > 0:
                return f"{elapsed:.0f}s / {total:.0f}s  (min-green in {min_left:.0f}s)"
            if c.sensor.detected:
                return f"{elapsed:.0f}s / {total:.0f}s  ⚡ cutting short…"

        return f"{elapsed:.0f}s / {total:.0f}s  ({remaining:.0f}s left)"

    def _refresh(self):
        L = self.ctrl.leds
        self._fza_r.set_on(L['fza_red'])
        self._fza_y.set_on(L['fza_yellow'])
        self._fza_g.set_on(L['fza_green'])
        self._fga1_r.set_on(L['fga1_red'])
        self._fga1_g.set_on(L['fga1_green'])
        self._fga2_r.set_on(L['fga2_red'])
        self._fga2_g.set_on(L['fga2_green'])
        self._status_var.set(_STATE_LABELS.get(self.ctrl.state, self.ctrl.state))
        self._timer_var.set(self._timer_text())

        if self.mock:
            detected = self.ctrl.sensor.detected
            self._sensor_lbl.config(
                text="Fahrzeug erkannt!" if detected else "Kein Fahrzeug",
                fg="#ff4444" if detected else "#88aa88")

    def _poll(self):
        self._refresh()
        self.root.after(80, self._poll)


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    ctrl = RPiZA(mock=_MOCK)
    RPiZAGui(root, ctrl, mock=_MOCK)

    def on_close():
        ctrl.cleanup()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
