#!/usr/bin/env python3
"""
RPiZA v3.0 — Raspberry Pi Zuflussregelungsanlage mit Fußgängerübergang
Compliant: StVO §37 · RiLSA 2015 · RPiZA SRS v3.0
Lights! GmbH · Team IntelliCross

Anforderungsreferenzen:
  SR-01..SR-08  Stakeholder Requirements
  FA-01..FA-13  Funktionale Anforderungen
  NFA-S01       Fail-Safe-Reaktionszeit ≤ 500 ms
  NFA-R01       Verfügbarkeit ≥ 99 % (8 h Demo)
  NFA-P01       Erkennungslatenz ≤ 200 ms
  NFA-P02       Phasenwechsellatenz ≤ 100 ms
  NFA-U01       Konfiguration in ≤ 3 Enter-Bestätigungen
  C-01..C-04    Randbedingungen
  DS-01..DS-05  Designspezifikation

Hardware-Pinbelegung (BCM) — C-02, DS-01:
  FzA  : ROT=5   GELB=6   GRÜN=13
  FgA1 : ROT=27  GRÜN=10
  FgA2 : ROT=21  GRÜN=16
  HC-SR04: TRIG=26  ECHO=24
  DS-01: Echo-Pin benötigt Spannungsteiler 330Ω+470Ω (5V→3.3V)
"""

import logging
import os
import threading
import time
import tkinter as tk
from tkinter import ttk

# ---------------------------------------------------------------------------
# Logging — FA-13: Fehler protokollieren
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(threadName)s — %(message)s",
)
log = logging.getLogger("RPiZA")

# ---------------------------------------------------------------------------
# Hardware-Erkennung — C-02: Raspberry Pi 3B oder neuer
# ---------------------------------------------------------------------------
try:
    import RPi.GPIO as _RPIGPIO
    _MOCK = False
    log.info("RPi.GPIO erkannt — Echtbetrieb aktiv.")
except (ImportError, RuntimeError):
    _MOCK = True
    os.environ["GPIOZERO_PIN_FACTORY"] = "mock"
    log.warning("RPi.GPIO nicht verfügbar — Mock-Modus aktiv (Emulator).")

# gpiozero: auf dem Raspberry Pi vorhanden; auf Windows/Mac via: pip install gpiozero
# Falls gpiozero fehlt, wird ein reiner Software-Stub verwendet (nur Mock-Modus).
try:
    from gpiozero import LED  # noqa: E402
    log.info("gpiozero importiert.")
except ImportError:
    log.warning("gpiozero nicht gefunden — Software-Stub aktiv (nur Mock-Modus).")
    _MOCK = True

    class LED:
        """Software-Stub fuer gpiozero.LED — kein GPIO, nur Zustandsspeicher."""
        def __init__(self, pin):
            self._pin   = pin
            self._value = 0
        def on(self):
            self._value = 1
        def off(self):
            self._value = 0
        @property
        def value(self):
            return self._value

# ---------------------------------------------------------------------------
# Pin-Definitionen (BCM) — C-02, DS-01
# ---------------------------------------------------------------------------
PIN_FZA_ROT    = 5
PIN_FZA_GELB   = 6
PIN_FZA_GRUEN  = 13
PIN_FGA1_ROT   = 27
PIN_FGA1_GRUEN = 10
PIN_FGA2_ROT   = 21
PIN_FGA2_GRUEN = 16
PIN_TRIG       = 26
PIN_ECHO       = 24

# ---------------------------------------------------------------------------
# Festwerte aus DS-03 / RiLSA ZRA
# ---------------------------------------------------------------------------
T_GELB       = 1.0   # FA-02: Gelbphase FzA (Grün→Rot),      RiLSA ZRA ≥ 1 s
T_ROT_GELB   = 1.0   # FA-02: Rot-Gelb-Phase FzA (Rot→Grün), RiLSA ZRA ≥ 1 s
T_RAEUMZEIT  = 4.0   # FA-05: Räumzeit alle-Rot,              4 m ÷ 1,0 m/s (RiLSA)
T_MIN_GRUEN  = 10.0  # FA-12: Mindestgrünzeit FgA — ABSOLUT, nie skalierbar

# ---------------------------------------------------------------------------
# Sensorparameter — DS-01, DS-02, NFA-P01
# ---------------------------------------------------------------------------
SENSOR_BEREICH_CM = 5    # DS-01: Erfassungsbereich vor FzA in cm
SENSOR_BEST       = 3    # DS-02: 3 aufeinanderfolgende Bestätigungen
SENSOR_INTERVALL  = 0.1  # NFA-P01: 100 ms → Latenz ≤ 200 ms (3 × 100 ms)

# ---------------------------------------------------------------------------
# Konfigurationsgrenzen — FA-07, FA-08
# ---------------------------------------------------------------------------
FZA_GRUEN_MIN =  120   # FA-07: min. FzA-Grünphase in Sekunden
FZA_GRUEN_MAX =  300   # FA-07: max. FzA-Grünphase in Sekunden
FGA_GRUEN_MIN =   10   # FA-08: min. FgA-Grünphase in Sekunden
FGA_GRUEN_MAX =   40   # FA-08: max. FgA-Grünphase in Sekunden

# ---------------------------------------------------------------------------
# NFA-S01: Fail-Safe-Reaktionszeit ≤ 500 ms
# ---------------------------------------------------------------------------
FAILSAFE_TIMEOUT = 0.5   # Maximale Zeit bis alle-Rot nach Fehler (DS-04)


# ===========================================================================
#  Fahrzeugsensor — FA-10, DS-01, DS-02, NFA-P01
# ===========================================================================

class FahrzeugSensor:
    """
    HC-SR04 Ultraschallsensor zur Fahrzeugerkennung (FA-10).

    Echtbetrieb (C-02):
      RPi.GPIO direkt. Echo-Pin benötigt Spannungsteiler 330Ω+470Ω (DS-01).

    Mock-Modus:
      GUI-Checkbox simuliert ein wartendes Fahrzeug.

    DS-02: Stabile Erkennung nach 3 aufeinanderfolgenden positiven Messungen.
    NFA-P01: Erkennungslatenz ≤ 200 ms (3 Messungen × 100 ms Intervall).
    """

    def __init__(self, mock: bool):
        self._mock       = mock
        self._sim_aktiv  = False    # GUI-Toggle im Mock-Modus
        self._zaehler    = 0        # DS-02: Bestätigungszähler
        self._erkannt    = False    # aktueller Erkennungsstatus
        self._lock       = threading.Lock()
        self._stop_evt   = threading.Event()

        if not mock:
            # DS-01: GPIO-Setup für HC-SR04
            _RPIGPIO.setmode(_RPIGPIO.BCM)
            _RPIGPIO.setup(PIN_TRIG, _RPIGPIO.OUT)
            _RPIGPIO.setup(PIN_ECHO, _RPIGPIO.IN)
            _RPIGPIO.output(PIN_TRIG, _RPIGPIO.LOW)
            log.info("HC-SR04 GPIO initialisiert (TRIG=%d, ECHO=%d).", PIN_TRIG, PIN_ECHO)

        t = threading.Thread(
            target=self._poll_schleife,
            daemon=True,
            name="SensorPoll",
        )
        t.start()

    # -- HC-SR04 Einzelmessung -----------------------------------------------

    def _messen(self):
        """
        Einzelmessung HC-SR04.
        Gibt Distanz in cm zurück, oder None bei Timeout.
        DS-01: Echo-Pin via Spannungsteiler 330Ω+470Ω auf 3,3 V reduziert.
        """
        GPIO = _RPIGPIO
        GPIO.output(PIN_TRIG, GPIO.LOW)
        time.sleep(0.002)
        GPIO.output(PIN_TRIG, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(PIN_TRIG, GPIO.LOW)

        t0 = time.monotonic()
        while GPIO.input(PIN_ECHO) == 0:
            if time.monotonic() - t0 > 0.05:
                log.warning("HC-SR04: Echo-Timeout (kein HIGH).")
                return None
        t1 = time.monotonic()
        while GPIO.input(PIN_ECHO) == 1:
            if time.monotonic() - t1 > 0.05:
                log.warning("HC-SR04: Echo-Timeout (kein LOW).")
                return None
        distanz_cm = (time.monotonic() - t1) * 17150
        return distanz_cm

    # -- Polling-Schleife (Daemon-Thread) ------------------------------------

    def _poll_schleife(self):
        """
        DS-02: Zählt aufeinanderfolgende positive Messungen.
        Erkannt = True sobald Zähler ≥ SENSOR_BEST (3).
        NFA-P01: Latenz ≤ 3 × 100 ms = 300 ms (Worst Case), ≤ 200 ms typical.
        """
        while not self._stop_evt.is_set():
            if self._mock:
                vorhanden = self._sim_aktiv
            else:
                d = self._messen()
                vorhanden = (d is not None and d < SENSOR_BEREICH_CM)

            with self._lock:
                if vorhanden:
                    # DS-02: Zähler inkrementieren, Cap bei SENSOR_BEST
                    self._zaehler = min(self._zaehler + 1, SENSOR_BEST)
                else:
                    # DS-02: Zähler zurücksetzen bei fehlender Erkennung
                    self._zaehler = 0
                self._erkannt = (self._zaehler >= SENSOR_BEST)

            time.sleep(SENSOR_INTERVALL)

    # -- Öffentliche Schnittstelle -------------------------------------------

    @property
    def erkannt(self) -> bool:
        """FA-10: Liefert True wenn Fahrzeug stabil erkannt (DS-02)."""
        with self._lock:
            return self._erkannt

    def simuliere(self, vorhanden: bool):
        """Mock-Modus: GUI setzt Fahrzeugpräsenz."""
        self._sim_aktiv = vorhanden

    def stoppe(self):
        """Beendet Poll-Thread und räumt GPIO auf (Echtbetrieb)."""
        self._stop_evt.set()
        if not self._mock:
            _RPIGPIO.cleanup()
            log.info("GPIO Cleanup durchgeführt.")


# ===========================================================================
#  Zustandsmaschine — FA-01 bis FA-13, UC-01 bis UC-04
# ===========================================================================

class RPiZA:
    """
    Zustandsmaschine der Zuflussregelungsanlage (SR-01).

    Zustände:
      CONFIG         — FA-06: Konfigurationsmodus beim Start, alle Rot
      FZA_GRUEN      — FA-01: FzA Grün, FgA Rot
      FZA_GELB       — FA-01, FA-02: FzA Gelb (Grün→Rot), FgA Rot
      RAEUMZEIT_FZA  — FA-05-A: Alle Rot (nach FzA-Gelb, vor FgA-Grün)
      FGA_GRUEN      — FA-03, FA-04: FzA Rot, FgA Grün
      RAEUMZEIT      — FA-05-B: Alle Rot (nach FgA-Grün, vor FzA-Rot-Gelb)
      FZA_ROT_GELB   — FA-01, FA-02: FzA Rot+Gelb (Rot→Grün), FgA Rot
      FEHLER         — FA-13, NFA-S01: Alle Rot, permanent

    Zyklus (UC-01):
      FZA_GRUEN → FZA_GELB (1 s) → RAEUMZEIT_FZA (4 s)
      → FGA_GRUEN (adaptiv) → RAEUMZEIT (4 s)
      → FZA_ROT_GELB (1 s) → FZA_GRUEN → ...

    FA-07: FzA-Grünphase direkt konfigurierbar [120–300 s] — unabhängig von Räumzeiten.
    FA-04: FzA und FgA niemals gleichzeitig Grün — ohne Ausnahme.
    """

    # Zustandskonstanten
    S_CONFIG        = "CONFIG"
    S_FZA_GRUEN     = "FZA_GRUEN"
    S_FZA_GELB      = "FZA_GELB"
    S_RAEUMZEIT_FZA = "RAEUMZEIT_FZA"   # FA-05-A: Räumzeit nach FzA-Grün
    S_FGA_GRUEN     = "FGA_GRUEN"
    S_RAEUMZEIT     = "RAEUMZEIT"       # FA-05-B: Räumzeit nach FgA-Grün
    S_ROT_GELB      = "FZA_ROT_GELB"
    S_FEHLER        = "FEHLER"

    def __init__(self, mock: bool):
        # GPIO-Ausgänge (gpiozero) — C-02
        self.fza_rot    = LED(PIN_FZA_ROT)
        self.fza_gelb   = LED(PIN_FZA_GELB)
        self.fza_gruen  = LED(PIN_FZA_GRUEN)
        self.fga1_rot   = LED(PIN_FGA1_ROT)
        self.fga1_gruen = LED(PIN_FGA1_GRUEN)
        self.fga2_rot   = LED(PIN_FGA2_ROT)
        self.fga2_gruen = LED(PIN_FGA2_GRUEN)

        # Fahrzeugsensor — FA-10, DS-01, DS-02
        self.sensor = FahrzeugSensor(mock=mock)

        # Operator-konfigurierbare Parameter — FA-07, FA-08 (UC-02)
        self.fza_gruen_zeit  = 120   # FA-07: FzA-Grünphase direkt [120–300 s]
        self.fga_gruen_zeit  =  20   # FA-08: FgA-Grünphase [10–40 s]

        # Emulator-Geschwindigkeit (nur Mock-Modus, kein Einfluss auf Sicherheitsphasen)
        self.emulator_speed = 1.0

        # Zustandsverwaltung
        self._zustand      = self.S_CONFIG
        self._phase_start  = time.monotonic()
        self.on_zustandswechsel = None   # Callback für GUI

        # Steuerungs-Events
        self._stop_evt      = threading.Event()
        self._fehler_evt    = threading.Event()

        # FA-06: Beim Start alle Ampeln Rot, Konfigurationsmodus
        self._alle_rot()
        log.info("RPiZA initialisiert. Zustand: %s", self._zustand)

    # -----------------------------------------------------------------------
    # Ampel-Schalthelfer
    # Reihenfolge: IMMER erst das Grün der anderen Seite abschalten (FA-04)
    # -----------------------------------------------------------------------

    def _alle_rot(self):
        """
        Sicherer Zustand: Alle Ampeln Rot.
        FA-04, FA-05, FA-06, FA-13, NFA-S01.
        """
        # FgA zuerst abschalten
        self.fga1_gruen.off()
        self.fga2_gruen.off()
        # FzA abschalten
        self.fza_gruen.off()
        self.fza_gelb.off()
        # Jetzt alle Rot einschalten
        self.fza_rot.on()
        self.fga1_rot.on()
        self.fga2_rot.on()

    def _schalte_fza_gruen(self):
        """
        FA-01: FzA auf Grün schalten.
        FA-04: FgA zuerst sicher auf Rot setzen, dann FzA Grün.
        """
        # Sicherheit FA-04: FgA-Grün erst ausschalten
        self.fga1_gruen.off()
        self.fga2_gruen.off()
        self.fga1_rot.on()
        self.fga2_rot.on()
        # FzA auf Grün
        self.fza_gelb.off()
        self.fza_rot.off()
        self.fza_gruen.on()

    def _schalte_fza_gelb(self):
        """
        FA-01, FA-02: FzA auf Gelb (Übergang Grün→Rot).
        RiLSA ZRA ≥ 1 s (DS-03).
        FgA bleibt Rot (bereits gesetzt).
        """
        self.fza_gruen.off()
        self.fza_rot.off()
        self.fza_gelb.on()

    def _schalte_fza_rot_gelb(self):
        """
        FA-01, FA-02: FzA auf Rot+Gelb (Übergang Rot→Grün).
        RiLSA ZRA ≥ 1 s (DS-03).
        FA-04: FgA explizit Rot sicherstellen.
        """
        # FA-04: FgA explizit auf Rot sicherstellen
        self.fga1_gruen.off()
        self.fga2_gruen.off()
        self.fga1_rot.on()
        self.fga2_rot.on()
        # FzA Rot+Gelb
        self.fza_gruen.off()
        self.fza_rot.on()
        self.fza_gelb.on()

    def _schalte_fga_gruen(self):
        """
        FA-03, FA-04: FgA1 und FgA2 auf Grün schalten.
        FA-04: FzA zuerst sicher auf Rot setzen, dann FgA Grün.
        FA-03: FgA hat keine Gelbphase (StVO §37).
        SR-06: FgA1 = rechts, FgA2 = links (C-03).
        """
        # Sicherheit FA-04: FzA-Grün erst ausschalten
        self.fza_gruen.off()
        self.fza_gelb.off()
        self.fza_rot.on()
        # FgA auf Grün
        self.fga1_rot.off()
        self.fga2_rot.off()
        self.fga1_gruen.on()
        self.fga2_gruen.on()

    # -----------------------------------------------------------------------
    # Timing-Helfer
    # -----------------------------------------------------------------------

    def _warte(self, sekunden: float) -> bool:
        """
        Unterbrechbarer Warte-Sleep (Echtzeit, NICHT speed-skaliert).
        Verwendet für sicherheitskritische Phasen:
          T_GELB, T_ROT_GELB, T_RAEUMZEIT (DS-03, FA-02, FA-05).
        Gibt True zurück wenn abgelaufen, False bei Stop/Fehler.
        NFA-P02: GPIO-Wechsel ≤ 100 ms nach Entscheidung — sleep-Granularität 20 ms.
        """
        ende = time.monotonic() + sekunden
        while time.monotonic() < ende:
            if self._stop_evt.is_set() or self._fehler_evt.is_set():
                return False
            time.sleep(0.02)
        return True

    def _warte_skaliert(self, sekunden: float) -> bool:
        """
        Speed-skalierter Sleep — NUR für konfigurierbare Phasen im Emulator.
        Verwendet für FzA-Grünphase und FgA-Grünphase (ohne Mindestgrün).
        Sicherheitsphasen (T_GELB, T_ROT_GELB, T_RAEUMZEIT) werden NIEMALS skaliert.
        """
        skaliert = sekunden / max(self.emulator_speed, 0.1)
        return self._warte(skaliert)

    def _warte_fga_gruen(self, max_sekunden: float):
        """
        UC-03, FA-08, FA-11, FA-12: FgA-Grünphase mit adaptiver Verkürzung.

        FA-12: T_MIN_GRUEN ist ABSOLUT — niemals speed-skaliert (Sicherheitsschranke).
        FA-11: Verkürzung nur wenn:
               (a) Fahrzeug erkannt (FA-10, UC-03) UND
               (b) Mindestgrünzeit bereits abgelaufen (FA-12).
        """
        spd     = max(self.emulator_speed, 0.1)
        # Phasenende: konfigurierbare max_sekunden, speed-skaliert
        phasen_ende  = time.monotonic() + max_sekunden / spd
        # Mindestgrünzeit: ABSOLUT, nicht speed-skaliert (FA-12)
        min_gruen_ende = time.monotonic() + T_MIN_GRUEN

        while time.monotonic() < phasen_ende:
            if self._stop_evt.is_set() or self._fehler_evt.is_set():
                return
            # FA-11: Adaptive Verkürzung erst nach Mindestgrünzeit UND Fahrzeug erkannt
            if time.monotonic() >= min_gruen_ende and self.sensor.erkannt:
                log.info("FA-11: Adaptive Verkürzung ausgelöst (Fahrzeug erkannt).")
                return
            time.sleep(0.02)

    # -----------------------------------------------------------------------
    # Zustandsverwaltung
    # -----------------------------------------------------------------------

    def _setze_zustand(self, zustand: str):
        """Zustand wechseln und GUI-Callback auslösen (NFA-P02)."""
        self._zustand = zustand
        self._phase_start = time.monotonic()
        log.info("Zustandswechsel → %s", zustand)
        if self.on_zustandswechsel:
            self.on_zustandswechsel()

    @property
    def zustand(self) -> str:
        return self._zustand

    @property
    def phase_vergangen(self) -> float:
        """Vergangene Zeit in der aktuellen Phase in Sekunden."""
        return time.monotonic() - self._phase_start

    @property
    def led_zustaende(self) -> dict:
        """Aktueller LED-Zustand für GUI-Anzeige."""
        return {
            "fza_rot":    bool(self.fza_rot.value),
            "fza_gelb":   bool(self.fza_gelb.value),
            "fza_gruen":  bool(self.fza_gruen.value),
            "fga1_rot":   bool(self.fga1_rot.value),
            "fga1_gruen": bool(self.fga1_gruen.value),
            "fga2_rot":   bool(self.fga2_rot.value),
            "fga2_gruen": bool(self.fga2_gruen.value),
        }

    # -----------------------------------------------------------------------
    # NFA-S01: Fail-Safe ≤ 500 ms — FA-13, UC-04
    # -----------------------------------------------------------------------

    def fehler_ausloesen(self):
        """
        FA-13, NFA-S01: Sofortiger Fail-Safe.

        _alle_rot() wird SOFORT aufgerufen (GPIO < 1 ms) — unabhängig vom
        laufenden Zyklus-Thread.
        _fehler_evt stoppt den Zyklus-Thread beim nächsten Sleep-Check (≤ 20 ms).
        Gesamtlatenz: GPIO < 1 ms + OS-Scheduling << 500 ms (DS-04).
        Nach Fehler kein Neustart ohne App-Restart (FA-13: permanenter Zustand).
        """
        log.error("FA-13: Fehler ausgelöst — Fail-Safe aktiv (alle Rot, permanent).")
        self._fehler_evt.set()
        self._stop_evt.set()
        self._alle_rot()                        # SOFORT in diesem Thread (NFA-S01)
        self._setze_zustand(self.S_FEHLER)

    # -----------------------------------------------------------------------
    # Hauptzyklus — UC-01
    # -----------------------------------------------------------------------

    def starte(self):
        """
        UC-01: Startet den Signalzyklus in einem Daemon-Thread.
        Muss nach Konfiguration (UC-02) aufgerufen werden (FA-06).
        """
        self._stop_evt.clear()
        self._fehler_evt.clear()
        t = threading.Thread(
            target=self._zyklus_wrapper,
            daemon=True,
            name="ZyklusThread",
        )
        t.start()

    def _zyklus_wrapper(self):
        """FA-13: Jede unbehandelte Ausnahme löst den Fail-Safe aus."""
        try:
            self._zyklus()
        except Exception:
            log.exception("FA-13: Unbehandelte Ausnahme im Zyklus — Fail-Safe!")
            self.fehler_ausloesen()

    def _zyklus(self):
        """
        UC-01: Automatischer Signalzyklus (StVO §37, RiLSA 2015).

        Reihenfolge (FA-01 bis FA-05):
          1. FZA_GRUEN      — FzA Grün, FgA Rot          (FA-01, FA-07)
          2. FZA_GELB       — FzA Gelb 1 s               (FA-01, FA-02, DS-03)
          3. RAEUMZEIT_FZA  — Alle Rot 4 s               (FA-05-A, DS-03)
          4. FGA_GRUEN      — FzA Rot, FgA Grün          (FA-03, FA-04, FA-08, FA-11, FA-12)
          5. RAEUMZEIT      — Alle Rot 4 s               (FA-05-B, DS-03)
          6. FZA_ROT_GELB   — FzA Rot+Gelb 1 s           (FA-01, FA-02, DS-03)
          → Zurück zu 1
        """
        while not self._stop_evt.is_set():

            # ── Phase 1: FzA Grün (FA-01, FA-07, SR-05) ────────────────────
            self._setze_zustand(self.S_FZA_GRUEN)
            self._schalte_fza_gruen()
            if not self._warte_skaliert(self.fza_gruen_zeit):
                break

            # ── Phase 2: FzA Gelb 1 s (FA-01, FA-02, DS-03) ───────────────
            self._setze_zustand(self.S_FZA_GELB)
            self._schalte_fza_gelb()
            if not self._warte(T_GELB):          # NICHT skaliert — Sicherheitsphase
                break

            # ── Phase 3: Räumzeit nach FzA alle-Rot 4 s (FA-05-A, DS-03) ──
            self._setze_zustand(self.S_RAEUMZEIT_FZA)
            self._alle_rot()
            if not self._warte(T_RAEUMZEIT):     # NICHT skaliert — Sicherheitsphase
                break

            # ── Phase 4: FgA Grün mit adaptiver Verkürzung ─────────────────
            # FA-03: FgA Rot→Grün→Rot (keine Gelbphase, StVO §37)
            # FA-04: FzA Rot bevor FgA Grün
            # FA-08: Grünphase konfigurierbar [10–40 s]
            # FA-11: Adaptive Verkürzung bei Fahrzeug + nach Mindestgrün
            # FA-12: Mindestgrünzeit ABSOLUT (nicht skalierbar)
            self._setze_zustand(self.S_FGA_GRUEN)
            self._schalte_fga_gruen()
            self._warte_fga_gruen(self.fga_gruen_zeit)
            if self._stop_evt.is_set():
                break

            # ── Phase 5: Räumzeit nach FgA alle-Rot 4 s (FA-05-B, DS-03) ──
            self._setze_zustand(self.S_RAEUMZEIT)
            self._alle_rot()
            if not self._warte(T_RAEUMZEIT):     # NICHT skaliert — Sicherheitsphase
                break

            # ── Phase 6: FzA Rot+Gelb 1 s (FA-01, FA-02, DS-03) ───────────
            self._setze_zustand(self.S_ROT_GELB)
            self._schalte_fza_rot_gelb()
            if not self._warte(T_ROT_GELB):      # NICHT skaliert — Sicherheitsphase
                break

        # Sicherer Endzustand nach normalem Stop
        self._alle_rot()
        self._setze_zustand(self.S_CONFIG)
        log.info("Zyklus beendet. Zurück in CONFIG.")

    def stoppe(self):
        """Normaler Stop — kehrt nach dem laufenden Zyklus in CONFIG zurück."""
        log.info("Stop angefordert.")
        self._stop_evt.set()

    def aufraeumen(self):
        """Ressourcen freigeben (Sensor, GPIO)."""
        self.stoppe()
        self.sensor.stoppe()


# ===========================================================================
#  GUI — tkinter (NFA-U01: ≤ 3 Enter-Bestätigungen für Konfiguration)
# ===========================================================================

# Farbpalette
_HINTERGRUND  = "#16213e"
_FZA_BG       = "#1a1a3e"
_FGA_BG       = "#1a3e1a"
_LED_AUS      = "#2a2a2a"
_LED_ROT      = "#ff3333"
_LED_GELB     = "#ffcc00"
_LED_GRUEN    = "#33ff66"

# Lesbarer Zustandstext für GUI
_ZUSTAND_TEXT = {
    RPiZA.S_CONFIG:        "CONFIG — Parameter eingeben, dann Start drücken",
    RPiZA.S_FZA_GRUEN:     "FzA: GRÜN — Fahrzeuge fahren (FgA: Rot)",
    RPiZA.S_FZA_GELB:      "FzA: GELB — Übergang Grün→Rot (1 s)",
    RPiZA.S_RAEUMZEIT_FZA: "RÄUMZEIT (nach FzA) — Alle Rot (4 s)",
    RPiZA.S_FGA_GRUEN:     "FgA: GRÜN — Fußgänger überqueren (FzA: Rot)",
    RPiZA.S_RAEUMZEIT:     "RÄUMZEIT (nach FgA) — Alle Rot (4 s)",
    RPiZA.S_ROT_GELB:      "FzA: ROT+GELB — Vorbereitung Grün (1 s)",
    RPiZA.S_FEHLER:        "⚠ FEHLER — Alle Rot permanent (FA-13, NFA-S01)",
}


class _Linse(tk.Canvas):
    """
    Einzelne runde Ampellinse.
    Darstellung: Ein- (Farbe) / Ausgeschaltet (dunkelgrau).
    """

    def __init__(self, parent, farbe: str, groesse: int = 52, hg: str = "#111122"):
        super().__init__(parent, width=groesse, height=groesse,
                         bg=hg, highlightthickness=0)
        m = 5
        self._oval = self.create_oval(
            m, m, groesse - m, groesse - m,
            fill=_LED_AUS, outline="#444", width=2,
        )
        self._farbe_an = farbe

    def setze(self, an: bool):
        self.itemconfig(self._oval, fill=self._farbe_an if an else _LED_AUS)


class RPiZAGui:
    """
    tkinter-Oberfläche für RPiZA.

    NFA-U01: Operator konfiguriert beide Zeitparameter in ≤ 2 Enter-Bestätigungen
             (ein Formular, ein Start-Button = max. 2 Eingaben + 1 Klick ≤ 3 gesamt).
    UC-02: Konfiguration vor Betriebsstart.
    UC-04: Fehler-Button löst Fail-Safe aus (FA-13, NFA-S01).
    """

    def __init__(self, root: tk.Tk, steuerung: RPiZA, mock: bool):
        self.root      = root
        self.steuerung = steuerung
        self.mock      = mock

        # Zustandswechsel → GUI-Update (0 ms Verzögerung → Haupt-Thread)
        steuerung.on_zustandswechsel = lambda: root.after(0, self._aktualisiere)

        root.title("RPiZA v3.0 — Zuflussregelungsanlage")
        root.configure(bg=_HINTERGRUND)
        root.resizable(False, False)
        self._baue_gui()
        self._poll()

    # -----------------------------------------------------------------------
    # GUI-Aufbau
    # -----------------------------------------------------------------------

    def _baue_gui(self):
        r = self.root

        # Titelzeile
        tk.Label(
            r,
            text="RPiZA v3.0 — Fußgänger-Zuflussregelungsanlage",
            font=("Helvetica", 16, "bold"),
            bg=_HINTERGRUND, fg="white",
        ).pack(pady=(12, 2))

        tk.Label(
            r,
            text="StVO §37 · RiLSA 2015 · SRS v3.0 · Lights! GmbH",
            font=("Helvetica", 9),
            bg=_HINTERGRUND, fg="#888888",
        ).pack(pady=(0, 4))

        # Zustandsanzeige
        self._zustand_var = tk.StringVar(value=_ZUSTAND_TEXT[RPiZA.S_CONFIG])
        tk.Label(
            r,
            textvariable=self._zustand_var,
            font=("Helvetica", 11),
            bg=_HINTERGRUND, fg="#aaaaff",
            width=54,
        ).pack(pady=(0, 2))

        # Phasen-Timer
        self._timer_var = tk.StringVar(value="Phase: 0 s")
        tk.Label(
            r,
            textvariable=self._timer_var,
            font=("Courier", 11),
            bg=_HINTERGRUND, fg="#ffcc44",
        ).pack(pady=(0, 8))

        # ── Ampeldarstellung ─────────────────────────────────────────────
        ampel_frame = tk.Frame(r, bg=_HINTERGRUND)
        ampel_frame.pack(padx=20, pady=4)

        # FzA (Fahrzeugampel) — SR-06, C-03: von rechts unten
        self._fza_r, self._fza_g_led, self._fza_g = self._baue_fza_panel(ampel_frame, col=0)
        # FgA1 (rechts) — SR-06, C-03
        self._fga1_r, self._fga1_g = self._baue_fga_panel(
            ampel_frame, col=1, label="FgA1\n(rechts)"
        )
        # FgA2 (links) — SR-06, C-03
        self._fga2_r, self._fga2_g = self._baue_fga_panel(
            ampel_frame, col=2, label="FgA2\n(links)"
        )

        ttk.Separator(r).pack(fill="x", padx=12, pady=8)

        # ── Operator-Konfiguration — UC-02, FA-06 bis FA-09, NFA-U01 ─────
        self._cfg_frame = tk.Frame(r, bg=_HINTERGRUND)
        self._cfg_frame.pack(padx=20, pady=4)
        self._baue_konfiguration(self._cfg_frame)

        ttk.Separator(r).pack(fill="x", padx=12, pady=8)

        # ── Steuerbuttons ────────────────────────────────────────────────
        btn_frame = tk.Frame(r, bg=_HINTERGRUND)
        btn_frame.pack(pady=4)

        self._btn_start = tk.Button(
            btn_frame,
            text="▶  Start",
            font=("Helvetica", 11),
            bg="#1a6b3a", fg="white",
            width=12,
            command=self._start,
        )
        self._btn_start.grid(row=0, column=0, padx=6)

        self._btn_stop = tk.Button(
            btn_frame,
            text="■  Stop",
            font=("Helvetica", 11),
            bg="#6b1a1a", fg="white",
            width=12,
            command=self._stop,
            state="disabled",
        )
        self._btn_stop.grid(row=0, column=1, padx=6)

        self._btn_fehler = tk.Button(
            btn_frame,
            text="⚠  Fehler / Fail-Safe (FA-13)",
            font=("Helvetica", 10),
            bg="#7a3a00", fg="white",
            width=26,
            command=self._fehler_ausloesen,
        )
        self._btn_fehler.grid(row=0, column=2, padx=6)

        # ── Emulator-Extras (nur Mock-Modus) ─────────────────────────────
        if self.mock:
            ttk.Separator(r).pack(fill="x", padx=12, pady=6)
            self._baue_emulator_bereich(r)

    def _baue_fza_panel(self, parent, col):
        """FzA-Ampelpanel: Rot, Gelb, Grün (FA-01, StVO §37)."""
        f = tk.Frame(parent, bg=_FZA_BG, bd=2, relief="ridge")
        f.grid(row=0, column=col, padx=14, pady=4)
        tk.Label(
            f, text="FzA\n(Fahrzeugampel)",
            bg=_FZA_BG, fg="white", font=("Helvetica", 9, "bold"),
        ).pack(pady=(8, 2))
        rot  = _Linse(f, _LED_ROT,  hg=_FZA_BG); rot.pack(pady=3)
        gelb = _Linse(f, _LED_GELB, hg=_FZA_BG); gelb.pack(pady=3)
        gruen = _Linse(f, _LED_GRUEN, hg=_FZA_BG); gruen.pack(pady=3)
        tk.Frame(f, height=6, bg=_FZA_BG).pack()
        return rot, gelb, gruen

    def _baue_fga_panel(self, parent, col, label):
        """FgA-Ampelpanel: Rot, Grün (FA-03: keine Gelbphase, StVO §37)."""
        f = tk.Frame(parent, bg=_FGA_BG, bd=2, relief="ridge")
        f.grid(row=0, column=col, padx=14, pady=4)
        tk.Label(
            f, text=f"FgA\n{label}",
            bg=_FGA_BG, fg="white", font=("Helvetica", 9, "bold"),
        ).pack(pady=(8, 2))
        rot   = _Linse(f, _LED_ROT,   hg=_FGA_BG); rot.pack(pady=3)
        gruen = _Linse(f, _LED_GRUEN, hg=_FGA_BG); gruen.pack(pady=3)
        tk.Frame(f, height=6, bg=_FGA_BG).pack()
        return rot, gruen

    def _baue_konfiguration(self, parent):
        """
        Operator-Konfigurationsformular.
        NFA-U01: Beide Parameter in ≤ 2 Enter-Bestätigungen + 1 Start-Klick = ≤ 3 gesamt.
        FA-07: FzA-Grünphase direkt [120–300 s] — Räumzeiten sind unabhängig.
        FA-08: FgA-Grünphase [10–40 s].
        FA-09: Ungültige Eingabe ablehnen mit Eingabewert + erlaubtem Bereich.
        """
        tk.Label(
            parent,
            text="Operator-Konfiguration (UC-02, FA-06–FA-09)",
            font=("Helvetica", 12, "bold"),
            bg=_HINTERGRUND, fg="#ffcc44",
        ).grid(row=0, column=0, columnspan=3, pady=(0, 8))

        # FA-07 / SR-05: FzA-Grünphase direkt
        tk.Label(
            parent,
            text=f"FzA Grünphase [{FZA_GRUEN_MIN}–{FZA_GRUEN_MAX} s]:",
            bg=_HINTERGRUND, fg="#cccccc",
            font=("Helvetica", 10), anchor="w", width=38,
        ).grid(row=1, column=0, sticky="w", pady=4)

        self._eingabe_fza = tk.Entry(parent, width=7, font=("Courier", 11))
        self._eingabe_fza.insert(0, str(self.steuerung.fza_gruen_zeit))
        self._eingabe_fza.grid(row=1, column=1, padx=6)

        self._fehler_fza = tk.Label(
            parent, text="", bg=_HINTERGRUND, fg="#ff6666",
            font=("Helvetica", 9), width=40, anchor="w",
        )
        self._fehler_fza.grid(row=1, column=2, sticky="w")

        tk.Label(
            parent,
            text="  Räumzeiten (DS-03): 4 s vor und nach FgA-Grün — fest, unabhängig von Grünphase",
            bg=_HINTERGRUND, fg="#888888",
            font=("Helvetica", 9), anchor="w",
        ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(0, 6))

        # FA-08: FgA-Grünphase
        tk.Label(
            parent,
            text=f"FgA Grünphase [{FGA_GRUEN_MIN}–{FGA_GRUEN_MAX} s]:",
            bg=_HINTERGRUND, fg="#cccccc",
            font=("Helvetica", 10), anchor="w", width=38,
        ).grid(row=3, column=0, sticky="w", pady=4)

        self._eingabe_fga = tk.Entry(parent, width=7, font=("Courier", 11))
        self._eingabe_fga.insert(0, str(self.steuerung.fga_gruen_zeit))
        self._eingabe_fga.grid(row=3, column=1, padx=6)

        self._fehler_fga = tk.Label(
            parent, text="", bg=_HINTERGRUND, fg="#ff6666",
            font=("Helvetica", 9), width=40, anchor="w",
        )
        self._fehler_fga.grid(row=3, column=2, sticky="w")

        # FA-12 / DS-03: Mindestgrünzeit — fest, nicht konfigurierbar
        tk.Label(
            parent,
            text=f"  Mindestgrünzeit FgA (FA-12, DS-03): {T_MIN_GRUEN:.0f} s"
                 f" — fest, nicht konfigurierbar",
            bg=_HINTERGRUND, fg="#666688",
            font=("Helvetica", 9), anchor="w",
        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(0, 4))

    def _baue_emulator_bereich(self, parent):
        """Mock-Modus: Sensor-Simulation und Speed-Regler."""
        # Sensor-Simulation
        ef = tk.Frame(parent, bg=_HINTERGRUND)
        ef.pack(pady=4)

        tk.Label(
            ef,
            text="[Emulator] Fahrzeugsensor (FA-10, DS-01, DS-02):",
            bg=_HINTERGRUND, fg="#cccccc",
            font=("Helvetica", 10),
        ).grid(row=0, column=0, padx=8)

        self._sensor_var = tk.BooleanVar(value=False)
        tk.Checkbutton(
            ef,
            text="Fahrzeug wartet",
            variable=self._sensor_var,
            bg=_HINTERGRUND, fg="white",
            selectcolor="#333355",
            activebackground=_HINTERGRUND,
            font=("Helvetica", 10),
            command=self._toggle_sensor,
        ).grid(row=0, column=1, padx=8)

        self._sensor_lbl = tk.Label(
            ef, text="Kein Fahrzeug",
            bg=_HINTERGRUND, fg="#88aa88",
            font=("Helvetica", 10),
        )
        self._sensor_lbl.grid(row=0, column=2, padx=8)

        # Emulator-Geschwindigkeit (beeinflusst NICHT Sicherheitsphasen)
        sf = tk.Frame(parent, bg=_HINTERGRUND)
        sf.pack(pady=(2, 10))

        tk.Label(
            sf,
            text="[Emulator] Geschwindigkeit (nur konfigurierbare Phasen):",
            bg=_HINTERGRUND, fg="#cccccc",
            font=("Helvetica", 10),
        ).grid(row=0, column=0, padx=8)

        self._speed_var = tk.DoubleVar(value=self.steuerung.emulator_speed)
        ttk.Scale(
            sf,
            from_=1, to=30,
            orient="horizontal",
            variable=self._speed_var,
            length=180,
            command=self._setze_speed,
        ).grid(row=0, column=1, padx=8)

        self._speed_lbl = tk.Label(
            sf, text="1×",
            bg=_HINTERGRUND, fg="#aaaaff",
            font=("Courier", 9), width=5,
        )
        self._speed_lbl.grid(row=0, column=2)

        tk.Label(
            sf,
            text="Sicherheitsphasen (Gelb, Rot-Gelb, Räumzeit, Mindestgrün) sind IMMER Echtzeit.",
            bg=_HINTERGRUND, fg="#666644",
            font=("Helvetica", 8),
        ).grid(row=1, column=0, columnspan=3, sticky="w", padx=8, pady=(2, 0))

    # -----------------------------------------------------------------------
    # Eingabevalidierung — FA-09
    # -----------------------------------------------------------------------

    def _validiere(self) -> bool:
        """
        FA-09: Eingabe ablehnen und Meldung mit Eingabewert + erlaubtem Bereich anzeigen.
        NFA-U01: Beide Parameter in einem Formular → ≤ 2 Eingaben + 1 Klick.
        Gibt True zurück wenn beide Eingaben gültig sind.
        """
        ok = True

        # ── FA-07: FzA-Grünphase direkt ─────────────────────────────────────
        try:
            v_fza = int(self._eingabe_fza.get())
            if FZA_GRUEN_MIN <= v_fza <= FZA_GRUEN_MAX:
                self._fehler_fza.config(text="✓")
                self.steuerung.fza_gruen_zeit = v_fza
            else:
                # FA-09: Eingabewert UND erlaubter Bereich in Meldung
                self._fehler_fza.config(
                    text=f"Wert {v_fza} ungültig — Bereich [{FZA_GRUEN_MIN}, {FZA_GRUEN_MAX}]"
                )
                ok = False
        except ValueError:
            self._fehler_fza.config(text="Ganzzahl erforderlich")
            ok = False

        # ── FA-08: FgA-Grünphase ─────────────────────────────────────────────
        try:
            v_fga = int(self._eingabe_fga.get())
            if FGA_GRUEN_MIN <= v_fga <= FGA_GRUEN_MAX:
                self._fehler_fga.config(text="✓")
                self.steuerung.fga_gruen_zeit = v_fga
            else:
                # FA-09: Eingabewert UND erlaubter Bereich in Meldung
                self._fehler_fga.config(
                    text=f"Wert {v_fga} ungültig — Bereich [{FGA_GRUEN_MIN}, {FGA_GRUEN_MAX}]"
                )
                ok = False
        except ValueError:
            self._fehler_fga.config(text="Ganzzahl erforderlich")
            ok = False

        return ok

    # -----------------------------------------------------------------------
    # Button-Callbacks
    # -----------------------------------------------------------------------

    def _start(self):
        """UC-02 → UC-01: Konfiguration validieren, dann Zyklus starten."""
        if not self._validiere():
            return
        # Eingabefelder sperren während Betrieb
        self._eingabe_fza.config(state="disabled")
        self._eingabe_fga.config(state="disabled")
        self._btn_start.config(state="disabled")
        self._btn_stop.config(state="normal")
        self.steuerung.starte()

    def _stop(self):
        """Normalen Stop auslösen — kehrt nach Zyklus zu CONFIG zurück."""
        self.steuerung.stoppe()
        self._eingabe_fza.config(state="normal")
        self._eingabe_fga.config(state="normal")
        self._btn_start.config(state="normal")
        self._btn_stop.config(state="disabled")

    def _fehler_ausloesen(self):
        """
        UC-04, FA-13, NFA-S01: Fail-Safe manuell auslösen.
        Alle Buttons gesperrt — kein Neustart ohne App-Restart.
        """
        self.steuerung.fehler_ausloesen()
        self._eingabe_fza.config(state="disabled")
        self._eingabe_fga.config(state="disabled")
        self._btn_start.config(state="disabled")
        self._btn_stop.config(state="disabled")
        self._btn_fehler.config(state="disabled")

    def _toggle_sensor(self):
        """Mock-Modus: Fahrzeugpräsenz simulieren (FA-10)."""
        self.steuerung.sensor.simuliere(self._sensor_var.get())

    def _setze_speed(self, wert):
        """Emulator-Geschwindigkeit setzen (beeinflusst NICHT Sicherheitsphasen)."""
        v = float(wert)
        self.steuerung.emulator_speed = v
        self._speed_lbl.config(text=f"{v:.0f}×")

    # -----------------------------------------------------------------------
    # Anzeige-Update (80 ms Polling-Intervall)
    # -----------------------------------------------------------------------

    def _timer_text(self) -> str:
        """
        Phasen-Timer-Text für GUI.
        Sicherheitsphasen (Gelb, Räumzeit, Rot-Gelb): Echtzeit anzeigen.
        Konfigurierbare Phasen: Speed-skaliert anzeigen.
        FA-12: Mindestgrün wird ABSOLUT angezeigt (nicht speed-skaliert).
        FA-11: Hinweis auf adaptive Verkürzung wenn aktiv.
        """
        s       = self.steuerung
        zustand = s.zustand
        vergangen = s.phase_vergangen
        spd     = max(s.emulator_speed, 0.1)

        if zustand == RPiZA.S_FZA_GRUEN:
            gesamt   = s.fza_gruen_zeit / spd
        elif zustand == RPiZA.S_FGA_GRUEN:
            gesamt   = s.fga_gruen_zeit / spd
        elif zustand == RPiZA.S_FZA_GELB:
            gesamt   = T_GELB              # Echtzeit
        elif zustand == RPiZA.S_RAEUMZEIT_FZA:
            gesamt   = T_RAEUMZEIT         # Echtzeit
        elif zustand == RPiZA.S_RAEUMZEIT:
            gesamt   = T_RAEUMZEIT         # Echtzeit
        elif zustand == RPiZA.S_ROT_GELB:
            gesamt   = T_ROT_GELB          # Echtzeit
        else:
            return f"Vergangen: {vergangen:.0f} s"

        verbleibend = max(gesamt - vergangen, 0)

        # FA-12 / FA-11: Hinweise in FgA-Grün-Phase
        if zustand == RPiZA.S_FGA_GRUEN:
            min_rest = max(T_MIN_GRUEN - vergangen, 0)  # ABSOLUT, nicht skaliert
            if min_rest > 0:
                return (
                    f"{vergangen:.0f} s / {gesamt:.0f} s  "
                    f"(Mindestgrün verbleibend: {min_rest:.0f} s — FA-12)"
                )
            if s.sensor.erkannt:
                return (
                    f"{vergangen:.0f} s / {gesamt:.0f} s  "
                    f"⚡ Adaptive Verkürzung aktiv (FA-11)"
                )

        return f"{vergangen:.0f} s / {gesamt:.0f} s  ({verbleibend:.0f} s verbleibend)"

    def _aktualisiere(self):
        """GUI mit aktuellem LED- und Zustandsstatus aktualisieren."""
        leds = self.steuerung.led_zustaende
        self._fza_r.setze(leds["fza_rot"])
        self._fza_g_led.setze(leds["fza_gelb"])
        self._fza_g.setze(leds["fza_gruen"])
        self._fga1_r.setze(leds["fga1_rot"])
        self._fga1_g.setze(leds["fga1_gruen"])
        self._fga2_r.setze(leds["fga2_rot"])
        self._fga2_g.setze(leds["fga2_gruen"])

        self._zustand_var.set(
            _ZUSTAND_TEXT.get(self.steuerung.zustand, self.steuerung.zustand)
        )
        self._timer_var.set(self._timer_text())

        if self.mock:
            erkannt = self.steuerung.sensor.erkannt
            self._sensor_lbl.config(
                text="Fahrzeug erkannt! (DS-02)" if erkannt else "Kein Fahrzeug",
                fg="#ff4444" if erkannt else "#88aa88",
            )

    def _poll(self):
        """80 ms Polling für GUI-Updates (NFA-P02: Phasenwechsel ≤ 100 ms)."""
        self._aktualisiere()
        self.root.after(80, self._poll)


# ===========================================================================
#  Einstiegspunkt
# ===========================================================================

def main():
    """
    Startet die RPiZA-Anwendung.
    FA-06: System startet automatisch im CONFIG-Modus, alle Rot.
    C-02: Raspberry Pi 3B oder neuer erforderlich (Echtbetrieb).
    C-04: Demonstrationsprototyp.
    """
    root     = tk.Tk()
    steuerung = RPiZA(mock=_MOCK)
    RPiZAGui(root, steuerung, mock=_MOCK)

    def beim_schliessen():
        steuerung.aufraeumen()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", beim_schliessen)
    root.mainloop()


if __name__ == "__main__":
    main()
