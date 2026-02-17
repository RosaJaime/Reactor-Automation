"""
Touch-Friendly ODrive Reactor Agitator HMI (PySide6)

Run:
  pip install PySide6
  python odrive_hmi.py

Backends:
  - BACKEND = "real" (default)
  - BACKEND = "sim"  (available for testing)

Real backend (open-loop LOCKIN_SPIN):
  - UI uses RPM; backend converts to turns/s = rpm / 60
  - On EVERY velocity change:
      axis.config.general_lockin.vel = turns/s
      axis.requested_state = AxisState.LOCKIN_SPIN
    (firmware latches vel only on re-entry)
  - connect() applies a safe baseline once:
      IDLE, disable startup calibrations and closed-loop flags (when present),
      conservative current limits, and a general_lockin profile (current/ramp_time/ramp_distance/accel,
      finish_* False if available)
  - stop() transitions to IDLE
  - Backend methods do not sleep; timing is handled by the QTimer-driven run engine

Screens:
  - Home
  - Recipe Builder
  - ODrive Configuration
  - Data Logger

Threading rules:
  - All ODrive reads/writes are executed in the worker thread via queued signals/slots
  - UI remains non-blocking

Auto-reconnect:
  - Worker thread supervises connection and reconnects automatically after disconnects or reboots.
  - Disconnect during a run stops the RunEngine safely (no auto-resume).
  - UI uses status text (no popup storms) for transient disconnects.
"""

from __future__ import annotations

import csv
import json
import math
import sqlite3
import sys
import time
import uuid
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from PySide6.QtCore import (
    QObject,
    Qt,
    QThread,
    QTimer,
    QStandardPaths,
    Signal,
    Slot,
    QMetaObject,
    QDateTime,
)
from PySide6.QtGui import QDoubleValidator, QFont, QIntValidator, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

# Prefer QtCharts if present; otherwise use a simple custom painter plot.
QTCHARTS_AVAILABLE = False
try:
    from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis, QDateTimeAxis  # type: ignore

    QTCHARTS_AVAILABLE = True
except Exception:
    QTCHARTS_AVAILABLE = False


# ----------------------------
# App Configuration
# ----------------------------

BACKEND = "real"  # "sim" or "real"

WINDOW_W = 1280
WINDOW_H = 800

VEL_MIN_RPM = 0.0
VEL_MAX_RPM = 2000.0
VEL_STEP_RPM = 1.0

TIME_STEP_S = 10
TIME_MAX_S = 24 * 60 * 60  # 24h cap

ENGINE_TICK_MS = 200

LOG_POLL_MS = 1000  # 1 Hz
DB_HEARTBEAT_S_DEFAULT = 60

SUPERVISOR_TICK_MS = 500
RECONNECT_PERIOD_S = 0.75
FIND_ANY_TIMEOUT_S = 0.5

TAG_VBUS_V = "vbus_voltage"
TAG_IBUS_A = "ibus"
TAG_PWR_W = "p_w"       # computed vbus*ibus
TAG_CMD_RPM = "cmd_rpm"  # last commanded
TAG_VEL_RPM = "vel_rpm"  # actual estimate

DEFAULT_ENABLED_TAGS = [TAG_PWR_W, TAG_CMD_RPM]

TAG_DEADBAND: Dict[str, float] = {
    TAG_VBUS_V: 0.1,
    TAG_IBUS_A: 0.05,
    TAG_PWR_W: 2.0,
    TAG_CMD_RPM: 0.5,
    TAG_VEL_RPM: 2.0,
}
TAG_HEARTBEAT_S: Dict[str, int] = {t: DB_HEARTBEAT_S_DEFAULT for t in TAG_DEADBAND}

TAG_LABELS: Dict[str, str] = {
    TAG_PWR_W: "Electrical input power (W)",
    TAG_CMD_RPM: "Commanded RPM",
    TAG_VEL_RPM: "Actual velocity RPM",
    TAG_VBUS_V: "Bus voltage (V)",
    TAG_IBUS_A: "Bus current (A)",
}
TAG_UNITS: Dict[str, str] = {
    TAG_PWR_W: "W",
    TAG_CMD_RPM: "rpm",
    TAG_VEL_RPM: "rpm",
    TAG_VBUS_V: "V",
    TAG_IBUS_A: "A",
}
PLOT_GROUP_RPM = "rpm"
PLOT_GROUP_OTHER = "other"
TAG_PLOT_GROUP: Dict[str, str] = {
    TAG_CMD_RPM: PLOT_GROUP_RPM,
    TAG_VEL_RPM: PLOT_GROUP_RPM,
    TAG_PWR_W: PLOT_GROUP_OTHER,
    TAG_VBUS_V: PLOT_GROUP_OTHER,
    TAG_IBUS_A: PLOT_GROUP_OTHER,
}


# ----------------------------
# Utilities
# ----------------------------

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def epoch_s() -> int:
    return int(time.time())


def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return float(default)


def format_mmss(seconds: int) -> str:
    seconds = max(0, int(seconds))
    m, s = divmod(seconds, 60)
    return f"{m:02d}:{s:02d}"


def parse_mmss(text: str) -> Optional[int]:
    t = text.strip()
    if not t:
        return None
    if ":" not in t:
        return int(t) if t.isdigit() else None
    parts = t.split(":")
    if len(parts) != 2:
        return None
    mm, ss = parts[0].strip(), parts[1].strip()
    if not (mm.isdigit() and ss.isdigit()):
        return None
    s = int(ss)
    if s < 0 or s > 59:
        return None
    m = int(mm)
    if m < 0:
        return None
    return m * 60 + s


def app_config_dir() -> Path:
    base = QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation) or str(Path.home() / ".config")
    d = Path(base) / "PolyJouleOdriveHMI"
    d.mkdir(parents=True, exist_ok=True)
    return d


def downloads_dir() -> Path:
    base = QStandardPaths.writableLocation(QStandardPaths.DownloadLocation) or str(Path.home() / "Downloads")
    d = Path(base)
    if not d.exists():
        d = Path.home() / "Output"
    d.mkdir(parents=True, exist_ok=True)
    return d


# ----------------------------
# Recipe Model + Storage
# ----------------------------

@dataclass
class Step:
    velocity_rpm: float
    duration_s: int


@dataclass
class Recipe:
    id: str
    name: str
    steps: List[Step]


def recipe_total_seconds(r: Recipe) -> int:
    return sum(max(0, int(s.duration_s)) for s in r.steps)


def recipe_summary(r: Recipe) -> str:
    n = len(r.steps)
    return f"{n} step{'s' if n != 1 else ''} • {format_mmss(recipe_total_seconds(r))} total"


class RecipeStore:
    def __init__(self) -> None:
        self._path = app_config_dir() / "recipes.json"
        self._recipes: List[Recipe] = []

    def recipes(self) -> List[Recipe]:
        return list(self._recipes)

    def load(self) -> None:
        if not self._path.exists():
            self._recipes = self._seed()
            self.save()
            return
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            out: List[Recipe] = []
            for rj in data.get("recipes", []):
                steps = [Step(float(s["velocity_rpm"]), int(s["duration_s"])) for s in rj.get("steps", [])]
                out.append(Recipe(id=str(rj["id"]), name=str(rj["name"]), steps=steps))
            self._recipes = out
        except Exception:
            try:
                self._path.replace(self._path.with_suffix(".corrupt.json"))
            except Exception:
                pass
            self._recipes = self._seed()
            self.save()

    def save(self) -> None:
        obj = {"recipes": [{"id": r.id, "name": r.name, "steps": [asdict(s) for s in r.steps]} for r in self._recipes]}
        self._path.write_text(json.dumps(obj, indent=2), encoding="utf-8")

    def upsert(self, recipe: Recipe) -> None:
        for i, r in enumerate(self._recipes):
            if r.id == recipe.id:
                self._recipes[i] = recipe
                self.save()
                return
        self._recipes.append(recipe)
        self.save()

    @staticmethod
    def _seed() -> List[Recipe]:
        return [
            Recipe(
                id=str(uuid.uuid4()),
                name="Gentle Mix",
                steps=[Step(120.0, 60), Step(200.0, 120), Step(120.0, 60)],
            )
        ]


# ----------------------------
# ODrive Configuration Schema
# ----------------------------

@dataclass(frozen=True)
class SettingSpec:
    key: str
    label: str
    kind: str  # "bool" or "float"
    default: Any
    min_value: Optional[float] = None
    max_value: Optional[float] = None


# Defaults chosen for an agitator that can see significant load:
# - Raise current limits above the prior ultra-conservative values, but keep them moderate.
# - Keep watchdog disabled by default (app does not feed watchdog).
# - Encoder defaults assume a typical incremental encoder is present (8192 CPR) but can be edited.
IMPORTANT_SETTINGS: List[SettingSpec] = [
    # --- Startup behavior (keep off for open-loop / avoid surprise calibrations) ---
    SettingSpec("axis0.config.startup_motor_calibration", "Startup: motor calibration", "bool", False),
    SettingSpec("axis0.config.startup_encoder_offset_calibration", "Startup: encoder offset calibration", "bool", False),
    SettingSpec("axis0.config.startup_encoder_index_search", "Startup: encoder index search", "bool", False),
    SettingSpec("axis0.config.startup_closed_loop_control", "Startup: closed loop control", "bool", False),

    # --- Motor torque capability (limits) ---
    SettingSpec("axis0.config.motor.current_soft_max", "Motor: current soft max (A)", "float", 20.0, 0.0, 60.0),
    SettingSpec("axis0.config.motor.current_hard_max", "Motor: current hard max (A)", "float", 30.0, 0.0, 100.0),

    # --- Open-loop lock-in profile (open-loop torque & ramping) ---
    SettingSpec("axis0.config.general_lockin.current", "Lock-in: current (A)", "float", 6.0, 0.0, 60.0),
    SettingSpec("axis0.config.general_lockin.ramp_time", "Lock-in: ramp time (s)", "float", 1.0, 0.0, 30.0),
    SettingSpec("axis0.config.general_lockin.ramp_distance", "Lock-in: ramp distance (turns)", "float", 1.0, 0.0, 100.0),
    SettingSpec("axis0.config.general_lockin.accel", "Lock-in: accel (turn/s²)", "float", 20.0, 0.0, 2000.0),
    SettingSpec("axis0.config.general_lockin.finish_on_vel", "Lock-in: finish on vel", "bool", False),
    SettingSpec("axis0.config.general_lockin.finish_on_distance", "Lock-in: finish on distance", "bool", False),

    # --- DC bus / supply-side limiting & protection (prevents “weak under load” from supply caps) ---
    SettingSpec("config.dc_max_positive_current", "DC bus: max positive current (A)", "float", 25.0, 0.0, 120.0),
    SettingSpec("config.dc_max_negative_current", "DC bus: max regen current (A, negative)", "float", -2.0, -120.0, 0.0),
    SettingSpec("config.dc_bus_undervoltage_trip_level", "DC bus: undervoltage trip (V)", "float", 10.0, 0.0, 60.0),
    SettingSpec("config.dc_bus_overvoltage_trip_level", "DC bus: overvoltage trip (V)", "float", 30.0, 0.0, 80.0),

    # --- Brake resistor / regen clamp support (optional but important for spinning loads) ---
    SettingSpec("config.enable_brake_resistor", "Brake resistor: enabled", "bool", False),
    SettingSpec("config.brake_resistance", "Brake resistor: resistance (ohm)", "float", 2.0, 0.1, 20.0),

    # --- Motor model (helps torque reporting and sanity; do not change unless you know your motor) ---
    SettingSpec("axis0.config.motor.pole_pairs", "Motor: pole pairs", "int", 7, 1, 50),
    SettingSpec("axis0.config.motor.torque_constant", "Motor: torque constant (Nm/A)", "float", 0.0306, 0.0001, 1.0),

    # --- Current loop tuning (torque response) ---
    SettingSpec("axis0.config.motor.current_control_bandwidth", "Motor: current control bandwidth", "float", 1000.0, 10.0, 5000.0),

    # --- Encoder essentials (so vel_estimate / Actual RPM can be meaningful) ---
    SettingSpec("inc_encoder0.config.enabled", "Encoder: incremental enabled", "bool", True),
    SettingSpec("inc_encoder0.config.cpr", "Encoder: incremental CPR", "int", 8192, 1, 200000),

    # --- Safety watchdog (leave disabled unless you also feed it) ---
    SettingSpec("axis0.config.enable_watchdog", "Safety: enable watchdog", "bool", False),
    SettingSpec("axis0.config.watchdog_timeout", "Safety: watchdog timeout (s)", "float", 1.0, 0.1, 10.0),
]
DEFAULT_CONFIG_MAP: Dict[str, Any] = {s.key: s.default for s in IMPORTANT_SETTINGS}


# ----------------------------
# Backends
# ----------------------------

class OdriveInterface:
    def connect(self) -> bool: raise NotImplementedError
    def is_connected(self) -> bool: raise NotImplementedError
    def set_velocity_rpm(self, rpm: float) -> None: raise NotImplementedError
    def stop(self) -> None: raise NotImplementedError
    def get_state(self) -> str: raise NotImplementedError
    def read_config(self, keys: List[str]) -> Tuple[Dict[str, Any], List[str]]: raise NotImplementedError
    def apply_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]: raise NotImplementedError
    def get_tags(self, tags: Sequence[str]) -> Dict[str, float]: raise NotImplementedError
    def get_json(self) -> str: raise NotImplementedError


class SimulatedOdrive(OdriveInterface):
    def __init__(self) -> None:
        self._connected = False
        self._state = "Disconnected"
        self._target_rpm = 0.0
        self._actual_rpm = 0.0
        self._last_update = time.monotonic()
        self._config: Dict[str, Any] = dict(DEFAULT_CONFIG_MAP)
        self._vbus = 24.0

    def connect(self) -> bool:
        self._connected = True
        self._state = "Idle"
        self._target_rpm = 0.0
        self._actual_rpm = 0.0
        self._last_update = time.monotonic()
        return True

    def is_connected(self) -> bool:
        return self._connected

    def set_velocity_rpm(self, rpm: float) -> None:
        self._target_rpm = float(rpm)
        self._state = "Running" if abs(self._target_rpm) > 1e-6 else "Idle"

    def stop(self) -> None:
        self._target_rpm = 0.0
        self._state = "Idle"

    def get_state(self) -> str:
        return self._state

    def read_config(self, keys: List[str]) -> Tuple[Dict[str, Any], List[str]]:
        values, missing = {}, []
        for k in keys:
            if k in self._config:
                values[k] = self._config[k]
            else:
                missing.append(k)
        return values, missing

    def apply_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]:
        missing: List[str] = []
        for k, v in values.items():
            if k in self._config:
                self._config[k] = v
            else:
                missing.append(k)
        return True, "Simulation config saved", missing

    def _update_actual(self) -> None:
        now = time.monotonic()
        dt = max(0.0, now - self._last_update)
        self._last_update = now
        tau = 0.5
        alpha = 1.0 - math.exp(-dt / max(1e-6, tau))
        self._actual_rpm += (self._target_rpm - self._actual_rpm) * alpha
        if abs(self._actual_rpm) < 0.2:
            self._actual_rpm = 0.0

    def get_tags(self, tags: Sequence[str]) -> Dict[str, float]:
        self._update_actual()
        rpm_mag = abs(self._actual_rpm)
        ibus = 0.2 + (rpm_mag / max(1.0, VEL_MAX_RPM)) * 2.0
        out: Dict[str, float] = {}
        for t in tags:
            if t == TAG_VBUS_V:
                out[t] = float(self._vbus)
            elif t == TAG_IBUS_A:
                out[t] = float(ibus)
            elif t == TAG_VEL_RPM:
                out[t] = float(self._actual_rpm)
        return out

    def get_json(self) -> str:
        return json.dumps({"simulated": True, "config": self._config}, indent=2)


class RealOdrive(OdriveInterface):
    def __init__(self) -> None:
        self._connected = False
        self._state = "Disconnected"
        self._odrv = None
        self._axis = None
        self._AxisState = None

    def _mark_disconnected(self, reason: str = "Disconnected") -> None:
        self._connected = False
        self._state = reason
        self._odrv = None
        self._axis = None

    def connect(self) -> bool:
        try:
            import odrive
            from odrive.enums import AxisState

            self._AxisState = AxisState
            self._state = "Connecting..."
            self._odrv = odrive.find_any(timeout=FIND_ANY_TIMEOUT_S)
            if self._odrv is None:
                self._mark_disconnected("Disconnected (reconnecting)")
                return False

            self._axis = self._odrv.axis0
            self._axis.requested_state = AxisState.IDLE

            # Disable startup calibration/closed-loop flags when present.
            for attr in (
                "startup_motor_calibration",
                "startup_encoder_offset_calibration",
                "startup_encoder_index_search",
                "startup_closed_loop_control",
            ):
                try:
                    setattr(self._axis.config, attr, False)
                except Exception:
                    pass

            # Conservative current limits when present.
            try:
                self._axis.config.motor.current_soft_max = 5.0
            except Exception:
                pass
            try:
                self._axis.config.motor.current_hard_max = 8.0
            except Exception:
                pass

            # general_lockin baseline.
            gl = self._axis.config.general_lockin
            for name, val in (
                ("current", 2.0),
                ("ramp_time", 1.0),
                ("ramp_distance", 1.0),
                ("accel", 10.0),
            ):
                try:
                    setattr(gl, name, val)
                except Exception:
                    pass
            for name, val in (("finish_on_vel", False), ("finish_on_distance", False), ("vel", 0.0)):
                try:
                    setattr(gl, name, val)
                except Exception:
                    pass

            self._connected = True
            self._state = "Idle (LOCKIN ready)"
            return True
        except Exception:
            self._mark_disconnected("Disconnected (reconnecting)")
            return False

    def is_connected(self) -> bool:
        return self._connected

    def set_velocity_rpm(self, rpm: float) -> None:
        if not self._connected or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")
        rpm = float(rpm)
        if abs(rpm) < 1e-6:
            self.stop()
            return
        turns_per_s = rpm / 60.0
        gl = self._axis.config.general_lockin
        gl.vel = float(turns_per_s)
        # Firmware latches vel on re-entry; always request LOCKIN_SPIN.
        self._axis.requested_state = self._AxisState.LOCKIN_SPIN
        self._state = f"LOCKIN_SPIN ({rpm:.0f} rpm)"

    def stop(self) -> None:
        if not self._connected or self._axis is None or self._AxisState is None:
            self._state = "Disconnected"
            return
        try:
            try:
                self._axis.config.general_lockin.vel = 0.0
            except Exception:
                pass
            self._axis.requested_state = self._AxisState.IDLE
        finally:
            self._state = "Idle"

    def get_state(self) -> str:
        return self._state

    @staticmethod
    def _get_attr_path(root: Any, key: str) -> Any:
        obj = root
        for part in key.split("."):
            obj = getattr(obj, part)
        return obj

    @staticmethod
    def _set_attr_path(root: Any, key: str, value: Any) -> None:
        parts = key.split(".")
        parent = root
        for part in parts[:-1]:
            parent = getattr(parent, part)
        setattr(parent, parts[-1], value)

    def read_config(self, keys: List[str]) -> Tuple[Dict[str, Any], List[str]]:
        if not self._connected or self._odrv is None:
            raise RuntimeError("ODrive not connected")
        values, missing = {}, []
        for k in keys:
            try:
                values[k] = self._get_attr_path(self._odrv, k)
            except Exception:
                missing.append(k)
        return values, missing

    def apply_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]:
        if not self._connected or self._odrv is None or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")
        missing: List[str] = []
        try:
            self._axis.requested_state = self._AxisState.IDLE
        except Exception:
            pass
        for k, v in values.items():
            try:
                self._set_attr_path(self._odrv, k, v)
            except Exception:
                missing.append(k)
        msg = "Config saved (device rebooting)"
        try:
            self._state = "Saving config (rebooting)..."
            self._odrv.save_configuration()
        except Exception:
            # Expected during reboot.
            pass
        self._mark_disconnected("Rebooting (reconnecting)")
        if missing:
            msg = f"Config saved (device rebooting); {len(missing)} unsupported field(s) skipped"
        return True, msg, missing

    def get_tags(self, tags: Sequence[str]) -> Dict[str, float]:
        if not self._connected or self._odrv is None or self._axis is None:
            raise RuntimeError("ODrive not connected")
        out: Dict[str, float] = {}
        for t in tags:
            try:
                if t == TAG_VBUS_V:
                    out[t] = float(getattr(self._odrv, "vbus_voltage"))
                elif t == TAG_IBUS_A:
                    out[t] = float(getattr(self._odrv, "ibus"))
                elif t == TAG_VEL_RPM:
                    turns_per_s = float(getattr(self._axis, "vel_estimate"))
                    out[t] = turns_per_s * 60.0
            except Exception:
                pass
        return out

    def get_json(self) -> str:
        if not self._connected or self._odrv is None:
            raise RuntimeError("ODrive not connected")
        return str(self._odrv.get_json())


# ----------------------------
# Worker Thread (ALL ODrive I/O)
# ----------------------------

class OdriveWorker(QObject):
    connected_changed = Signal(bool, str)         # connected, backend state
    state_changed = Signal(str)                   # backend state
    rpm_commanded = Signal(float)                 # last commanded rpm
    error = Signal(str)

    config_read = Signal(object, object)          # dict, missing list
    config_applied = Signal(bool, str, object)    # ok, message, missing list
    export_json_done = Signal(bool, str)          # ok, path/message
    tags_sample = Signal(int, object)             # ts, dict

    def __init__(self, backend: OdriveInterface) -> None:
        super().__init__()
        self._backend = backend
        self._poll_timer: Optional[QTimer] = None
        self._polled_tags: List[str] = [TAG_VBUS_V, TAG_IBUS_A, TAG_VEL_RPM]
        self._supervisor_timer: Optional[QTimer] = None

        self._connecting = False
        self._last_connect_attempt_mono = 0.0
        self._connected_cache = False
        self._state_cache = "Disconnected"

        self._last_poll_error_mono = 0.0
        self._poll_error_cooldown_s = 5.0

    @Slot()
    def on_thread_started(self) -> None:
        if self._supervisor_timer is None:
            self._supervisor_timer = QTimer(self)
            self._supervisor_timer.setInterval(SUPERVISOR_TICK_MS)
            self._supervisor_timer.timeout.connect(self._supervise_connection)
            self._supervisor_timer.start()

    def _is_disconnect_error(self, e: Exception) -> bool:
        s = str(e).lower()
        return any(
            n in s
            for n in (
                "not connected",
                "disconnected",
                "object lost",
                "usb",
                "broken pipe",
                "timed out",
                "timeout",
                "channel",
                "ioerror",
                "oserror",
                "device",
                "fibre",
                "endpoint",
                "no such device",
            )
        )

    def _emit_state(self, state: str) -> None:
        if state != self._state_cache:
            self._state_cache = state
            self.state_changed.emit(state)

    def _set_connected(self, connected: bool, state: str) -> None:
        if connected != self._connected_cache:
            self._connected_cache = connected
            self.connected_changed.emit(connected, state)
        self._emit_state(state)

    def _mark_disconnected(self, reason: str) -> None:
        try:
            self.stop_tag_polling()
        except Exception:
            pass
        try:
            if hasattr(self._backend, "_mark_disconnected"):
                getattr(self._backend, "_mark_disconnected")(reason)
        except Exception:
            pass
        self._set_connected(False, reason)

    @Slot()
    def _supervise_connection(self) -> None:
        try:
            if self._backend.is_connected():
                self._set_connected(True, self._backend.get_state())
                return
        except Exception:
            self._mark_disconnected("Disconnected (reconnecting)")
            return

        now_m = time.monotonic()
        if self._connecting or (now_m - self._last_connect_attempt_mono) < RECONNECT_PERIOD_S:
            return

        self._connecting = True
        self._last_connect_attempt_mono = now_m
        try:
            ok = self._backend.connect()
            st = self._backend.get_state() or ("Connected" if ok else "Disconnected (reconnecting)")
            self._set_connected(bool(ok), st)
        except Exception:
            self._mark_disconnected("Disconnected (reconnecting)")
        finally:
            self._connecting = False

    @Slot()
    def connect_backend(self) -> None:
        if self._connecting:
            return
        self._connecting = True
        self._last_connect_attempt_mono = time.monotonic()
        try:
            ok = self._backend.connect()
            self._set_connected(bool(ok), self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
            else:
                self.error.emit(f"Connect failed: {e}")
                self._set_connected(False, "Error")
        finally:
            self._connecting = False

    @Slot(float)
    def set_velocity_rpm(self, rpm: float) -> None:
        if not self._backend.is_connected():
            return
        try:
            self._backend.set_velocity_rpm(float(rpm))
            self.rpm_commanded.emit(float(rpm))
            self._emit_state(self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
            else:
                self.error.emit(f"Motor command failed: {e}")

    @Slot()
    def stop(self) -> None:
        if not self._backend.is_connected():
            return
        try:
            self._backend.stop()
            self.rpm_commanded.emit(0.0)
            self._emit_state(self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
            else:
                self.error.emit(f"Stop failed: {e}")

    @Slot(object)
    def read_config(self, keys: List[str]) -> None:
        if not self._backend.is_connected():
            self.config_read.emit({}, list(keys))
            return
        try:
            values, missing = self._backend.read_config(keys)
            self.config_read.emit(values, missing)
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
            else:
                self.error.emit(f"Read config failed: {e}")
            self.config_read.emit({}, list(keys))

    @Slot(object)
    def apply_config(self, values: Dict[str, Any]) -> None:
        if not self._backend.is_connected():
            self.config_applied.emit(False, "ODrive not connected (reconnecting)", list(values.keys()))
            return
        try:
            ok, msg, missing = self._backend.apply_config(values)
            self.config_applied.emit(bool(ok), str(msg), missing)
            if not self._backend.is_connected():
                self._set_connected(False, self._backend.get_state())
            else:
                self._emit_state(self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.config_applied.emit(False, "ODrive disconnected during save (reconnecting)", list(values.keys()))
            else:
                self.error.emit(f"Apply config failed: {e}")
                self.config_applied.emit(False, str(e), list(values.keys()))

    @Slot()
    def export_json(self) -> None:
        if not self._backend.is_connected():
            self.export_json_done.emit(False, "ODrive not connected (reconnecting)")
            return
        try:
            txt = self._backend.get_json()
            out = downloads_dir() / f"odrive_get_json_{time.strftime('%Y%m%d_%H%M%S')}.json"
            out.write_text(txt, encoding="utf-8")
            self.export_json_done.emit(True, str(out))
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.export_json_done.emit(False, "ODrive disconnected during export (reconnecting)")
            else:
                self.export_json_done.emit(False, f"Export failed: {e}")

    @Slot(object)
    def set_polled_tags(self, tags: List[str]) -> None:
        self._polled_tags = list(tags)

    @Slot()
    def start_tag_polling(self) -> None:
        if self._poll_timer is None:
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(LOG_POLL_MS)
            self._poll_timer.timeout.connect(self._poll_tags)
        if not self._poll_timer.isActive():
            self._poll_timer.start()

    @Slot()
    def stop_tag_polling(self) -> None:
        if self._poll_timer and self._poll_timer.isActive():
            self._poll_timer.stop()

    @Slot()
    def _poll_tags(self) -> None:
        if not self._backend.is_connected():
            return
        try:
            values = self._backend.get_tags(self._polled_tags)
            self.tags_sample.emit(epoch_s(), values)
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                return
            now_m = time.monotonic()
            if (now_m - self._last_poll_error_mono) >= self._poll_error_cooldown_s:
                self._last_poll_error_mono = now_m
                self.error.emit(f"Tag polling failed: {e}")


# ----------------------------
# Run Engine (QTimer state machine; UI thread only)
# ----------------------------

class RunEngine(QObject):
    status_changed = Signal(str)           # "Idle", "Running", "Stopped"
    run_mode_changed = Signal(str)         # "idle" | "single" | "recipe"
    active_rpm_changed = Signal(float)
    remaining_changed = Signal(int)
    step_info_changed = Signal(str)

    request_set_rpm = Signal(float)
    request_stop = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._timer = QTimer(self)
        self._timer.setInterval(ENGINE_TICK_MS)
        self._timer.timeout.connect(self._tick)

        self._mode = "idle"
        self._status = "Idle"
        self._active_rpm = 0.0

        self._single_end_t = 0.0
        self._recipe: Optional[Recipe] = None
        self._step_idx = -1
        self._step_end_t = 0.0

    def is_running(self) -> bool:
        return self._mode in ("single", "recipe")

    def run_mode(self) -> str:
        return self._mode

    def _set_status(self, status: str) -> None:
        if status != self._status:
            self._status = status
            self.status_changed.emit(status)

    def _set_mode(self, mode: str) -> None:
        if mode != self._mode:
            self._mode = mode
            self.run_mode_changed.emit(mode)

    @Slot(float, int)
    def start_single(self, rpm: float, duration_s: int) -> None:
        self.stop(user_initiated=False)
        self._set_mode("single")
        self._active_rpm = float(rpm)
        now = time.monotonic()
        self._single_end_t = now + max(0, int(duration_s))

        self.step_info_changed.emit("")
        self.request_set_rpm.emit(self._active_rpm)
        self.active_rpm_changed.emit(self._active_rpm)
        self._set_status("Running")

        self._timer.start()
        self._tick()

    @Slot(object)
    def start_recipe(self, recipe: Recipe) -> None:
        self.stop(user_initiated=False)
        self._set_mode("recipe")
        self._recipe = recipe
        self._step_idx = 0
        self._start_current_step(time.monotonic())
        self._set_status("Running")

        self._timer.start()
        self._tick()

    def _start_current_step(self, now: float) -> None:
        assert self._recipe is not None
        step = self._recipe.steps[self._step_idx]
        self._active_rpm = float(step.velocity_rpm)
        self.request_set_rpm.emit(self._active_rpm)
        self.active_rpm_changed.emit(self._active_rpm)

        self._step_end_t = now + max(0, int(step.duration_s))
        info = f"Step {self._step_idx + 1}/{len(self._recipe.steps)} • {step.velocity_rpm:g} rpm • {format_mmss(step.duration_s)}"
        self.step_info_changed.emit(info)

    @Slot(int)
    def adjust_single_remaining(self, delta_s: int) -> None:
        if self._mode != "single":
            return
        now = time.monotonic()
        remaining = int(round(self._single_end_t - now))
        new_remaining = int(clamp(remaining + int(delta_s), 0, TIME_MAX_S))
        self._single_end_t = now + new_remaining
        self._tick()

    @Slot(bool)
    def stop(self, user_initiated: bool = True) -> None:
        if self._timer.isActive():
            self._timer.stop()
        if self._mode != "idle":
            self.request_stop.emit()

        self._set_mode("idle")
        self._recipe = None
        self._step_idx = -1
        self._active_rpm = 0.0

        self.active_rpm_changed.emit(0.0)
        self.remaining_changed.emit(0)
        self.step_info_changed.emit("")
        self._set_status("Stopped" if user_initiated else "Idle")

    @Slot()
    def _tick(self) -> None:
        now = time.monotonic()

        if self._mode == "single":
            remaining = int(round(self._single_end_t - now))
            if remaining <= 0:
                self.request_stop.emit()
                self._set_mode("idle")
                self._active_rpm = 0.0
                self.active_rpm_changed.emit(0.0)
                self.remaining_changed.emit(0)
                self._set_status("Idle")
                self._timer.stop()
                return
            self.remaining_changed.emit(remaining)
            return

        if self._mode == "recipe":
            if not self._recipe:
                self.stop(user_initiated=False)
                return
            step_remaining = int(round(self._step_end_t - now))
            if step_remaining <= 0:
                self._step_idx += 1
                if self._step_idx >= len(self._recipe.steps):
                    self.request_stop.emit()
                    self._set_mode("idle")
                    self._active_rpm = 0.0
                    self.active_rpm_changed.emit(0.0)
                    self.remaining_changed.emit(0)
                    self.step_info_changed.emit("")
                    self._set_status("Idle")
                    self._timer.stop()
                    return
                self._start_current_step(now)
                step_remaining = int(round(self._step_end_t - now))
            self.remaining_changed.emit(max(0, step_remaining))
            return

        self.remaining_changed.emit(0)


# ----------------------------
# Historian (SQLite + deadband/heartbeat compression)
# ----------------------------

class HistoryDB:
    def __init__(self, db_path: Path) -> None:
        self._conn = sqlite3.connect(str(db_path))
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute(
            "CREATE TABLE IF NOT EXISTS samples (ts INTEGER NOT NULL, tag TEXT NOT NULL, value REAL NOT NULL)"
        )
        self._conn.execute("CREATE INDEX IF NOT EXISTS idx_samples_tag_ts ON samples(tag, ts)")
        self._conn.commit()
        self._last: Dict[str, Tuple[int, float]] = {}

    def close(self) -> None:
        self._conn.close()

    def insert_if_needed(self, ts: int, tag: str, value: float, deadband: float, heartbeat_s: int) -> bool:
        prev = self._last.get(tag)
        v = float(value)
        if prev is None:
            self._conn.execute("INSERT INTO samples(ts, tag, value) VALUES(?, ?, ?)", (ts, tag, v))
            self._conn.commit()
            self._last[tag] = (ts, v)
            return True
        prev_ts, prev_val = prev
        if abs(v - prev_val) >= float(deadband) or (ts - prev_ts) >= int(heartbeat_s):
            self._conn.execute("INSERT INTO samples(ts, tag, value) VALUES(?, ?, ?)", (ts, tag, v))
            self._conn.commit()
            self._last[tag] = (ts, v)
            return True
        return False

    def insert_many_if_needed(self, ts: int, samples: Sequence[Tuple[str, float, float, int]]) -> List[str]:
        inserted: List[str] = []
        rows: List[Tuple[int, str, float]] = []
        for tag, value, deadband, heartbeat_s in samples:
            v = float(value)
            prev = self._last.get(tag)
            if prev is None:
                rows.append((ts, tag, v))
                inserted.append(tag)
                self._last[tag] = (ts, v)
                continue
            prev_ts, prev_val = prev
            if abs(v - prev_val) >= float(deadband) or (ts - prev_ts) >= int(heartbeat_s):
                rows.append((ts, tag, v))
                inserted.append(tag)
                self._last[tag] = (ts, v)
        if rows:
            self._conn.executemany("INSERT INTO samples(ts, tag, value) VALUES(?, ?, ?)", rows)
            self._conn.commit()
        return inserted

    def query_window(self, now_ts: int, tag: str, minutes: int) -> List[Tuple[int, float]]:
        start_ts = now_ts - max(1, int(minutes)) * 60
        cur = self._conn.execute(
            "SELECT ts, value FROM samples WHERE tag=? AND ts>=? AND ts<=? ORDER BY ts ASC",
            (tag, start_ts, now_ts),
        )
        return [(int(r[0]), float(r[1])) for r in cur.fetchall()]

    def query_window_multi(self, now_ts: int, tags: Sequence[str], minutes: int) -> Dict[str, List[Tuple[int, float]]]:
        return {t: self.query_window(now_ts, t, minutes) for t in tags}

    def export_window_csv_multi(self, out_path: Path, now_ts: int, tags: Sequence[str], minutes: int) -> int:
        start_ts = now_ts - max(1, int(minutes)) * 60
        tags = list(tags)
        if not tags:
            out_path.write_text("ts_epoch_s,ts_iso,tag,value,unit\n", encoding="utf-8")
            return 0

        placeholders = ",".join(["?"] * len(tags))
        q = (
            f"SELECT ts, tag, value FROM samples WHERE tag IN ({placeholders}) AND ts>=? AND ts<=?"
            f" ORDER BY ts ASC, tag ASC"
        )
        cur = self._conn.execute(q, (*tags, start_ts, now_ts))
        rows = [(int(r[0]), str(r[1]), float(r[2])) for r in cur.fetchall()]

        with out_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["ts_epoch_s", "ts_iso", "tag", "value", "unit"])
            for ts, tag, val in rows:
                ts_iso = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts))
                w.writerow([ts, ts_iso, tag, f"{val:.6g}", TAG_UNITS.get(tag, "")])

        return len(rows)


# ----------------------------
# Touch-friendly Widgets
# ----------------------------

class TouchButton(QPushButton):
    def __init__(self, text: str, min_h: int = 70, min_w: int = 140) -> None:
        super().__init__(text)
        self.setMinimumHeight(min_h)
        self.setMinimumWidth(min_w)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


class RpmControl(QWidget):
    value_changed = Signal(float)

    def __init__(self, title: str, vmin: float, vmax: float, step: float, compact: bool = False) -> None:
        super().__init__()
        self._vmin, self._vmax, self._step = float(vmin), float(vmax), float(step)

        min_h = 70 if not compact else 58
        btn_w = 110 if not compact else 90
        spacing = 12 if not compact else 8

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8 if not compact else 6)

        lbl = QLabel(title)
        lbl.setObjectName("SectionTitle" if not compact else "SectionTitleCompact")
        root.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(spacing)

        self.btn_minus = TouchButton("−", min_h=min_h, min_w=btn_w)
        self.btn_plus = TouchButton("+", min_h=min_h, min_w=btn_w)

        self.edit = QLineEdit()
        self.edit.setMinimumHeight(min_h)
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText("120")
        self.edit.setValidator(QDoubleValidator(self._vmin, self._vmax, 2, self.edit))

        units = QLabel("rpm")
        units.setMinimumWidth(70 if not compact else 60)
        units.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        units.setObjectName("UnitsLabel")

        row.addWidget(self.btn_minus)
        row.addWidget(self.edit, 1)
        row.addWidget(units)
        row.addWidget(self.btn_plus)
        root.addLayout(row)

        self.btn_minus.clicked.connect(lambda: self.set_value(self.value() - self._step))
        self.btn_plus.clicked.connect(lambda: self.set_value(self.value() + self._step))
        self.edit.editingFinished.connect(self._emit_if_valid)
        self._emit_if_valid()

    def value(self) -> float:
        try:
            return float(self.edit.text().strip() or self._vmin)
        except Exception:
            return float(self._vmin)

    def set_value(self, v: float) -> None:
        v = clamp(float(v), self._vmin, self._vmax)
        self.edit.setText(f"{v:g}")
        self.value_changed.emit(v)

    def set_enabled(self, enabled: bool) -> None:
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)
        self.edit.setEnabled(enabled)

    @Slot()
    def _emit_if_valid(self) -> None:
        self.set_value(self.value())


class TimeControl(QWidget):
    value_changed = Signal(int)

    def __init__(self, title: str, step_s: int, compact: bool = False) -> None:
        super().__init__()
        self._step_s = int(step_s)
        self._adjust_cb: Optional[Callable[[int], int]] = None

        min_h = 70 if not compact else 58
        btn_w = 110 if not compact else 90
        spacing = 12 if not compact else 8

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8 if not compact else 6)

        lbl = QLabel(title)
        lbl.setObjectName("SectionTitle" if not compact else "SectionTitleCompact")
        root.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(spacing)

        self.btn_minus = TouchButton("−", min_h=min_h, min_w=btn_w)
        self.btn_plus = TouchButton("+", min_h=min_h, min_w=btn_w)

        self.edit = QLineEdit()
        self.edit.setMinimumHeight(min_h)
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText("00:00")
        self.edit.setPlaceholderText("mm:ss")

        hint = QLabel("mm:ss")
        hint.setMinimumWidth(70 if not compact else 60)
        hint.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        hint.setObjectName("UnitsLabel")

        row.addWidget(self.btn_minus)
        row.addWidget(self.edit, 1)
        row.addWidget(hint)
        row.addWidget(self.btn_plus)
        root.addLayout(row)

        self.btn_minus.clicked.connect(lambda: self._adjust(-self._step_s))
        self.btn_plus.clicked.connect(lambda: self._adjust(+self._step_s))
        self.edit.editingFinished.connect(self._emit_if_valid)
        self._emit_if_valid()

    def set_adjust_callback(self, cb: Optional[Callable[[int], int]]) -> None:
        self._adjust_cb = cb

    def seconds(self) -> int:
        s = parse_mmss(self.edit.text())
        return int(clamp(int(s if s is not None else 0), 0, TIME_MAX_S))

    def set_seconds(self, s: int) -> None:
        s = int(clamp(int(s), 0, TIME_MAX_S))
        self.edit.setText(format_mmss(s))
        self.value_changed.emit(s)

    def set_enabled(self, enabled: bool) -> None:
        self.set_buttons_enabled(enabled)
        self.set_edit_enabled(enabled)

    def set_buttons_enabled(self, enabled: bool) -> None:
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)

    def set_edit_enabled(self, enabled: bool) -> None:
        self.edit.setEnabled(enabled)

    def _adjust(self, delta_s: int) -> None:
        if self._adjust_cb is not None:
            self.set_seconds(int(self._adjust_cb(int(delta_s))))
        else:
            self.set_seconds(self.seconds() + int(delta_s))

    @Slot()
    def _emit_if_valid(self) -> None:
        self.set_seconds(self.seconds())


# ----------------------------
# Shared Header/Footer
# ----------------------------

class StatusHeader(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("StatusBar")
        sb = QHBoxLayout(self)
        sb.setContentsMargins(14, 10, 14, 10)
        sb.setSpacing(14)

        self.lbl_state = QLabel("State: Idle")
        self.lbl_vel = QLabel("Velocity: 0 rpm")
        self.lbl_rem = QLabel("Remaining: 00:00")
        self.lbl_recipe = QLabel("Recipe: (none)")
        self.lbl_step = QLabel("")
        self.lbl_backend = QLabel("")

        for w in (self.lbl_state, self.lbl_vel, self.lbl_rem, self.lbl_recipe):
            w.setObjectName("StatusLabel")
        self.lbl_step.setObjectName("StatusStep")
        self.lbl_backend.setObjectName("StatusBackend")

        sb.addWidget(self.lbl_state)
        sb.addWidget(self.lbl_vel)
        sb.addWidget(self.lbl_rem)
        sb.addWidget(self.lbl_recipe, 1)
        sb.addWidget(self.lbl_step, 2)
        sb.addWidget(self.lbl_backend, 1)

    def set_status(self, ui_state: str, rpm: float, remaining_s: int, recipe_name: str, step_info: str, backend: str) -> None:
        self.lbl_state.setText(f"State: {ui_state}")
        self.lbl_vel.setText(f"Velocity: {rpm:g} rpm")
        self.lbl_rem.setText(f"Remaining: {format_mmss(remaining_s)}")
        self.lbl_recipe.setText(f"Recipe: {recipe_name}")
        self.lbl_step.setText(step_info)
        self.lbl_backend.setText(backend)


class StartStopFooter(QWidget):
    start_clicked = Signal()
    stop_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        bottom = QHBoxLayout(self)
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.setSpacing(14)

        self.btn_start = TouchButton("Start", min_h=76, min_w=240)
        self.btn_stop = TouchButton("Stop", min_h=76, min_w=240)
        self.btn_start.setObjectName("StartButton")
        self.btn_stop.setObjectName("StopButton")

        bottom.addWidget(self.btn_start, 1)
        bottom.addWidget(self.btn_stop, 1)

        self.btn_start.clicked.connect(self.start_clicked.emit)
        self.btn_stop.clicked.connect(self.stop_clicked.emit)

    def set_running_ui(self, running: bool) -> None:
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)


# ----------------------------
# Home Screen Recipe Cards
# ----------------------------

class RecipeCard(QFrame):
    select_clicked = Signal(str)
    edit_clicked = Signal(str)

    def __init__(self, recipe: Recipe) -> None:
        super().__init__()
        self.recipe = recipe
        self.setObjectName("RecipeCard")
        self.setFrameShape(QFrame.StyledPanel)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)

        name = QLabel(recipe.name)
        name.setObjectName("RecipeName")
        summary = QLabel(recipe_summary(recipe))
        summary.setObjectName("RecipeSummary")

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_select = TouchButton("Select", min_h=64, min_w=160)
        btn_edit = TouchButton("Edit", min_h=64, min_w=160)
        btn_row.addWidget(btn_select, 1)
        btn_row.addWidget(btn_edit, 1)

        root.addWidget(name)
        root.addWidget(summary)
        root.addLayout(btn_row)

        btn_select.clicked.connect(lambda: self.select_clicked.emit(self.recipe.id))
        btn_edit.clicked.connect(lambda: self.edit_clicked.emit(self.recipe.id))

    def set_selected(self, selected: bool) -> None:
        self.setProperty("selected", "true" if selected else "false")
        self.style().unpolish(self)
        self.style().polish(self)


# ----------------------------
# Recipe Builder
# ----------------------------

class StepRow(QFrame):
    remove_clicked = Signal(int)
    move_up_clicked = Signal(int)
    move_down_clicked = Signal(int)

    def __init__(self, index: int, step: Step) -> None:
        super().__init__()
        self._index = index
        self.setObjectName("StepRow")
        self.setFrameShape(QFrame.StyledPanel)

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        header = QHBoxLayout()
        header.setSpacing(10)

        self.lbl = QLabel(f"Step {index + 1}")
        self.lbl.setObjectName("StepIndex")
        header.addWidget(self.lbl, 1)

        btn_h, btn_w = 54, 120
        self.btn_up = TouchButton("Up", min_h=btn_h, min_w=btn_w)
        self.btn_down = TouchButton("Down", min_h=btn_h, min_w=btn_w)
        self.btn_remove = TouchButton("Remove", min_h=btn_h, min_w=btn_w)
        header.addWidget(self.btn_up)
        header.addWidget(self.btn_down)
        header.addWidget(self.btn_remove)
        root.addLayout(header)

        controls = QHBoxLayout()
        controls.setSpacing(12)
        self.vel = RpmControl("Velocity", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM, compact=True)
        self.tim = TimeControl("Time", TIME_STEP_S, compact=True)
        self.vel.set_value(step.velocity_rpm)
        self.tim.set_seconds(step.duration_s)
        controls.addWidget(self.vel, 1)
        controls.addWidget(self.tim, 1)
        root.addLayout(controls)

        self.btn_remove.clicked.connect(lambda: self.remove_clicked.emit(self._index))
        self.btn_up.clicked.connect(lambda: self.move_up_clicked.emit(self._index))
        self.btn_down.clicked.connect(lambda: self.move_down_clicked.emit(self._index))

    def set_index(self, idx: int) -> None:
        self._index = idx
        self.lbl.setText(f"Step {idx + 1}")

    def get_step(self) -> Step:
        return Step(velocity_rpm=float(self.vel.value()), duration_s=int(self.tim.seconds()))


class RecipeBuilderScreen(QWidget):
    back_clicked = Signal()
    cancel_clicked = Signal()
    save_clicked = Signal(Recipe)

    def __init__(self) -> None:
        super().__init__()
        self._editing_id: Optional[str] = None

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(12)
        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("Recipe Builder")
        self.title.setObjectName("ScreenTitle")
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        root.addLayout(top)

        name_row = QHBoxLayout()
        name_row.setSpacing(12)
        lbl_name = QLabel("Name")
        lbl_name.setObjectName("FieldLabel")
        lbl_name.setMinimumWidth(120)
        self.name_edit = QLineEdit()
        self.name_edit.setMinimumHeight(66)
        self.name_edit.setPlaceholderText("Enter recipe name")
        name_row.addWidget(lbl_name)
        name_row.addWidget(self.name_edit, 1)
        root.addLayout(name_row)

        self.steps_area = QScrollArea()
        self.steps_area.setWidgetResizable(True)
        self.steps_area.setFrameShape(QFrame.NoFrame)
        self.steps_container = QWidget()
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setContentsMargins(0, 0, 0, 0)
        self.steps_layout.setSpacing(10)
        self.steps_layout.addStretch(1)
        self.steps_area.setWidget(self.steps_container)
        root.addWidget(self.steps_area, 1)

        actions = QHBoxLayout()
        actions.setSpacing(12)
        self.btn_add = TouchButton("Add Step", min_h=66, min_w=220)
        self.btn_cancel = TouchButton("Cancel", min_h=66, min_w=220)
        self.btn_save = TouchButton("Save", min_h=66, min_w=220)
        actions.addWidget(self.btn_add, 1)
        actions.addWidget(self.btn_cancel, 1)
        actions.addWidget(self.btn_save, 1)
        root.addLayout(actions)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_cancel.clicked.connect(self.cancel_clicked.emit)
        self.btn_add.clicked.connect(self._on_add_step)
        self.btn_save.clicked.connect(self._on_save)

    def _rows(self) -> List[StepRow]:
        rows: List[StepRow] = []
        for i in range(self.steps_layout.count() - 1):
            w = self.steps_layout.itemAt(i).widget()
            if isinstance(w, StepRow):
                rows.append(w)
        return rows

    def _clear_rows(self) -> None:
        while self.steps_layout.count() > 1:
            item = self.steps_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _add_row(self, step: Step) -> None:
        idx = len(self._rows())
        row = StepRow(idx, step)
        row.remove_clicked.connect(self._on_remove)
        row.move_up_clicked.connect(self._on_move_up)
        row.move_down_clicked.connect(self._on_move_down)
        self.steps_layout.insertWidget(self.steps_layout.count() - 1, row)

    def _reindex(self) -> None:
        for i, r in enumerate(self._rows()):
            r.set_index(i)

    def load_recipe(self, recipe: Optional[Recipe]) -> None:
        self._clear_rows()
        if recipe is None:
            self._editing_id = None
            self.title.setText("Recipe Builder (New)")
            self.name_edit.setText("")
            self._add_row(Step(120.0, 60))
            return
        self._editing_id = recipe.id
        self.title.setText("Recipe Builder (Edit)")
        self.name_edit.setText(recipe.name)
        for s in recipe.steps:
            self._add_row(Step(float(s.velocity_rpm), int(s.duration_s)))

    @Slot()
    def _on_add_step(self) -> None:
        self._add_row(Step(120.0, 60))
        self._reindex()

    @Slot(int)
    def _on_remove(self, idx: int) -> None:
        rows = self._rows()
        if not (0 <= idx < len(rows)):
            return
        rows[idx].deleteLater()
        self.steps_layout.takeAt(idx)
        self._reindex()

    @Slot(int)
    def _on_move_up(self, idx: int) -> None:
        if idx <= 0:
            return
        rows = self._rows()
        if idx >= len(rows):
            return
        w = rows[idx]
        self.steps_layout.removeWidget(w)
        self.steps_layout.insertWidget(idx - 1, w)
        self._reindex()

    @Slot(int)
    def _on_move_down(self, idx: int) -> None:
        rows = self._rows()
        if not (0 <= idx < len(rows) - 1):
            return
        w = rows[idx]
        self.steps_layout.removeWidget(w)
        self.steps_layout.insertWidget(idx + 1, w)
        self._reindex()

    @Slot()
    def _on_save(self) -> None:
        name = self.name_edit.text().strip()
        steps = [r.get_step() for r in self._rows()]

        if not name:
            QMessageBox.warning(self, "Validation", "Recipe name is required.")
            return
        if len(steps) < 1:
            QMessageBox.warning(self, "Validation", "Add at least one step.")
            return
        for i, s in enumerate(steps, start=1):
            if s.duration_s <= 0:
                QMessageBox.warning(self, "Validation", f"Step {i}: duration must be > 0.")
                return
            if not (VEL_MIN_RPM <= float(s.velocity_rpm) <= VEL_MAX_RPM):
                QMessageBox.warning(self, "Validation", f"Step {i}: velocity out of range.")
                return

        rid = self._editing_id or str(uuid.uuid4())
        self.save_clicked.emit(Recipe(id=rid, name=name, steps=steps))


# ----------------------------
# ODrive Configuration Screen
# ----------------------------

class SettingRowWidget(QFrame):
    def __init__(self, spec: SettingSpec) -> None:
        super().__init__()
        self.spec = spec
        self.setObjectName("ConfigRow")
        self.setFrameShape(QFrame.StyledPanel)

        root = QHBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(12)

        self.lbl = QLabel(spec.label)
        self.lbl.setObjectName("ConfigLabel")
        self.lbl.setMinimumWidth(520)

        if spec.kind == "bool":
            self.editor: QWidget = QComboBox()
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            cb.setMinimumHeight(58)
            cb.addItems(["False", "True"])
        else:
            self.editor = QLineEdit()
            le = self.editor  # type: ignore[assignment]
            assert isinstance(le, QLineEdit)
            le.setMinimumHeight(58)
            le.setAlignment(Qt.AlignCenter)
            dv = QDoubleValidator(le)
            if spec.min_value is not None:
                dv.setBottom(float(spec.min_value))
            if spec.max_value is not None:
                dv.setTop(float(spec.max_value))
            dv.setDecimals(4)
            le.setValidator(dv)

        self.note = QLabel("")
        self.note.setObjectName("ConfigNote")
        self.note.setMinimumWidth(220)
        self.note.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

        root.addWidget(self.lbl, 2)
        root.addWidget(self.editor, 1)
        root.addWidget(self.note, 1)

    def set_value(self, value: Any) -> None:
        self.note.setText("")
        self.set_enabled(True)
        if self.spec.kind == "bool":
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            cb.setCurrentIndex(1 if bool(value) else 0)
        else:
            le = self.editor  # type: ignore[assignment]
            assert isinstance(le, QLineEdit)
            le.setText(f"{safe_float(value, float(self.spec.default)):g}")

    def set_missing(self, missing: bool) -> None:
        if missing:
            self.note.setText("Not supported")
            self.set_enabled(False)
        else:
            self.note.setText("")
            self.set_enabled(True)

    def set_enabled(self, enabled: bool) -> None:
        self.editor.setEnabled(enabled)

    def get_value(self) -> Any:
        if self.spec.kind == "bool":
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            return bool(cb.currentIndex() == 1)
        le = self.editor  # type: ignore[assignment]
        assert isinstance(le, QLineEdit)
        txt = le.text().strip()
        return float(txt) if txt else float(self.spec.default)


class ODriveConfigScreen(QWidget):
    back_clicked = Signal()
    save_clicked = Signal(object)          # dict key->value
    request_refresh = Signal()
    restore_defaults_clicked = Signal()
    export_json_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._rows: Dict[str, SettingRowWidget] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(12)
        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("ODrive Configuration")
        self.title.setObjectName("ScreenTitle")
        self.btn_refresh = TouchButton("Refresh", min_h=66, min_w=170)
        self.btn_export = TouchButton("Export JSON", min_h=66, min_w=190)
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        top.addWidget(self.btn_refresh)
        top.addWidget(self.btn_export)
        root.addLayout(top)

        self.status = QLabel("Loading…")
        self.status.setObjectName("InfoLabel")
        root.addWidget(self.status)

        self.area = QScrollArea()
        self.area.setWidgetResizable(True)
        self.area.setFrameShape(QFrame.NoFrame)

        self.container = QWidget()
        self.vbox = QVBoxLayout(self.container)
        self.vbox.setContentsMargins(0, 0, 0, 0)
        self.vbox.setSpacing(10)
        self.vbox.addStretch(1)
        self.area.setWidget(self.container)
        root.addWidget(self.area, 1)

        for spec in IMPORTANT_SETTINGS:
            row = SettingRowWidget(spec)
            self._rows[spec.key] = row
            self.vbox.insertWidget(self.vbox.count() - 1, row)

        actions = QHBoxLayout()
        actions.setSpacing(12)
        self.btn_restore = TouchButton("Restore Defaults", min_h=66, min_w=260)
        self.btn_cancel = TouchButton("Cancel", min_h=66, min_w=200)
        self.btn_save = TouchButton("Save", min_h=66, min_w=200)
        actions.addWidget(self.btn_restore, 1)
        actions.addWidget(self.btn_cancel, 1)
        actions.addWidget(self.btn_save, 1)
        root.addLayout(actions)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_cancel.clicked.connect(self.back_clicked.emit)
        self.btn_save.clicked.connect(lambda: self.save_clicked.emit(self.gather_values()))
        self.btn_restore.clicked.connect(self.restore_defaults_clicked.emit)
        self.btn_refresh.clicked.connect(self.request_refresh.emit)
        self.btn_export.clicked.connect(self.export_json_clicked.emit)

    def set_loading(self, text: str) -> None:
        self.status.setText(text)

    def apply_values(self, values: Dict[str, Any], missing: List[str]) -> None:
        missing_set = set(missing)
        for key, row in self._rows.items():
            if key in values:
                row.set_value(values[key])
            else:
                row.set_value(DEFAULT_CONFIG_MAP.get(key, row.spec.default))
            row.set_missing(key in missing_set)
        self.status.setText(f"Loaded with {len(missing)} unsupported field(s) skipped" if missing else "Loaded")

    def set_defaults(self) -> None:
        for key, row in self._rows.items():
            row.set_value(DEFAULT_CONFIG_MAP.get(key, row.spec.default))
            row.set_missing(False)
        self.status.setText("Defaults restored (not saved)")

    def gather_values(self) -> Dict[str, Any]:
        return {k: row.get_value() for k, row in self._rows.items() if row.editor.isEnabled()}


# ----------------------------
# Plot Widgets
# ----------------------------

class SimplePlotWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(360)
        self._data: Dict[str, List[Tuple[int, float]]] = {}
        self._tags: List[str] = []
        self._minutes = 10

    def set_series_data(self, data: Dict[str, List[Tuple[int, float]]], tags: List[str], minutes: int) -> None:
        self._data = {k: list(v) for k, v in data.items()}
        self._tags = list(tags)
        self._minutes = int(minutes)
        self.update()

    def append_points(self, points: List[Tuple[str, int, float]], tags: List[str], minutes: int) -> None:
        self._tags = list(tags)
        self._minutes = int(minutes)
        for tag, ts, val in points:
            self._data.setdefault(tag, []).append((ts, val))
        cutoff = epoch_s() - self._minutes * 60
        for t in list(self._data.keys()):
            self._data[t] = [(ts, v) for ts, v in self._data[t] if ts >= cutoff]
        self.update()

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        rect = self.rect().adjusted(10, 10, -10, -10)
        p.drawRect(rect)

        tags = [t for t in self._tags if t in self._data]
        if not tags:
            p.drawText(rect, Qt.AlignCenter, "No data")
            return

        now_ts = epoch_s()
        x0, x1 = now_ts - self._minutes * 60, now_ts
        if x1 <= x0:
            x1 = x0 + 1

        y_min, y_max = float("inf"), float("-inf")
        for t in tags:
            for _, v in self._data.get(t, []):
                y_min, y_max = min(y_min, v), max(y_max, v)
        if not math.isfinite(y_min) or not math.isfinite(y_max):
            p.drawText(rect, Qt.AlignCenter, "No data")
            return
        if abs(y_max - y_min) < 1e-6:
            y_max = y_min + 1.0

        plot = rect.adjusted(50, 10, -10, -30)
        p.drawRect(plot)

        def map_x(ts: int) -> float:
            return plot.left() + (ts - x0) / (x1 - x0) * plot.width()

        def map_y(v: float) -> float:
            return plot.bottom() - (v - y_min) / (y_max - y_min) * plot.height()

        p.drawText(rect.left() + 6, plot.top() + 14, f"{y_max:.3g}")
        p.drawText(rect.left() + 6, plot.bottom(), f"{y_min:.3g}")
        p.drawText(plot.left(), rect.bottom() - 6, f"-{self._minutes} min")
        p.drawText(plot.right() - 60, rect.bottom() - 6, "now")

        colors = [Qt.cyan, Qt.yellow, Qt.green, Qt.magenta, Qt.red, Qt.blue, Qt.gray, Qt.white]
        for i, t in enumerate(tags):
            pts = self._data.get(t, [])
            if len(pts) < 2:
                continue
            pen = p.pen()
            pen.setWidth(2)
            pen.setColor(colors[i % len(colors)])
            p.setPen(pen)
            last = None
            for ts, v in pts:
                x, y = map_x(ts), map_y(v)
                if last is not None:
                    p.drawLine(last[0], last[1], x, y)
                last = (x, y)

        p.setPen(self.palette().text().color())
        legend = "  ".join(TAG_LABELS.get(t, t) for t in tags)
        p.drawText(rect.adjusted(0, 0, 0, -rect.height() + 20), Qt.AlignLeft | Qt.AlignVCenter, legend)


class QtChartsPlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._chart = QChart()
        self._chart.legend().setVisible(True)
        self._chart.legend().setAlignment(Qt.AlignBottom)
        self._view = QChartView(self._chart)
        self._view.setRenderHint(QPainter.Antialiasing)
        self._view.setMinimumHeight(360)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._view)

        self._series: Dict[str, QLineSeries] = {}
        self._minutes = 10
        self._tags: List[str] = []

        self._x = QDateTimeAxis()
        self._x.setFormat("HH:mm:ss")
        self._x.setTitleText("Time")
        self._chart.addAxis(self._x, Qt.AlignBottom)

        self._y_rpm = QValueAxis()
        self._y_rpm.setTitleText("RPM")
        self._chart.addAxis(self._y_rpm, Qt.AlignLeft)

        self._y_other = QValueAxis()
        self._y_other.setTitleText("Value")
        self._chart.addAxis(self._y_other, Qt.AlignRight)

    def _axis_for_tag(self, tag: str) -> QValueAxis:
        return self._y_rpm if TAG_PLOT_GROUP.get(tag) == PLOT_GROUP_RPM else self._y_other

    def set_series_data(self, data: Dict[str, List[Tuple[int, float]]], tags: List[str], minutes: int) -> None:
        self._minutes = int(minutes)
        self._tags = list(tags)

        for s in list(self._series.values()):
            self._chart.removeSeries(s)
        self._series.clear()

        for tag in self._tags:
            series = QLineSeries()
            series.setName(TAG_LABELS.get(tag, tag))
            for ts, val in data.get(tag, []):
                series.append(float(ts) * 1000.0, float(val))
            self._chart.addSeries(series)
            series.attachAxis(self._x)
            series.attachAxis(self._axis_for_tag(tag))
            self._series[tag] = series

        self._rescale_axes()

    def append_points(self, points: List[Tuple[str, int, float]], tags: List[str], minutes: int) -> None:
        self._minutes = int(minutes)
        self._tags = list(tags)

        for tag in self._tags:
            if tag not in self._series:
                s = QLineSeries()
                s.setName(TAG_LABELS.get(tag, tag))
                self._chart.addSeries(s)
                s.attachAxis(self._x)
                s.attachAxis(self._axis_for_tag(tag))
                self._series[tag] = s

        for tag in list(self._series.keys()):
            if tag not in self._tags:
                s = self._series.pop(tag)
                self._chart.removeSeries(s)

        cutoff_ms = (epoch_s() - self._minutes * 60) * 1000.0
        for tag, ts, val in points:
            if tag in self._series:
                self._series[tag].append(float(ts) * 1000.0, float(val))

        for s in self._series.values():
            n = s.count()
            if n <= 0:
                continue
            idx = 0
            while idx < n and s.at(idx).x() < cutoff_ms:
                idx += 1
            if idx > 0:
                s.removePoints(0, idx)

        self._rescale_axes()

    def _rescale_axes(self) -> None:
        now_ts = epoch_s()
        start_ts = now_ts - self._minutes * 60
        self._x.setRange(QDateTime.fromSecsSinceEpoch(start_ts), QDateTime.fromSecsSinceEpoch(now_ts))

        rpm_vals: List[float] = []
        other_vals: List[float] = []
        for tag, s in self._series.items():
            pts = s.pointsVector()
            if not pts:
                continue
            if TAG_PLOT_GROUP.get(tag) == PLOT_GROUP_RPM:
                rpm_vals += [float(pt.y()) for pt in pts]
            else:
                other_vals += [float(pt.y()) for pt in pts]

        def apply_axis(axis: QValueAxis, vals: List[float]) -> None:
            if not vals:
                axis.setRange(0.0, 1.0)
                return
            vmin, vmax = min(vals), max(vals)
            if abs(vmax - vmin) < 1e-9:
                vmax = vmin + 1.0
            pad = 0.05 * (vmax - vmin)
            axis.setRange(vmin - pad, vmax + pad)

        apply_axis(self._y_rpm, rpm_vals)
        apply_axis(self._y_other, other_vals)


def make_plot_widget() -> QWidget:
    return QtChartsPlot() if QTCHARTS_AVAILABLE else SimplePlotWidget()


# ----------------------------
# Data Logger Screen
# ----------------------------

class DataLoggerScreen(QWidget):
    back_clicked = Signal()
    start_clicked = Signal()
    stop_clicked = Signal()
    window_changed = Signal(int)
    tags_changed = Signal(object)          # list[str]
    export_clicked = Signal(int, object)   # minutes, list[str]

    def __init__(self) -> None:
        super().__init__()
        self._minutes = 10

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(12)
        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("Data Logger")
        self.title.setObjectName("ScreenTitle")
        self.btn_export = TouchButton("Export CSV", min_h=66, min_w=200)
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        top.addWidget(self.btn_export)
        root.addLayout(top)

        self.status = StatusHeader()
        root.addWidget(self.status)

        body = QHBoxLayout()
        body.setSpacing(12)
        left = QVBoxLayout()
        left.setSpacing(12)

        lbl_tags = QLabel("Tags")
        lbl_tags.setObjectName("SectionTitle")
        left.addWidget(lbl_tags)

        self._tag_checks: Dict[str, QCheckBox] = {}
        for tag in [TAG_PWR_W, TAG_CMD_RPM, TAG_VEL_RPM, TAG_VBUS_V, TAG_IBUS_A]:
            cb = QCheckBox(TAG_LABELS.get(tag, tag))
            cb.setChecked(tag in DEFAULT_ENABLED_TAGS)
            cb.setMinimumHeight(44)
            cb.stateChanged.connect(lambda _=0: self.tags_changed.emit(self.selected_tags()))
            self._tag_checks[tag] = cb
            left.addWidget(cb)

        left.addStretch(1)

        win_row = QHBoxLayout()
        win_row.setSpacing(12)
        lbl_win = QLabel("Minutes to display")
        lbl_win.setObjectName("FieldLabel")
        lbl_win.setMinimumWidth(260)
        self.edit_min = QLineEdit()
        self.edit_min.setMinimumHeight(66)
        self.edit_min.setAlignment(Qt.AlignCenter)
        self.edit_min.setValidator(QIntValidator(1, 24 * 60, self.edit_min))
        self.edit_min.setText(str(self._minutes))
        self.btn_apply = TouchButton("Apply", min_h=66, min_w=170)
        win_row.addWidget(lbl_win)
        win_row.addWidget(self.edit_min, 1)
        win_row.addWidget(self.btn_apply)
        left.addLayout(win_row)

        body.addLayout(left, 1)

        self.plot = make_plot_widget()
        body.addWidget(self.plot, 2)
        root.addLayout(body, 1)

        self.footer = StartStopFooter()
        root.addWidget(self.footer)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.footer.start_clicked.connect(self.start_clicked.emit)
        self.footer.stop_clicked.connect(self.stop_clicked.emit)
        self.btn_apply.clicked.connect(self._on_apply)
        self.edit_min.editingFinished.connect(self._on_apply)
        self.btn_export.clicked.connect(self._on_export)

    def set_status(self, ui_state: str, rpm: float, remaining_s: int, recipe_name: str, step_info: str, backend: str) -> None:
        self.status.set_status(ui_state, rpm, remaining_s, recipe_name, step_info, backend)

    def set_running_ui(self, running: bool) -> None:
        self.footer.set_running_ui(running)

    def minutes_window(self) -> int:
        try:
            return max(1, int(self.edit_min.text()))
        except Exception:
            return 10

    def selected_tags(self) -> List[str]:
        return [t for t, cb in self._tag_checks.items() if cb.isChecked()]

    def set_series_data(self, data: Dict[str, List[Tuple[int, float]]], tags: List[str], minutes: int) -> None:
        self.plot.set_series_data(data, tags, minutes)  # type: ignore[attr-defined]

    def append_points(self, points: List[Tuple[str, int, float]], tags: List[str], minutes: int) -> None:
        self.plot.append_points(points, tags, minutes)  # type: ignore[attr-defined]

    @Slot()
    def _on_apply(self) -> None:
        self.window_changed.emit(self.minutes_window())

    @Slot()
    def _on_export(self) -> None:
        self.export_clicked.emit(self.minutes_window(), self.selected_tags())


# ----------------------------
# Home Screen
# ----------------------------

class HomeScreen(QWidget):
    start_clicked = Signal()
    stop_clicked = Signal()
    open_builder_clicked = Signal()
    recipe_select_clicked = Signal(str)
    recipe_edit_clicked = Signal(str)
    open_config_clicked = Signal()
    open_datalogger_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        self.status_bar = StatusHeader()
        root.addWidget(self.status_bar)

        mid = QHBoxLayout()
        mid.setSpacing(14)

        left = QVBoxLayout()
        left.setSpacing(12)
        self.rpm_ctl = RpmControl("Velocity", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM, compact=False)
        self.time_ctl = TimeControl("Timer", TIME_STEP_S, compact=False)
        self.btn_builder = TouchButton("Recipe Builder", min_h=70, min_w=240)
        self.btn_config = TouchButton("ODrive Configuration", min_h=70, min_w=240)
        self.btn_datalogger = TouchButton("Data Logger", min_h=70, min_w=240)

        left.addWidget(self.rpm_ctl)
        left.addWidget(self.time_ctl)
        left.addWidget(self.btn_builder)
        left.addWidget(self.btn_config)
        left.addWidget(self.btn_datalogger)
        left.addStretch(1)
        mid.addLayout(left, 1)

        recipes_col = QVBoxLayout()
        recipes_col.setSpacing(8)
        title = QLabel("Recipes")
        title.setObjectName("SectionTitle")
        recipes_col.addWidget(title)

        self.recipes_area = QScrollArea()
        self.recipes_area.setWidgetResizable(True)
        self.recipes_area.setFrameShape(QFrame.NoFrame)
        self.recipes_container = QWidget()
        self.recipes_layout = QVBoxLayout(self.recipes_container)
        self.recipes_layout.setContentsMargins(0, 0, 0, 0)
        self.recipes_layout.setSpacing(10)
        self.recipes_layout.addStretch(1)
        self.recipes_area.setWidget(self.recipes_container)
        recipes_col.addWidget(self.recipes_area, 1)

        mid.addLayout(recipes_col, 2)
        root.addLayout(mid, 1)

        self.footer = StartStopFooter()
        root.addWidget(self.footer)

        self.footer.start_clicked.connect(self.start_clicked.emit)
        self.footer.stop_clicked.connect(self.stop_clicked.emit)
        self.btn_builder.clicked.connect(self.open_builder_clicked.emit)
        self.btn_config.clicked.connect(self.open_config_clicked.emit)
        self.btn_datalogger.clicked.connect(self.open_datalogger_clicked.emit)

    def set_recipe_cards(self, recipes: List[Recipe], selected_id: Optional[str]) -> None:
        while self.recipes_layout.count() > 1:
            item = self.recipes_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

        for r in recipes:
            card = RecipeCard(r)
            card.set_selected(selected_id == r.id)
            card.select_clicked.connect(self.recipe_select_clicked.emit)
            card.edit_clicked.connect(self.recipe_edit_clicked.emit)
            self.recipes_layout.insertWidget(self.recipes_layout.count() - 1, card)

    def set_status(self, ui_state: str, rpm: float, remaining_s: int, recipe_name: str, step_info: str, backend: str) -> None:
        self.status_bar.set_status(ui_state, rpm, remaining_s, recipe_name, step_info, backend)

    def set_running_ui(self, running: bool, run_mode: str, recipe_selected: bool) -> None:
        """
        Enforce home-screen behaviors:
          - Start disabled while running
          - Stop enabled while running
          - RPM input disabled while running
          - Recipe selection disabled while running
          - Builder/config navigation disabled while running
          - Data Logger navigation enabled during run
          - Manual timer disabled when recipe selected while idle
          - While running SINGLE mode: timer text disabled, +/- enabled (adjust callback is set by controller)
        """
        self.footer.set_running_ui(running)
        self.rpm_ctl.set_enabled(not running)

        if running:
            if run_mode == "single":
                self.time_ctl.set_buttons_enabled(True)
                self.time_ctl.set_edit_enabled(False)
            else:
                self.time_ctl.set_enabled(False)
        else:
            self.time_ctl.set_enabled(not recipe_selected)

        self.btn_builder.setEnabled(not running)
        self.btn_config.setEnabled(not running)
        self.btn_datalogger.setEnabled(True)
        self.recipes_area.setEnabled(not running)


# ----------------------------
# Controller
# ----------------------------

class AppController(QObject):
    request_connect = Signal()
    request_read_config = Signal(object)          # list[str]
    request_apply_config = Signal(object)         # dict
    request_export_json = Signal()
    request_set_polled_tags = Signal(object)      # list[str]
    request_start_poll = Signal()
    request_stop_poll = Signal()

    def __init__(self, stack: QStackedWidget, home: HomeScreen, builder: RecipeBuilderScreen, cfg: ODriveConfigScreen, dlog: DataLoggerScreen) -> None:
        super().__init__()
        self.stack, self.home, self.builder, self.cfg, self.dlog = stack, home, builder, cfg, dlog

        self.store = RecipeStore()
        self.store.load()
        self.selected_recipe_id: Optional[str] = None

        self.connected = False
        self.backend_state = "Disconnected"

        self._ui_state = "Idle"
        self._ui_rpm = 0.0
        self._ui_remaining = 0
        self._ui_step_info = ""
        self._run_mode = "idle"
        self._cmd_rpm = 0.0

        self._dlog_minutes = 10
        self._dlog_selected_tags: List[str] = list(DEFAULT_ENABLED_TAGS)

        self.history_db = HistoryDB(app_config_dir() / "history.sqlite")

        backend: OdriveInterface = SimulatedOdrive() if BACKEND == "sim" else RealOdrive()
        self.worker_thread = QThread()
        self.worker = OdriveWorker(backend)
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.on_thread_started)

        self.engine = RunEngine()
        self.engine.request_set_rpm.connect(self.worker.set_velocity_rpm)
        self.engine.request_stop.connect(self.worker.stop)

        self.request_connect.connect(self.worker.connect_backend)
        self.request_read_config.connect(self.worker.read_config)
        self.request_apply_config.connect(self.worker.apply_config)
        self.request_export_json.connect(self.worker.export_json)
        self.request_set_polled_tags.connect(self.worker.set_polled_tags)
        self.request_start_poll.connect(self.worker.start_tag_polling)
        self.request_stop_poll.connect(self.worker.stop_tag_polling)

        self.worker.connected_changed.connect(self._on_connected)
        self.worker.state_changed.connect(self._on_backend_state)
        self.worker.rpm_commanded.connect(lambda rpm: setattr(self, "_cmd_rpm", float(rpm)))
        self.worker.error.connect(self._on_worker_error)
        self.worker.config_read.connect(self._on_config_read)
        self.worker.config_applied.connect(self._on_config_applied)
        self.worker.export_json_done.connect(self._on_export_json_done)
        self.worker.tags_sample.connect(self._on_tags_sample)

        self.engine.status_changed.connect(lambda s: self._set_ui(status=s))
        self.engine.run_mode_changed.connect(lambda m: setattr(self, "_run_mode", m))
        self.engine.active_rpm_changed.connect(self._on_engine_rpm)
        self.engine.remaining_changed.connect(self._on_engine_remaining)
        self.engine.step_info_changed.connect(lambda x: self._set_ui(step=x))

        self.home.start_clicked.connect(self.on_start)
        self.home.stop_clicked.connect(self.on_stop)
        self.home.open_builder_clicked.connect(self.on_open_builder_new)
        self.home.recipe_select_clicked.connect(self.on_select_recipe)
        self.home.recipe_edit_clicked.connect(self.on_edit_recipe)
        self.home.open_config_clicked.connect(self.on_open_config)
        self.home.open_datalogger_clicked.connect(self.on_open_datalogger)

        self.builder.back_clicked.connect(self.on_back_home)
        self.builder.cancel_clicked.connect(self.on_back_home)
        self.builder.save_clicked.connect(self.on_builder_save)

        self.cfg.back_clicked.connect(self.on_back_home)
        self.cfg.request_refresh.connect(self._refresh_odrive_config)
        self.cfg.save_clicked.connect(self._save_odrive_config)
        self.cfg.restore_defaults_clicked.connect(self.cfg.set_defaults)
        self.cfg.export_json_clicked.connect(self._export_odrive_json)

        self.dlog.back_clicked.connect(self.on_back_home)
        self.dlog.start_clicked.connect(self.on_start)
        self.dlog.stop_clicked.connect(self.on_stop)
        self.dlog.window_changed.connect(self._on_dlog_window_changed)
        self.dlog.tags_changed.connect(self._on_dlog_tags_changed)
        self.dlog.export_clicked.connect(self._export_datalogger_csv)

        self.stack.currentChanged.connect(self._on_screen_changed)

        self._cmd_log_timer = QTimer(self)
        self._cmd_log_timer.setInterval(LOG_POLL_MS)
        self._cmd_log_timer.timeout.connect(self._log_cmd_rpm)
        self._cmd_log_timer.start()

        self._last_modal_error_mono = 0.0
        self._modal_error_cooldown_s = 8.0

        self.worker_thread.start()
        self.request_connect.emit()
        self.request_set_polled_tags.emit([TAG_VBUS_V, TAG_IBUS_A, TAG_VEL_RPM])

        self._refresh_recipes()
        self._refresh_status_all()
        self._apply_controls()

    def shutdown(self) -> None:
        try:
            self.request_stop_poll.emit()
        except Exception:
            pass
        try:
            if self.engine.is_running():
                self.engine.stop(user_initiated=True)
                QMetaObject.invokeMethod(self.worker, "stop", Qt.BlockingQueuedConnection)
        except Exception:
            pass
        self.worker_thread.quit()
        self.worker_thread.wait(1500)
        try:
            self.history_db.close()
        except Exception:
            pass

    def _show_modal_error(self, parent: QWidget, title: str, msg: str) -> None:
        now_m = time.monotonic()
        if (now_m - self._last_modal_error_mono) < self._modal_error_cooldown_s:
            return
        self._last_modal_error_mono = now_m
        QMessageBox.critical(parent, title, msg)

    def _selected_recipe(self) -> Optional[Recipe]:
        if not self.selected_recipe_id:
            return None
        for r in self.store.recipes():
            if r.id == self.selected_recipe_id:
                return r
        return None

    def _refresh_recipes(self) -> None:
        self.home.set_recipe_cards(self.store.recipes(), self.selected_recipe_id)

    def _refresh_status_all(self) -> None:
        r = self._selected_recipe()
        rname = r.name if r else "(none)"
        self.home.set_status(self._ui_state, self._ui_rpm, self._ui_remaining, rname, self._ui_step_info, self.backend_state)
        self.dlog.set_status(self._ui_state, self._ui_rpm, self._ui_remaining, rname, self._ui_step_info, self.backend_state)

    def _apply_controls(self) -> None:
        r_selected = self._selected_recipe() is not None
        running = self.engine.is_running()

        # While running SINGLE mode, +/- must adjust remaining time immediately.
        if running and self._run_mode == "single" and not r_selected:
            self.home.time_ctl.set_adjust_callback(self._adjust_time_from_home)
        else:
            self.home.time_ctl.set_adjust_callback(None)

        self.home.set_running_ui(running=running, run_mode=self._run_mode, recipe_selected=r_selected)
        self.dlog.set_running_ui(running=running)

    def _adjust_time_from_home(self, delta_s: int) -> int:
        if self._run_mode != "single":
            return self._ui_remaining
        self.engine.adjust_single_remaining(int(delta_s))
        return int(clamp(self._ui_remaining, 0, TIME_MAX_S))

    def _set_ui(self, status: Optional[str] = None, step: Optional[str] = None) -> None:
        if status is not None:
            self._ui_state = status
        if step is not None:
            self._ui_step_info = step
        self._refresh_status_all()
        self._apply_controls()

    # ---- Worker callbacks ----

    @Slot(bool, str)
    def _on_connected(self, connected: bool, state: str) -> None:
        self.connected = bool(connected)
        self.backend_state = state

        if connected:
            self.request_start_poll.emit()
            if self.stack.currentWidget() is self.cfg:
                self._refresh_odrive_config()
        else:
            self.request_stop_poll.emit()
            if self.engine.is_running():
                self.engine.stop(user_initiated=True)
            if self.stack.currentWidget() is self.cfg:
                self.cfg.set_loading("Disconnected (reconnecting…)")

        self._refresh_status_all()
        self._apply_controls()

    @Slot(str)
    def _on_backend_state(self, state: str) -> None:
        self.backend_state = state
        self._refresh_status_all()

    @Slot(str)
    def _on_worker_error(self, msg: str) -> None:
        if self.engine.is_running():
            self.engine.stop(user_initiated=True)
        self._show_modal_error(self.stack.currentWidget(), "ODrive Error", msg)
        self._refresh_status_all()
        self._apply_controls()

    @Slot(object, object)
    def _on_config_read(self, values_obj: Any, missing_obj: Any) -> None:
        if self.stack.currentWidget() is self.cfg:
            self.cfg.apply_values(dict(values_obj or {}), list(missing_obj or []))

    @Slot(bool, str, object)
    def _on_config_applied(self, ok: bool, message: str, missing_obj: Any) -> None:
        missing = list(missing_obj or [])
        if ok:
            QMessageBox.information(self.cfg, "Configuration", f"{message}\n\nUnsupported skipped: {len(missing)}")
            self.on_back_home()
        else:
            self._show_modal_error(self.cfg, "Configuration", message)

    @Slot(bool, str)
    def _on_export_json_done(self, ok: bool, msg: str) -> None:
        if ok:
            QMessageBox.information(self.cfg, "Export JSON", f"Saved:\n{msg}")
        else:
            self._show_modal_error(self.cfg, "Export JSON", msg)

    @Slot(int, object)
    def _on_tags_sample(self, ts: int, values_obj: Any) -> None:
        values: Dict[str, float] = {str(k): safe_float(v) for k, v in dict(values_obj or {}).items()}

        vbus, ibus = values.get(TAG_VBUS_V, float("nan")), values.get(TAG_IBUS_A, float("nan"))
        if math.isfinite(vbus) and math.isfinite(ibus):
            values[TAG_PWR_W] = float(vbus * ibus)
        values[TAG_CMD_RPM] = float(self._cmd_rpm)

        samples: List[Tuple[str, float, float, int]] = []
        for tag in [TAG_VBUS_V, TAG_IBUS_A, TAG_VEL_RPM, TAG_PWR_W, TAG_CMD_RPM]:
            if tag in values and math.isfinite(values[tag]):
                samples.append((tag, float(values[tag]), float(TAG_DEADBAND.get(tag, 0.0)), int(TAG_HEARTBEAT_S.get(tag, DB_HEARTBEAT_S_DEFAULT))))

        try:
            inserted = self.history_db.insert_many_if_needed(ts=ts, samples=samples)
        except Exception as e:
            self._show_modal_error(self.dlog, "Data Logger DB", f"Failed to write sample(s): {e}")
            return

        if self.stack.currentWidget() is self.dlog and inserted:
            selected = set(self._dlog_selected_tags)
            points = [(tag, ts, float(values[tag])) for tag in inserted if tag in selected]
            if points:
                self.dlog.append_points(points, tags=self._dlog_selected_tags, minutes=self._dlog_minutes)

    # ---- Engine callbacks ----

    @Slot(float)
    def _on_engine_rpm(self, rpm: float) -> None:
        self._ui_rpm = float(rpm)
        self._cmd_rpm = float(rpm)
        self._refresh_status_all()
        self._apply_controls()

    @Slot(int)
    def _on_engine_remaining(self, seconds: int) -> None:
        self._ui_remaining = int(seconds)
        self._refresh_status_all()
        if self.engine.is_running() and self._run_mode == "single":
            self.home.time_ctl.set_seconds(int(clamp(self._ui_remaining, 0, TIME_MAX_S)))

    # ---- Screen changes ----

    @Slot(int)
    def _on_screen_changed(self, _idx: int) -> None:
        if self.stack.currentWidget() is self.dlog:
            self._dlog_minutes = self.dlog.minutes_window()
            self._dlog_selected_tags = self.dlog.selected_tags() or list(DEFAULT_ENABLED_TAGS)
            self._refresh_datalogger_plot()

    # ---- Home actions ----

    @Slot()
    def on_start(self) -> None:
        if self.engine.is_running():
            return
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.home, "ODrive Not Connected", "Cannot start: ODrive is not connected.")
            return

        rpm = clamp(self.home.rpm_ctl.value(), VEL_MIN_RPM, VEL_MAX_RPM)
        recipe = self._selected_recipe()
        if recipe is None:
            duration = self.home.time_ctl.seconds()
            if duration <= 0:
                QMessageBox.information(self.home, "Start", "Set a timer or select a recipe.")
                return
            self.home.time_ctl.set_seconds(int(clamp(duration, 0, TIME_MAX_S)))
            self.engine.start_single(rpm, duration)
            return

        if len(recipe.steps) < 1:
            QMessageBox.warning(self.home, "Start", "Selected recipe has no steps.")
            return
        self.engine.start_recipe(recipe)

    @Slot()
    def on_stop(self) -> None:
        self.engine.stop(user_initiated=True)

    @Slot()
    def on_open_builder_new(self) -> None:
        if self.engine.is_running():
            return
        self.builder.load_recipe(None)
        self.stack.setCurrentWidget(self.builder)

    @Slot()
    def on_open_config(self) -> None:
        if self.engine.is_running():
            return
        self.stack.setCurrentWidget(self.cfg)
        self._refresh_odrive_config()

    @Slot()
    def on_open_datalogger(self) -> None:
        self.stack.setCurrentWidget(self.dlog)
        self._dlog_minutes = self.dlog.minutes_window()
        self._dlog_selected_tags = self.dlog.selected_tags() or list(DEFAULT_ENABLED_TAGS)
        self._refresh_datalogger_plot()

    @Slot()
    def on_back_home(self) -> None:
        self.stack.setCurrentWidget(self.home)
        self._refresh_recipes()
        self._refresh_status_all()
        self._apply_controls()

    @Slot(str)
    def on_select_recipe(self, recipe_id: str) -> None:
        if self.engine.is_running():
            return
        self.selected_recipe_id = None if self.selected_recipe_id == recipe_id else recipe_id
        self._refresh_recipes()
        self._refresh_status_all()
        self._apply_controls()

    @Slot(str)
    def on_edit_recipe(self, recipe_id: str) -> None:
        if self.engine.is_running():
            return
        r = next((x for x in self.store.recipes() if x.id == recipe_id), None)
        if r is None:
            return
        self.builder.load_recipe(r)
        self.stack.setCurrentWidget(self.builder)

    @Slot(Recipe)
    def on_builder_save(self, recipe: Recipe) -> None:
        self.store.upsert(recipe)
        self.on_back_home()

    # ---- ODrive Config actions ----

    def _refresh_odrive_config(self) -> None:
        keys = [s.key for s in IMPORTANT_SETTINGS]
        self.cfg.set_loading("Loading from device…")
        if BACKEND == "real" and not self.connected:
            self.cfg.apply_values(DEFAULT_CONFIG_MAP, keys)
            self.cfg.set_loading("Not connected (reconnecting…) — showing defaults")
            return
        self.request_read_config.emit(keys)

    @Slot(object)
    def _save_odrive_config(self, values_obj: Any) -> None:
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.cfg, "Configuration", "Cannot save: ODrive is not connected.")
            return
        self.cfg.set_loading("Applying & saving config (device will reboot)…")
        self.request_apply_config.emit(dict(values_obj or {}))

    @Slot()
    def _export_odrive_json(self) -> None:
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.cfg, "Export JSON", "Cannot export: ODrive is not connected.")
            return
        self.cfg.set_loading("Exporting JSON…")
        self.request_export_json.emit()

    # ---- Data Logger actions ----

    @Slot(int)
    def _on_dlog_window_changed(self, minutes: int) -> None:
        self._dlog_minutes = max(1, int(minutes))
        if self.stack.currentWidget() is self.dlog:
            self._refresh_datalogger_plot()

    @Slot(object)
    def _on_dlog_tags_changed(self, tags_obj: Any) -> None:
        tags = list(tags_obj or [])
        self._dlog_selected_tags = tags if tags else list(DEFAULT_ENABLED_TAGS)
        if self.stack.currentWidget() is self.dlog:
            self._refresh_datalogger_plot()

    def _refresh_datalogger_plot(self) -> None:
        now_ts = epoch_s()
        tags = self._dlog_selected_tags
        minutes = self._dlog_minutes
        try:
            data = self.history_db.query_window_multi(now_ts, tags, minutes)
        except Exception as e:
            self._show_modal_error(self.dlog, "Data Logger DB", f"Failed to load window:\n{e}")
            data = {t: [] for t in tags}
        self.dlog.set_series_data(data, tags=tags, minutes=minutes)

    @Slot()
    def _log_cmd_rpm(self) -> None:
        ts = epoch_s()
        try:
            self.history_db.insert_if_needed(
                ts=ts,
                tag=TAG_CMD_RPM,
                value=float(self._cmd_rpm),
                deadband=float(TAG_DEADBAND.get(TAG_CMD_RPM, 0.0)),
                heartbeat_s=int(TAG_HEARTBEAT_S.get(TAG_CMD_RPM, DB_HEARTBEAT_S_DEFAULT)),
            )
        except Exception:
            pass

    @Slot(int, object)
    def _export_datalogger_csv(self, minutes: int, tags_obj: Any) -> None:
        minutes = max(1, int(minutes))
        tags = list(tags_obj or [])
        if not tags:
            QMessageBox.information(self.dlog, "Export", "Select at least one tag to export.")
            return
        out = downloads_dir() / f"datalogger_{minutes}min_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            n = self.history_db.export_window_csv_multi(out, epoch_s(), tags, minutes)
            QMessageBox.information(self.dlog, "Export", f"Exported {n} row(s) to:\n{out}")
        except Exception as e:
            self._show_modal_error(self.dlog, "Export", f"Failed to export CSV:\n{e}")


# ----------------------------
# Main Window
# ----------------------------

class MainWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ODrive Reactor Agitator HMI")
        self.setFixedSize(WINDOW_W, WINDOW_H)

        self.stack = QStackedWidget()
        self.home = HomeScreen()
        self.builder = RecipeBuilderScreen()
        self.cfg = ODriveConfigScreen()
        self.dlog = DataLoggerScreen()

        self.stack.addWidget(self.home)
        self.stack.addWidget(self.builder)
        self.stack.addWidget(self.cfg)
        self.stack.addWidget(self.dlog)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.stack)

        self.controller = AppController(self.stack, self.home, self.builder, self.cfg, self.dlog)

    def closeEvent(self, event) -> None:
        self.controller.shutdown()
        event.accept()


# ----------------------------
# Styling
# ----------------------------

def apply_style(app: QApplication) -> None:
    base_font = QFont()
    base_font.setPointSize(12)
    app.setFont(base_font)

    app.setStyleSheet(
        """
        QWidget { background: #101317; color: #F2F5F7; font-size: 20px; }

        QLabel#ScreenTitle { font-size: 28px; font-weight: 700; }
        QLabel#SectionTitle { font-size: 24px; font-weight: 650; }
        QLabel#SectionTitleCompact { font-size: 20px; font-weight: 650; }
        QLabel#FieldLabel { font-size: 22px; font-weight: 650; }
        QLabel#UnitsLabel { font-size: 18px; color: #D0D7DE; }
        QLabel#InfoLabel { font-size: 18px; color: #D0D7DE; }

        QFrame#StatusBar { background: #161B22; border: 2px solid #2A313A; border-radius: 14px; }
        QLabel#StatusLabel { font-size: 20px; font-weight: 650; }
        QLabel#StatusStep { font-size: 18px; color: #D0D7DE; }
        QLabel#StatusBackend { font-size: 16px; color: #9FB0C0; }

        QLineEdit {
            background: #0B0F14; border: 2px solid #2A313A; border-radius: 14px;
            padding: 8px 12px; font-size: 26px;
        }

        QComboBox {
            background: #0B0F14; border: 2px solid #2A313A; border-radius: 14px;
            padding: 8px 12px; font-size: 22px; min-height: 58px;
        }

        QCheckBox { spacing: 12px; font-size: 20px; }
        QCheckBox::indicator { width: 28px; height: 28px; }

        QPushButton {
            background: #2B3440; border: 2px solid #3A4656; border-radius: 16px;
            padding: 10px 14px; font-size: 24px; font-weight: 650;
        }
        QPushButton:hover { background: #354152; }
        QPushButton:pressed { background: #202733; }
        QPushButton:disabled { background: #1C222B; border: 2px solid #2A313A; color: #7A8794; }

        QPushButton#StartButton { background: #1E5F3C; border: 2px solid #2D7B52; }
        QPushButton#StartButton:hover { background: #21704A; }
        QPushButton#StopButton { background: #6B1E1E; border: 2px solid #8B2A2A; }
        QPushButton#StopButton:hover { background: #7A2424; }

        QFrame#RecipeCard { background: #161B22; border: 2px solid #2A313A; border-radius: 16px; }
        QFrame#RecipeCard[selected="true"] { border: 4px solid #2F81F7; }
        QLabel#RecipeName { font-size: 24px; font-weight: 700; }
        QLabel#RecipeSummary { font-size: 18px; color: #D0D7DE; }

        QFrame#StepRow { background: #161B22; border: 2px solid #2A313A; border-radius: 16px; }
        QLabel#StepIndex { font-size: 20px; font-weight: 700; }

        QFrame#ConfigRow { background: #161B22; border: 2px solid #2A313A; border-radius: 16px; }
        QLabel#ConfigLabel { font-size: 20px; font-weight: 650; }
        QLabel#ConfigNote { font-size: 18px; color: #9FB0C0; }

        QScrollArea { border: none; }
        """
    )


# ----------------------------
# Entry Point
# ----------------------------

def main() -> int:
    app = QApplication(sys.argv)
    apply_style(app)
    w = MainWindow()
    w.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
