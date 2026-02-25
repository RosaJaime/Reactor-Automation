"""
Touch-Friendly ODrive Reactor Agitator HMI (PySide6)

Run:
  pip install PySide6
  python odrive_hmi.pyw

Backends:
  - BACKEND = "real" (default)
  - BACKEND = "sim"  (available for testing)

Real backend (closed-loop velocity control):
  - UI uses RPM; backend converts to turns/s = rpm / 60
  - Backend writes `axis.controller.input_vel` and requests `AxisState.CLOSED_LOOP_CONTROL`
  - connect() applies a safe baseline once:
      IDLE, disable startup flags (when present), conservative current limits,
      and velocity-control controller settings when available
  - stop() zeros velocity command and transitions to IDLE
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
import os
import sqlite3
import sys
import time
import uuid
import webbrowser
from dataclasses import dataclass, asdict, replace
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

try:
    from odrive_configuration import HMI_DEFAULT_CONFIG_VALUES, ODRIVE_MOTOR_PRESETS, SCRIPT_DEFAULT_ASSIGNMENTS
except Exception:
    HMI_DEFAULT_CONFIG_VALUES = {}
    ODRIVE_MOTOR_PRESETS = {}
    SCRIPT_DEFAULT_ASSIGNMENTS = []

from PySide6.QtCore import (
    QObject,
    QEvent,
    Qt,
    QThread,
    QTimer,
    QStandardPaths,
    Signal,
    Slot,
    QMetaObject,
    QDateTime,
    QSignalBlocker,
)
from PySide6.QtGui import QDoubleValidator, QFont, QIntValidator, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QScroller,
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
START_FULLSCREEN = True

WINDOW_W = 1280
WINDOW_H = 800

VEL_MIN_RPM = 0.0
VEL_MAX_RPM = 5000.0  # absolute UI limit; user-configurable HMI max defaults lower
VEL_STEP_RPM = 1.0
HMI_MAX_VEL_DEFAULT_RPM = 2400.0

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
TAG_PWR_W = "p_w"        # computed vbus*ibus
TAG_CMD_RPM = "cmd_rpm"  # last commanded
TAG_VEL_RPM = "vel_rpm"  # actual estimate
TAG_TORQUE_NM = "torque_nm"  # actual/estimated torque
TAG_MOTOR_TEMP_C = "motor_temp_c"  # motor thermistor temperature

DEFAULT_ENABLED_TAGS = [TAG_PWR_W, TAG_CMD_RPM]

TAG_DEADBAND: Dict[str, float] = {
    TAG_VBUS_V: 0.1,
    TAG_IBUS_A: 0.05,
    TAG_PWR_W: 0.05,
    TAG_CMD_RPM: 0.1,
    TAG_VEL_RPM: 0.1,
    TAG_TORQUE_NM: 0.01,
    TAG_MOTOR_TEMP_C: 0.2,
}
TAG_HEARTBEAT_S: Dict[str, int] = {t: DB_HEARTBEAT_S_DEFAULT for t in TAG_DEADBAND}

TAG_LABELS: Dict[str, str] = {
    TAG_PWR_W: "Electrical input power (W)",
    TAG_CMD_RPM: "Commanded RPM",
    TAG_VEL_RPM: "Actual velocity RPM",
    TAG_TORQUE_NM: "Actual torque (Nm)",
    TAG_VBUS_V: "Bus voltage (V)",
    TAG_IBUS_A: "Bus current (A)",
    TAG_MOTOR_TEMP_C: "Motor thermistor temperature (C)",
}
TAG_UNITS: Dict[str, str] = {
    TAG_PWR_W: "W",
    TAG_CMD_RPM: "rpm",
    TAG_VEL_RPM: "rpm",
    TAG_TORQUE_NM: "Nm",
    TAG_VBUS_V: "V",
    TAG_IBUS_A: "A",
    TAG_MOTOR_TEMP_C: "C",
}
PLOT_GROUP_RPM = "rpm"
PLOT_GROUP_OTHER = "other"
TAG_PLOT_GROUP: Dict[str, str] = {
    TAG_CMD_RPM: PLOT_GROUP_RPM,
    TAG_VEL_RPM: PLOT_GROUP_RPM,
    TAG_TORQUE_NM: PLOT_GROUP_OTHER,
    TAG_PWR_W: PLOT_GROUP_OTHER,
    TAG_VBUS_V: PLOT_GROUP_OTHER,
    TAG_IBUS_A: PLOT_GROUP_OTHER,
    TAG_MOTOR_TEMP_C: PLOT_GROUP_OTHER,
}

# DPI/UI scaling: keep fixed window size but scale fonts/margins so content remains touch-friendly
UI_SCALE = 1.0


def ui(px: float) -> int:
    return int(round(float(px) * float(UI_SCALE)))


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


def enable_touch_scrolling(widget: QWidget) -> None:
    """
    Enable single-finger kinetic scrolling on touchscreens.
    Use the viewport for scroll areas/list widgets so gestures are captured where content is drawn.
    """
    try:
        target = widget.viewport() if hasattr(widget, "viewport") else widget  # type: ignore[assignment]
        if not isinstance(target, QWidget):
            return
        target.setAttribute(Qt.WA_AcceptTouchEvents, True)
        QScroller.grabGesture(target, QScroller.LeftMouseButtonGesture)
    except Exception:
        pass


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
    velocity_rpm: float = 0.0
    duration_s: int = 0
    name: str = ""
    kind: str = "motor"  # "motor" | "data_entry"
    prompt: str = ""
    response_kind: str = "text"  # "text" | "numeric" | "pass_fail"
    allow_skip: bool = True


def is_data_entry_step(step: Step) -> bool:
    return str(getattr(step, "kind", "motor") or "motor") == "data_entry"


@dataclass
class Recipe:
    id: str
    name: str
    steps: List[Step]


def recipe_total_seconds(r: Recipe) -> int:
    return sum(max(0, int(s.duration_s)) for s in r.steps if not is_data_entry_step(s))


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
                steps = []
                for i, s in enumerate(rj.get("steps", []), start=1):
                    kind = str(s.get("kind") or "motor")
                    steps.append(
                        Step(
                            float(s.get("velocity_rpm", 0.0)),
                            int(s.get("duration_s", 0)),
                            str(s.get("name") or f"Step {i}"),
                            kind=kind,
                            prompt=str(s.get("prompt") or ""),
                            response_kind=str(s.get("response_kind") or "text"),
                            allow_skip=bool(s.get("allow_skip", True)),
                        )
                    )
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

    def delete(self, recipe_id: str) -> bool:
        rid = str(recipe_id)
        old_n = len(self._recipes)
        self._recipes = [r for r in self._recipes if str(r.id) != rid]
        changed = len(self._recipes) != old_n
        if changed:
            self.save()
        return bool(changed)

    @staticmethod
    def _seed() -> List[Recipe]:
        return [
            Recipe(
                id=str(uuid.uuid4()),
                name="Gentle Mix",
                steps=[
                    Step(120.0, 60, "Step 1"),
                    Step(200.0, 120, "Step 2"),
                    Step(120.0, 60, "Step 3"),
                ],
            )
        ]


class RecipeRunStore:
    def __init__(self) -> None:
        self._path = app_config_dir() / "recipe_runs.json"
        self._runs: List[Dict[str, Any]] = []

    def load(self) -> None:
        if not self._path.exists():
            self._runs = []
            return
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            self._runs = [dict(x) for x in list(data.get("runs", []))]
        except Exception:
            self._runs = []

    def save(self) -> None:
        try:
            self._path.write_text(json.dumps({"runs": self._runs}, indent=2), encoding="utf-8")
        except Exception:
            pass

    def add_run(self, run: Dict[str, Any]) -> None:
        self._runs.insert(0, dict(run))
        self._runs = self._runs[:200]
        self.save()

    def runs(self) -> List[Dict[str, Any]]:
        return list(self._runs)


# ----------------------------
# ODrive Configuration Schema
# ----------------------------

@dataclass(frozen=True)
class SettingSpec:
    key: str
    label: str
    kind: str  # "bool", "float", "int", or "protocol"
    default: Any
    min_value: Optional[float] = None
    max_value: Optional[float] = None


ODRIVE_CONFIG_FIELDS: List[SettingSpec] = [
    SettingSpec("axis0.config.startup_motor_calibration", "Startup: motor calibration", "bool", False),
    SettingSpec("axis0.config.startup_encoder_offset_calibration", "Startup: encoder offset calibration", "bool", False),
    SettingSpec("axis0.config.startup_encoder_index_search", "Startup: encoder index search", "bool", False),
    SettingSpec("axis0.config.startup_closed_loop_control", "Startup: closed loop control", "bool", False),

    SettingSpec("axis0.config.motor.current_soft_max", "Motor: current soft max (A)", "float", 20.0, 0.0, 60.0),
    SettingSpec("axis0.config.motor.current_hard_max", "Motor: current hard max (A)", "float", 30.0, 0.0, 100.0),

    SettingSpec("axis0.config.general_lockin.current", "Lock-in: current (A)", "float", 6.0, 0.0, 60.0),
    SettingSpec("axis0.config.general_lockin.ramp_time", "Lock-in: ramp time (s)", "float", 1.0, 0.0, 30.0),
    SettingSpec("axis0.config.general_lockin.ramp_distance", "Lock-in: ramp distance (turns)", "float", 1.0, 0.0, 100.0),
    SettingSpec("axis0.config.general_lockin.accel", "Lock-in: accel (turn/s²)", "float", 20.0, 0.0, 2000.0),
    SettingSpec("axis0.config.general_lockin.finish_on_vel", "Lock-in: finish on vel", "bool", False),
    SettingSpec("axis0.config.general_lockin.finish_on_distance", "Lock-in: finish on distance", "bool", False),

    SettingSpec("config.dc_max_positive_current", "DC bus: max positive current (A)", "float", 25.0, 0.0, 120.0),
    SettingSpec("config.dc_max_negative_current", "DC bus: max regen current (A, negative)", "float", -2.0, -120.0, 0.0),
    SettingSpec("config.dc_bus_undervoltage_trip_level", "DC bus: undervoltage trip (V)", "float", 10.0, 0.0, 60.0),
    SettingSpec("config.dc_bus_overvoltage_trip_level", "DC bus: overvoltage trip (V)", "float", 30.0, 0.0, 80.0),
    SettingSpec("config.brake_resistor0.enable", "Brake resistor: enable", "bool", True),
    SettingSpec("config.brake_resistor0.resistance", "Brake resistor: resistance (ohm)", "float", 2.0, 0.1, 50.0),

    SettingSpec("axis0.config.motor.pole_pairs", "Motor: pole pairs", "int", 7, 1, 50),
    SettingSpec("axis0.config.motor.torque_constant", "Motor: torque constant (Nm/A)", "float", 0.0306, 0.0001, 1.0),
    SettingSpec("axis0.config.motor.calibration_current", "Motor: calibration current (A)", "float", 10.0, 0.0, 100.0),
    SettingSpec("axis0.config.motor.resistance_calib_max_voltage", "Motor: resistance calib max voltage (V)", "float", 2.0, 0.1, 30.0),
    SettingSpec("axis0.config.calibration_lockin.current", "Calibration lock-in: current (A)", "float", 10.0, 0.0, 100.0),

    SettingSpec("axis0.config.motor.current_control_bandwidth", "Motor: current control bandwidth", "float", 1000.0, 10.0, 5000.0),
    SettingSpec("axis0.controller.config.vel_limit", "Controller: velocity limit (turn/s)", "float", 10.0, 0.1, 200.0),
    SettingSpec("axis0.controller.config.vel_limit_tolerance", "Controller: vel limit tolerance", "float", 1.2, 1.0, 5.0),
    SettingSpec("axis0.controller.config.vel_ramp_rate", "Controller: vel ramp rate (turn/s^2)", "float", 10.0, 0.0, 2000.0),

    SettingSpec("inc_encoder0.config.enabled", "Encoder: incremental enabled", "bool", True),
    SettingSpec("inc_encoder0.config.cpr", "Encoder: incremental CPR", "int", 8192, 1, 200000),

    SettingSpec("axis0.config.enable_watchdog", "Interface: enable watchdog", "bool", False),
    SettingSpec("axis0.config.watchdog_timeout", "Interface: watchdog timeout (s)", "float", 1.0, 0.1, 10.0),
    SettingSpec("can.config.protocol", "CAN: protocol", "protocol", "Protocol.NONE"),
    SettingSpec("can.config.baud_rate", "CAN: baud rate", "int", 250000, 10000, 1000000),
    SettingSpec("axis0.config.can.node_id", "CAN: node ID", "int", 63, 0, 63),
    SettingSpec("axis0.config.can.heartbeat_msg_rate_ms", "CAN: heartbeat msg rate (ms)", "int", 100, 0, 10000),
    SettingSpec("axis0.config.can.encoder_msg_rate_ms", "CAN: encoder msg rate (ms)", "int", 0, 0, 10000),
    SettingSpec("axis0.config.can.iq_msg_rate_ms", "CAN: iq msg rate (ms)", "int", 0, 0, 10000),
    SettingSpec("axis0.config.can.torques_msg_rate_ms", "CAN: torques msg rate (ms)", "int", 0, 0, 10000),
    SettingSpec("axis0.config.can.error_msg_rate_ms", "CAN: error msg rate (ms)", "int", 0, 0, 10000),
    SettingSpec("axis0.config.can.temperature_msg_rate_ms", "CAN: temperature msg rate (ms)", "int", 0, 0, 10000),
    SettingSpec("axis0.config.can.bus_voltage_msg_rate_ms", "CAN: bus voltage msg rate (ms)", "int", 0, 0, 10000),
    SettingSpec("config.enable_uart_a", "UART A: enable", "bool", False),
]
ODRIVE_CONFIG_FIELDS = [
    replace(_spec, default=HMI_DEFAULT_CONFIG_VALUES.get(_spec.key, _spec.default))
    for _spec in ODRIVE_CONFIG_FIELDS
]
DEFAULT_CONFIG_MAP: Dict[str, Any] = {s.key: s.default for s in ODRIVE_CONFIG_FIELDS}


# ----------------------------
# Backends
# ----------------------------

class OdriveInterface:
    def connect(self) -> bool: raise NotImplementedError
    def is_connected(self) -> bool: raise NotImplementedError
    def feed_watchdog(self) -> str: raise NotImplementedError
    def get_runtime_fault(self) -> str: raise NotImplementedError
    def set_velocity_rpm(self, rpm: float) -> None: raise NotImplementedError
    def stop(self) -> None: raise NotImplementedError
    def is_calibrated(self) -> Tuple[bool, str]: raise NotImplementedError
    def calibrate(self) -> Tuple[bool, str]: raise NotImplementedError
    def get_state(self) -> str: raise NotImplementedError
    def read_config(self, keys: List[str]) -> Tuple[Dict[str, Any], List[str]]: raise NotImplementedError
    def apply_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]: raise NotImplementedError
    def save_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]: raise NotImplementedError
    def erase_config(self) -> Tuple[bool, str]: raise NotImplementedError
    def clear_errors(self) -> Tuple[bool, str]: raise NotImplementedError
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

    def feed_watchdog(self) -> str:
        # Simulation backend has no watchdog; no-op for API compatibility.
        return "Simulation"

    def get_runtime_fault(self) -> str:
        return ""

    def set_velocity_rpm(self, rpm: float) -> None:
        self._target_rpm = float(rpm)
        self._state = "Running" if abs(self._target_rpm) > 1e-6 else "Idle"

    def stop(self) -> None:
        self._target_rpm = 0.0
        self._state = "Idle"

    def is_calibrated(self) -> Tuple[bool, str]:
        return True, "Simulation backend"

    def calibrate(self) -> Tuple[bool, str]:
        self._state = "Idle"
        return True, "Simulation calibration complete"

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
        return True, "Simulation config applied (not saved)", missing

    def save_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]:
        ok, _msg, missing = self.apply_config(values)
        return ok, "Simulation config saved (no reboot)", missing

    def erase_config(self) -> Tuple[bool, str]:
        self._config = dict(DEFAULT_CONFIG_MAP)
        self._state = "Idle"
        return True, "Simulation config reset to defaults"

    def clear_errors(self) -> Tuple[bool, str]:
        self._state = "Idle"
        return True, "Simulation errors cleared"

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
        torque_nm = 0.02 * ibus + 0.0005 * rpm_mag
        motor_temp_c = 25.0 + min(45.0, rpm_mag * 0.02 + ibus * 1.5)
        out: Dict[str, float] = {}
        for t in tags:
            if t == TAG_VBUS_V:
                out[t] = float(self._vbus)
            elif t == TAG_IBUS_A:
                out[t] = float(ibus)
            elif t == TAG_VEL_RPM:
                out[t] = float(self._actual_rpm)
            elif t == TAG_TORQUE_NM:
                out[t] = float(torque_nm)
            elif t == TAG_MOTOR_TEMP_C:
                out[t] = float(motor_temp_c)
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
            from odrive.enums import AxisState, ControlMode, InputMode

            self._AxisState = AxisState
            self._state = "Connecting..."
            self._odrv = odrive.find_any(timeout=FIND_ANY_TIMEOUT_S)
            if self._odrv is None:
                self._mark_disconnected("Disconnected (reconnecting)")
                return False

            self._axis = self._odrv.axis0
            self._axis.requested_state = AxisState.IDLE

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

            try:
                self._axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
            except Exception:
                pass
            try:
                self._axis.controller.config.input_mode = InputMode.VEL_RAMP
            except Exception:
                pass
            try:
                self._axis.controller.input_vel = 0.0
            except Exception:
                pass

            self._connected = True
            self._state = "Idle (velocity ready)"
            return True
        except Exception:
            self._mark_disconnected("Disconnected (reconnecting)")
            return False

    def is_connected(self) -> bool:
        return self._connected

    def feed_watchdog(self) -> str:
        if not self._connected or self._axis is None:
            return "Disconnected"
        try:
            if not bool(getattr(self._axis.config, "enable_watchdog")):
                return "Disabled"
        except Exception:
            return "Unknown"
        try:
            feed = getattr(self._axis, "watchdog_feed", None)
            if callable(feed):
                feed()
                return "Fed"
            return "Unsupported"
        except Exception:
            # Avoid interrupting normal operation on firmware variants that do not
            # expose watchdog feed in the same way.
            return "Feed error"

    def get_runtime_fault(self) -> str:
        if not self._connected or self._axis is None:
            return ""
        runtime_fault = False
        parts: List[str] = []
        probes = [
            ("odrv.active_errors", (lambda: getattr(self._odrv, "active_errors")) if self._odrv is not None else None),
            ("axis.active_errors", lambda: getattr(self._axis, "active_errors")),
            ("axis.disarm_reason", lambda: getattr(self._axis, "disarm_reason")),
            ("axis.procedure_result", lambda: getattr(self._axis, "procedure_result")),
            ("motor.active_errors", lambda: getattr(self._axis.motor, "active_errors")),
            ("encoder.active_errors", lambda: getattr(self._axis.encoder, "active_errors")),
        ]
        for name, fn in probes:
            if fn is None:
                continue
            try:
                val = fn()
            except Exception:
                continue
            sval = self._format_runtime_error_value(name, val)
            if not sval:
                continue
            if name != "axis.procedure_result" and self._is_active_fault_value(name, sval, val):
                runtime_fault = True
            if sval not in ("0", "ODriveError.NONE", "AxisError.NONE", "ProcedureResult.SUCCESS"):
                parts.append(f"{name}={sval}")
        if not runtime_fault:
            return ""
        return f"ODrive runtime fault: {'; '.join(parts)}"

    @staticmethod
    def _is_active_fault_value(name: str, sval: str, raw_val: Any) -> bool:
        if name == "axis.procedure_result":
            return False
        txt = str(sval or "").strip().lower()
        if not txt:
            return False
        if txt in {"0", "none", "no error", "axiserror.none", "odriveerror.none", "motorerror.none", "encodererror.none"}:
            return False
        if "no error" in txt:
            return False
        if "error(s):" in txt and "none" in txt and "violation" not in txt:
            return False
        try:
            iv = int(raw_val)
            if iv == 0:
                return False
        except Exception:
            pass
        return True

    def set_velocity_rpm(self, rpm: float) -> None:
        if not self._connected or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")
        rpm = float(rpm)
        if abs(rpm) < 1e-6:
            self.stop()
            return
        turns_per_s = rpm / 60.0
        # Some firmware/hardware combinations ignore the first input_vel command if it is
        # written before the axis finishes entering CLOSED_LOOP_CONTROL. Request state first,
        # then write the target (twice) so the very first Start command moves the motor.
        self._axis.requested_state = self._AxisState.CLOSED_LOOP_CONTROL
        self._axis.controller.input_vel = float(turns_per_s)
        self._axis.controller.input_vel = float(turns_per_s)
        self._state = f"CLOSED_LOOP_CONTROL ({rpm:.0f} rpm)"

    def stop(self) -> None:
        if not self._connected or self._axis is None or self._AxisState is None:
            self._state = "Disconnected"
            return
        try:
            try:
                self._axis.controller.input_vel = 0.0
            except Exception:
                pass
            self._axis.requested_state = self._AxisState.IDLE
        finally:
            self._state = "Idle"

    def _calibration_status(self) -> Tuple[bool, str]:
        if not self._connected or self._axis is None:
            raise RuntimeError("ODrive not connected")

        # Newer ODrive object models may not expose axis.encoder or motor.is_calibrated.
        # On those firmwares, a valid commutation mapper offset is a practical indicator
        # that calibration completed successfully for velocity control.
        try:
            active_errors = int(getattr(self._axis, "active_errors"))
        except Exception:
            active_errors = 0
        try:
            disarm_reason = int(getattr(self._axis, "disarm_reason"))
        except Exception:
            disarm_reason = 0

        motor_ok = False
        try:
            motor_ok = bool(getattr(self._axis.motor, "is_calibrated"))
        except Exception:
            try:
                motor_ok = bool(getattr(self._axis.motor.config, "pre_calibrated"))
            except Exception:
                motor_ok = False
        if not motor_ok:
            try:
                motor_ok = bool(getattr(self._axis.commutation_mapper.config, "offset_valid"))
            except Exception:
                motor_ok = False

        encoder_ready = True
        try:
            encoder_ready = bool(getattr(self._axis.encoder, "is_ready"))
        except Exception:
            # Some firmware variants expose calibration state differently; motor calibration is the minimum gate.
            encoder_ready = True
        # If the newer mapper-based API exists, accept commutation mapper validity as sufficient
        # for this app's velocity-control gating. pos_vel_mapper offset may remain invalid and
        # closed-loop velocity can still work (as observed on some firmware/hardware combinations).
        if motor_ok and active_errors == 0 and disarm_reason == 0:
            return True, "Motor calibrated"

        if motor_ok and encoder_ready:
            return True, "Motor calibrated"
        if not motor_ok and not encoder_ready:
            return False, "Motor and encoder are not calibrated"
        if not motor_ok:
            return False, "Motor is not calibrated"
        return False, "Encoder is not calibrated"

    def is_calibrated(self) -> Tuple[bool, str]:
        return self._calibration_status()

    def _error_summary(self) -> str:
        if not self._connected or self._axis is None:
            return ""
        parts: List[str] = []
        probes = [
            ("axis.active_errors", lambda: getattr(self._axis, "active_errors")),
            ("axis.disarm_reason", lambda: getattr(self._axis, "disarm_reason")),
            ("axis.procedure_result", lambda: getattr(self._axis, "procedure_result")),
            ("motor.active_errors", lambda: getattr(self._axis.motor, "active_errors")),
            ("encoder.active_errors", lambda: getattr(self._axis.encoder, "active_errors")),
        ]
        if self._odrv is not None:
            probes.insert(0, ("odrv.active_errors", lambda: getattr(self._odrv, "active_errors")))
        for name, fn in probes:
            try:
                val = fn()
            except Exception:
                continue
            sval = self._format_runtime_error_value(name, val)
            if sval and sval not in ("0", "ODriveError.NONE", "AxisError.NONE", "ProcedureResult.SUCCESS"):
                parts.append(f"{name}={sval}")
        return "; ".join(parts)

    @staticmethod
    def _format_runtime_error_value(name: str, val: Any) -> str:
        try:
            sval = str(val)
        except Exception:
            return ""
        if not sval:
            return ""
        # If firmware already returns a symbolic enum string, keep it.
        if "." in sval and not sval.isdigit():
            return sval
        try:
            iv = int(val)
        except Exception:
            return sval
        enum_candidates: List[str] = []
        if name == "axis.procedure_result":
            enum_candidates = ["ProcedureResult"]
        elif name == "axis.disarm_reason":
            enum_candidates = ["ODriveError"]
        elif name == "axis.active_errors":
            enum_candidates = ["AxisError", "ODriveError"]
        elif name == "motor.active_errors":
            enum_candidates = ["MotorError"]
        elif name == "encoder.active_errors":
            enum_candidates = ["EncoderError"]
        elif name == "odrv.active_errors":
            enum_candidates = ["ODriveError"]
        if not enum_candidates:
            return sval
        try:
            from odrive import enums as odrive_enums  # type: ignore
        except Exception:
            return sval
        for enum_name in enum_candidates:
            enum_type = getattr(odrive_enums, enum_name, None)
            if enum_type is None:
                continue
            try:
                resolved = enum_type(iv)
                rtxt = str(resolved)
                if rtxt and rtxt != str(iv):
                    return rtxt
                # Some enum/flag types stringify to the raw number even when valid.
                if hasattr(resolved, "name") and getattr(resolved, "name", None):
                    return f"{enum_name}.{resolved.name}"
            except Exception:
                continue
        return sval

    def _clear_errors(self) -> None:
        if self._axis is None:
            return
        for obj in (self._axis, self._odrv):
            if obj is None:
                continue
            try:
                clr = getattr(obj, "clear_errors", None)
                if callable(clr):
                    clr()
            except Exception:
                pass

    def clear_errors(self) -> Tuple[bool, str]:
        if not self._connected:
            return False, "ODrive not connected"
        try:
            self._clear_errors()
            try:
                if self._AxisState is not None and self._axis is not None:
                    self._axis.requested_state = self._AxisState.IDLE
            except Exception:
                pass
            self._state = "Idle (errors cleared)"
            return True, "ODrive errors cleared"
        except Exception as e:
            return False, f"Failed to clear errors: {e}"

    def _wait_axis_idle(self, timeout_s: float) -> bool:
        if self._axis is None or self._AxisState is None:
            return False
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            try:
                cur = getattr(self._axis, "current_state")
                if int(cur) == int(self._AxisState.IDLE):
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False

    def _run_state_sequence(self, state_name: str, timeout_s: float) -> Tuple[bool, str]:
        if self._axis is None or self._AxisState is None:
            return False, "ODrive not connected"
        if not hasattr(self._AxisState, state_name):
            return False, f"{state_name} is not available on this ODrive firmware"
        try:
            self._axis.requested_state = getattr(self._AxisState, state_name)
        except Exception as e:
            return False, f"Failed to request {state_name}: {e}"
        if not self._wait_axis_idle(timeout_s):
            return False, f"{state_name} timed out"
        return True, f"{state_name} complete"

    def calibrate(self) -> Tuple[bool, str]:
        if not self._connected or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")

        self._state = "Calibrating..."
        old_soft_max: Optional[float] = None
        old_hard_max: Optional[float] = None
        old_dc_max_positive_current: Optional[float] = None
        try:
            self._clear_errors()
            # Calibration can legitimately require more current than the conservative
            # runtime limits used after connect(). Raise them temporarily if needed.
            try:
                motor_cfg = self._axis.config.motor
                cal_i = float(getattr(motor_cfg, "calibration_current"))
                old_soft_max = float(getattr(motor_cfg, "current_soft_max"))
                old_hard_max = float(getattr(motor_cfg, "current_hard_max"))
                req_soft = max(old_soft_max, cal_i + 1.0)
                req_hard = max(old_hard_max, cal_i + 3.0)
                if req_hard <= req_soft:
                    req_hard = req_soft + 1.0
                motor_cfg.current_soft_max = req_soft
                motor_cfg.current_hard_max = req_hard
            except Exception:
                pass
            try:
                if self._odrv is not None:
                    old_dc_max_positive_current = float(getattr(self._odrv.config, "dc_max_positive_current"))
                    # Bus current during calibration can exceed conservative runtime limits.
                    req_dc_pos = max(old_dc_max_positive_current, 30.0, cal_i * 2.0)  # type: ignore[name-defined]
                    self._odrv.config.dc_max_positive_current = float(req_dc_pos)
            except Exception:
                pass
            try:
                self._axis.controller.input_vel = 0.0
            except Exception:
                pass
            self._axis.requested_state = self._AxisState.IDLE
            time.sleep(0.1)

            ran_any = False
            errs: List[str] = []
            if hasattr(self._AxisState, "FULL_CALIBRATION_SEQUENCE"):
                ran_any = True
                ok_full, msg_full = self._run_state_sequence("FULL_CALIBRATION_SEQUENCE", 120.0)
                if not ok_full:
                    errs.append(msg_full)
            else:
                for state_name, timeout_s in (
                    ("MOTOR_CALIBRATION", 60.0),
                    ("ENCODER_OFFSET_CALIBRATION", 60.0),
                ):
                    if hasattr(self._AxisState, state_name):
                        ran_any = True
                        ok_seq, msg_seq = self._run_state_sequence(state_name, timeout_s)
                        if not ok_seq:
                            errs.append(msg_seq)
                            break
            if not ran_any:
                self._state = "Idle"
                return False, "No supported calibration states found on this ODrive firmware"

            ok, msg = self._calibration_status()
            self._state = "Idle (velocity ready)" if ok else "Idle (not calibrated)"
            if ok:
                return True, "Calibration complete"
            detail = f"Calibration incomplete: {msg}"
            err_text = self._error_summary()
            if errs:
                detail += f" | Sequence: {'; '.join(errs)}"
            if err_text:
                detail += f" | ODrive: {err_text}"
            return False, detail
        except Exception as e:
            self._state = "Idle"
            raise RuntimeError(f"Calibration failed: {e}")
        finally:
            try:
                if old_soft_max is not None:
                    self._axis.config.motor.current_soft_max = float(old_soft_max)
                if old_hard_max is not None:
                    self._axis.config.motor.current_hard_max = float(old_hard_max)
                if old_dc_max_positive_current is not None and self._odrv is not None:
                    self._odrv.config.dc_max_positive_current = float(old_dc_max_positive_current)
            except Exception:
                pass

    def get_state(self) -> str:
        return self._state

    @staticmethod
    def _get_attr_path(root: Any, key: str) -> Any:
        obj = root
        for part in key.split("."):
            obj = getattr(obj, part)
        return obj

    @staticmethod
    def _resolve_config_value(value: Any) -> Any:
        if not isinstance(value, str) or "." not in value:
            return value
        enum_type_name, enum_member = value.split(".", 1)
        try:
            from odrive import enums as odrive_enums  # type: ignore
            enum_type = getattr(odrive_enums, enum_type_name, None)
            if enum_type is None:
                return value
            return getattr(enum_type, enum_member, value)
        except Exception:
            return value

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
        missing = self._apply_config_values(values)
        self._state = "Config applied"
        msg = "Config applied (not saved)"
        if missing:
            msg = f"Config applied; {len(missing)} unsupported field(s) skipped"
        return True, msg, missing

    def save_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]:
        if not self._connected or self._odrv is None or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")
        missing = self._apply_config_values(values)
        msg = "Config saved (device rebooting)"
        try:
            self._state = "Saving config (rebooting)..."
            self._odrv.save_configuration()
        except Exception:
            pass
        self._mark_disconnected("Rebooting (reconnecting)")
        if missing:
            msg = f"Config saved (device rebooting); {len(missing)} unsupported field(s) skipped"
        return True, msg, missing

    def erase_config(self) -> Tuple[bool, str]:
        if not self._connected or self._odrv is None or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")
        try:
            self._axis.requested_state = self._AxisState.IDLE
        except Exception:
            pass
        if not hasattr(self._odrv, "erase_configuration"):
            return False, "erase_configuration() is not available on this ODrive firmware"
        try:
            self._state = "Erasing config (rebooting)..."
            self._odrv.erase_configuration()
        except Exception as e:
            raise RuntimeError(f"Erase config failed: {e}")
        self._mark_disconnected("Rebooting (reconnecting)")
        return True, "Configuration erased (device rebooting)"

    def _apply_config_values(self, values: Dict[str, Any]) -> List[str]:
        if not self._connected or self._odrv is None or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")
        missing: List[str] = []
        try:
            self._axis.requested_state = self._AxisState.IDLE
        except Exception:
            pass
        for k, v in values.items():
            try:
                self._set_attr_path(self._odrv, k, self._resolve_config_value(v))
            except Exception:
                missing.append(k)
        return missing

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
                elif t == TAG_TORQUE_NM:
                    torque_val = None
                    # Firmware variants expose torque estimate/setpoint on different objects.
                    for getter in (
                        lambda: getattr(self._axis.controller, "torque_estimate"),
                        lambda: getattr(self._axis.controller, "effective_torque_setpoint"),
                        lambda: getattr(self._axis.controller, "torque_setpoint"),
                        lambda: getattr(self._axis, "torque_estimate"),
                        lambda: getattr(self._axis.motor, "torque_estimate"),
                    ):
                        try:
                            torque_val = float(getter())
                            break
                        except Exception:
                            continue
                    # Fallback: estimate torque from q-axis current * torque constant.
                    if torque_val is None:
                        iq_val = None
                        for getter in (
                            lambda: getattr(getattr(self._axis.motor, "foc"), "Iq_measured"),
                            lambda: getattr(getattr(self._axis.motor, "foc"), "Iq_setpoint"),
                            lambda: getattr(getattr(self._axis.motor, "current_control"), "Iq_measured"),
                            lambda: getattr(getattr(self._axis.motor, "current_control"), "Iq_setpoint"),
                        ):
                            try:
                                iq_val = float(getter())
                                break
                            except Exception:
                                continue
                        if iq_val is not None:
                            kt = None
                            for getter in (
                                lambda: getattr(getattr(self._axis.config, "motor"), "torque_constant"),
                                lambda: getattr(self._axis.motor, "torque_constant"),
                            ):
                                try:
                                    kt = float(getter())
                                    break
                                except Exception:
                                    continue
                            if kt is not None and math.isfinite(kt):
                                torque_val = float(iq_val * kt)
                    if torque_val is not None and math.isfinite(torque_val):
                        out[t] = float(torque_val)
                elif t == TAG_MOTOR_TEMP_C:
                    temp_val = None
                    for getter in (
                        lambda: getattr(getattr(self._axis.motor, "motor_thermistor"), "temperature"),
                        lambda: getattr(getattr(self._axis.motor, "motor_thermistor"), "temp"),
                        lambda: getattr(getattr(self._axis.motor, "fet_thermistor"), "temperature"),
                    ):
                        try:
                            temp_val = float(getter())
                            break
                        except Exception:
                            continue
                    if temp_val is not None and math.isfinite(temp_val):
                        out[t] = float(temp_val)
            except Exception:
                pass
        return out

    def get_json(self) -> str:
        if not self._connected or self._odrv is None:
            raise RuntimeError("ODrive not connected")
        try:
            getter = getattr(self._odrv, "get_json", None)
            if callable(getter):
                return str(getter())
        except Exception:
            pass

        # Firmware/API fallback: export a practical snapshot when get_json() is unavailable.
        snap: Dict[str, Any] = {
            "export_kind": "odrive_snapshot_fallback",
            "reason": "get_json() not available on this ODrive API object",
            "backend_state": self._state,
        }
        try:
            snap["vbus_voltage"] = float(getattr(self._odrv, "vbus_voltage"))
        except Exception:
            pass
        try:
            snap["ibus"] = float(getattr(self._odrv, "ibus"))
        except Exception:
            pass
        try:
            snap["axis0"] = {
                "current_state": int(getattr(self._axis, "current_state")) if self._axis is not None else None,
                "active_errors": str(getattr(self._axis, "active_errors")) if self._axis is not None else None,
                "disarm_reason": str(getattr(self._axis, "disarm_reason")) if self._axis is not None else None,
                "procedure_result": str(getattr(self._axis, "procedure_result")) if self._axis is not None else None,
            }
        except Exception:
            pass

        try:
            keys = [s.key for s in ODRIVE_CONFIG_FIELDS]
            values, missing = self.read_config(keys)
            # Make sure everything is JSON serializable.
            clean_values: Dict[str, Any] = {}
            for k, v in values.items():
                if isinstance(v, (str, int, float, bool)) or v is None:
                    clean_values[k] = v
                else:
                    clean_values[k] = str(v)
            snap["config_fields"] = clean_values
            snap["unsupported_settings"] = list(missing)
        except Exception as e:
            snap["config_read_error"] = str(e)

        return json.dumps(snap, indent=2)


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
    config_saved = Signal(bool, str, object)      # ok, message, missing list
    config_erased = Signal(bool, str)             # ok, message
    export_json_done = Signal(bool, str)          # ok, path/message
    errors_cleared = Signal(bool, str)            # ok, message
    tags_sample = Signal(int, object)             # ts, dict
    calibration_checked = Signal(bool, str)       # calibrated, detail
    calibration_done = Signal(bool, str)          # calibrated, detail
    watchdog_status_changed = Signal(str)         # watchdog feed/runtime status

    def __init__(self, backend: OdriveInterface) -> None:
        super().__init__()
        self._backend = backend
        self._poll_timer: Optional[QTimer] = None
        self._fault_poll_timer: Optional[QTimer] = None
        self._polled_tags: List[str] = [TAG_VBUS_V, TAG_IBUS_A, TAG_VEL_RPM, TAG_TORQUE_NM]
        self._supervisor_timer: Optional[QTimer] = None

        self._connecting = False
        self._last_connect_attempt_mono = 0.0
        self._connected_cache = False
        self._state_cache = "Disconnected"

        self._last_poll_error_mono = 0.0
        self._poll_error_cooldown_s = 5.0
        self._watchdog_status_cache = "Unknown"
        self._last_runtime_fault_msg = ""

    @Slot()
    def on_thread_started(self) -> None:
        if self._supervisor_timer is None:
            self._supervisor_timer = QTimer(self)
            self._supervisor_timer.setInterval(SUPERVISOR_TICK_MS)
            self._supervisor_timer.timeout.connect(self._supervise_connection)
            self._supervisor_timer.start()
        if self._fault_poll_timer is None:
            self._fault_poll_timer = QTimer(self)
            self._fault_poll_timer.setInterval(100)  # non-invasive runtime fault polling
            self._fault_poll_timer.timeout.connect(self._poll_runtime_fault)
            self._fault_poll_timer.start()

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

    def _emit_watchdog_status(self, status: str) -> None:
        status = str(status or "Unknown")
        if status != self._watchdog_status_cache:
            self._watchdog_status_cache = status
            self.watchdog_status_changed.emit(status)

    def _poll_runtime_fault(self) -> None:
        if not self._backend.is_connected():
            self._last_runtime_fault_msg = ""
            return
        try:
            msg = str(self._backend.get_runtime_fault() or "").strip()
        except Exception:
            return
        if not msg:
            self._last_runtime_fault_msg = ""
            return
        if msg == self._last_runtime_fault_msg:
            return
        self._last_runtime_fault_msg = msg
        self.error.emit(msg)

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
        self._emit_watchdog_status("Disconnected")
        self._last_runtime_fault_msg = ""

    @Slot()
    def _supervise_connection(self) -> None:
        try:
            if self._backend.is_connected():
                try:
                    self._emit_watchdog_status(self._backend.feed_watchdog())
                except Exception:
                    pass
                self._poll_runtime_fault()
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

    @Slot()
    def check_calibration(self) -> None:
        if not self._backend.is_connected():
            self.calibration_checked.emit(False, "ODrive not connected")
            return
        try:
            calibrated, detail = self._backend.is_calibrated()
            self.calibration_checked.emit(bool(calibrated), str(detail))
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.calibration_checked.emit(False, "Disconnected (reconnecting)")
            else:
                self.error.emit(f"Calibration check failed: {e}")
                self.calibration_checked.emit(False, str(e))

    @Slot()
    def run_calibration(self) -> None:
        if not self._backend.is_connected():
            self.calibration_done.emit(False, "ODrive not connected")
            return
        try:
            calibrated, detail = self._backend.calibrate()
            self._emit_state(self._backend.get_state())
            self.calibration_done.emit(bool(calibrated), str(detail))
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.calibration_done.emit(False, "Disconnected (reconnecting)")
            else:
                self.calibration_done.emit(False, str(e))

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

    @Slot(object)
    def save_config(self, values: Dict[str, Any]) -> None:
        if not self._backend.is_connected():
            self.config_saved.emit(False, "ODrive not connected (reconnecting)", list(values.keys()))
            return
        try:
            ok, msg, missing = self._backend.save_config(values)
            self.config_saved.emit(bool(ok), str(msg), missing)
            if not self._backend.is_connected():
                self._set_connected(False, self._backend.get_state())
            else:
                self._emit_state(self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.config_saved.emit(False, "ODrive disconnected during save (reconnecting)", list(values.keys()))
            else:
                self.error.emit(f"Save config failed: {e}")
                self.config_saved.emit(False, str(e), list(values.keys()))

    @Slot()
    def erase_config(self) -> None:
        if not self._backend.is_connected():
            self.config_erased.emit(False, "ODrive not connected (reconnecting)")
            return
        try:
            ok, msg = self._backend.erase_config()
            self.config_erased.emit(bool(ok), str(msg))
            if not self._backend.is_connected():
                self._set_connected(False, self._backend.get_state())
            else:
                self._emit_state(self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.config_erased.emit(False, "ODrive disconnected during erase (reconnecting)")
            else:
                self.error.emit(f"Erase config failed: {e}")
                self.config_erased.emit(False, str(e))

    @Slot()
    def clear_errors(self) -> None:
        if not self._backend.is_connected():
            self.errors_cleared.emit(False, "ODrive not connected (reconnecting)")
            return
        try:
            ok, msg = self._backend.clear_errors()
            self.errors_cleared.emit(bool(ok), str(msg))
            if not self._backend.is_connected():
                self._set_connected(False, self._backend.get_state())
            else:
                self._emit_state(self._backend.get_state())
        except Exception as e:
            if self._is_disconnect_error(e):
                self._mark_disconnected("Disconnected (reconnecting)")
                self.errors_cleared.emit(False, "ODrive disconnected while clearing errors (reconnecting)")
            else:
                self.errors_cleared.emit(False, str(e))

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
            self._emit_watchdog_status("Disconnected")
            return
        try:
            try:
                self._emit_watchdog_status(self._backend.feed_watchdog())
            except Exception:
                pass
            self._poll_runtime_fault()
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
    recipe_step_changed = Signal(int, int)       # step_idx, total_steps
    recipe_data_entry_requested = Signal(object, int, int)  # Step, step_idx, total_steps

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
        self._recipe_override_rpm: Optional[float] = None
        self._waiting_data_entry = False
        self._paused = False
        self._single_remaining_s = 0
        self._recipe_step_remaining_s = 0

    def is_running(self) -> bool:
        return self._mode in ("single", "recipe")

    def is_paused(self) -> bool:
        return bool(self._paused)

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

    def _clear_pause(self) -> None:
        self._paused = False
        self._single_remaining_s = 0
        self._recipe_step_remaining_s = 0

    @Slot(float, int)
    def start_single(self, rpm: float, duration_s: int) -> None:
        self.stop(user_initiated=False)
        self._set_mode("single")
        self._clear_pause()
        self._recipe_override_rpm = None
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
        self._clear_pause()
        self._recipe = recipe
        self._step_idx = 0
        self._recipe_override_rpm = None
        self._waiting_data_entry = False
        self._start_current_step(time.monotonic())
        self._set_status("Running")

        self._timer.start()
        self._tick()

    def _start_current_step(self, now: float) -> None:
        assert self._recipe is not None
        step = self._recipe.steps[self._step_idx]
        self.recipe_step_changed.emit(int(self._step_idx), int(len(self._recipe.steps)))
        step_name = (step.name or f"Step {self._step_idx + 1}").strip()
        if is_data_entry_step(step):
            self._waiting_data_entry = True
            self._active_rpm = 0.0
            self.request_stop.emit()
            self.active_rpm_changed.emit(0.0)
            self.remaining_changed.emit(0)
            prompt = (step.prompt or step_name or "Enter value").strip()
            info = f"{step_name} • {self._step_idx + 1}/{len(self._recipe.steps)} • Data entry"
            self.step_info_changed.emit(info)
            self.recipe_data_entry_requested.emit(step, int(self._step_idx), int(len(self._recipe.steps)))
            return

        self._waiting_data_entry = False
        self._active_rpm = float(step.velocity_rpm)
        self.request_set_rpm.emit(self._active_rpm)
        self.active_rpm_changed.emit(self._active_rpm)

        self._step_end_t = now + max(0, int(step.duration_s))
        info = f"{step_name} • {self._step_idx + 1}/{len(self._recipe.steps)} • {step.velocity_rpm:g} rpm • {format_mmss(step.duration_s)}"
        self.step_info_changed.emit(info)

    @Slot()
    def continue_after_data_entry(self) -> None:
        if self._mode != "recipe" or self._recipe is None or not self._waiting_data_entry:
            return
        self._paused = False
        self._waiting_data_entry = False
        now = time.monotonic()
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
        self._recipe_override_rpm = None
        self._start_current_step(now)
        if not self._timer.isActive():
            self._timer.start()
        self._tick()

    @Slot()
    def skip_current_recipe_step(self) -> None:
        if self._mode != "recipe" or self._recipe is None:
            return
        if self._waiting_data_entry:
            self.continue_after_data_entry()
            return
        now = time.monotonic()
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
        self._recipe_override_rpm = None
        self._start_current_step(now)
        if not self._timer.isActive() and not self._paused:
            self._timer.start()
        self._tick()

    @Slot(float)
    def override_rpm(self, rpm: float) -> None:
        """
        Apply an immediate RPM change while running:
          - SINGLE: updates active rpm immediately and persists.
          - RECIPE: overrides step rpm until the next step boundary, then step rpm takes over.
        """
        if self._mode == "single":
            self._active_rpm = float(rpm)
            self.request_set_rpm.emit(self._active_rpm)
            self.active_rpm_changed.emit(self._active_rpm)
            return

        if self._mode == "recipe":
            self._recipe_override_rpm = float(rpm)
            self._active_rpm = float(rpm)
            self.request_set_rpm.emit(self._active_rpm)
            self.active_rpm_changed.emit(self._active_rpm)
            return

    @Slot(int)
    def adjust_single_remaining(self, delta_s: int) -> None:
        if self._mode != "single":
            return
        if self._paused:
            self._single_remaining_s = int(clamp(int(self._single_remaining_s) + int(delta_s), 0, TIME_MAX_S))
            self.remaining_changed.emit(int(self._single_remaining_s))
            return
        now = time.monotonic()
        remaining = int(round(self._single_end_t - now))
        new_remaining = int(clamp(remaining + int(delta_s), 0, TIME_MAX_S))
        self._single_end_t = now + new_remaining
        self._tick()

    @Slot()
    def pause(self) -> None:
        if self._paused or self._mode not in ("single", "recipe"):
            return
        now = time.monotonic()
        if self._mode == "single":
            self._single_remaining_s = max(0, int(round(self._single_end_t - now)))
        elif self._mode == "recipe":
            if self._waiting_data_entry:
                self._recipe_step_remaining_s = 0
            else:
                self._recipe_step_remaining_s = max(0, int(round(self._step_end_t - now)))
        self._paused = True
        if self._timer.isActive():
            self._timer.stop()
        self.request_stop.emit()
        self._set_status("Paused")
        if self._mode == "single":
            self.remaining_changed.emit(int(self._single_remaining_s))
        elif self._mode == "recipe":
            self.remaining_changed.emit(int(self._recipe_step_remaining_s if not self._waiting_data_entry else 0))

    @Slot()
    def resume(self) -> None:
        if not self._paused or self._mode not in ("single", "recipe"):
            return
        now = time.monotonic()
        if self._mode == "single":
            self._single_end_t = now + max(0, int(self._single_remaining_s))
            if abs(self._active_rpm) > 1e-9:
                self.request_set_rpm.emit(self._active_rpm)
                self.active_rpm_changed.emit(self._active_rpm)
        elif self._mode == "recipe":
            if not self._waiting_data_entry:
                self._step_end_t = now + max(0, int(self._recipe_step_remaining_s))
                if abs(self._active_rpm) > 1e-9:
                    self.request_set_rpm.emit(self._active_rpm)
                    self.active_rpm_changed.emit(self._active_rpm)
        self._paused = False
        self._set_status("Running")
        if not self._timer.isActive():
            self._timer.start()
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
        self._clear_pause()
        self._recipe_override_rpm = None
        self._waiting_data_entry = False

        self.active_rpm_changed.emit(0.0)
        self.remaining_changed.emit(0)
        self.step_info_changed.emit("")
        self._set_status("Stopped" if user_initiated else "Idle")

    @Slot()
    def _tick(self) -> None:
        if self._paused:
            return
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
            if self._waiting_data_entry:
                self.remaining_changed.emit(0)
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
                self._recipe_override_rpm = None
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

    def insert_force(self, ts: int, tag: str, value: float) -> None:
        v = float(value)
        self._conn.execute("INSERT INTO samples(ts, tag, value) VALUES(?, ?, ?)", (int(ts), str(tag), v))
        self._conn.commit()
        self._last[str(tag)] = (int(ts), v)

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
            "SELECT ts, value FROM samples WHERE tag=? AND ts>=? AND ts<=? ORDER BY ts ASC, rowid ASC",
            (tag, start_ts, now_ts),
        )
        return [(int(r[0]), float(r[1])) for r in cur.fetchall()]

    def query_window_multi(self, now_ts: int, tags: Sequence[str], minutes: int) -> Dict[str, List[Tuple[int, float]]]:
        return {t: self.query_window(now_ts, t, minutes) for t in tags}

    def query_range(self, start_ts: int, end_ts: int, tag: str) -> List[Tuple[int, float]]:
        lo = int(min(start_ts, end_ts))
        hi = int(max(start_ts, end_ts))
        cur = self._conn.execute(
            "SELECT ts, value FROM samples WHERE tag=? AND ts>=? AND ts<=? ORDER BY ts ASC, rowid ASC",
            (tag, lo, hi),
        )
        return [(int(r[0]), float(r[1])) for r in cur.fetchall()]

    def query_range_multi(self, start_ts: int, end_ts: int, tags: Sequence[str]) -> Dict[str, List[Tuple[int, float]]]:
        return {t: self.query_range(start_ts, end_ts, t) for t in tags}

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
        self.setMinimumHeight(ui(min_h))
        self.setMinimumWidth(ui(min_w))
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


class RpmControl(QWidget):
    value_changed = Signal(float)
    edit_focused = Signal()

    def __init__(self, title: str, vmin: float, vmax: float, step: float, compact: bool = False) -> None:
        super().__init__()
        self.setObjectName("RpmControl")
        self._vmin, self._vmax, self._step = float(vmin), float(vmax), float(step)
        self._hold_dir = 0
        self._hold_started_t = 0.0
        self._hold_timer = QTimer(self)
        self._hold_timer.setSingleShot(True)
        self._hold_timer.timeout.connect(self._hold_tick)

        min_h = ui(70 if not compact else 58)
        btn_w = ui(110 if not compact else 90)
        spacing = ui(12 if not compact else 8)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(ui(8 if not compact else 6))

        lbl = QLabel(title)
        lbl.setObjectName("SectionTitle" if not compact else "SectionTitleCompact")
        root.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(spacing)

        self.btn_minus = TouchButton("−", min_h=min_h, min_w=btn_w)
        self.btn_plus = TouchButton("+", min_h=min_h, min_w=btn_w)

        self.edit = QLineEdit()
        self.edit.setMinimumHeight(min_h)
        self.edit.setMinimumWidth(ui(180 if not compact else 140))
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText("120")
        self.edit.setValidator(QDoubleValidator(self._vmin, self._vmax, 2, self.edit))

        units = QLabel("rpm")
        units.setMinimumWidth(ui(54 if not compact else 46))
        units.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        units.setObjectName("UnitsLabel")

        row.addWidget(self.btn_minus)
        row.addWidget(self.edit, 1)
        row.addWidget(units)
        row.addWidget(self.btn_plus)
        root.addLayout(row)

        self.btn_minus.pressed.connect(lambda: self._begin_hold(-1))
        self.btn_minus.released.connect(self._end_hold)
        self.btn_plus.pressed.connect(lambda: self._begin_hold(+1))
        self.btn_plus.released.connect(self._end_hold)
        self.edit.editingFinished.connect(self._emit_if_valid)
        self.edit.installEventFilter(self)
        self._emit_if_valid()

    def value(self) -> float:
        try:
            return float(self.edit.text().strip() or self._vmin)
        except Exception:
            return float(self._vmin)

    def set_value(self, v: float, emit_signal: bool = True) -> None:
        v = clamp(float(v), self._vmin, self._vmax)
        if self.edit.text().strip() != f"{v:g}":
            self.edit.setText(f"{v:g}")
        if emit_signal:
            self.value_changed.emit(v)

    def set_range(self, vmin: float, vmax: float) -> None:
        self._vmin = float(vmin)
        self._vmax = float(max(vmin, vmax))
        try:
            self.edit.setValidator(QDoubleValidator(self._vmin, self._vmax, 2, self.edit))
        except Exception:
            pass
        self.set_value(self.value(), emit_signal=False)

    def set_enabled(self, enabled: bool) -> None:
        if not enabled:
            self._end_hold()
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)
        self.edit.setEnabled(enabled)

    def _begin_hold(self, direction: int) -> None:
        try:
            if not self.isEnabled():
                return
            self._hold_dir = -1 if int(direction) < 0 else 1
            self._hold_started_t = time.monotonic()
            self._nudge(self._hold_dir)
            # Initial delay keeps short taps as single-step changes.
            self._hold_timer.start(450)
        except Exception:
            self._hold_dir = 0

    @Slot()
    def _end_hold(self) -> None:
        self._hold_dir = 0
        try:
            self._hold_timer.stop()
        except Exception:
            pass

    def _hold_interval_ms(self, held_s: float) -> int:
        if held_s >= 6.0:
            return 40
        if held_s >= 4.0:
            return 80
        if held_s >= 2.5:
            return 140
        return 220

    def _nudge(self, direction: int) -> None:
        self.set_value(self.value() + (float(direction) * self._step))

    @Slot()
    def _hold_tick(self) -> None:
        try:
            if int(self._hold_dir) == 0:
                return
            held_s = max(0.0, float(time.monotonic() - self._hold_started_t))
            self._nudge(int(self._hold_dir))
            self._hold_timer.start(int(self._hold_interval_ms(held_s)))
        except Exception:
            self._end_hold()

    @Slot()
    def _emit_if_valid(self) -> None:
        self.set_value(self.value(), emit_signal=True)

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        try:
            if watched is self.edit and event.type() == QEvent.FocusIn:
                self.edit_focused.emit()
        except Exception:
            pass
        return super().eventFilter(watched, event)


class TimeControl(QWidget):
    value_changed = Signal(int)
    edit_focused = Signal()

    def __init__(self, title: str, step_s: int, compact: bool = False) -> None:
        super().__init__()
        self.setObjectName("TimeControl")
        self._step_s = int(step_s)
        self._adjust_cb: Optional[Callable[[int], None]] = None
        self._hold_dir = 0
        self._hold_started_t = 0.0
        self._hold_timer = QTimer(self)
        self._hold_timer.setSingleShot(True)
        self._hold_timer.timeout.connect(self._hold_tick)

        min_h = ui(70 if not compact else 58)
        btn_w = ui(110 if not compact else 90)
        spacing = ui(12 if not compact else 8)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(ui(8 if not compact else 6))

        lbl = QLabel(title)
        lbl.setObjectName("SectionTitle" if not compact else "SectionTitleCompact")
        root.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(spacing)

        self.btn_minus = TouchButton("−", min_h=min_h, min_w=btn_w)
        self.btn_plus = TouchButton("+", min_h=min_h, min_w=btn_w)

        self.edit = QLineEdit()
        self.edit.setMinimumHeight(min_h)
        self.edit.setMinimumWidth(ui(180 if not compact else 140))
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText("00:00")
        self.edit.setPlaceholderText("mm:ss")

        hint = QLabel("mm:ss")
        hint.setMinimumWidth(ui(54 if not compact else 46))
        hint.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        hint.setObjectName("UnitsLabel")

        row.addWidget(self.btn_minus)
        row.addWidget(self.edit, 1)
        row.addWidget(hint)
        row.addWidget(self.btn_plus)
        root.addLayout(row)

        self.btn_minus.pressed.connect(lambda: self._begin_hold(-1))
        self.btn_minus.released.connect(self._end_hold)
        self.btn_plus.pressed.connect(lambda: self._begin_hold(+1))
        self.btn_plus.released.connect(self._end_hold)
        self.edit.editingFinished.connect(self._emit_if_valid)
        self.edit.installEventFilter(self)
        self._emit_if_valid()

    def set_adjust_callback(self, cb: Optional[Callable[[int], None]]) -> None:
        self._adjust_cb = cb

    def seconds(self) -> int:
        s = parse_mmss(self.edit.text())
        return int(clamp(int(s if s is not None else 0), 0, TIME_MAX_S))

    def text_mmss(self) -> str:
        return self.edit.text().strip()

    def set_seconds(self, s: int, emit_signal: bool = True) -> None:
        s = int(clamp(int(s), 0, TIME_MAX_S))
        txt = format_mmss(s)
        if self.edit.text().strip() != txt:
            self.edit.setText(txt)
        if emit_signal:
            self.value_changed.emit(s)

    def set_text_mmss(self, text: str, emit_signal: bool = True) -> None:
        s = parse_mmss(text)
        if s is None:
            s = 0
        self.set_seconds(int(s), emit_signal=emit_signal)

    def set_enabled(self, enabled: bool) -> None:
        if not enabled:
            self._end_hold()
        self.set_buttons_enabled(enabled)
        self.set_edit_enabled(enabled)

    def set_buttons_enabled(self, enabled: bool) -> None:
        if not enabled:
            self._end_hold()
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)

    def set_edit_enabled(self, enabled: bool) -> None:
        self.edit.setEnabled(enabled)

    def set_dimmed(self, dimmed: bool) -> None:
        self.setProperty("dimmed", "true" if dimmed else "false")
        self.style().unpolish(self)
        self.style().polish(self)

    def _adjust(self, delta_s: int) -> None:
        # During a running SINGLE run, the +/- buttons must adjust remaining time only,
        # without updating the entry text. This is implemented by using _adjust_cb.
        if self._adjust_cb is not None:
            self._adjust_cb(int(delta_s))
            return
        self.set_seconds(self.seconds() + int(delta_s), emit_signal=True)

    def _begin_hold(self, direction: int) -> None:
        try:
            if not self.isEnabled():
                return
            self._hold_dir = -1 if int(direction) < 0 else 1
            self._hold_started_t = time.monotonic()
            self._adjust(int(self._hold_dir) * int(self._step_s))
            self._hold_timer.start(450)
        except Exception:
            self._hold_dir = 0

    @Slot()
    def _end_hold(self) -> None:
        self._hold_dir = 0
        try:
            self._hold_timer.stop()
        except Exception:
            pass

    def _hold_interval_ms(self, held_s: float) -> int:
        if held_s >= 6.0:
            return 40
        if held_s >= 4.0:
            return 80
        if held_s >= 2.5:
            return 140
        return 220

    @Slot()
    def _hold_tick(self) -> None:
        try:
            if int(self._hold_dir) == 0:
                return
            held_s = max(0.0, float(time.monotonic() - self._hold_started_t))
            self._adjust(int(self._hold_dir) * int(self._step_s))
            self._hold_timer.start(int(self._hold_interval_ms(held_s)))
        except Exception:
            self._end_hold()

    @Slot()
    def _emit_if_valid(self) -> None:
        self.set_seconds(self.seconds(), emit_signal=True)

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        try:
            if watched is self.edit and event.type() == QEvent.FocusIn:
                self.edit_focused.emit()
        except Exception:
            pass
        return super().eventFilter(watched, event)


# ----------------------------
# Shared Header/Footer
# ----------------------------

class StatusHeader(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("StatusBar")
        sb = QHBoxLayout(self)
        sb.setContentsMargins(ui(16), ui(12), ui(16), ui(12))
        sb.setSpacing(ui(14))

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
        bottom.setSpacing(ui(14))

        self.btn_start = TouchButton("Start", min_h=76, min_w=240)
        self.btn_stop = TouchButton("Stop", min_h=76, min_w=240)
        self.btn_start.setObjectName("StartButton")
        self.btn_stop.setObjectName("StopButton")
        self._running = False
        self._motor_controls_enabled = True

        bottom.addWidget(self.btn_start, 1)
        bottom.addWidget(self.btn_stop, 1)

        self.btn_start.clicked.connect(self.start_clicked.emit)
        self.btn_stop.clicked.connect(self.stop_clicked.emit)

    def set_running_ui(self, running: bool) -> None:
        self._running = bool(running)
        self._apply_enable_state()

    def set_motor_controls_enabled(self, enabled: bool) -> None:
        self._motor_controls_enabled = bool(enabled)
        self._apply_enable_state()

    def _apply_enable_state(self) -> None:
        self.btn_start.setEnabled((not self._running) and self._motor_controls_enabled)
        self.btn_stop.setEnabled(self._running and self._motor_controls_enabled)


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
        root.setContentsMargins(ui(14), ui(12), ui(14), ui(12))
        root.setSpacing(ui(10))

        name = QLabel(recipe.name)
        name.setObjectName("RecipeName")
        summary = QLabel(recipe_summary(recipe))
        summary.setObjectName("RecipeSummary")

        btn_row = QHBoxLayout()
        btn_row.setSpacing(ui(10))
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
    drag_reorder_requested = Signal(int, int)  # from_idx, drop_global_y
    text_field_focused = Signal(object)  # QWidget

    def __init__(self, index: int, step: Step) -> None:
        super().__init__()
        self._index = index
        self._drag_press_y: Optional[int] = None
        self._dragging = False
        self.setObjectName("StepRow")
        self.setFrameShape(QFrame.StyledPanel)

        root = QVBoxLayout(self)
        root.setContentsMargins(ui(12), ui(10), ui(12), ui(10))
        root.setSpacing(ui(8))

        header = QHBoxLayout()
        header.setSpacing(ui(10))

        self.lbl_num = QLabel(f"{index + 1}")
        self.lbl_num.setObjectName("StepIndex")
        self.lbl_num.setMinimumWidth(ui(36))
        self.lbl_num.setAlignment(Qt.AlignCenter)
        header.addWidget(self.lbl_num)

        self.lbl = QLabel(f"Step {index + 1}")
        self.lbl.setObjectName("StepIndex")
        header.addWidget(self.lbl, 1)

        self.kind_combo = QComboBox()
        self.kind_combo.setMinimumHeight(ui(54))
        self.kind_combo.addItem("Motor Run", "motor")
        self.kind_combo.addItem("Data Entry", "data_entry")
        header.addWidget(self.kind_combo)

        self.name_edit = QLineEdit()
        self.name_edit.setMinimumHeight(ui(54))
        self.name_edit.setPlaceholderText(f"Step {index + 1}")
        self.name_edit.setText(str(step.name or f"Step {index + 1}"))
        self.name_edit.installEventFilter(self)
        header.addWidget(self.name_edit, 2)

        btn_h, btn_w = ui(54), ui(120)
        self.btn_remove = TouchButton("Remove", min_h=btn_h, min_w=btn_w)
        header.addWidget(self.btn_remove)
        root.addLayout(header)

        self.motor_controls = QWidget()
        controls = QHBoxLayout(self.motor_controls)
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(ui(12))
        self.vel = RpmControl("Velocity", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM, compact=True)
        self.tim = TimeControl("Time", TIME_STEP_S, compact=True)
        self.vel.set_value(float(step.velocity_rpm), emit_signal=False)
        self.tim.set_seconds(int(step.duration_s), emit_signal=False)
        controls.addWidget(self.vel, 1)
        controls.addWidget(self.tim, 1)
        self.chk_motor_allow_skip = QCheckBox("Allow skip")
        self.chk_motor_allow_skip.setMinimumHeight(ui(54))
        self.chk_motor_allow_skip.setChecked(bool(getattr(step, "allow_skip", True)))
        controls.addWidget(self.chk_motor_allow_skip)
        root.addWidget(self.motor_controls)

        self.data_controls = QWidget()
        data = QHBoxLayout(self.data_controls)
        data.setContentsMargins(0, 0, 0, 0)
        data.setSpacing(ui(12))
        self.prompt_edit = QLineEdit()
        self.prompt_edit.setMinimumHeight(ui(54))
        self.prompt_edit.setPlaceholderText("Prompt (shown during run)")
        self.prompt_edit.setText(str(step.prompt or ""))
        self.prompt_edit.installEventFilter(self)
        self.response_combo = QComboBox()
        self.response_combo.setMinimumHeight(ui(54))
        self.response_combo.addItem("Text", "text")
        self.response_combo.addItem("Numeric", "numeric")
        self.response_combo.addItem("Pass / Fail", "pass_fail")
        self.chk_allow_skip = QCheckBox("Allow skip")
        self.chk_allow_skip.setMinimumHeight(ui(54))
        self.chk_allow_skip.setChecked(bool(getattr(step, "allow_skip", True)))
        data.addWidget(self.prompt_edit, 2)
        data.addWidget(self.response_combo, 1)
        data.addWidget(self.chk_allow_skip, 1)
        root.addWidget(self.data_controls)

        self.btn_remove.clicked.connect(lambda: self.remove_clicked.emit(self._index))
        self.kind_combo.currentIndexChanged.connect(self._refresh_kind_ui)
        self.vel.edit_focused.connect(lambda: self.text_field_focused.emit(self.vel.edit))
        self.tim.edit_focused.connect(lambda: self.text_field_focused.emit(self.tim.edit))

        kind = str(getattr(step, "kind", "motor") or "motor")
        self.kind_combo.setCurrentIndex(1 if kind == "data_entry" else 0)
        rk = str(getattr(step, "response_kind", "text") or "text")
        for i in range(self.response_combo.count()):
            if str(self.response_combo.itemData(i)) == rk:
                self.response_combo.setCurrentIndex(i)
                break
        self._refresh_kind_ui()

    def set_index(self, idx: int) -> None:
        self._index = idx
        self.lbl_num.setText(f"{idx + 1}")
        self.lbl.setText(f"Step {idx + 1}")
        if not self.name_edit.text().strip() or self.name_edit.text().startswith("Step "):
            self.name_edit.setPlaceholderText(f"Step {idx + 1}")

    def get_step(self) -> Step:
        name = self.name_edit.text().strip() or f"Step {self._index + 1}"
        kind = str(self.kind_combo.currentData() or "motor")
        if kind == "data_entry":
            prompt_txt = self.prompt_edit.text().strip()
            return Step(
                velocity_rpm=0.0,
                duration_s=0,
                name=prompt_txt or name,
                kind="data_entry",
                prompt=prompt_txt,
                response_kind=str(self.response_combo.currentData() or "text"),
                allow_skip=bool(self.chk_allow_skip.isChecked()),
            )
        return Step(
            velocity_rpm=float(self.vel.value()),
            duration_s=int(self.tim.seconds()),
            name=name,
            kind="motor",
            allow_skip=bool(self.chk_motor_allow_skip.isChecked()),
        )

    @Slot()
    def _refresh_kind_ui(self) -> None:
        is_data = str(self.kind_combo.currentData() or "motor") == "data_entry"
        self.motor_controls.setVisible(not is_data)
        self.data_controls.setVisible(is_data)
        self.name_edit.setVisible(not is_data)

    def mousePressEvent(self, event) -> None:
        try:
            if event.button() == Qt.LeftButton:
                self._drag_press_y = int(event.globalPosition().y())  # type: ignore[attr-defined]
                self._dragging = False
        except Exception:
            self._drag_press_y = None
            self._dragging = False
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        try:
            if self._drag_press_y is not None and (event.buttons() & Qt.LeftButton):
                gy = int(event.globalPosition().y())  # type: ignore[attr-defined]
                if abs(gy - int(self._drag_press_y)) >= QApplication.startDragDistance():
                    self._dragging = True
        except Exception:
            pass
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        try:
            if self._dragging and event.button() == Qt.LeftButton:
                gy = int(event.globalPosition().y())  # type: ignore[attr-defined]
                self.drag_reorder_requested.emit(int(self._index), int(gy))
        except Exception:
            pass
        self._drag_press_y = None
        self._dragging = False
        super().mouseReleaseEvent(event)

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        try:
            if watched in (self.name_edit, self.prompt_edit) and event.type() == QEvent.FocusIn:
                self.text_field_focused.emit(watched)
        except Exception:
            pass
        return super().eventFilter(watched, event)


class RecipeBuilderScreen(QWidget):
    back_clicked = Signal()
    cancel_clicked = Signal()
    save_clicked = Signal(Recipe)
    export_clicked = Signal(Recipe)
    delete_clicked = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._editing_id: Optional[str] = None
        self._vel_max_rpm = float(HMI_MAX_VEL_DEFAULT_RPM)

        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top = QHBoxLayout()
        top.setSpacing(ui(12))
        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("Recipe Builder")
        self.title.setObjectName("ScreenTitle")
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        root.addLayout(top)
        info_row = QHBoxLayout()
        info_row.setSpacing(ui(10))

        name_row = QHBoxLayout()
        name_row.setSpacing(ui(12))
        lbl_name = QLabel("Name")
        lbl_name.setObjectName("FieldLabel")
        lbl_name.setMinimumWidth(ui(120))
        self.name_edit = QLineEdit()
        self.name_edit.setMinimumHeight(ui(66))
        self.name_edit.setPlaceholderText("Enter recipe name")
        name_row.addWidget(lbl_name)
        name_row.addWidget(self.name_edit, 1)
        root.addLayout(name_row)

        self.steps_area = QScrollArea()
        self.steps_area.setWidgetResizable(True)
        self.steps_area.setFrameShape(QFrame.NoFrame)
        enable_touch_scrolling(self.steps_area)
        self.steps_container = QWidget()
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setContentsMargins(0, 0, 0, 0)
        self.steps_layout.setSpacing(ui(10))
        self._steps_bottom_spacer = QWidget()
        self._steps_bottom_spacer.setFixedHeight(ui(280))
        self.steps_layout.addWidget(self._steps_bottom_spacer)
        self.steps_layout.addStretch(1)
        self.steps_area.setWidget(self.steps_container)
        root.addWidget(self.steps_area, 1)

        actions = QHBoxLayout()
        actions.setSpacing(ui(12))
        self.btn_add = TouchButton("Add Step", min_h=66, min_w=220)
        self.btn_export = TouchButton("Export JSON", min_h=66, min_w=220)
        self.btn_delete = TouchButton("Delete Recipe", min_h=66, min_w=220)
        self.btn_delete.setObjectName("StopButton")
        self.btn_cancel = TouchButton("Cancel", min_h=66, min_w=220)
        self.btn_save = TouchButton("Save", min_h=66, min_w=220)
        actions.addWidget(self.btn_add, 1)
        actions.addWidget(self.btn_export, 1)
        actions.addWidget(self.btn_delete, 1)
        actions.addWidget(self.btn_cancel, 1)
        actions.addWidget(self.btn_save, 1)
        root.addLayout(actions)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_cancel.clicked.connect(self.cancel_clicked.emit)
        self.btn_add.clicked.connect(self._on_add_step)
        self.btn_export.clicked.connect(self._on_export)
        self.btn_delete.clicked.connect(self._on_delete)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_delete.setEnabled(False)

    def _rows(self) -> List[StepRow]:
        rows: List[StepRow] = []
        for i in range(self.steps_layout.count()):
            w = self.steps_layout.itemAt(i).widget()
            if isinstance(w, StepRow):
                rows.append(w)
        return rows

    def _clear_rows(self) -> None:
        for row in self._rows():
            self.steps_layout.removeWidget(row)
            row.deleteLater()

    def _add_row(self, step: Step) -> None:
        idx = len(self._rows())
        row = StepRow(idx, step)
        row.vel.set_range(VEL_MIN_RPM, self._vel_max_rpm)
        row.remove_clicked.connect(self._on_remove)
        row.drag_reorder_requested.connect(self._on_drag_reorder)
        row.text_field_focused.connect(self._on_step_text_field_focused)
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
            self.btn_delete.setEnabled(False)
            self._add_row(Step(name="Run Name", kind="data_entry", prompt="Run name", response_kind="text", allow_skip=True))
            self._add_row(Step(120.0, 60, "Step 2", kind="motor"))
            return
        self._editing_id = recipe.id
        self.title.setText("Recipe Builder (Edit)")
        self.name_edit.setText(recipe.name)
        self.btn_delete.setEnabled(True)
        for s in recipe.steps:
            self._add_row(
                Step(
                    velocity_rpm=float(getattr(s, "velocity_rpm", 0.0)),
                    duration_s=int(getattr(s, "duration_s", 0)),
                    name=str(getattr(s, "name", "") or ""),
                    kind=str(getattr(s, "kind", "motor") or "motor"),
                    prompt=str(getattr(s, "prompt", "") or ""),
                    response_kind=str(getattr(s, "response_kind", "text") or "text"),
                    allow_skip=bool(getattr(s, "allow_skip", True)),
                )
            )

    @Slot()
    def _on_add_step(self) -> None:
        self._add_row(Step(120.0, 60, kind="motor"))
        self._reindex()

    @Slot(int)
    def _on_remove(self, idx: int) -> None:
        rows = self._rows()
        if not (0 <= idx < len(rows)):
            return
        rows[idx].deleteLater()
        self.steps_layout.takeAt(idx)
        self._reindex()

    @Slot(int, int)
    def _on_drag_reorder(self, from_idx: int, drop_global_y: int) -> None:
        rows = self._rows()
        if not (0 <= int(from_idx) < len(rows)):
            return
        target_idx = int(from_idx)
        best_dist = None
        for i, row in enumerate(rows):
            try:
                cy = int(row.mapToGlobal(row.rect().center()).y())
            except Exception:
                continue
            dist = abs(int(drop_global_y) - cy)
            if best_dist is None or dist < best_dist:
                best_dist = dist
                target_idx = i
        if target_idx == int(from_idx):
            return
        w = rows[int(from_idx)]
        self.steps_layout.removeWidget(w)
        self.steps_layout.insertWidget(int(target_idx), w)
        self._reindex()

    @Slot()
    def _on_save(self) -> None:
        recipe = self._build_recipe_from_form()
        if recipe is None:
            return
        self.save_clicked.emit(recipe)

    @Slot()
    def _on_export(self) -> None:
        recipe = self._build_recipe_from_form()
        if recipe is None:
            return
        self.export_clicked.emit(recipe)

    @Slot()
    def _on_delete(self) -> None:
        if not self._editing_id:
            return
        self.delete_clicked.emit(str(self._editing_id))

    @Slot(object)
    def _on_step_text_field_focused(self, widget: object) -> None:
        try:
            if not isinstance(widget, QWidget):
                return
            QTimer.singleShot(0, lambda w=widget: self._scroll_widget_near_top(w))
        except Exception:
            pass

    def _scroll_widget_near_top(self, widget: QWidget) -> None:
        try:
            sb = self.steps_area.verticalScrollBar()
            top_margin = ui(90)
            y = int(widget.mapTo(self.steps_container, widget.rect().topLeft()).y())
            sb.setValue(max(0, y - top_margin))
        except Exception:
            pass

    def _build_recipe_from_form(self) -> Optional[Recipe]:
        name = self.name_edit.text().strip()
        steps = [r.get_step() for r in self._rows()]

        if not name:
            QMessageBox.warning(self, "Validation", "Recipe name is required.")
            return
        if len(steps) < 1:
            QMessageBox.warning(self, "Validation", "Add at least one step.")
            return
        for i, s in enumerate(steps, start=1):
            if is_data_entry_step(s):
                if not (s.prompt or "").strip():
                    QMessageBox.warning(self, "Validation", f"Step {i}: data entry prompt is required.")
                    return
                if str(s.response_kind) not in {"text", "numeric", "pass_fail"}:
                    QMessageBox.warning(self, "Validation", f"Step {i}: invalid data entry type.")
                    return
            else:
                if s.duration_s <= 0:
                    QMessageBox.warning(self, "Validation", f"Step {i}: duration must be > 0.")
                    return
                if not (VEL_MIN_RPM <= float(s.velocity_rpm) <= float(self._vel_max_rpm)):
                    QMessageBox.warning(self, "Validation", f"Step {i}: velocity out of range.")
                    return

        rid = self._editing_id or str(uuid.uuid4())
        return Recipe(id=rid, name=name, steps=steps)

    def set_velocity_limit(self, vmax_rpm: float) -> None:
        self._vel_max_rpm = float(clamp(float(vmax_rpm), VEL_MIN_RPM, VEL_MAX_RPM))
        for row in self._rows():
            row.vel.set_range(VEL_MIN_RPM, self._vel_max_rpm)


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
        root.setContentsMargins(ui(12), ui(10), ui(12), ui(10))
        root.setSpacing(ui(12))

        self.lbl = QLabel(spec.label)
        self.lbl.setObjectName("ConfigLabel")
        self.lbl.setMinimumWidth(ui(520))

        if spec.kind == "bool":
            self.editor: QWidget = QComboBox()
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            cb.setMinimumHeight(ui(58))
            cb.addItems(["False", "True"])
        elif spec.kind == "protocol":
            self.editor = QComboBox()
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            cb.setMinimumHeight(ui(58))
            cb.addItem("Disabled (NONE)", "Protocol.NONE")
            cb.addItem("Simple", "Protocol.SIMPLE")
        elif spec.kind == "int":
            self.editor = QLineEdit()
            le = self.editor  # type: ignore[assignment]
            assert isinstance(le, QLineEdit)
            le.setMinimumHeight(ui(58))
            le.setAlignment(Qt.AlignCenter)
            iv = QIntValidator(le)
            if spec.min_value is not None:
                iv.setBottom(int(spec.min_value))
            if spec.max_value is not None:
                iv.setTop(int(spec.max_value))
            le.setValidator(iv)
        else:
            self.editor = QLineEdit()
            le = self.editor  # type: ignore[assignment]
            assert isinstance(le, QLineEdit)
            le.setMinimumHeight(ui(58))
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
        self.note.setMinimumWidth(ui(220))
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
        elif self.spec.kind == "protocol":
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            current_name = None
            try:
                current_name = str(getattr(value, "name"))
            except Exception:
                current_name = None
            if current_name:
                target = f"Protocol.{current_name}"
            else:
                raw = str(value)
                if raw in ("Protocol.NONE", "Protocol.SIMPLE"):
                    target = raw
                elif raw == "0":
                    target = "Protocol.NONE"
                elif raw == "1":
                    target = "Protocol.SIMPLE"
                else:
                    target = str(self.spec.default)
            idx = cb.findData(target)
            cb.setCurrentIndex(idx if idx >= 0 else 0)
        elif self.spec.kind == "int":
            le = self.editor  # type: ignore[assignment]
            assert isinstance(le, QLineEdit)
            try:
                iv = int(value)
            except Exception:
                iv = int(self.spec.default)
            le.setText(str(iv))
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
        if self.spec.kind == "protocol":
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            return str(cb.currentData() or self.spec.default)

        le = self.editor  # type: ignore[assignment]
        assert isinstance(le, QLineEdit)
        txt = le.text().strip()
        if not txt:
            return self.spec.default
        if self.spec.kind == "int":
            try:
                return int(txt)
            except Exception:
                return int(self.spec.default)
        try:
            return float(txt)
        except Exception:
            return float(self.spec.default)


class ODriveConfigScreen(QWidget):
    back_clicked = Signal()
    apply_clicked = Signal(object)         # dict key->value
    save_reboot_clicked = Signal(object)   # dict key->value
    erase_reset_clicked = Signal()
    clear_errors_clicked = Signal()
    request_refresh = Signal()
    export_json_clicked = Signal()
    hmi_max_velocity_changed = Signal(float)

    def __init__(self) -> None:
        super().__init__()
        self._rows: Dict[str, SettingRowWidget] = {}
        self._motor_presets: Dict[str, Dict[str, Any]] = dict(ODRIVE_MOTOR_PRESETS)
        self._search_last_query = ""
        self._search_match_keys: List[str] = []
        self._search_match_idx = -1

        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top = QHBoxLayout()
        top.setSpacing(ui(10))
        self.btn_back = TouchButton("Back", min_h=66, min_w=150)
        self.title = QLabel("ODrive Configuration")
        self.title.setObjectName("ScreenTitle")
        self.title.setMinimumWidth(ui(260))
        self.title.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_refresh = TouchButton("Refresh", min_h=66, min_w=150)
        self.btn_export = TouchButton("Export JSON", min_h=66, min_w=170)
        self.btn_odrive_gui = TouchButton("ODrive GUI", min_h=66, min_w=150)
        self.btn_clear_errors = TouchButton("Clear Errors", min_h=66, min_w=170)
        self.edit_search = QLineEdit()
        self.edit_search.setMinimumHeight(ui(58))
        self.edit_search.setPlaceholderText("Search label or odrivetool property")
        self._search_w_collapsed = ui(170)
        self._search_w_expanded = ui(520)
        self.edit_search.setMinimumWidth(self._search_w_collapsed)
        self.edit_search.setMaximumWidth(self._search_w_collapsed)
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        top.addStretch(1)
        top.addWidget(self.edit_search)
        top.addWidget(self.btn_odrive_gui)
        root.addLayout(top)

        info_row = QHBoxLayout()
        info_row.setSpacing(ui(10))
        self.status = QLabel("Loading…")
        self.status.setObjectName("InfoLabel")
        self.status.setMinimumWidth(ui(180))
        info_row.addWidget(self.status)
        self.watchdog_status = QLabel("Watchdog: Unknown")
        self.watchdog_status.setObjectName("InfoLabel")
        self.watchdog_status.setMinimumWidth(ui(220))
        info_row.addWidget(self.watchdog_status)
        info_row.addStretch(1)
        info_row.addWidget(self.btn_clear_errors)
        info_row.addWidget(self.btn_refresh)
        info_row.addWidget(self.btn_export)
        root.addLayout(info_row)

        preset_row = QHBoxLayout()
        preset_row.setSpacing(ui(12))
        self.lbl_motor_preset = QLabel("Motor preset")
        self.lbl_motor_preset.setObjectName("FieldLabel")
        self.lbl_motor_preset.setMinimumWidth(ui(220))
        self.cmb_motor_preset = QComboBox()
        self.cmb_motor_preset.setMinimumHeight(ui(58))
        self.cmb_motor_preset.addItem("Custom / no preset", "")
        for name in self._motor_presets.keys():
            self.cmb_motor_preset.addItem(name, name)
        preset_row.addWidget(self.lbl_motor_preset)
        preset_row.addWidget(self.cmb_motor_preset, 1)
        root.addLayout(preset_row)

        hmi_max_row = QHBoxLayout()
        hmi_max_row.setSpacing(ui(12))
        self.lbl_hmi_max_vel = QLabel("HMI max velocity (rpm)")
        self.lbl_hmi_max_vel.setObjectName("FieldLabel")
        self.lbl_hmi_max_vel.setMinimumWidth(ui(320))
        self.edit_hmi_max_vel = QLineEdit()
        self.edit_hmi_max_vel.setMinimumHeight(ui(58))
        self.edit_hmi_max_vel.setAlignment(Qt.AlignCenter)
        self.edit_hmi_max_vel.setValidator(QDoubleValidator(VEL_MIN_RPM, VEL_MAX_RPM, 1, self.edit_hmi_max_vel))
        self.btn_apply_hmi_max_vel = TouchButton("Apply HMI Limit", min_h=66, min_w=240)
        hmi_max_row.addWidget(self.lbl_hmi_max_vel)
        hmi_max_row.addWidget(self.edit_hmi_max_vel, 1)
        hmi_max_row.addWidget(self.btn_apply_hmi_max_vel)
        root.addLayout(hmi_max_row)

        self.area = QScrollArea()
        self.area.setWidgetResizable(True)
        self.area.setFrameShape(QFrame.NoFrame)
        enable_touch_scrolling(self.area)

        self.container = QWidget()
        self.vbox = QVBoxLayout(self.container)
        self.vbox.setContentsMargins(0, 0, 0, 0)
        self.vbox.setSpacing(ui(10))
        self.vbox.addStretch(1)
        self.area.setWidget(self.container)
        root.addWidget(self.area, 1)

        for spec in ODRIVE_CONFIG_FIELDS:
            row = SettingRowWidget(spec)
            self._rows[spec.key] = row
            row.editor.installEventFilter(self)
            self.vbox.insertWidget(self.vbox.count() - 1, row)

        actions = QHBoxLayout()
        actions.setSpacing(ui(12))
        self.btn_apply = TouchButton("Apply", min_h=66, min_w=180)
        self.btn_save = TouchButton("Save && Reboot", min_h=66, min_w=240)
        self.btn_erase_reset = TouchButton("Erase && Reset Defaults", min_h=66, min_w=320)
        self.btn_cancel = TouchButton("Cancel", min_h=66, min_w=200)
        actions.addWidget(self.btn_apply, 1)
        actions.addWidget(self.btn_save, 1)
        actions.addWidget(self.btn_erase_reset, 1)
        actions.addWidget(self.btn_cancel, 1)
        root.addLayout(actions)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_cancel.clicked.connect(self.back_clicked.emit)
        self.btn_apply.clicked.connect(lambda: self.apply_clicked.emit(self.gather_values()))
        self.btn_save.clicked.connect(lambda: self.save_reboot_clicked.emit(self.gather_values()))
        self.btn_erase_reset.clicked.connect(self.erase_reset_clicked.emit)
        self.btn_clear_errors.clicked.connect(self.clear_errors_clicked.emit)
        self.btn_refresh.clicked.connect(self.request_refresh.emit)
        self.btn_export.clicked.connect(self.export_json_clicked.emit)
        self.btn_odrive_gui.clicked.connect(self._open_odrive_gui)
        self.edit_search.returnPressed.connect(self._on_search_trigger)
        self.edit_search.textEdited.connect(self._on_search_text_edited)
        self.edit_search.installEventFilter(self)
        try:
            app = QApplication.instance()
            if app is not None:
                app.installEventFilter(self)
        except Exception:
            pass
        self.cmb_motor_preset.currentIndexChanged.connect(self._on_motor_preset_changed)
        self.btn_apply_hmi_max_vel.clicked.connect(self._emit_hmi_max_velocity)
        self.edit_hmi_max_vel.editingFinished.connect(self._emit_hmi_max_velocity)

    def set_loading(self, text: str) -> None:
        self.status.setText(text)

    def set_watchdog_status(self, status: str) -> None:
        self.watchdog_status.setText(f"Watchdog: {status}")

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
        self.status.setText("Defaults restored (not applied)")

    def gather_values(self) -> Dict[str, Any]:
        return {k: row.get_value() for k, row in self._rows.items() if row.editor.isEnabled()}

    def eventFilter(self, watched: QObject, event: QEvent) -> bool:
        try:
            if (
                self.isVisible()
                and event.type() == QEvent.MouseButtonPress
                and self.edit_search.hasFocus()
                and watched is not self.edit_search
                and isinstance(watched, QWidget)
            ):
                # Clicking any other widget on the config screen should collapse
                # the expanded search box, even if the target is non-focusable.
                if (watched is self) or self.isAncestorOf(watched):
                    self.edit_search.clearFocus()
            if watched is self.edit_search:
                if event.type() == QEvent.FocusIn:
                    self._set_search_expanded(True)
                elif event.type() == QEvent.FocusOut:
                    self._set_search_expanded(False)
            elif event.type() == QEvent.FocusIn:
                for row in self._rows.values():
                    if watched is row.editor:
                        QTimer.singleShot(0, lambda r=row: self._scroll_row_near_top(r))
                        break
        except Exception:
            pass
        return super().eventFilter(watched, event)

    def _scroll_row_near_top(self, row: SettingRowWidget) -> None:
        try:
            sb = self.area.verticalScrollBar()
            y = row.pos().y()
            top_margin = ui(80)
            sb.setValue(max(0, int(y - top_margin)))
        except Exception:
            pass

    def _set_search_expanded(self, expanded: bool) -> None:
        w = int(self._search_w_expanded if expanded else self._search_w_collapsed)
        try:
            self.edit_search.setMinimumWidth(w)
            self.edit_search.setMaximumWidth(w)
            if expanded:
                # Keep the expanded search bar visible above adjacent top-row buttons.
                self.edit_search.raise_()
        except Exception:
            pass

    @Slot(str)
    def _on_search_text_edited(self, _text: str) -> None:
        self._apply_search_filter(live=True)

    def _apply_search_filter(self, live: bool = False) -> None:
        query = str(self.edit_search.text() or "").strip().lower()
        if not query:
            self._search_last_query = ""
            self._search_match_keys = []
            self._search_match_idx = -1
            for row in self._rows.values():
                row.setVisible(True)
            if live:
                self.status.setText("Loaded")
            return

        self._search_last_query = query
        self._search_match_idx = -1
        self._search_match_keys = [
            spec.key
            for spec in ODRIVE_CONFIG_FIELDS
            if (query in str(spec.label).lower()) or (query in str(spec.key).lower())
        ]
        match_set = set(self._search_match_keys)
        for key, row in self._rows.items():
            row.setVisible(key in match_set)

        if not self._search_match_keys:
            self.status.setText(f"No config fields match '{query}'")
            return

        first_key = self._search_match_keys[0]
        row = self._rows.get(first_key)
        if row is not None:
            QTimer.singleShot(0, lambda r=row: self._scroll_row_near_top(r))
        self.status.setText(f"{len(self._search_match_keys)} match(es) for '{query}'")

    @Slot()
    def _on_search_trigger(self) -> None:
        query = str(self.edit_search.text() or "").strip().lower()
        if not query:
            self.status.setText("Enter search text (label or property path)")
            return

        if query != self._search_last_query:
            self._apply_search_filter(live=False)

        if not self._search_match_keys:
            self.status.setText(f"No config fields match '{query}'")
            return

        self._search_match_idx = (self._search_match_idx + 1) % len(self._search_match_keys)
        key = self._search_match_keys[self._search_match_idx]
        row = self._rows.get(key)
        if row is None:
            self.status.setText(f"Search match missing row: {key}")
            return
        QTimer.singleShot(0, lambda r=row: self._scroll_row_near_top(r))
        try:
            row.editor.setFocus()
        except Exception:
            pass
        self.status.setText(
            f"Search {self._search_match_idx + 1}/{len(self._search_match_keys)}: {row.spec.label} [{row.spec.key}]"
        )

    @Slot()
    def _open_odrive_gui(self) -> None:
        url = "https://gui.odriverobotics.com/configuration"
        try:
            ok = bool(webbrowser.open(url))
        except Exception:
            ok = False
        if not ok:
            self.status.setText(f"Failed to open browser: {url}")

    def set_hmi_max_velocity(self, rpm: float, emit_signal: bool = False) -> None:
        rpm = float(clamp(float(rpm), VEL_MIN_RPM, VEL_MAX_RPM))
        with QSignalBlocker(self.edit_hmi_max_vel):
            self.edit_hmi_max_vel.setText(f"{rpm:g}")
        if emit_signal:
            self.hmi_max_velocity_changed.emit(float(rpm))

    def hmi_max_velocity(self) -> float:
        try:
            return float(clamp(float(self.edit_hmi_max_vel.text().strip() or HMI_MAX_VEL_DEFAULT_RPM), VEL_MIN_RPM, VEL_MAX_RPM))
        except Exception:
            return float(HMI_MAX_VEL_DEFAULT_RPM)

    @Slot()
    def _emit_hmi_max_velocity(self) -> None:
        self.set_hmi_max_velocity(self.hmi_max_velocity(), emit_signal=True)

    @Slot(int)
    def _on_motor_preset_changed(self, _idx: int) -> None:
        key = str(self.cmb_motor_preset.currentData() or "")
        if not key:
            return
        preset = self._motor_presets.get(key, {})
        hmi_values = dict(preset.get("hmi_values") or {})
        if not hmi_values:
            self.status.setText(f"Preset selected: {key} (no mapped HMI fields)")
            return
        applied = 0
        skipped = 0
        for field_key, value in hmi_values.items():
            row = self._rows.get(field_key)
            if row is None:
                skipped += 1
                continue
            row.set_value(value)
            applied += 1
        suffix = f", {skipped} unmapped" if skipped else ""
        self.status.setText(f"Loaded motor preset '{key}' into form ({applied} field(s){suffix}; not applied)")


# ----------------------------
# Settings Screens
# ----------------------------

class SettingsHubScreen(QWidget):
    back_clicked = Signal()
    open_hmi_clicked = Signal()
    open_odrive_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top = QHBoxLayout()
        top.setSpacing(ui(12))
        self.btn_back = TouchButton("Back", min_h=80, min_w=210)
        self.title = QLabel("Settings")
        self.title.setObjectName("ScreenTitle")
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        root.addLayout(top)

        self.info = QLabel("Select a settings category")
        self.info.setObjectName("InfoLabel")
        root.addWidget(self.info)

        self.btn_hmi = TouchButton("HMI", min_h=110, min_w=320)
        self.btn_odrive = TouchButton("ODrive", min_h=110, min_w=320)
        root.addWidget(self.btn_hmi)
        root.addWidget(self.btn_odrive)
        root.addStretch(1)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_hmi.clicked.connect(self.open_hmi_clicked.emit)
        self.btn_odrive.clicked.connect(self.open_odrive_clicked.emit)


class HMISettingsScreen(QWidget):
    back_clicked = Signal()
    fullscreen_changed = Signal(bool)
    close_program_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._fullscreen_enabled = False
        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top = QHBoxLayout()
        top.setSpacing(ui(12))
        self.btn_back = TouchButton("Back", min_h=80, min_w=210)
        self.title = QLabel("HMI Settings")
        self.title.setObjectName("ScreenTitle")
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        root.addLayout(top)

        self.info = QLabel("Display")
        self.info.setObjectName("SectionTitle")
        root.addWidget(self.info)

        self.btn_fullscreen_toggle = TouchButton("Switch to Full Screen", min_h=90, min_w=420)
        root.addWidget(self.btn_fullscreen_toggle)

        self.note = QLabel("Press Esc to exit full screen at any time.")
        self.note.setObjectName("InfoLabel")
        root.addWidget(self.note)

        self.btn_close_program = TouchButton("Close Program", min_h=90, min_w=320)
        self.btn_close_program.setObjectName("StopButton")
        root.addWidget(self.btn_close_program)
        root.addStretch(1)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_fullscreen_toggle.clicked.connect(self._toggle_fullscreen_clicked)
        self.btn_close_program.clicked.connect(self.close_program_clicked.emit)
        self._refresh_fullscreen_button_text()

    def set_fullscreen_state(self, enabled: bool, emit_signal: bool = False) -> None:
        self._fullscreen_enabled = bool(enabled)
        self._refresh_fullscreen_button_text()
        if emit_signal:
            self.fullscreen_changed.emit(bool(enabled))

    @Slot()
    def _toggle_fullscreen_clicked(self) -> None:
        self.set_fullscreen_state(not self._fullscreen_enabled, emit_signal=True)

    def _refresh_fullscreen_button_text(self) -> None:
        self.btn_fullscreen_toggle.setText("Switch to Windowed Mode" if self._fullscreen_enabled else "Switch to Full Screen")


# ----------------------------
# Plot Widgets
# ----------------------------

class SimplePlotWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.setMinimumHeight(ui(360))
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

        rect = self.rect().adjusted(ui(10), ui(10), -ui(10), -ui(10))
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

        plot = rect.adjusted(ui(50), ui(10), -ui(10), -ui(30))
        p.drawRect(plot)

        def map_x(ts: int) -> float:
            return plot.left() + (ts - x0) / (x1 - x0) * plot.width()

        def map_y(v: float) -> float:
            return plot.bottom() - (v - y_min) / (y_max - y_min) * plot.height()

        p.drawText(rect.left() + ui(6), plot.top() + ui(14), f"{y_max:.3g}")
        p.drawText(rect.left() + ui(6), plot.bottom(), f"{y_min:.3g}")
        p.drawText(plot.left(), rect.bottom() - ui(6), f"-{self._minutes} min")
        p.drawText(plot.right() - ui(60), rect.bottom() - ui(6), "now")

        colors = [Qt.cyan, Qt.yellow, Qt.green, Qt.magenta, Qt.red, Qt.blue, Qt.gray, Qt.white]
        for i, t in enumerate(tags):
            pts = self._data.get(t, [])
            if len(pts) < 2:
                continue
            pen = p.pen()
            pen.setWidth(max(2, ui(2)))
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
        p.drawText(rect.adjusted(0, 0, 0, -rect.height() + ui(20)), Qt.AlignLeft | Qt.AlignVCenter, legend)


class QtChartsPlot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._chart = QChart()
        self._chart.legend().setVisible(True)
        self._chart.legend().setAlignment(Qt.AlignBottom)
        self._view = QChartView(self._chart)
        self._view.setRenderHint(QPainter.Antialiasing)
        self._view.setMinimumHeight(ui(360))

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self._view)

        self._series: Dict[str, QLineSeries] = {}
        self._minutes = 10
        self._tags: List[str] = []
        self._y_axes: Dict[str, QValueAxis] = {}

        self._x = QDateTimeAxis()
        self._x.setFormat("HH:mm:ss")
        self._x.setTitleText("Time")
        self._chart.addAxis(self._x, Qt.AlignBottom)

    def _axis_group_for_tag(self, tag: str) -> str:
        unit = str(TAG_UNITS.get(tag, "") or "value")
        return unit

    def _axis_title_for_group(self, group: str) -> str:
        g = str(group or "value")
        return "RPM" if g.lower() == "rpm" else g

    def _sync_y_axes(self) -> None:
        wanted_groups: List[str] = []
        for tag in self._tags:
            grp = self._axis_group_for_tag(tag)
            if grp not in wanted_groups:
                wanted_groups.append(grp)

        # Remove axes for groups no longer present.
        for grp in list(self._y_axes.keys()):
            if grp in wanted_groups:
                continue
            axis = self._y_axes.pop(grp)
            for s in self._series.values():
                try:
                    s.detachAxis(axis)
                except Exception:
                    pass
            try:
                self._chart.removeAxis(axis)
            except Exception:
                pass

        # Create missing axes and alternate sides.
        for i, grp in enumerate(wanted_groups):
            axis = self._y_axes.get(grp)
            if axis is None:
                axis = QValueAxis()
                self._y_axes[grp] = axis
                align = Qt.AlignLeft if (i % 2 == 0) else Qt.AlignRight
                self._chart.addAxis(axis, align)
            axis.setTitleText(self._axis_title_for_group(grp))

        # Reattach all existing series to the correct y-axis.
        for tag, s in self._series.items():
            self._attach_series_axes(s, tag)

    def _attach_series_axes(self, series: QLineSeries, tag: str) -> None:
        try:
            attached = list(series.attachedAxes())
        except Exception:
            attached = []
        try:
            if self._x not in attached:
                series.attachAxis(self._x)
        except Exception:
            pass
        for axis in self._y_axes.values():
            try:
                if axis in attached:
                    series.detachAxis(axis)
            except Exception:
                pass
        grp = self._axis_group_for_tag(tag)
        axis = self._y_axes.get(grp)
        if axis is not None:
            try:
                attached_after = list(series.attachedAxes())
            except Exception:
                attached_after = []
            try:
                if axis not in attached_after:
                    series.attachAxis(axis)
            except Exception:
                pass

    def set_series_data(self, data: Dict[str, List[Tuple[int, float]]], tags: List[str], minutes: int) -> None:
        self._minutes = int(minutes)
        self._tags = list(tags)

        for s in list(self._series.values()):
            self._chart.removeSeries(s)
        self._series.clear()

        self._sync_y_axes()

        for tag in self._tags:
            series = QLineSeries()
            series.setName(TAG_LABELS.get(tag, tag))
            for ts, val in data.get(tag, []):
                series.append(float(ts) * 1000.0, float(val))
            self._chart.addSeries(series)
            self._attach_series_axes(series, tag)
            self._series[tag] = series

        self._rescale_axes()

    def append_points(self, points: List[Tuple[str, int, float]], tags: List[str], minutes: int) -> None:
        self._minutes = int(minutes)
        self._tags = list(tags)
        self._sync_y_axes()

        for tag in self._tags:
            if tag not in self._series:
                s = QLineSeries()
                s.setName(TAG_LABELS.get(tag, tag))
                self._chart.addSeries(s)
                self._attach_series_axes(s, tag)
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

        vals_by_group: Dict[str, List[float]] = {}
        for tag, s in self._series.items():
            pts = s.pointsVector()
            if not pts:
                continue
            grp = self._axis_group_for_tag(tag)
            vals_by_group.setdefault(grp, []).extend(float(pt.y()) for pt in pts)

        def apply_axis(axis: QValueAxis, vals: List[float]) -> bool:
            if not vals:
                axis.setRange(0.0, 1.0)
                return False
            vmin, vmax = min(vals), max(vals)
            if abs(vmax - vmin) < 1e-9:
                vmax = vmin + 1.0
            pad = 0.05 * (vmax - vmin)
            axis.setRange(vmin - pad, vmax + pad)
            return True

        any_active = False
        for grp, axis in self._y_axes.items():
            active = apply_axis(axis, vals_by_group.get(grp, []))
            axis.setVisible(active)
            any_active = any_active or active
        if not any_active:
            for axis in self._y_axes.values():
                axis.setVisible(True)
                break


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
        self._selected_tags: List[str] = list(DEFAULT_ENABLED_TAGS)

        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top = QHBoxLayout()
        top.setSpacing(ui(12))
        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("Data Logger")
        self.title.setObjectName("ScreenTitle")
        self.btn_options = TouchButton("Options", min_h=66, min_w=180)
        self.btn_export = TouchButton("Export CSV", min_h=66, min_w=200)
        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        top.addWidget(self.btn_options)
        top.addWidget(self.btn_export)
        root.addLayout(top)

        self.status = StatusHeader()
        root.addWidget(self.status)

        self.plot = make_plot_widget()
        root.addWidget(self.plot, 1)

        self.footer = StartStopFooter()
        root.addWidget(self.footer)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.footer.start_clicked.connect(self.start_clicked.emit)
        self.footer.stop_clicked.connect(self.stop_clicked.emit)
        self.btn_options.clicked.connect(self._open_options_dialog)
        self.btn_export.clicked.connect(self._on_export)

    def set_status(self, ui_state: str, rpm: float, remaining_s: int, recipe_name: str, step_info: str, backend: str) -> None:
        self.status.set_status(ui_state, rpm, remaining_s, recipe_name, step_info, backend)

    def set_running_ui(self, running: bool, motor_controls_enabled: bool = True) -> None:
        self.footer.set_running_ui(running)
        self.footer.set_motor_controls_enabled(motor_controls_enabled)

    def minutes_window(self) -> int:
        return int(clamp(int(self._minutes), 1, 24 * 60))

    def selected_tags(self) -> List[str]:
        return list(self._selected_tags)

    def set_selected_tags(self, tags: List[str]) -> None:
        known = {
            TAG_PWR_W, TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM, TAG_MOTOR_TEMP_C, TAG_VBUS_V, TAG_IBUS_A
        }
        out = [t for t in (tags or []) if t in known]
        self._selected_tags = list(out or DEFAULT_ENABLED_TAGS)

    def set_minutes_window(self, minutes: int) -> None:
        minutes = int(clamp(int(minutes), 1, 24 * 60))
        self._minutes = minutes

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

    @Slot()
    def _open_options_dialog(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Data Logger Options")
        dlg.setModal(True)
        dlg.setMinimumWidth(ui(700))

        root = QVBoxLayout(dlg)
        root.setContentsMargins(ui(16), ui(16), ui(16), ui(16))
        root.setSpacing(ui(12))

        lbl_tags = QLabel("Tags")
        lbl_tags.setObjectName("SectionTitle")
        root.addWidget(lbl_tags)

        tag_checks: Dict[str, QCheckBox] = {}
        ordered_tags = [TAG_PWR_W, TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM, TAG_MOTOR_TEMP_C, TAG_VBUS_V, TAG_IBUS_A]
        wanted = set(self._selected_tags)
        for tag in ordered_tags:
            cb = QCheckBox(TAG_LABELS.get(tag, tag))
            cb.setMinimumHeight(ui(44))
            cb.setChecked(tag in wanted)
            tag_checks[tag] = cb
            root.addWidget(cb)

        win_row = QHBoxLayout()
        win_row.setSpacing(ui(12))
        lbl_win = QLabel("Minutes to display")
        lbl_win.setObjectName("FieldLabel")
        lbl_win.setMinimumWidth(ui(260))
        edit_min = QLineEdit()
        edit_min.setMinimumHeight(ui(66))
        edit_min.setAlignment(Qt.AlignCenter)
        edit_min.setValidator(QIntValidator(1, 24 * 60, edit_min))
        edit_min.setText(str(self._minutes))
        win_row.addWidget(lbl_win)
        win_row.addWidget(edit_min, 1)
        root.addLayout(win_row)

        buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        btn_apply = buttons.addButton("Apply", QDialogButtonBox.AcceptRole)
        btn_apply.setMinimumHeight(ui(58))
        root.addWidget(buttons)

        buttons.rejected.connect(dlg.reject)
        buttons.accepted.connect(dlg.accept)

        if dlg.exec() != QDialog.Accepted:
            return

        try:
            minutes = max(1, int(edit_min.text()))
        except Exception:
            minutes = self._minutes
        self._minutes = int(clamp(int(minutes), 1, 24 * 60))

        selected = [t for t in ordered_tags if tag_checks[t].isChecked()]
        self._selected_tags = list(selected or DEFAULT_ENABLED_TAGS)

        self.tags_changed.emit(self.selected_tags())
        self.window_changed.emit(self.minutes_window())


# ----------------------------
# Recipe Run Monitor Screen
# ----------------------------

class RecipeRunStepBox(QFrame):
    skip_clicked = Signal(int)

    def __init__(self, index: int, step: Step) -> None:
        super().__init__()
        self._index = int(index)
        self._base_duration = int(max(0, step.duration_s))
        self._can_skip = bool(getattr(step, "allow_skip", True) and (not is_data_entry_step(step)))
        self.setObjectName("RecipeRunStepBox")
        self.setProperty("status", "pending")
        self.setFrameShape(QFrame.StyledPanel)

        root = QVBoxLayout(self)
        root.setContentsMargins(ui(10), ui(8), ui(10), ui(8))
        root.setSpacing(ui(6))

        self.lbl_title = QLabel(f"{index + 1}. {step.name or f'Step {index + 1}'}")
        self.lbl_title.setObjectName("FieldLabel")
        self.lbl_meta = QLabel(f"{step.velocity_rpm:g} rpm • {format_mmss(self._base_duration)}")
        self.lbl_meta.setObjectName("InfoLabel")
        self.lbl_status = QLabel("Pending")
        self.lbl_status.setObjectName("InfoLabel")
        self.lbl_error = QLabel("")
        self.lbl_error.setObjectName("InfoLabel")
        self.lbl_error.setWordWrap(True)
        self.btn_skip = TouchButton("Skip", min_h=58, min_w=140)
        self.btn_skip.setVisible(False)
        self.btn_skip.clicked.connect(lambda: self.skip_clicked.emit(self._index))

        root.addWidget(self.lbl_title)
        root.addWidget(self.lbl_meta)
        root.addWidget(self.lbl_status)
        root.addWidget(self.lbl_error)
        root.addWidget(self.btn_skip, 0, Qt.AlignLeft)

    def set_state(self, status: str, remaining_s: Optional[int] = None, error_text: str = "") -> None:
        status = str(status)
        self.setProperty("status", status)
        self.style().unpolish(self)
        self.style().polish(self)
        if status == "running":
            rem = max(0, int(remaining_s or 0))
            self.lbl_status.setText(f"In progress • {format_mmss(rem)} remaining")
        elif status == "done":
            self.lbl_status.setText("Completed")
        elif status == "error":
            self.lbl_status.setText("Error")
        else:
            self.lbl_status.setText("Pending")
        self.lbl_error.setText(str(error_text or ""))
        self.btn_skip.setVisible(bool(status == "running" and self._can_skip))

    def hide_skip(self) -> None:
        self.btn_skip.setVisible(False)

    def can_skip(self) -> bool:
        return bool(self._can_skip)


class RecipeRunMonitorScreen(QWidget):
    back_clicked = Signal()
    stop_clicked = Signal()
    pause_resume_clicked = Signal()
    skip_step_clicked = Signal(int)
    tags_changed = Signal(object)  # list[str]
    generate_report_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._plot_minutes = 10
        self._step_boxes: List[RecipeRunStepBox] = []
        self._active_step_idx = -1

        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top = QHBoxLayout()
        top.setSpacing(ui(12))
        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("Recipe Run")
        self.title.setObjectName("ScreenTitle")
        self.lbl_recipe = QLabel("")
        self.lbl_recipe.setObjectName("InfoLabel")
        self.btn_options = TouchButton("Options", min_h=66, min_w=180)
        top.addWidget(self.btn_back)
        top.addWidget(self.title)
        top.addWidget(self.lbl_recipe, 1)
        top.addWidget(self.btn_options)
        root.addLayout(top)

        body = QHBoxLayout()
        body.setSpacing(ui(12))

        left = QVBoxLayout()
        left.setSpacing(ui(10))

        self.steps_area = QScrollArea()
        self.steps_area.setWidgetResizable(True)
        self.steps_area.setFrameShape(QFrame.NoFrame)
        enable_touch_scrolling(self.steps_area)
        self.steps_container = QWidget()
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setContentsMargins(0, 0, 0, 0)
        self.steps_layout.setSpacing(ui(8))
        self.steps_layout.addStretch(1)
        self.steps_area.setWidget(self.steps_container)
        left.addWidget(self.steps_area, 2)

        audit_title = QLabel("Audit Trail")
        audit_title.setObjectName("SectionTitleCompact")
        left.addWidget(audit_title)
        self.audit_list = QListWidget()
        enable_touch_scrolling(self.audit_list)
        self.audit_list.setMinimumHeight(ui(180))
        left.addWidget(self.audit_list, 1)

        body.addLayout(left, 1)

        self.plot = make_plot_widget()
        body.addWidget(self.plot, 2)
        root.addLayout(body, 1)

        footer_row = QHBoxLayout()
        footer_row.setSpacing(ui(12))
        footer_row.addStretch(1)
        self.btn_report = TouchButton("Generate Report", min_h=72, min_w=260)
        self.btn_pause_resume = TouchButton("Pause", min_h=72, min_w=260)
        self.btn_stop = TouchButton("Stop", min_h=72, min_w=220)
        self.btn_stop.setObjectName("StopButton")
        footer_row.addWidget(self.btn_report)
        footer_row.addWidget(self.btn_pause_resume)
        footer_row.addWidget(self.btn_stop)
        root.addLayout(footer_row)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_stop.clicked.connect(self.stop_clicked.emit)
        self.btn_pause_resume.clicked.connect(self.pause_resume_clicked.emit)
        self.btn_options.clicked.connect(self._open_options_dialog)
        self.btn_report.clicked.connect(self.generate_report_clicked.emit)
        self.set_pause_resume_action("pause", enabled=False)

    def selected_tags(self) -> List[str]:
        return list(getattr(self, "_selected_tags", [TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM]))

    def set_selected_tags(self, tags: List[str]) -> None:
        known = {TAG_PWR_W, TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM, TAG_MOTOR_TEMP_C, TAG_VBUS_V, TAG_IBUS_A}
        out = [t for t in (tags or []) if t in known]
        self._selected_tags = list(out or [TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM])

    def set_recipe(self, recipe: Recipe) -> None:
        self.lbl_recipe.setText(f"Recipe: {recipe.name} ({format_mmss(recipe_total_seconds(recipe))})")
        self._plot_minutes = int(clamp(int(math.ceil(recipe_total_seconds(recipe) / 60.0)), 1, 30))
        self._active_step_idx = -1
        self.audit_list.clear()
        while self.steps_layout.count() > 1:
            item = self.steps_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._step_boxes = []
        for i, step in enumerate(recipe.steps):
            box = RecipeRunStepBox(i, step)
            box.skip_clicked.connect(self.skip_step_clicked.emit)
            self._step_boxes.append(box)
            self.steps_layout.insertWidget(self.steps_layout.count() - 1, box)

    def append_audit(self, text: str) -> None:
        self.audit_list.addItem(f"{time.strftime('%H:%M:%S')}  {text}")
        self.audit_list.scrollToBottom()

    def set_step_running(self, idx: int, remaining_s: int) -> None:
        self._active_step_idx = int(idx)
        for i, box in enumerate(self._step_boxes):
            if i == idx:
                box.set_state("running", remaining_s=int(remaining_s))
            else:
                box.hide_skip()
        if 0 <= idx < len(self._step_boxes):
            self.steps_area.ensureWidgetVisible(self._step_boxes[idx])

    def set_step_remaining(self, idx: int, remaining_s: int) -> None:
        if 0 <= idx < len(self._step_boxes):
            self._step_boxes[idx].set_state("running", remaining_s=int(remaining_s))

    def set_step_done(self, idx: int) -> None:
        if 0 <= idx < len(self._step_boxes):
            self._step_boxes[idx].set_state("done")

    def set_step_error(self, idx: int, error_text: str) -> None:
        if 0 <= idx < len(self._step_boxes):
            self._step_boxes[idx].set_state("error", error_text=error_text)

    def set_series_data(self, data: Dict[str, List[Tuple[int, float]]], tags: List[str]) -> None:
        self.plot.set_series_data(data, tags, self._plot_minutes)  # type: ignore[attr-defined]

    def append_points(self, points: List[Tuple[str, int, float]], tags: List[str]) -> None:
        self.plot.append_points(points, tags, self._plot_minutes)  # type: ignore[attr-defined]

    def set_plot_minutes(self, minutes: int) -> None:
        self._plot_minutes = int(clamp(int(minutes), 1, 30))

    def set_pause_resume_action(self, mode: str, enabled: bool = True) -> None:
        m = str(mode or "pause").strip().lower()
        if m == "resume":
            txt = "Resume"
        elif m in ("clear_error_resume", "clear-error-resume"):
            txt = "Clear Error && Resume"
        else:
            txt = "Pause"
        self.btn_pause_resume.setText(txt)
        self.btn_pause_resume.setEnabled(bool(enabled))

    @Slot()
    def _open_options_dialog(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Recipe Run Trend Options")
        dlg.setModal(True)
        dlg.setMinimumWidth(ui(700))

        root = QVBoxLayout(dlg)
        root.setContentsMargins(ui(16), ui(16), ui(16), ui(16))
        root.setSpacing(ui(12))

        lbl_tags = QLabel("Tags")
        lbl_tags.setObjectName("SectionTitle")
        root.addWidget(lbl_tags)

        tag_checks: Dict[str, QCheckBox] = {}
        ordered_tags = [TAG_PWR_W, TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM, TAG_MOTOR_TEMP_C, TAG_VBUS_V, TAG_IBUS_A]
        wanted = set(self.selected_tags())
        for tag in ordered_tags:
            cb = QCheckBox(TAG_LABELS.get(tag, tag))
            cb.setMinimumHeight(ui(44))
            cb.setChecked(tag in wanted)
            tag_checks[tag] = cb
            root.addWidget(cb)

        buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        btn_apply = buttons.addButton("Apply", QDialogButtonBox.AcceptRole)
        btn_apply.setMinimumHeight(ui(58))
        root.addWidget(buttons)

        buttons.rejected.connect(dlg.reject)
        buttons.accepted.connect(dlg.accept)

        if dlg.exec() != QDialog.Accepted:
            return

        selected = [t for t in ordered_tags if tag_checks[t].isChecked()]
        self.set_selected_tags(selected)
        self.tags_changed.emit(self.selected_tags())


# ----------------------------
# Home Screen
# ----------------------------

class HomeScreen(QWidget):
    start_clicked = Signal()
    stop_clicked = Signal()
    calibrate_clicked = Signal()
    open_settings_clicked = Signal()
    open_builder_clicked = Signal()
    import_recipe_clicked = Signal()
    recipe_select_clicked = Signal(str)
    recipe_edit_clicked = Signal(str)
    open_datalogger_clicked = Signal()
    open_recipe_run_clicked = Signal()
    open_previous_runs_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(ui(18), ui(16), ui(18), ui(16))
        root.setSpacing(ui(12))

        top_row = QHBoxLayout()
        top_row.setSpacing(ui(12))
        self.status_bar = StatusHeader()
        self.btn_settings = TouchButton("\u2699", min_h=66, min_w=66)
        self.btn_settings.setObjectName("SettingsButton")
        self.btn_settings.setToolTip("Settings")
        top_row.addWidget(self.status_bar, 1)
        top_row.addWidget(self.btn_settings)
        root.addLayout(top_row)

        mid = QHBoxLayout()
        mid.setSpacing(ui(14))

        left = QVBoxLayout()
        left.setSpacing(ui(12))
        self.rpm_ctl = RpmControl("Velocity", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM, compact=False)
        self.time_ctl = TimeControl("Timer", TIME_STEP_S, compact=False)
        self.btn_datalogger = TouchButton("Data Logger", min_h=70, min_w=240)
        self.btn_recipe_run = TouchButton("Recipe Run", min_h=70, min_w=240)
        self.btn_previous_runs = TouchButton("Previous Runs", min_h=70, min_w=240)
        self.btn_calibrate = TouchButton("Calibrate Motor", min_h=70, min_w=240)

        left.addWidget(self.rpm_ctl)
        left.addWidget(self.time_ctl)
        left.addWidget(self.btn_calibrate)
        left.addWidget(self.btn_datalogger)
        left.addWidget(self.btn_recipe_run)
        left.addWidget(self.btn_previous_runs)
        left.addStretch(1)
        mid.addLayout(left, 1)

        recipes_col = QVBoxLayout()
        recipes_col.setSpacing(ui(8))
        title = QLabel("Recipes")
        title.setObjectName("SectionTitle")
        recipes_col.addWidget(title)

        self.recipes_area = QScrollArea()
        self.recipes_area.setWidgetResizable(True)
        self.recipes_area.setFrameShape(QFrame.NoFrame)
        enable_touch_scrolling(self.recipes_area)
        self.recipes_container = QWidget()
        self.recipes_layout = QVBoxLayout(self.recipes_container)
        self.recipes_layout.setContentsMargins(0, 0, 0, 0)
        self.recipes_layout.setSpacing(ui(10))
        self.recipes_layout.addStretch(1)
        self.recipes_area.setWidget(self.recipes_container)
        recipes_col.addWidget(self.recipes_area, 1)
        import_row = QHBoxLayout()
        import_row.setSpacing(ui(10))
        self.btn_import_recipe = TouchButton("Import JSON", min_h=66, min_w=220)
        self.btn_add_recipe = TouchButton("+ New Recipe", min_h=66, min_w=220)
        import_row.addWidget(self.btn_import_recipe, 1)
        import_row.addWidget(self.btn_add_recipe, 1)
        recipes_col.addLayout(import_row)

        mid.addLayout(recipes_col, 2)
        root.addLayout(mid, 1)

        self.footer = StartStopFooter()
        root.addWidget(self.footer)

        self.footer.start_clicked.connect(self.start_clicked.emit)
        self.footer.stop_clicked.connect(self.stop_clicked.emit)
        self.btn_calibrate.clicked.connect(self.calibrate_clicked.emit)
        self.btn_settings.clicked.connect(self.open_settings_clicked.emit)
        self.btn_datalogger.clicked.connect(self.open_datalogger_clicked.emit)
        self.btn_recipe_run.clicked.connect(self.open_recipe_run_clicked.emit)
        self.btn_previous_runs.clicked.connect(self.open_previous_runs_clicked.emit)
        self.btn_import_recipe.clicked.connect(self.import_recipe_clicked.emit)
        self.btn_add_recipe.clicked.connect(self.open_builder_clicked.emit)

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

    def set_running_ui(
        self,
        running: bool,
        run_mode: str,
        recipe_selected: bool,
        motor_controls_enabled: bool,
        connected: bool,
        calibrated: bool,
        calibrating: bool,
        recipe_run_active: bool = False,
    ) -> None:
        """
        Home-screen behaviors:
          - Start disabled while running
          - Stop enabled while running
          - RPM control enabled while running (SINGLE + RECIPE override)
          - Recipe selection disabled while running
          - Builder/config navigation disabled while running
          - Data Logger navigation enabled during run
          - Manual timer disabled when recipe selected while idle
          - While running SINGLE mode: timer text disabled, +/- enabled (adjusts remaining)
        """
        self.footer.set_running_ui(running)
        self.footer.set_motor_controls_enabled(motor_controls_enabled)
        self.rpm_ctl.set_enabled(motor_controls_enabled)

        if running:
            if run_mode == "single":
                self.time_ctl.setEnabled(True)
                self.time_ctl.set_dimmed(False)
                self.time_ctl.set_buttons_enabled(motor_controls_enabled)
                self.time_ctl.set_edit_enabled(False)
            else:
                self.time_ctl.setEnabled(True)
                self.time_ctl.set_dimmed(True)
                self.time_ctl.set_enabled(False)
        else:
            if recipe_selected:
                self.time_ctl.setEnabled(False)
                self.time_ctl.set_dimmed(True)
                self.time_ctl.set_enabled(False)
            else:
                self.time_ctl.setEnabled(True)
                self.time_ctl.set_dimmed(False)
                self.time_ctl.set_enabled(True)

        self.btn_calibrate.setEnabled((not running) and connected and (not calibrating))
        self.btn_calibrate.setText("Calibrating..." if calibrating else ("Recalibrate Motor" if calibrated else "Calibrate Motor"))
        self.btn_datalogger.setEnabled(True)
        self.btn_recipe_run.setEnabled(bool(recipe_run_active))
        self.btn_previous_runs.setEnabled(not running)
        self.btn_import_recipe.setEnabled(not running)
        self.btn_add_recipe.setEnabled(not running)
        self.recipes_area.setEnabled(not running)


# ----------------------------
# Controller
# ----------------------------

class AppController(QObject):
    request_connect = Signal()
    request_check_calibration = Signal()
    request_run_calibration = Signal()
    request_read_config = Signal(object)          # list[str]
    request_apply_config = Signal(object)         # dict (live apply, no save)
    request_save_config = Signal(object)          # dict (save + reboot)
    request_erase_config = Signal()               # erase + reboot
    request_clear_errors = Signal()
    request_export_json = Signal()
    request_set_polled_tags = Signal(object)      # list[str]
    request_start_poll = Signal()
    request_stop_poll = Signal()

    def __init__(
        self,
        stack: QStackedWidget,
        home: HomeScreen,
        builder: RecipeBuilderScreen,
        settings: SettingsHubScreen,
        hmi_settings: HMISettingsScreen,
        recipe_run: RecipeRunMonitorScreen,
        cfg: ODriveConfigScreen,
        dlog: DataLoggerScreen,
    ) -> None:
        super().__init__()
        self.stack, self.home, self.builder, self.settings, self.hmi_settings, self.recipe_run, self.cfg, self.dlog = (
            stack,
            home,
            builder,
            settings,
            hmi_settings,
            recipe_run,
            cfg,
            dlog,
        )

        self.store = RecipeStore()
        self.store.load()
        self.run_store = RecipeRunStore()
        self.run_store.load()
        self.selected_recipe_id: Optional[str] = None

        self.connected = False
        self.backend_state = "Disconnected"
        self.motor_calibrated = (BACKEND != "real")
        self.calibration_detail = "Unknown"
        self._calibration_busy = False

        self._ui_state = "Idle"
        self._ui_rpm = 0.0
        self._ui_remaining = 0
        self._ui_step_info = ""
        self._run_mode = "idle"
        self._last_run_mode = "idle"
        self._cmd_rpm = 0.0

        self._dlog_minutes = 10
        self._dlog_selected_tags: List[str] = list(DEFAULT_ENABLED_TAGS)
        self._reset_defaults_after_erase_pending = False
        self._reset_defaults_auto_save_in_progress = False
        self._hmi_fullscreen_pref = bool(START_FULLSCREEN)
        self._hmi_max_velocity_rpm = float(HMI_MAX_VEL_DEFAULT_RPM)
        self._recipe_monitor_recipe: Optional[Recipe] = None
        self._recipe_monitor_step_idx = -1
        self._recipe_monitor_failed = False
        self._recipe_monitor_error_paused = False
        self._recipe_resume_after_clear_error = False
        self._recipe_monitor_plot_tags: List[str] = [TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM]
        self._current_recipe_run_record: Optional[Dict[str, Any]] = None
        self._cfg_runtime_fault_popup_suppressed = False
        self._cfg_op_in_progress = ""
        self._last_cfg_apply_values: Dict[str, Any] = {}

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
        self.request_check_calibration.connect(self.worker.check_calibration)
        self.request_run_calibration.connect(self.worker.run_calibration)
        self.request_read_config.connect(self.worker.read_config)
        self.request_apply_config.connect(self.worker.apply_config)
        self.request_save_config.connect(self.worker.save_config)
        self.request_erase_config.connect(self.worker.erase_config)
        self.request_clear_errors.connect(self.worker.clear_errors)
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
        self.worker.config_saved.connect(self._on_config_saved)
        self.worker.config_erased.connect(self._on_config_erased)
        self.worker.export_json_done.connect(self._on_export_json_done)
        self.worker.errors_cleared.connect(self._on_errors_cleared)
        self.worker.tags_sample.connect(self._on_tags_sample)
        self.worker.calibration_checked.connect(self._on_calibration_checked)
        self.worker.calibration_done.connect(self._on_calibration_done)
        self.worker.watchdog_status_changed.connect(self.cfg.set_watchdog_status)

        self.engine.status_changed.connect(lambda s: self._set_ui(status=s))
        self.engine.run_mode_changed.connect(lambda m: setattr(self, "_run_mode", m))
        self.engine.run_mode_changed.connect(self._on_engine_run_mode_changed)
        self.engine.active_rpm_changed.connect(self._on_engine_rpm)
        self.engine.remaining_changed.connect(self._on_engine_remaining)
        self.engine.step_info_changed.connect(lambda x: self._set_ui(step=x))
        self.engine.recipe_step_changed.connect(self._on_engine_recipe_step_changed)
        self.engine.recipe_data_entry_requested.connect(self._on_engine_recipe_data_entry_requested)

        self.home.start_clicked.connect(self.on_start)
        self.home.stop_clicked.connect(self.on_stop)
        self.home.calibrate_clicked.connect(self.on_calibrate)
        self.home.open_settings_clicked.connect(self.on_open_settings)
        self.home.open_builder_clicked.connect(self.on_open_builder_new)
        self.home.import_recipe_clicked.connect(self.on_import_recipe)
        self.home.recipe_select_clicked.connect(self.on_select_recipe)
        self.home.recipe_edit_clicked.connect(self.on_edit_recipe)
        self.home.open_datalogger_clicked.connect(self.on_open_datalogger)
        self.home.open_previous_runs_clicked.connect(self.on_open_previous_runs)
        self.home.open_recipe_run_clicked.connect(self.on_open_recipe_run)

        self.home.rpm_ctl.value_changed.connect(self._on_home_rpm_changed)
        self.home.time_ctl.value_changed.connect(self._on_home_time_changed)
        self.home.rpm_ctl.edit_focused.connect(self._on_home_manual_field_focused)
        self.home.time_ctl.edit_focused.connect(self._on_home_manual_field_focused)

        self.builder.back_clicked.connect(self.on_back_home)
        self.builder.cancel_clicked.connect(self.on_back_home)
        self.builder.save_clicked.connect(self.on_builder_save)
        self.builder.export_clicked.connect(self.on_builder_export)
        self.builder.delete_clicked.connect(self.on_builder_delete)

        self.settings.back_clicked.connect(self.on_back_home)
        self.settings.open_hmi_clicked.connect(self.on_open_hmi_settings)
        self.settings.open_odrive_clicked.connect(self.on_open_config)

        self.hmi_settings.back_clicked.connect(self.on_back_settings)
        self.recipe_run.back_clicked.connect(self.on_back_home)
        self.recipe_run.stop_clicked.connect(self.on_stop)
        self.recipe_run.pause_resume_clicked.connect(self.on_recipe_pause_resume)
        self.recipe_run.skip_step_clicked.connect(self.on_recipe_skip_step)
        self.recipe_run.tags_changed.connect(self._on_recipe_run_tags_changed)
        self.recipe_run.generate_report_clicked.connect(self.on_generate_current_recipe_report)
        self.hmi_settings.fullscreen_changed.connect(self.on_hmi_fullscreen_changed)

        self.cfg.back_clicked.connect(self.on_back_home)
        self.cfg.request_refresh.connect(self._refresh_odrive_config)
        self.cfg.apply_clicked.connect(self._apply_odrive_config)
        self.cfg.save_reboot_clicked.connect(self._save_odrive_config)
        self.cfg.erase_reset_clicked.connect(self._erase_reset_odrive_config)
        self.cfg.clear_errors_clicked.connect(self._clear_odrive_errors)
        self.cfg.export_json_clicked.connect(self._export_odrive_json)
        self.cfg.hmi_max_velocity_changed.connect(self.on_hmi_max_velocity_changed)

        self.dlog.back_clicked.connect(self.on_back_home)
        self.dlog.start_clicked.connect(self.on_start)
        self.dlog.stop_clicked.connect(self.on_stop)
        self.dlog.window_changed.connect(self._on_dlog_window_changed)
        self.dlog.tags_changed.connect(self._on_dlog_tags_changed)
        self.dlog.export_clicked.connect(self._export_datalogger_csv)

        self.stack.currentChanged.connect(self._on_screen_changed)

        self._last_modal_error_mono = 0.0
        self._modal_error_cooldown_s = 8.0
        self._last_odrive_error_modal_mono = 0.0
        self._odrive_error_modal_cooldown_s = 1.0

        self._ui_state_path = app_config_dir() / "ui_state.json"
        self._ui_state_saving = False
        self._ui_state_debounce = QTimer(self)
        self._ui_state_debounce.setSingleShot(True)
        self._ui_state_debounce.setInterval(400)
        self._ui_state_debounce.timeout.connect(self._save_ui_state_now)

        self._loading_ui_state = True
        self._load_ui_state()
        self._loading_ui_state = False
        self.hmi_settings.set_fullscreen_state(self._hmi_fullscreen_pref, emit_signal=False)
        self.cfg.set_hmi_max_velocity(self._hmi_max_velocity_rpm, emit_signal=False)
        self.recipe_run.set_selected_tags(self._recipe_monitor_plot_tags)
        self._apply_hmi_velocity_limit()

        self.worker_thread.start()
        self.request_connect.emit()
        self.request_set_polled_tags.emit([TAG_VBUS_V, TAG_IBUS_A, TAG_VEL_RPM, TAG_TORQUE_NM])

        self._refresh_recipes()
        self._refresh_status_all()
        self._apply_controls()

    def shutdown(self) -> None:
        self._save_ui_state_now()
        try:
            self.request_stop_poll.emit()
        except Exception:
            pass
        try:
            if self.engine.is_running():
                self.engine.stop(user_initiated=True)
            QMetaObject.invokeMethod(self.worker, "stop", Qt.QueuedConnection)
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
        box = QMessageBox(parent)
        box.setIcon(QMessageBox.Critical)
        box.setWindowTitle(title)
        box.setText(str(msg))
        try:
            box.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
        except Exception:
            pass
        box.setStandardButtons(QMessageBox.Ok)
        box.exec()

    def _show_odrive_error_with_clear(self, parent: QWidget, msg: str) -> None:
        now_m = time.monotonic()
        if (now_m - self._last_odrive_error_modal_mono) < self._odrive_error_modal_cooldown_s:
            return
        self._last_odrive_error_modal_mono = now_m
        box = QMessageBox(parent)
        box.setIcon(QMessageBox.Critical)
        box.setWindowTitle("ODrive Error")
        box.setText(str(msg))
        try:
            box.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
        except Exception:
            pass
        btn_clear = box.addButton("Clear Error", QMessageBox.ActionRole)
        btn_close = box.addButton(QMessageBox.Close)
        box.setDefaultButton(btn_close)
        box.exec()
        if box.clickedButton() is btn_clear:
            self._clear_odrive_errors()

    def _selected_recipe(self) -> Optional[Recipe]:
        if not self.selected_recipe_id:
            return None
        for r in self.store.recipes():
            if r.id == self.selected_recipe_id:
                return r
        return None

    def _refresh_recipes(self) -> None:
        self.home.set_recipe_cards(self.store.recipes(), self.selected_recipe_id)

    def _recipe_run_audit(self, text: str) -> None:
        msg = str(text)
        self.recipe_run.append_audit(msg)
        if self._current_recipe_run_record is not None:
            self._current_recipe_run_record.setdefault("audit", []).append(
                {"ts": int(epoch_s()), "text": msg}
            )

    def _refresh_status_all(self) -> None:
        r = self._selected_recipe()
        rname = r.name if r else "(none)"
        self.home.set_status(self._ui_state, self._ui_rpm, self._ui_remaining, rname, self._ui_step_info, self.backend_state)
        self.dlog.set_status(self._ui_state, self._ui_rpm, self._ui_remaining, rname, self._ui_step_info, self.backend_state)

    def _apply_controls(self) -> None:
        r_selected = self._selected_recipe() is not None
        running = self.engine.is_running()
        motor_controls_enabled = bool(self.connected and self.motor_calibrated and (not self._calibration_busy))
        recipe_run_active = bool(running and self._run_mode == "recipe" and self._recipe_monitor_recipe is not None)

        if running and self._run_mode == "single" and not r_selected:
            self.home.time_ctl.set_adjust_callback(self._adjust_time_from_home)
        else:
            self.home.time_ctl.set_adjust_callback(None)

        self.home.set_running_ui(
            running=running,
            run_mode=self._run_mode,
            recipe_selected=r_selected,
            motor_controls_enabled=motor_controls_enabled,
            connected=self.connected,
            calibrated=self.motor_calibrated,
            calibrating=self._calibration_busy,
            recipe_run_active=recipe_run_active,
        )
        self.dlog.set_running_ui(running=running, motor_controls_enabled=motor_controls_enabled)

        recipe_pause_enabled = bool(self._recipe_monitor_recipe is not None and self._run_mode == "recipe")
        if not recipe_pause_enabled:
            self.recipe_run.set_pause_resume_action("pause", enabled=False)
        elif self._recipe_monitor_error_paused:
            self.recipe_run.set_pause_resume_action("clear_error_resume", enabled=True)
        elif self.engine.is_paused():
            self.recipe_run.set_pause_resume_action("resume", enabled=True)
        else:
            self.recipe_run.set_pause_resume_action("pause", enabled=True)

    def _adjust_time_from_home(self, delta_s: int) -> None:
        if self._run_mode != "single":
            return
        self.engine.adjust_single_remaining(int(delta_s))

    def _set_ui(self, status: Optional[str] = None, step: Optional[str] = None) -> None:
        if status is not None:
            self._ui_state = status
        if step is not None:
            self._ui_step_info = step
        self._refresh_status_all()
        self._apply_controls()

    # ---- UI state persistence ----

    def _known_tags(self) -> List[str]:
        return [TAG_PWR_W, TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM, TAG_MOTOR_TEMP_C, TAG_VBUS_V, TAG_IBUS_A]

    def _load_ui_state(self) -> None:
        if not self._ui_state_path.exists():
            self._apply_loaded_ui_state_defaults()
            return
        try:
            raw = json.loads(self._ui_state_path.read_text(encoding="utf-8"))
        except Exception:
            self._apply_loaded_ui_state_defaults()
            return

        home_rpm = clamp(safe_float(raw.get("home_rpm", 120.0), 120.0), VEL_MIN_RPM, VEL_MAX_RPM)
        home_timer = raw.get("home_timer_mmss", "00:00")
        secs = parse_mmss(str(home_timer)) if home_timer is not None else None
        if secs is None:
            secs = 0
        secs = int(clamp(int(secs), 0, TIME_MAX_S))
        timer_mmss = format_mmss(secs)

        dlog_minutes = raw.get("dlog_minutes", 10)
        try:
            dlog_minutes_i = int(dlog_minutes)
        except Exception:
            dlog_minutes_i = 10
        dlog_minutes_i = int(clamp(dlog_minutes_i, 1, 24 * 60))

        tags_raw = raw.get("dlog_tags", list(DEFAULT_ENABLED_TAGS))
        tags_list = [str(t) for t in (tags_raw or []) if str(t) in set(self._known_tags())]
        if not tags_list:
            tags_list = list(DEFAULT_ENABLED_TAGS)

        recipe_id = raw.get("selected_recipe_id")
        recipe_id = str(recipe_id) if recipe_id else None
        if recipe_id and not any(r.id == recipe_id for r in self.store.recipes()):
            recipe_id = None

        fullscreen_pref = raw.get("hmi_fullscreen", START_FULLSCREEN)
        self._hmi_fullscreen_pref = bool(fullscreen_pref)
        self._hmi_max_velocity_rpm = float(
            clamp(safe_float(raw.get("hmi_max_velocity_rpm", HMI_MAX_VEL_DEFAULT_RPM), HMI_MAX_VEL_DEFAULT_RPM), VEL_MIN_RPM, VEL_MAX_RPM)
        )
        home_rpm = clamp(home_rpm, VEL_MIN_RPM, self._hmi_velocity_limit())

        self.home.rpm_ctl.set_value(home_rpm, emit_signal=False)
        self.home.time_ctl.set_text_mmss(timer_mmss, emit_signal=False)

        self._dlog_minutes = dlog_minutes_i
        self._dlog_selected_tags = list(tags_list)
        self.dlog.set_minutes_window(self._dlog_minutes)
        self.dlog.set_selected_tags(self._dlog_selected_tags)

        self.selected_recipe_id = recipe_id

    def _apply_loaded_ui_state_defaults(self) -> None:
        self.home.rpm_ctl.set_value(120.0, emit_signal=False)
        self.home.time_ctl.set_text_mmss("00:00", emit_signal=False)
        self._dlog_minutes = 10
        self._dlog_selected_tags = list(DEFAULT_ENABLED_TAGS)
        self.dlog.set_minutes_window(self._dlog_minutes)
        self.dlog.set_selected_tags(self._dlog_selected_tags)
        self.selected_recipe_id = None
        self._hmi_fullscreen_pref = bool(START_FULLSCREEN)
        self._hmi_max_velocity_rpm = float(HMI_MAX_VEL_DEFAULT_RPM)

    def _schedule_ui_state_save(self) -> None:
        if self._loading_ui_state:
            return
        if self._ui_state_debounce.isActive():
            self._ui_state_debounce.start()
        else:
            self._ui_state_debounce.start()

    def _collect_ui_state(self) -> Dict[str, Any]:
        home_rpm = clamp(self.home.rpm_ctl.value(), VEL_MIN_RPM, self._hmi_velocity_limit())
        secs = parse_mmss(self.home.time_ctl.text_mmss())
        if secs is None:
            secs = 0
        secs = int(clamp(int(secs), 0, TIME_MAX_S))
        timer_mmss = format_mmss(secs)

        tags = self._dlog_selected_tags or list(DEFAULT_ENABLED_TAGS)
        tags = [t for t in tags if t in set(self._known_tags())]
        if not tags:
            tags = list(DEFAULT_ENABLED_TAGS)

        minutes = int(clamp(int(self._dlog_minutes), 1, 24 * 60))
        recipe_id = self.selected_recipe_id or None

        return {
            "home_rpm": float(home_rpm),
            "home_timer_mmss": str(timer_mmss),
            "dlog_tags": list(tags),
            "dlog_minutes": int(minutes),
            "selected_recipe_id": recipe_id,
            "hmi_fullscreen": bool(self._hmi_fullscreen_pref),
            "hmi_max_velocity_rpm": float(self._hmi_max_velocity_rpm),
        }

    def hmi_fullscreen_preference(self) -> bool:
        return bool(self._hmi_fullscreen_pref)

    def set_hmi_fullscreen_preference(self, enabled: bool) -> None:
        self._hmi_fullscreen_pref = bool(enabled)
        self._schedule_ui_state_save()

    def _hmi_velocity_limit(self) -> float:
        return float(clamp(float(self._hmi_max_velocity_rpm), VEL_MIN_RPM, VEL_MAX_RPM))

    def _apply_hmi_velocity_limit(self) -> None:
        vmax = self._hmi_velocity_limit()
        self.home.rpm_ctl.set_range(VEL_MIN_RPM, vmax)
        self.builder.set_velocity_limit(vmax)
        # Clamp current entry values to the new limit without emitting changes.
        self.home.rpm_ctl.set_value(self.home.rpm_ctl.value(), emit_signal=False)

    @Slot()
    def _save_ui_state_now(self) -> None:
        if self._ui_state_saving:
            return
        self._ui_state_saving = True
        try:
            obj = self._collect_ui_state()
            tmp = self._ui_state_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(obj, indent=2), encoding="utf-8")
            tmp.replace(self._ui_state_path)
        except Exception:
            pass
        finally:
            self._ui_state_saving = False

    # ---- Worker callbacks ----

    @Slot(bool, str)
    def _on_connected(self, connected: bool, state: str) -> None:
        self.connected = bool(connected)
        self.backend_state = state

        if connected:
            self._calibration_busy = False
            self.motor_calibrated = False if BACKEND == "real" else True
            self.calibration_detail = "Checking calibration..."
            self.request_start_poll.emit()
            self.request_check_calibration.emit()
            if self.stack.currentWidget() is self.cfg:
                self._refresh_odrive_config()
        else:
            self._calibration_busy = False
            self.motor_calibrated = False if BACKEND == "real" else True
            self.calibration_detail = "Disconnected"
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
        msg = str(msg or "")
        if self._cfg_runtime_fault_popup_suppressed and msg.lower().startswith("odrive runtime fault:"):
            # Config apply/save/erase can transiently report INVALID_STATE while writes succeed.
            if self.stack.currentWidget() is self.cfg:
                self.cfg.set_loading(f"Config op in progress ({self._cfg_op_in_progress or 'odrive'}): transient runtime fault suppressed")
            return
        self._calibration_busy = False
        if self._run_mode == "recipe" and self._recipe_monitor_recipe is not None:
            self._recipe_monitor_error_paused = True
            if self._recipe_monitor_step_idx >= 0:
                self.recipe_run.set_step_error(self._recipe_monitor_step_idx, msg)
            self._recipe_run_audit(f"Error: {msg}")
            if self.engine.is_running():
                self.engine.pause()
        elif self.engine.is_running():
            self.engine.stop(user_initiated=True)
        self._show_odrive_error_with_clear(self.stack.currentWidget(), msg)
        self._refresh_status_all()
        self._apply_controls()

    def _project_reset_config_values(self) -> Dict[str, Any]:
        values: Dict[str, Any] = dict(DEFAULT_CONFIG_MAP)
        for key, value in SCRIPT_DEFAULT_ASSIGNMENTS:
            values[str(key)] = value
        return values

    def _report_wrap_lines(self, text: str, max_chars: int = 110) -> List[str]:
        s = str(text or "")
        if not s:
            return [""]
        out: List[str] = []
        for para in s.splitlines() or [""]:
            p = para.strip()
            if not p:
                out.append("")
                continue
            while len(p) > max_chars:
                cut = p.rfind(" ", 0, max_chars + 1)
                if cut <= 0:
                    cut = max_chars
                out.append(p[:cut].rstrip())
                p = p[cut:].lstrip()
            out.append(p)
        return out

    def _draw_report_chart(
        self,
        canv: Any,
        page_w: float,
        page_h: float,
        title: str,
        series_by_tag: Dict[str, List[Tuple[int, float]]],
    ) -> None:
        margin = 48.0
        canv.setFont("Helvetica-Bold", 16)
        canv.drawString(margin, page_h - margin, str(title))

        chart_x = margin
        chart_y = 120.0
        chart_w = page_w - 2 * margin
        chart_h = page_h - 200.0

        canv.setLineWidth(1)
        canv.rect(chart_x, chart_y, chart_w, chart_h)

        active_tags = [t for t, pts in series_by_tag.items() if pts]
        if not active_tags:
            canv.setFont("Helvetica", 12)
            canv.drawString(chart_x + 12, chart_y + chart_h - 24, "No data in selected run window")
            return

        all_pts = [(ts, float(v)) for t in active_tags for ts, v in series_by_tag.get(t, [])]
        ts_min = min(ts for ts, _ in all_pts)
        ts_max = max(ts for ts, _ in all_pts)
        if ts_max <= ts_min:
            ts_max = ts_min + 1
        y_min = min(v for _, v in all_pts)
        y_max = max(v for _, v in all_pts)
        if not math.isfinite(y_min) or not math.isfinite(y_max):
            y_min, y_max = 0.0, 1.0
        if abs(y_max - y_min) < 1e-9:
            y_max = y_min + 1.0
        pad = 0.05 * (y_max - y_min)
        y_min -= pad
        y_max += pad

        def mx(ts: int) -> float:
            return chart_x + (float(ts - ts_min) / float(ts_max - ts_min)) * chart_w

        def my(v: float) -> float:
            return chart_y + (float(v - y_min) / float(y_max - y_min)) * chart_h

        canv.setFont("Helvetica", 10)
        canv.drawString(chart_x, chart_y - 18, time.strftime("%H:%M:%S", time.localtime(ts_min)))
        canv.drawRightString(chart_x + chart_w, chart_y - 18, time.strftime("%H:%M:%S", time.localtime(ts_max)))
        canv.drawRightString(chart_x - 6, chart_y + chart_h - 2, f"{y_max:.3g}")
        canv.drawRightString(chart_x - 6, chart_y - 2, f"{y_min:.3g}")

        colors = [
            (0.00, 0.75, 1.00),
            (1.00, 0.85, 0.10),
            (0.20, 0.90, 0.35),
            (1.00, 0.25, 0.75),
            (1.00, 0.35, 0.20),
            (0.45, 0.65, 1.00),
        ]

        legend_y = page_h - 78.0
        legend_x = margin
        for i, tag in enumerate(active_tags):
            pts = series_by_tag.get(tag, [])
            if len(pts) < 1:
                continue
            r, g, b = colors[i % len(colors)]
            canv.setStrokeColorRGB(r, g, b)
            canv.setFillColorRGB(r, g, b)
            canv.rect(legend_x, legend_y - 8, 10, 10, fill=1, stroke=0)
            canv.setFillColorRGB(1, 1, 1)
            canv.setFont("Helvetica", 10)
            label = f"{TAG_LABELS.get(tag, tag)} ({TAG_UNITS.get(tag, '')})"
            canv.drawString(legend_x + 14, legend_y - 1, label)
            legend_x += 14 + min(260, 8 * len(label))
            if legend_x > (page_w - margin - 220):
                legend_x = margin
                legend_y -= 16

            canv.setStrokeColorRGB(r, g, b)
            canv.setLineWidth(1.4)
            prev: Optional[Tuple[float, float]] = None
            for ts, val in pts:
                px, py = mx(int(ts)), my(float(val))
                if prev is not None:
                    canv.line(prev[0], prev[1], px, py)
                prev = (px, py)

        canv.setStrokeColorRGB(1, 1, 1)
        canv.setFillColorRGB(1, 1, 1)

    def _export_recipe_run_report(self, run: Dict[str, Any]) -> None:
        try:
            from reportlab.lib.pagesizes import letter  # type: ignore
            from reportlab.pdfgen import canvas  # type: ignore
        except Exception as e:
            self._show_modal_error(self.home, "Report Export", f"ReportLab is not installed:\n{e}")
            return

        run_id = str(run.get("run_id") or str(uuid.uuid4()))
        recipe_name = str(run.get("recipe_name") or "Recipe Run")
        started_ts = int(safe_float(run.get("started_ts", 0), 0))
        ended_ts = int(safe_float(run.get("ended_ts", epoch_s()), epoch_s()))
        if ended_ts <= 0:
            ended_ts = max(started_ts, epoch_s())
        if started_ts <= 0:
            started_ts = max(0, ended_ts - 60)
        if ended_ts < started_ts:
            ended_ts = started_ts

        out = downloads_dir() / f"recipe_run_report_{run_id[:8]}_{time.strftime('%Y%m%d_%H%M%S')}.pdf"

        try:
            data = self.history_db.query_range_multi(
                started_ts,
                ended_ts,
                [TAG_CMD_RPM, TAG_VEL_RPM, TAG_PWR_W, TAG_TORQUE_NM],
            )
        except Exception as e:
            self._show_modal_error(self.home, "Report Export", f"Failed to query run data:\n{e}")
            return

        c = canvas.Canvas(str(out), pagesize=letter)
        page_w, page_h = letter
        margin = 48.0

        # Page 1: Title + metadata + block diagram
        c.setTitle(f"Recipe Run Report - {run_id}")
        c.setFont("Helvetica-Bold", 20)
        c.drawString(margin, page_h - margin, "Recipe Run Report")
        c.setFont("Helvetica", 11)
        meta_lines = [
            f"Recipe: {recipe_name}",
            f"Run ID: {run_id}",
            f"Status: {str(run.get('status') or 'unknown')}",
            f"Started: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(started_ts)) if started_ts > 0 else 'Unknown'}",
            f"Ended: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ended_ts)) if ended_ts > 0 else 'Unknown'}",
            f"Duration: {format_mmss(max(0, ended_ts - started_ts))}",
        ]
        y = page_h - margin - 28
        for line in meta_lines:
            c.drawString(margin, y, line)
            y -= 15

        c.setFont("Helvetica-Bold", 14)
        c.drawString(margin, y - 10, "Block Diagram")
        y -= 26

        steps = list(run.get("recipe_steps") or [])
        if not steps:
            c.setFont("Helvetica", 11)
            c.drawString(margin, y, "No recipe step snapshot saved for this run.")
        else:
            box_x = margin
            box_w = page_w - 2 * margin
            box_h = 42.0
            for i, s in enumerate(steps):
                if y - box_h < 70:
                    c.showPage()
                    y = page_h - margin
                    c.setFont("Helvetica-Bold", 14)
                    c.drawString(margin, y, "Block Diagram (cont.)")
                    y -= 24
                kind = str((s or {}).get("kind", "motor") or "motor")
                name = str((s or {}).get("name") or f"Step {i+1}")
                if kind == "data_entry":
                    prompt = str((s or {}).get("prompt") or "")
                    rkind = str((s or {}).get("response_kind") or "text")
                    detail = f"Data entry [{rkind}] - {prompt}"
                else:
                    rpm = safe_float((s or {}).get("velocity_rpm", 0), 0.0)
                    dur = int(safe_float((s or {}).get("duration_s", 0), 0))
                    detail = f"{rpm:g} rpm for {format_mmss(dur)}"
                c.rect(box_x, y - box_h + 6, box_w, box_h, stroke=1, fill=0)
                c.setFont("Helvetica-Bold", 11)
                c.drawString(box_x + 8, y - 10, f"{i+1}. {name}")
                c.setFont("Helvetica", 10)
                c.drawString(box_x + 8, y - 25, detail[:150])
                y -= (box_h + 8)

        # Audit trail pages
        c.showPage()
        c.setFont("Helvetica-Bold", 16)
        c.drawString(margin, page_h - margin, "Audit Trail")
        y = page_h - margin - 24
        c.setFont("Helvetica", 10)
        audit = list(run.get("audit") or [])
        if not audit:
            c.drawString(margin, y, "No audit entries saved.")
        else:
            for entry in audit:
                ts = int(safe_float((entry or {}).get("ts", 0), 0))
                ttxt = time.strftime("%H:%M:%S", time.localtime(ts)) if ts > 0 else "--:--:--"
                prefix = f"{ttxt}  "
                lines = self._report_wrap_lines(f"{prefix}{str((entry or {}).get('text', ''))}", max_chars=115)
                for line in lines:
                    if y < 60:
                        c.showPage()
                        c.setFont("Helvetica-Bold", 16)
                        c.drawString(margin, page_h - margin, "Audit Trail (cont.)")
                        y = page_h - margin - 24
                        c.setFont("Helvetica", 10)
                    c.drawString(margin, y, line)
                    y -= 13

        # Trend pages
        chart_pages = [
            ("Velocity Trend", {TAG_CMD_RPM: data.get(TAG_CMD_RPM, []), TAG_VEL_RPM: data.get(TAG_VEL_RPM, [])}),
            ("Power Trend", {TAG_PWR_W: data.get(TAG_PWR_W, [])}),
            ("Torque Trend", {TAG_TORQUE_NM: data.get(TAG_TORQUE_NM, [])}),
        ]
        for title_txt, series in chart_pages:
            c.showPage()
            self._draw_report_chart(c, page_w, page_h, title_txt, series)

        c.save()
        ans = QMessageBox.question(
            self.home,
            "Report Export",
            f"Saved report:\n{out}\n\nOpen PDF now?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )
        if ans == QMessageBox.Yes:
            try:
                if hasattr(os, "startfile"):
                    os.startfile(str(out))  # type: ignore[attr-defined]
                else:
                    raise RuntimeError("Opening files is not supported on this platform build.")
            except Exception as e:
                self._show_modal_error(self.home, "Open PDF", f"Failed to open PDF:\n{e}")

    @Slot(bool, str)
    def _on_calibration_checked(self, calibrated: bool, detail: str) -> None:
        self.motor_calibrated = bool(calibrated)
        self.calibration_detail = str(detail)
        self._apply_controls()

    @Slot(bool, str)
    def _on_calibration_done(self, calibrated: bool, detail: str) -> None:
        self._calibration_busy = False
        self.motor_calibrated = bool(calibrated)
        self.calibration_detail = str(detail)
        self._apply_controls()
        if calibrated:
            QMessageBox.information(self.home, "Calibration", detail)
        else:
            self._show_modal_error(self.home, "Calibration", detail)

    @Slot(object, object)
    def _on_config_read(self, values_obj: Any, missing_obj: Any) -> None:
        if self.stack.currentWidget() is self.cfg:
            self.cfg.apply_values(dict(values_obj or {}), list(missing_obj or []))

    @Slot(bool, str, object)
    def _on_config_applied(self, ok: bool, message: str, missing_obj: Any) -> None:
        self._cfg_runtime_fault_popup_suppressed = False
        self._cfg_op_in_progress = ""
        _missing = list(missing_obj or [])
        if ok:
            self.cfg.set_loading("Applied (not saved)")
            ans = QMessageBox.question(
                self.cfg,
                "Configuration Applied",
                f"{message}\n\nSave to ODrive and reboot now?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.Yes,
            )
            if ans == QMessageBox.Yes:
                values = dict(self._last_cfg_apply_values or self.cfg.gather_values())
                self._save_odrive_config(values)
        else:
            self._show_modal_error(self.cfg, "Configuration", message)

    @Slot(bool, str, object)
    def _on_config_saved(self, ok: bool, message: str, missing_obj: Any) -> None:
        self._cfg_runtime_fault_popup_suppressed = False
        self._cfg_op_in_progress = ""
        _missing = list(missing_obj or [])
        if ok:
            QMessageBox.information(self.cfg, "Configuration Saved", str(message))
            self.on_back_home()
        else:
            self._show_modal_error(self.cfg, "Configuration", message)

    @Slot(bool, str)
    def _on_config_erased(self, ok: bool, message: str) -> None:
        self._cfg_runtime_fault_popup_suppressed = False
        self._cfg_op_in_progress = ""
        if ok:
            self.cfg.set_defaults()
            self.cfg.set_loading("Device erased (reconnecting) - form reset to project defaults")
            QMessageBox.information(
                self.cfg,
                "Configuration Erased",
                f"{message}\n\nThe form was reset to project defaults.\nReconnect and use 'Save & Reboot' to write these defaults to the device.",
            )
        else:
            self._show_modal_error(self.cfg, "Configuration", message)

    @Slot(bool, str)
    def _on_export_json_done(self, ok: bool, msg: str) -> None:
        if ok:
            QMessageBox.information(self.cfg, "Export JSON", f"Saved:\n{msg}")
            self.cfg.set_loading("Loaded")
        else:
            self._show_modal_error(self.cfg, "Export JSON", msg)
            self.cfg.set_loading("Loaded")

    @Slot(bool, str)
    def _on_errors_cleared(self, ok: bool, message: str) -> None:
        if self.stack.currentWidget() is self.cfg:
            self.cfg.set_loading("Loaded")
        parent = self.cfg if self.stack.currentWidget() is self.cfg else self.stack.currentWidget()
        auto_resume = bool(ok and self._recipe_resume_after_clear_error and self._recipe_monitor_error_paused and self._run_mode == "recipe")
        if ok:
            if self._recipe_monitor_error_paused and self._run_mode == "recipe":
                self._recipe_monitor_error_paused = False
            if auto_resume:
                self._recipe_run_audit("ODrive errors cleared; resuming recipe")
                self._recipe_resume_after_clear_error = False
                self.engine.resume()
                self._refresh_status_all()
                self._apply_controls()
            else:
                self._apply_controls()
                QMessageBox.information(parent, "Clear Errors", str(message))
        else:
            self._recipe_resume_after_clear_error = False
            self._show_modal_error(parent, "Clear Errors", str(message))

    @Slot(int, object)
    def _on_tags_sample(self, ts: int, values_obj: Any) -> None:
        values: Dict[str, float] = {str(k): safe_float(v) for k, v in dict(values_obj or {}).items()}

        vbus, ibus = values.get(TAG_VBUS_V, float("nan")), values.get(TAG_IBUS_A, float("nan"))
        if math.isfinite(vbus) and math.isfinite(ibus):
            values[TAG_PWR_W] = float(vbus * ibus)
        values[TAG_CMD_RPM] = float(self._cmd_rpm)

        samples: List[Tuple[str, float, float, int]] = []
        for tag in [TAG_VBUS_V, TAG_IBUS_A, TAG_VEL_RPM, TAG_TORQUE_NM, TAG_MOTOR_TEMP_C, TAG_PWR_W, TAG_CMD_RPM]:
            if tag in values and math.isfinite(values[tag]):
                samples.append(
                    (
                        tag,
                        float(values[tag]),
                        float(TAG_DEADBAND.get(tag, 0.0)),
                        int(TAG_HEARTBEAT_S.get(tag, DB_HEARTBEAT_S_DEFAULT)),
                    )
                )

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
        if self.stack.currentWidget() is self.recipe_run and inserted and self._recipe_monitor_recipe is not None:
            selected = set(self._recipe_monitor_plot_tags)
            points = [(tag, ts, float(values[tag])) for tag in inserted if tag in selected]
            if points:
                self.recipe_run.append_points(points, tags=self._recipe_monitor_plot_tags)

    # ---- Engine callbacks ----

    @Slot(float)
    def _on_engine_rpm(self, rpm: float) -> None:
        new_rpm = float(rpm)
        prev_rpm = float(self._cmd_rpm)
        self._ui_rpm = new_rpm
        if abs(new_rpm - prev_rpm) > 1e-9:
            ts = epoch_s()
            try:
                # Record an explicit step edge in the historian: old command value and
                # then the new command value at the same timestamp.
                self.history_db.insert_force(ts=ts, tag=TAG_CMD_RPM, value=prev_rpm)
                self.history_db.insert_force(ts=ts, tag=TAG_CMD_RPM, value=new_rpm)
            except Exception:
                pass
        self._cmd_rpm = new_rpm
        self._log_cmd_rpm()

        # Only force the Home RPM entry to follow the engine while running.
        # When stopped, keep whatever the user last typed (don't overwrite to 0).
        if self.engine.is_running() and not self.home.rpm_ctl.edit.hasFocus():
            self.home.rpm_ctl.set_value(float(rpm), emit_signal=False)

        self._refresh_status_all()
        self._apply_controls()

    @Slot(int)
    def _on_engine_remaining(self, seconds: int) -> None:
        # Remaining countdown is shown ONLY in the status header; the home timer entry must not count down.
        self._ui_remaining = int(seconds)
        if self._run_mode == "recipe" and self._recipe_monitor_recipe is not None and self._recipe_monitor_step_idx >= 0:
            self.recipe_run.set_step_remaining(self._recipe_monitor_step_idx, int(seconds))
        self._refresh_status_all()

    @Slot(int, int)
    def _on_engine_recipe_step_changed(self, step_idx: int, total_steps: int) -> None:
        if self._recipe_monitor_recipe is None:
            return
        prev = self._recipe_monitor_step_idx
        if 0 <= prev < total_steps and prev != step_idx and not self._recipe_monitor_failed:
            self.recipe_run.set_step_done(prev)
        self._recipe_monitor_step_idx = int(step_idx)
        try:
            step = self._recipe_monitor_recipe.steps[step_idx]
            step_name = step.name or f"Step {step_idx + 1}"
            self._recipe_run_audit(f"Step {step_idx + 1} started: {step_name}")
            self.recipe_run.set_step_running(step_idx, int(step.duration_s))
        except Exception:
            self.recipe_run.set_step_running(step_idx, 0)

    @Slot(object, int, int)
    def _on_engine_recipe_data_entry_requested(self, step_obj: Any, step_idx: int, total_steps: int) -> None:
        step = step_obj if isinstance(step_obj, Step) else Step(**dict(step_obj)) if isinstance(step_obj, dict) else None
        if step is None:
            self._recipe_run_audit(f"Step {int(step_idx)+1}: data entry prompt missing; continuing")
            self.engine.continue_after_data_entry()
            return

        parent = self.recipe_run if self.stack.currentWidget() is self.recipe_run else self.home
        title = (step.name or f"Step {int(step_idx)+1}").strip()
        prompt = (step.prompt or title or "Enter value").strip()
        response_kind = str(getattr(step, "response_kind", "text") or "text")
        allow_skip = bool(getattr(step, "allow_skip", True))

        while True:
            dlg = QDialog(parent)
            dlg.setWindowTitle(f"Recipe Data Entry ({int(step_idx)+1}/{int(total_steps)})")
            dlg.setModal(True)
            dlg.setMinimumWidth(ui(700))
            dlg.setWindowFlag(Qt.WindowCloseButtonHint, False)

            root = QVBoxLayout(dlg)
            root.setContentsMargins(ui(16), ui(16), ui(16), ui(16))
            root.setSpacing(ui(12))

            lbl_title = QLabel(title)
            lbl_title.setObjectName("SectionTitle")
            root.addWidget(lbl_title)

            lbl_prompt = QLabel(prompt)
            lbl_prompt.setObjectName("InfoLabel")
            lbl_prompt.setWordWrap(True)
            root.addWidget(lbl_prompt)

            editor: Optional[QWidget] = None
            if response_kind == "pass_fail":
                combo = QComboBox()
                combo.setMinimumHeight(ui(66))
                combo.addItem("Pass", "pass")
                combo.addItem("Fail", "fail")
                editor = combo
                root.addWidget(combo)
            else:
                edit = QLineEdit()
                edit.setMinimumHeight(ui(66))
                if response_kind == "numeric":
                    edit.setValidator(QDoubleValidator(edit))
                    edit.setPlaceholderText("Enter numeric value")
                else:
                    edit.setPlaceholderText("Enter text")
                editor = edit
                root.addWidget(edit)

            buttons = QDialogButtonBox()
            btn_submit = buttons.addButton("Submit", QDialogButtonBox.AcceptRole)
            btn_submit.setMinimumHeight(ui(58))
            btn_skip = None
            if allow_skip:
                btn_skip = buttons.addButton("Skip", QDialogButtonBox.ActionRole)
                btn_skip.setMinimumHeight(ui(58))
            root.addWidget(buttons)

            result: Dict[str, Any] = {"action": "cancel", "value": None}

            def on_accept() -> None:
                if response_kind == "pass_fail":
                    assert isinstance(editor, QComboBox)
                    result["action"] = "submit"
                    result["value"] = str(editor.currentData() or editor.currentText() or "")
                    dlg.accept()
                    return
                assert isinstance(editor, QLineEdit)
                txt = editor.text().strip()
                if response_kind == "numeric":
                    if not txt:
                        QMessageBox.warning(dlg, "Data Entry", "Enter a numeric value or choose Skip.")
                        return
                    try:
                        result["value"] = float(txt)
                    except Exception:
                        QMessageBox.warning(dlg, "Data Entry", "Numeric value is invalid.")
                        return
                else:
                    if not txt:
                        QMessageBox.warning(dlg, "Data Entry", "Enter a value or choose Skip.")
                        return
                    result["value"] = txt
                result["action"] = "submit"
                dlg.accept()

            buttons.accepted.connect(on_accept)
            if btn_skip is not None:
                btn_skip.clicked.connect(lambda: (result.update({"action": "skip", "value": None}), dlg.accept()))

            dlg.exec()

            action = str(result.get("action", "cancel"))
            if action == "cancel":
                if allow_skip:
                    action = "skip"
                else:
                    continue

            if action == "skip":
                self._recipe_run_audit(f"Step {int(step_idx)+1} data entry skipped: {prompt}")
                if self._current_recipe_run_record is not None:
                    self._current_recipe_run_record.setdefault("data_entries", []).append(
                        {
                            "ts": int(epoch_s()),
                            "step_idx": int(step_idx),
                            "step_name": title,
                            "prompt": prompt,
                            "response_kind": response_kind,
                            "skipped": True,
                            "value": None,
                        }
                    )
                self.engine.continue_after_data_entry()
                return

            val = result.get("value")
            val_txt = f"{val:g}" if isinstance(val, (int, float)) else str(val)
            self._recipe_run_audit(f"Step {int(step_idx)+1} data entry: {prompt} = {val_txt}")
            if self._current_recipe_run_record is not None:
                self._current_recipe_run_record.setdefault("data_entries", []).append(
                    {
                        "ts": int(epoch_s()),
                        "step_idx": int(step_idx),
                        "step_name": title,
                        "prompt": prompt,
                        "response_kind": response_kind,
                        "skipped": False,
                        "value": val,
                    }
                )
            self.engine.continue_after_data_entry()
            return

    @Slot(str)
    def _on_engine_run_mode_changed(self, mode: str) -> None:
        prev_mode = getattr(self, "_last_run_mode", "idle")
        if prev_mode == "recipe" and mode == "idle" and self._recipe_monitor_recipe is not None:
            if not self._recipe_monitor_failed and self._recipe_monitor_step_idx >= 0:
                self.recipe_run.set_step_done(self._recipe_monitor_step_idx)
                self._recipe_monitor_finish("Recipe completed")
            elif self._recipe_monitor_failed:
                self._recipe_monitor_finish("Recipe ended with error")
        self._last_run_mode = str(mode)

    # ---- Screen changes ----

    @Slot(int)
    def _on_screen_changed(self, _idx: int) -> None:
        if self.stack.currentWidget() is self.dlog:
            self._dlog_minutes = self.dlog.minutes_window()
            self._dlog_selected_tags = self.dlog.selected_tags() or list(DEFAULT_ENABLED_TAGS)
            self._refresh_datalogger_plot()
        if self.stack.currentWidget() is self.recipe_run:
            self._refresh_recipe_monitor_plot()
        self._schedule_ui_state_save()

    # ---- Home field changes ----

    @Slot(float)
    def _on_home_rpm_changed(self, rpm: float) -> None:
        rpm = float(clamp(float(rpm), VEL_MIN_RPM, self._hmi_velocity_limit()))
        self._switch_home_to_manual_timer_mode()
        if self.engine.is_running():
            self.engine.override_rpm(rpm)
        self._schedule_ui_state_save()

    @Slot(int)
    def _on_home_time_changed(self, _seconds: int) -> None:
        # Save the entry text (mm:ss) when edited while stopped. During a run, the entry is disabled.
        self._switch_home_to_manual_timer_mode()
        self._schedule_ui_state_save()

    @Slot()
    def _on_home_manual_field_focused(self) -> None:
        self._switch_home_to_manual_timer_mode()

    def _switch_home_to_manual_timer_mode(self) -> None:
        if self.engine.is_running():
            return
        if self.selected_recipe_id is None:
            return
        self.selected_recipe_id = None
        self._refresh_recipes()
        self._refresh_status_all()
        self._apply_controls()
        self._schedule_ui_state_save()

    # ---- Home actions ----

    @Slot()
    def on_start(self) -> None:
        if self.engine.is_running():
            return
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.home, "ODrive Not Connected", "Cannot start: ODrive is not connected.")
            return
        if BACKEND == "real" and not self.motor_calibrated:
            QMessageBox.warning(self.home, "Calibration Required", "Cannot start: calibrate the motor before starting.")
            return

        rpm = clamp(self.home.rpm_ctl.value(), VEL_MIN_RPM, self._hmi_velocity_limit())
        recipe = self._selected_recipe()
        if recipe is None:
            duration = self.home.time_ctl.seconds()
            if duration <= 0:
                QMessageBox.information(self.home, "Start", "Set a timer or select a recipe.")
                return
            self.engine.start_single(rpm, duration)
            return

        if len(recipe.steps) < 1:
            QMessageBox.warning(self.home, "Start", "Selected recipe has no steps.")
            return
        self._recipe_monitor_error_paused = False
        self._recipe_resume_after_clear_error = False
        self._recipe_monitor_begin(recipe)
        self.engine.start_recipe(recipe)

    @Slot()
    def on_stop(self) -> None:
        self._recipe_resume_after_clear_error = False
        if self._run_mode == "recipe" and self._recipe_monitor_recipe is not None and not self._recipe_monitor_failed:
            self._recipe_run_audit("Recipe stopped by user")
            if self._recipe_monitor_step_idx >= 0:
                self.recipe_run.set_step_error(self._recipe_monitor_step_idx, "Stopped by user")
            self._recipe_monitor_failed = True
            self._recipe_monitor_error_paused = False
        self.engine.stop(user_initiated=True)

    @Slot()
    def on_recipe_pause_resume(self) -> None:
        if self._run_mode != "recipe" or self._recipe_monitor_recipe is None:
            return
        if self._recipe_monitor_error_paused:
            self._recipe_resume_after_clear_error = True
            self._recipe_run_audit("Clear errors and resume requested")
            self._clear_odrive_errors()
            self._apply_controls()
            return
        if self.engine.is_paused():
            self._recipe_run_audit("Recipe resumed")
            self.engine.resume()
        else:
            self._recipe_run_audit("Recipe paused")
            self.engine.pause()
        self._refresh_status_all()
        self._apply_controls()

    @Slot(int)
    def on_recipe_skip_step(self, step_idx: int) -> None:
        if self._run_mode != "recipe" or self._recipe_monitor_recipe is None:
            return
        if self.engine.is_paused():
            return
        if int(step_idx) != int(self._recipe_monitor_step_idx):
            return
        try:
            step = self._recipe_monitor_recipe.steps[int(step_idx)]
        except Exception:
            return
        if is_data_entry_step(step):
            return
        if not bool(getattr(step, "allow_skip", True)):
            return
        self._recipe_run_audit(f"Step {int(step_idx)+1} skipped by user")
        self.engine.skip_current_recipe_step()

    @Slot()
    def on_calibrate(self) -> None:
        if self.engine.is_running() or self._calibration_busy:
            return
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.home, "ODrive Not Connected", "Cannot calibrate: ODrive is not connected.")
            return
        self._calibration_busy = True
        self.motor_calibrated = False if BACKEND == "real" else self.motor_calibrated
        self.calibration_detail = "Calibrating..."
        self._apply_controls()
        self.request_run_calibration.emit()

    @Slot()
    def on_open_builder_new(self) -> None:
        if self.engine.is_running():
            return
        self.builder.load_recipe(None)
        self.stack.setCurrentWidget(self.builder)

    @Slot()
    def on_open_settings(self) -> None:
        if self.engine.is_running():
            return
        self.stack.setCurrentWidget(self.settings)

    @Slot()
    def on_back_settings(self) -> None:
        self.stack.setCurrentWidget(self.settings)

    @Slot()
    def on_open_hmi_settings(self) -> None:
        if self.engine.is_running():
            return
        self.stack.setCurrentWidget(self.hmi_settings)

    @Slot(bool)
    def on_hmi_fullscreen_changed(self, enabled: bool) -> None:
        self._hmi_fullscreen_pref = bool(enabled)
        self._schedule_ui_state_save()

    @Slot(float)
    def on_hmi_max_velocity_changed(self, rpm: float) -> None:
        self._hmi_max_velocity_rpm = float(clamp(float(rpm), VEL_MIN_RPM, VEL_MAX_RPM))
        self._apply_hmi_velocity_limit()
        self._schedule_ui_state_save()

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
    def on_open_recipe_run(self) -> None:
        if not (self.engine.is_running() and self._run_mode == "recipe" and self._recipe_monitor_recipe is not None):
            return
        self.stack.setCurrentWidget(self.recipe_run)
        self._refresh_recipe_monitor_plot()

    @Slot()
    def on_open_previous_runs(self) -> None:
        dlg = QDialog(self.home)
        dlg.setWindowTitle("Previous Recipe Runs")
        dlg.setModal(True)
        dlg.setMinimumSize(ui(980), ui(620))

        root = QVBoxLayout(dlg)
        root.setContentsMargins(ui(16), ui(16), ui(16), ui(16))
        root.setSpacing(ui(12))

        title = QLabel("Previous Recipe Runs")
        title.setObjectName("ScreenTitle")
        root.addWidget(title)

        body = QHBoxLayout()
        body.setSpacing(ui(12))

        run_list = QListWidget()
        enable_touch_scrolling(run_list)
        detail_list = QListWidget()
        enable_touch_scrolling(detail_list)
        body.addWidget(run_list, 1)
        body.addWidget(detail_list, 2)
        root.addLayout(body, 1)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        btn_report = buttons.addButton("Generate Report", QDialogButtonBox.ActionRole)
        btn_report.setMinimumHeight(ui(58))
        root.addWidget(buttons)
        buttons.rejected.connect(dlg.reject)
        buttons.accepted.connect(dlg.accept)

        runs = self.run_store.runs()
        btn_report.setEnabled(False)
        if not runs:
            run_list.addItem("No previous recipe runs recorded yet.")
        else:
            for r in runs:
                started = int(safe_float(r.get("started_ts", 0), 0))
                started_txt = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(started)) if started > 0 else "Unknown time"
                status = str(r.get("status") or "unknown")
                recipe_name = str(r.get("recipe_name") or "(unknown recipe)")
                run_id = str(r.get("run_id") or "")
                label = f"{started_txt} | {recipe_name} | {status} | {run_id[:8]}"
                run_list.addItem(label)

            def load_detail(idx: int) -> None:
                detail_list.clear()
                if not (0 <= idx < len(runs)):
                    return
                r = runs[idx]
                started = int(safe_float(r.get('started_ts', 0), 0))
                ended = int(safe_float(r.get('ended_ts', 0), 0))
                detail_list.addItem(f"Run ID: {r.get('run_id', '')}")
                detail_list.addItem(f"Recipe: {r.get('recipe_name', '')}")
                detail_list.addItem(f"Status: {r.get('status', '')}")
                if started > 0:
                    detail_list.addItem(f"Started: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(started))}")
                if ended > 0:
                    detail_list.addItem(f"Ended: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(ended))}")
                detail_list.addItem("")
                detail_list.addItem("Audit Trail")
                for a in list(r.get("audit") or []):
                    ts = int(safe_float((a or {}).get("ts", 0), 0))
                    ttxt = time.strftime("%H:%M:%S", time.localtime(ts)) if ts > 0 else "--:--:--"
                    detail_list.addItem(f"{ttxt}  {str((a or {}).get('text', ''))}")
                btn_report.setEnabled(True)

            run_list.currentRowChanged.connect(load_detail)
            run_list.setCurrentRow(0)

            def on_generate_report() -> None:
                idx = int(run_list.currentRow())
                if not (0 <= idx < len(runs)):
                    return
                self._export_recipe_run_report(dict(runs[idx]))

            btn_report.clicked.connect(on_generate_report)

        dlg.exec()

    @Slot()
    def on_generate_current_recipe_report(self) -> None:
        run: Optional[Dict[str, Any]] = None
        if self._current_recipe_run_record is not None:
            run = dict(self._current_recipe_run_record)
            run.setdefault("status", "running")
            run.setdefault("started_ts", int(epoch_s()))
            run["ended_ts"] = int(epoch_s())
        else:
            runs = self.run_store.runs()
            if runs:
                run = dict(runs[0])
        if not run:
            QMessageBox.information(self.recipe_run, "Generate Report", "No recipe run is available to export.")
            return
        self._export_recipe_run_report(run)

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
        self._schedule_ui_state_save()

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
        # If the selected recipe was edited or newly created, keep selection stable where possible
        if self.selected_recipe_id == recipe.id or self.selected_recipe_id is None:
            self.selected_recipe_id = recipe.id if self.selected_recipe_id == recipe.id else self.selected_recipe_id
        self.on_back_home()
        self._schedule_ui_state_save()

    @Slot(Recipe)
    def on_builder_export(self, recipe: Recipe) -> None:
        try:
            safe_name = "".join(ch if (ch.isalnum() or ch in (" ", "-", "_")) else "_" for ch in str(recipe.name or "recipe"))
            safe_name = "_".join(safe_name.strip().split()) or "recipe"
            out = downloads_dir() / f"recipe_{safe_name}_{time.strftime('%Y%m%d_%H%M%S')}.json"
            payload = {
                "export_kind": "recipe",
                "recipe": {
                    "id": str(recipe.id),
                    "name": str(recipe.name),
                    "steps": [asdict(s) for s in recipe.steps],
                },
            }
            out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            QMessageBox.information(self.builder, "Recipe Export", f"Saved:\n{out}")
        except Exception as e:
            self._show_modal_error(self.builder, "Recipe Export", f"Failed to export recipe:\n{e}")

    @Slot(str)
    def on_builder_delete(self, recipe_id: str) -> None:
        rid = str(recipe_id or "").strip()
        if not rid:
            return
        recipe = next((r for r in self.store.recipes() if str(r.id) == rid), None)
        if recipe is None:
            self._show_modal_error(self.builder, "Delete Recipe", "Recipe not found.")
            return
        ans = QMessageBox.warning(
            self.builder,
            "Delete Recipe",
            f"Delete recipe '{recipe.name}'?\n\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            return
        if not self.store.delete(rid):
            self._show_modal_error(self.builder, "Delete Recipe", "Recipe could not be deleted.")
            return
        if self.selected_recipe_id == rid:
            self.selected_recipe_id = None
        self._refresh_recipes()
        self._refresh_status_all()
        self._apply_controls()
        self._schedule_ui_state_save()
        QMessageBox.information(self.home, "Delete Recipe", f"Deleted recipe:\n{recipe.name}")
        self.on_back_home()

    @Slot()
    def on_import_recipe(self) -> None:
        if self.engine.is_running():
            return
        try:
            path, _flt = QFileDialog.getOpenFileName(
                self.home,
                "Import Recipe JSON",
                str(downloads_dir()),
                "JSON Files (*.json);;All Files (*)",
            )
        except Exception as e:
            self._show_modal_error(self.home, "Import Recipe", f"Failed to open file picker:\n{e}")
            return
        if not path:
            return
        try:
            raw = json.loads(Path(path).read_text(encoding="utf-8"))
            rj = dict(raw.get("recipe")) if isinstance(raw, dict) and isinstance(raw.get("recipe"), dict) else dict(raw)
            name = str(rj.get("name") or "").strip()
            if not name:
                raise ValueError("Recipe name is missing")
            steps_in = list(rj.get("steps") or [])
            if not steps_in:
                raise ValueError("Recipe has no steps")
            steps: List[Step] = []
            for i, s in enumerate(steps_in, start=1):
                if not isinstance(s, dict):
                    raise ValueError(f"Step {i} is not an object")
                kind = str(s.get("kind") or "motor")
                step = Step(
                    velocity_rpm=float(s.get("velocity_rpm", 0.0)),
                    duration_s=int(s.get("duration_s", 0)),
                    name=str(s.get("name") or f"Step {i}"),
                    kind=kind,
                    prompt=str(s.get("prompt") or ""),
                    response_kind=str(s.get("response_kind") or "text"),
                    allow_skip=bool(s.get("allow_skip", True)),
                )
                if is_data_entry_step(step):
                    if not step.prompt.strip():
                        raise ValueError(f"Step {i}: data entry prompt is required")
                else:
                    if int(step.duration_s) <= 0:
                        raise ValueError(f"Step {i}: duration must be > 0")
                steps.append(step)

            imported_id = str(rj.get("id") or uuid.uuid4())
            existing_ids = {r.id for r in self.store.recipes()}
            if imported_id in existing_ids:
                imported_id = str(uuid.uuid4())
            recipe = Recipe(id=imported_id, name=name, steps=steps)
            self.store.upsert(recipe)
            self.selected_recipe_id = recipe.id
            self._refresh_recipes()
            self._refresh_status_all()
            self._apply_controls()
            self._schedule_ui_state_save()
            QMessageBox.information(self.home, "Import Recipe", f"Imported recipe:\n{recipe.name}")
        except Exception as e:
            self._show_modal_error(self.home, "Import Recipe", f"Failed to import recipe:\n{e}")

    # ---- ODrive Config actions ----

    def _refresh_odrive_config(self) -> None:
        keys = [s.key for s in ODRIVE_CONFIG_FIELDS]
        self.cfg.set_loading("Loading from device…")
        if BACKEND == "real" and not self.connected:
            self.cfg.apply_values(DEFAULT_CONFIG_MAP, keys)
            self.cfg.set_loading("Not connected (reconnecting…) — showing defaults")
            return
        self.request_read_config.emit(keys)

    @Slot(object)
    def _apply_odrive_config(self, values_obj: Any) -> None:
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.cfg, "Configuration", "Cannot apply: ODrive is not connected.")
            return
        self._last_cfg_apply_values = dict(values_obj or {})
        self._cfg_runtime_fault_popup_suppressed = True
        self._cfg_op_in_progress = "apply"
        self.cfg.set_loading("Applying config (not saved)...")
        self.request_apply_config.emit(dict(values_obj or {}))

    @Slot(object)
    def _save_odrive_config(self, values_obj: Any) -> None:
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.cfg, "Configuration", "Cannot save: ODrive is not connected.")
            return
        self._cfg_runtime_fault_popup_suppressed = True
        self._cfg_op_in_progress = "save/reboot"
        self.cfg.set_loading("Saving config (device will reboot)...")
        self.request_save_config.emit(dict(values_obj or {}))

    @Slot()
    def _erase_reset_odrive_config(self) -> None:
        self.cfg.set_defaults()
        if BACKEND == "real" and not self.connected:
            self.cfg.set_loading("Defaults restored (not applied) - connect ODrive to erase/reset")
            return
        ans = QMessageBox.warning(
            self.cfg,
            "Erase Configuration",
            "This will erase the ODrive configuration and reboot the device.\n\nContinue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            self.cfg.set_loading("Erase canceled - form reset to defaults (not applied)")
            return
        self._cfg_runtime_fault_popup_suppressed = True
        self._cfg_op_in_progress = "erase/reset"
        self.cfg.set_loading("Erasing configuration (device will reboot)...")
        self.request_erase_config.emit()

    @Slot()
    def _clear_odrive_errors(self) -> None:
        parent = self.cfg if self.stack.currentWidget() is self.cfg else self.stack.currentWidget()
        if BACKEND == "real" and not self.connected:
            self._show_modal_error(parent, "Clear Errors", "ODrive is not connected.")
            return
        if self.stack.currentWidget() is self.cfg:
            self.cfg.set_loading("Clearing ODrive errors...")
        self.request_clear_errors.emit()

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
        self._schedule_ui_state_save()

    @Slot(object)
    def _on_dlog_tags_changed(self, tags_obj: Any) -> None:
        tags = [str(t) for t in (list(tags_obj or []))]
        known = set(self._known_tags())
        tags = [t for t in tags if t in known]
        self._dlog_selected_tags = tags if tags else list(DEFAULT_ENABLED_TAGS)
        if self.stack.currentWidget() is self.dlog:
            self._refresh_datalogger_plot()
        self._schedule_ui_state_save()

    @Slot(object)
    def _on_recipe_run_tags_changed(self, tags_obj: Any) -> None:
        tags = [str(t) for t in (list(tags_obj or []))]
        known = set(self._known_tags())
        tags = [t for t in tags if t in known]
        self._recipe_monitor_plot_tags = tags if tags else [TAG_CMD_RPM, TAG_VEL_RPM, TAG_TORQUE_NM]
        self.recipe_run.set_selected_tags(self._recipe_monitor_plot_tags)
        if self.stack.currentWidget() is self.recipe_run:
            self._refresh_recipe_monitor_plot()

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

    def _refresh_recipe_monitor_plot(self) -> None:
        if self._recipe_monitor_recipe is None:
            self.recipe_run.set_series_data({t: [] for t in self._recipe_monitor_plot_tags}, self._recipe_monitor_plot_tags)
            return
        now_ts = epoch_s()
        started_ts = 0
        if self._current_recipe_run_record is not None:
            started_ts = int(safe_float(self._current_recipe_run_record.get("started_ts", 0), 0))
        if started_ts <= 0:
            started_ts = max(0, now_ts - int(recipe_total_seconds(self._recipe_monitor_recipe)))

        recipe_minutes = max(1, int(math.ceil(recipe_total_seconds(self._recipe_monitor_recipe) / 60.0)))
        elapsed_minutes = max(1, int(math.ceil(max(0, now_ts - started_ts) / 60.0)))
        minutes = int(clamp(max(recipe_minutes, elapsed_minutes), 1, 30))
        self.recipe_run.set_plot_minutes(minutes)

        window_start_ts = max(int(started_ts), int(now_ts - minutes * 60))
        try:
            data = self.history_db.query_range_multi(window_start_ts, now_ts, self._recipe_monitor_plot_tags)
        except Exception as e:
            self._show_modal_error(self.recipe_run, "Data Logger DB", f"Failed to load recipe run window:\n{e}")
            data = {t: [] for t in self._recipe_monitor_plot_tags}
        self.recipe_run.set_series_data(data, tags=self._recipe_monitor_plot_tags)

    def _recipe_monitor_begin(self, recipe: Recipe) -> None:
        self._recipe_monitor_recipe = recipe
        self._recipe_monitor_step_idx = -1
        self._recipe_monitor_failed = False
        self._recipe_monitor_error_paused = False
        self._recipe_resume_after_clear_error = False
        run_id = str(uuid.uuid4())
        self._current_recipe_run_record = {
            "run_id": run_id,
            "recipe_id": str(recipe.id),
            "recipe_name": str(recipe.name),
            "started_ts": int(epoch_s()),
            "ended_ts": None,
            "status": "running",
            "recipe_steps": [asdict(s) for s in recipe.steps],
            "audit": [],
        }
        self.recipe_run.set_recipe(recipe)
        self._recipe_run_audit(f"Recipe started: {recipe.name} (Run ID: {run_id})")
        self._refresh_recipe_monitor_plot()
        self.stack.setCurrentWidget(self.recipe_run)

    def _recipe_monitor_finish(self, reason: str) -> None:
        if self._recipe_monitor_recipe is None:
            return
        self._recipe_monitor_error_paused = False
        self._recipe_resume_after_clear_error = False
        self._recipe_run_audit(reason)
        if self._current_recipe_run_record is not None:
            self._current_recipe_run_record["ended_ts"] = int(epoch_s())
            rtxt = str(reason).lower()
            if "completed" in rtxt:
                status = "completed"
            elif "user" in rtxt or "stopped" in rtxt:
                status = "stopped"
            elif "error" in rtxt or self._recipe_monitor_failed:
                status = "error"
            else:
                status = "ended"
            self._current_recipe_run_record["status"] = status
            self.run_store.add_run(self._current_recipe_run_record)
            self._current_recipe_run_record = None
        self._recipe_monitor_recipe = None
        self._recipe_monitor_step_idx = -1

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
        tags = [str(t) for t in (list(tags_obj or []))]
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
        self.settings = SettingsHubScreen()
        self.hmi_settings = HMISettingsScreen()
        self.recipe_run = RecipeRunMonitorScreen()
        self.cfg = ODriveConfigScreen()
        self.dlog = DataLoggerScreen()

        self.stack.addWidget(self.home)
        self.stack.addWidget(self.builder)
        self.stack.addWidget(self.settings)
        self.stack.addWidget(self.hmi_settings)
        self.stack.addWidget(self.recipe_run)
        self.stack.addWidget(self.cfg)
        self.stack.addWidget(self.dlog)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.stack)

        self.controller = AppController(
            self.stack,
            self.home,
            self.builder,
            self.settings,
            self.hmi_settings,
            self.recipe_run,
            self.cfg,
            self.dlog,
        )
        self.hmi_settings.fullscreen_changed.connect(self._set_fullscreen_mode)
        self.hmi_settings.close_program_clicked.connect(self.close)

    def closeEvent(self, event) -> None:
        self.controller.shutdown()
        event.accept()

    def keyPressEvent(self, event) -> None:
        try:
            if event.key() == Qt.Key_Escape and self.isFullScreen():
                self.showNormal()
                self.hmi_settings.set_fullscreen_state(False, emit_signal=False)
                self.controller.set_hmi_fullscreen_preference(False)
                event.accept()
                return
        except Exception:
            pass
        super().keyPressEvent(event)

    @Slot(bool)
    def _set_fullscreen_mode(self, enabled: bool) -> None:
        if bool(enabled):
            self.showFullScreen()
            self.hmi_settings.set_fullscreen_state(True, emit_signal=False)
            return
        self.showNormal()
        self.hmi_settings.set_fullscreen_state(False, emit_signal=False)

    def start_in_fullscreen(self) -> bool:
        return bool(self.controller.hmi_fullscreen_preference())


# ----------------------------
# Styling
# ----------------------------

def compute_ui_scale(app: QApplication) -> float:
    try:
        screen = app.primaryScreen()
        if screen is None:
            return 1.0
        dpi = float(screen.logicalDotsPerInch())
        if not math.isfinite(dpi) or dpi <= 1.0:
            return 1.0
        # Match physical sizing when Qt styles use px values; clamp to avoid layout blowups.
        return float(clamp(dpi / 96.0, 1.0, 1.6))
    except Exception:
        return 1.0


def apply_style(app: QApplication) -> None:
    # Use pixel sizing here to avoid Qt point-size conversion warnings on some
    # Windows/Qt builds where inherited fonts report pointSize == -1.
    base_font = QFont(app.font())
    base_font.setPixelSize(max(14, ui(20)))
    app.setFont(base_font)

    # Scale stylesheet px sizes so fixed-size window remains comfortably touch-friendly at OS scaling (e.g., 125%).
    fs_widget = ui(20)
    fs_title = ui(28)
    fs_section = ui(24)
    fs_section_c = ui(20)
    fs_field = ui(22)
    fs_units = ui(18)
    fs_info = ui(18)

    fs_status = ui(20)
    fs_step = ui(18)
    fs_backend = ui(16)

    fs_lineedit = ui(26)
    fs_combo = ui(22)
    fs_btn = ui(24)

    indicator = ui(28)
    border_r = ui(14)
    border_r_btn = ui(16)

    app.setStyleSheet(
        f"""
        QWidget {{ background: #101317; color: #F2F5F7; font-size: {fs_widget}px; }}

        QLabel#ScreenTitle {{ font-size: {fs_title}px; font-weight: 700; }}
        QLabel#SectionTitle {{ font-size: {fs_section}px; font-weight: 650; }}
        QLabel#SectionTitleCompact {{ font-size: {fs_section_c}px; font-weight: 650; }}
        QLabel#FieldLabel {{ font-size: {fs_field}px; font-weight: 650; }}
        QLabel#UnitsLabel {{ font-size: {fs_units}px; color: #D0D7DE; }}
        QLabel#InfoLabel {{ font-size: {fs_info}px; color: #D0D7DE; }}

        QFrame#StatusBar {{ background: #161B22; border: 2px solid #2A313A; border-radius: {border_r}px; }}
        QLabel#StatusLabel {{ font-size: {fs_status}px; font-weight: 650; }}
        QLabel#StatusStep {{ font-size: {fs_step}px; color: #D0D7DE; }}
        QLabel#StatusBackend {{ font-size: {fs_backend}px; color: #9FB0C0; }}

        QLineEdit {{
            background: #0B0F14; border: 2px solid #2A313A; border-radius: {border_r}px;
            padding: {ui(8)}px {ui(12)}px; font-size: {fs_lineedit}px;
        }}

        QComboBox {{
            background: #0B0F14; border: 2px solid #2A313A; border-radius: {border_r}px;
            padding: {ui(8)}px {ui(12)}px; font-size: {fs_combo}px; min-height: {ui(58)}px;
        }}

        QCheckBox {{ spacing: {ui(12)}px; font-size: {fs_widget}px; }}
        QCheckBox::indicator {{ width: {indicator}px; height: {indicator}px; }}

        QPushButton {{
            background: #2B3440; border: 2px solid #3A4656; border-radius: {border_r_btn}px;
            padding: {ui(10)}px {ui(14)}px; font-size: {fs_btn}px; font-weight: 650;
        }}
        QPushButton:hover {{ background: #354152; }}
        QPushButton:pressed {{ background: #202733; }}
        QPushButton:disabled {{ background: #1C222B; border: 2px solid #2A313A; color: #7A8794; }}

        QPushButton#StartButton {{ background: #1E5F3C; border: 2px solid #2D7B52; }}
        QPushButton#StartButton:hover {{ background: #21704A; }}
        QPushButton#StopButton {{ background: #6B1E1E; border: 2px solid #8B2A2A; }}
        QPushButton#StopButton:hover {{ background: #7A2424; }}

        QFrame#RecipeCard {{ background: #161B22; border: 2px solid #2A313A; border-radius: {border_r_btn}px; }}
        QFrame#RecipeCard[selected="true"] {{ border: 4px solid #2F81F7; }}
        QLabel#RecipeName {{ font-size: {ui(24)}px; font-weight: 700; }}
        QLabel#RecipeSummary {{ font-size: {ui(18)}px; color: #D0D7DE; }}

        QFrame#StepRow {{ background: #161B22; border: 2px solid #2A313A; border-radius: {border_r_btn}px; }}
        QLabel#StepIndex {{ font-size: {ui(20)}px; font-weight: 700; }}

        QFrame#ConfigRow {{ background: #161B22; border: 2px solid #2A313A; border-radius: {border_r_btn}px; }}
        QLabel#ConfigLabel {{ font-size: {ui(20)}px; font-weight: 650; }}
        QLabel#ConfigNote {{ font-size: {ui(18)}px; color: #9FB0C0; }}
        QFrame#RecipeRunStepBox {{ background: #161B22; border: 2px solid #2A313A; border-radius: {border_r_btn}px; }}
        QFrame#RecipeRunStepBox[status="running"] {{ background: #163323; border: 2px solid #2D7B52; }}
        QFrame#RecipeRunStepBox[status="done"] {{ background: #1A222C; border: 2px solid #3A4656; }}
        QFrame#RecipeRunStepBox[status="error"] {{ background: #3A1717; border: 2px solid #8B2A2A; }}

        QWidget#TimeControl[dimmed="true"] QLabel {{ color: #7A8794; }}
        QWidget#TimeControl[dimmed="true"] QLineEdit {{
            background: #0E1218; border: 2px solid #232A33; color: #7A8794;
        }}

        QScrollArea {{ border: none; }}
        """
    )


def show_startup_splash(app: QApplication) -> QWidget:
    splash = QWidget(None, Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
    splash.setObjectName("StartupSplash")
    splash.setAttribute(Qt.WA_ShowWithoutActivating, True)
    splash.setFixedSize(460, 180)
    splash.setStyleSheet(
        """
        QWidget#StartupSplash {
            background: #101317;
            color: #F2F5F7;
            border: 2px solid #2A313A;
            border-radius: 14px;
        }
        QLabel#SplashTitle { font-size: 28px; font-weight: 700; color: #FFFFFF; }
        QLabel#SplashInfo { font-size: 16px; color: #FFFFFF; }
        """
    )

    root = QVBoxLayout(splash)
    root.setContentsMargins(20, 18, 20, 18)
    root.setSpacing(8)

    title = QLabel("Reactor Automation HMI")
    title.setObjectName("SplashTitle")
    info = QLabel("Opening program...")
    info.setObjectName("SplashInfo")
    info2 = QLabel("Loading UI and connecting background services")
    info2.setObjectName("SplashInfo")

    root.addStretch(1)
    root.addWidget(title)
    root.addWidget(info)
    root.addWidget(info2)
    root.addStretch(1)

    try:
        screen = app.primaryScreen()
        if screen is not None:
            g = screen.availableGeometry()
            x = g.x() + (g.width() - splash.width()) // 2
            y = g.y() + (g.height() - splash.height()) // 2
            splash.move(x, y)
    except Exception:
        pass

    splash.show()
    app.processEvents()
    return splash


# ----------------------------
# Entry Point
# ----------------------------

def main() -> int:
    app = QApplication(sys.argv)
    splash = show_startup_splash(app)

    global UI_SCALE
    UI_SCALE = compute_ui_scale(app)

    apply_style(app)
    w = MainWindow()
    if w.start_in_fullscreen():
        w.showFullScreen()
    else:
        w.show()
    splash.close()
    app.processEvents()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

