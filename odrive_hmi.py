"""
Touch-Friendly ODrive Reactor Agitator HMI (PySide6)

Run (simulation default):
  pip install PySide6
  python odrive_hmi.py

Switch to real ODrive backend:
  - Set BACKEND = "real"
  - Ensure the `odrive` Python package is installed and accessible on the target device

Real backend (open-loop LOCKIN_SPIN):
  - UI uses RPM; backend converts to turns/s = rpm / 60
  - On EVERY velocity change:
      axis.config.general_lockin.vel = turns/s
      axis.requested_state = AxisState.LOCKIN_SPIN
    (firmware latches vel only on re-entry)
  - connect() applies a safe baseline once:
      IDLE, disable startup calibrations/closed-loop flags, current limits,
      and a general_lockin profile (current/ramp_time/ramp_distance/accel, finish_* False if available)
  - stop() transitions to IDLE
  - Backend methods do not sleep; timing is handled by the QTimer-driven run engine

Added screens:
  - ODrive Configuration: view/edit important settings, restore defaults, save to device (worker thread), export to JSON
  - Velocity History: poll velocity at 1 Hz (worker thread), store compressed samples in SQLite, live table view, export window to CSV

Threading rules:
  - All ODrive reads/writes are executed in the worker thread via queued signals/slots
  - UI remains non-blocking
"""

from __future__ import annotations

import csv
import json
import sqlite3
import sys
import time
import uuid
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtCore import QObject, Qt, QThread, QTimer, QStandardPaths, Signal, Slot, QMetaObject
from PySide6.QtGui import QDoubleValidator, QFont, QIntValidator
from PySide6.QtWidgets import (
    QApplication,
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
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

# ----------------------------
# App Configuration
# ----------------------------

BACKEND = "real"  # "sim" (default) or "real"

WINDOW_W = 1280
WINDOW_H = 800

# UI units are RPM everywhere.
VEL_MIN_RPM = 0.0
VEL_MAX_RPM = 2000.0
VEL_STEP_RPM = 1.0

# Timer UI uses mm:ss; internal storage uses seconds.
TIME_STEP_S = 10
TIME_MAX_S = 24 * 60 * 60  # 24h cap

ENGINE_TICK_MS = 200  # Engine tick; countdown displayed in whole seconds

# Velocity polling for historian.
VEL_POLL_MS = 1000
HIST_TAG = "axis0.vel_estimate_rpm"
HIST_DEADBAND_RPM = 2.0
HIST_HEARTBEAT_S = 60

# ----------------------------
# Utilities
# ----------------------------


def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp v into [lo, hi]."""
    return max(lo, min(hi, v))


def format_mmss(seconds: int) -> str:
    """Format seconds into mm:ss (minutes may exceed 59)."""
    seconds = max(0, int(seconds))
    m = seconds // 60
    s = seconds % 60
    return f"{m:02d}:{s:02d}"


def parse_mmss(text: str) -> Optional[int]:
    """
    Parse mm:ss or plain seconds into an integer number of seconds.

    Accepted forms:
      - "mm:ss" or "m:ss"
      - "ss" (treated as seconds)
    Returns None for invalid input.
    """
    t = text.strip()
    if not t:
        return None

    if ":" not in t:
        if not t.isdigit():
            return None
        return int(t)

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
    """Return the per-user application config directory (created if missing)."""
    base = QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation)
    if not base:
        base = str(Path.home() / ".config")
    d = Path(base) / "PolyJouleOdriveHMI"
    d.mkdir(parents=True, exist_ok=True)
    return d


def downloads_dir() -> Path:
    """
    Return the per-user Downloads directory (created if missing).
    Falls back to ~/Output when the platform location is unavailable.
    """
    base = QStandardPaths.writableLocation(QStandardPaths.DownloadLocation)
    if not base:
        base = str(Path.home() / "Output")
    d = Path(base)
    d.mkdir(parents=True, exist_ok=True)
    return d


def epoch_s() -> int:
    """Return current epoch time in seconds."""
    return int(time.time())


# ----------------------------
# Recipe Model
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
    """Compute total duration of a recipe."""
    return sum(max(0, int(s.duration_s)) for s in r.steps)


def recipe_summary(r: Recipe) -> str:
    """Create a compact summary string for display on the home screen."""
    n = len(r.steps)
    return f"{n} step{'s' if n != 1 else ''} • {format_mmss(recipe_total_seconds(r))} total"


# ----------------------------
# Recipe Storage (JSON, per-user config directory)
# ----------------------------

class RecipeStore:
    """
    Persist recipes to a JSON file stored in the per-user config directory.

    File format:
      {
        "recipes": [
          {"id": "...", "name": "...", "steps": [{"velocity_rpm": 100.0, "duration_s": 30}, ...]},
          ...
        ]
      }
    """

    def __init__(self) -> None:
        self._path = app_config_dir() / "recipes.json"
        self._recipes: List[Recipe] = []

    def recipes(self) -> List[Recipe]:
        """Return a shallow copy of recipes."""
        return list(self._recipes)

    def load(self) -> None:
        """Load recipes from disk; seed defaults if file is missing or corrupted."""
        if not self._path.exists():
            self._recipes = self._seed_recipes()
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
            bad = self._path.with_suffix(".corrupt.json")
            try:
                self._path.replace(bad)
            except Exception:
                pass
            self._recipes = self._seed_recipes()
            self.save()

    def save(self) -> None:
        """Write recipes to disk."""
        obj = {
            "recipes": [
                {"id": r.id, "name": r.name, "steps": [asdict(s) for s in r.steps]}
                for r in self._recipes
            ]
        }
        self._path.write_text(json.dumps(obj, indent=2), encoding="utf-8")

    def upsert(self, recipe: Recipe) -> None:
        """Insert or update a recipe by ID."""
        for i, r in enumerate(self._recipes):
            if r.id == recipe.id:
                self._recipes[i] = recipe
                self.save()
                return
        self._recipes.append(recipe)
        self.save()

    @staticmethod
    def _seed_recipes() -> List[Recipe]:
        """Provide a small default recipe for first boot."""
        return [
            Recipe(
                id=str(uuid.uuid4()),
                name="Gentle Mix",
                steps=[
                    Step(velocity_rpm=120.0, duration_s=60),
                    Step(velocity_rpm=200.0, duration_s=120),
                    Step(velocity_rpm=120.0, duration_s=60),
                ],
            )
        ]


# ----------------------------
# ODrive Configuration Schema
# ----------------------------

@dataclass(frozen=True)
class SettingSpec:
    """
    Specification for a single configuration field displayed in the UI.

    key is an attribute path beginning at the ODrive root object (e.g., "axis0.config.motor.current_soft_max").
    """
    key: str
    label: str
    kind: str  # "bool" or "float"
    default: Any
    min_value: Optional[float] = None
    max_value: Optional[float] = None


IMPORTANT_SETTINGS: List[SettingSpec] = [
    SettingSpec("axis0.config.startup_motor_calibration", "Startup: motor calibration", "bool", False),
    SettingSpec("axis0.config.startup_encoder_offset_calibration", "Startup: encoder offset calibration", "bool", False),
    SettingSpec("axis0.config.startup_encoder_index_search", "Startup: encoder index search", "bool", False),
    SettingSpec("axis0.config.startup_closed_loop_control", "Startup: closed loop control", "bool", False),
    SettingSpec("axis0.config.motor.current_soft_max", "Motor: current soft max (A)", "float", 5.0, 0.0, 50.0),
    SettingSpec("axis0.config.motor.current_hard_max", "Motor: current hard max (A)", "float", 8.0, 0.0, 100.0),
    SettingSpec("axis0.config.general_lockin.current", "Lock-in: current (A)", "float", 2.0, 0.0, 20.0),
    SettingSpec("axis0.config.general_lockin.ramp_time", "Lock-in: ramp time (s)", "float", 1.0, 0.0, 30.0),
    SettingSpec("axis0.config.general_lockin.ramp_distance", "Lock-in: ramp distance (turns)", "float", 1.0, 0.0, 100.0),
    SettingSpec("axis0.config.general_lockin.accel", "Lock-in: accel (turn/s²)", "float", 10.0, 0.0, 1000.0),
    SettingSpec("axis0.config.general_lockin.finish_on_vel", "Lock-in: finish on vel", "bool", False),
    SettingSpec("axis0.config.general_lockin.finish_on_distance", "Lock-in: finish on distance", "bool", False),
]

DEFAULT_CONFIG_MAP: Dict[str, Any] = {s.key: s.default for s in IMPORTANT_SETTINGS}


# ----------------------------
# ODrive Interface + Backends
# ----------------------------

class OdriveInterface:
    """Backend interface used by the worker thread."""

    def connect(self) -> bool:
        raise NotImplementedError

    def is_connected(self) -> bool:
        raise NotImplementedError

    def set_velocity_rpm(self, rpm: float) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def get_state(self) -> str:
        raise NotImplementedError

    def read_config(self, keys: List[str]) -> Tuple[Dict[str, Any], List[str]]:
        raise NotImplementedError

    def apply_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]:
        raise NotImplementedError

    def get_velocity_rpm(self) -> float:
        raise NotImplementedError


class SimulatedOdrive(OdriveInterface):
    """
    Simulation backend that never blocks.

    It tracks:
      - target RPM set by set_velocity_rpm()
      - a simulated "actual" RPM that smoothly approaches the target
      - a mutable config map used by the configuration screen
    """

    def __init__(self) -> None:
        self._connected = False
        self._state = "Disconnected"
        self._target_rpm = 0.0
        self._actual_rpm = 0.0
        self._last_update = time.monotonic()
        self._config: Dict[str, Any] = dict(DEFAULT_CONFIG_MAP)

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
        values: Dict[str, Any] = {}
        missing: List[str] = []
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
        return True, "Simulation config updated", missing

    def get_velocity_rpm(self) -> float:
        now = time.monotonic()
        dt = max(0.0, now - self._last_update)
        self._last_update = now

        # First-order lag to emulate a controller response.
        tau = 0.5  # seconds
        alpha = 1.0 - pow(2.718281828, -dt / max(1e-6, tau))
        self._actual_rpm += (self._target_rpm - self._actual_rpm) * alpha

        # A small deadzone near zero to reduce tiny floating noise.
        if abs(self._actual_rpm) < 0.2:
            self._actual_rpm = 0.0

        return float(self._actual_rpm)


class RealOdrive(OdriveInterface):
    """
    Real backend using open-loop LOCKIN_SPIN.

    Behavior:
      - UI uses RPM; convert to turns/s = rpm / 60
      - On every velocity change: set general_lockin.vel then request LOCKIN_SPIN
      - stop() goes to IDLE
      - read/apply config use attribute traversal and skip unsupported fields
    """

    def __init__(self) -> None:
        self._connected = False
        self._state = "Disconnected"
        self._odrv = None
        self._axis = None
        self._AxisState = None

    def connect(self) -> bool:
        """
        Connect to ODrive and apply a safe open-loop baseline configuration.

        This method runs in the worker thread and may block briefly during discovery.
        """
        try:
            import odrive
            from odrive.enums import AxisState

            self._AxisState = AxisState
            self._state = "Connecting..."
            self._odrv = odrive.find_any(timeout=5)
            if self._odrv is None:
                self._connected = False
                self._state = "Error: ODrive not found"
                return False

            self._axis = self._odrv.axis0

            # Safety: start from IDLE.
            self._axis.requested_state = AxisState.IDLE

            # Disable automatic startup procedures that would attempt calibration or closed-loop.
            for attr, val in (
                ("startup_motor_calibration", False),
                ("startup_encoder_offset_calibration", False),
                ("startup_encoder_index_search", False),
                ("startup_closed_loop_control", False),
            ):
                try:
                    setattr(self._axis.config, attr, val)
                except Exception:
                    pass

            # Conservative current limits.
            self._axis.config.motor.current_soft_max = 5.0
            self._axis.config.motor.current_hard_max = 8.0

            # Configure open-loop lock-in profile.
            gl = self._axis.config.general_lockin
            gl.current = 2.0
            gl.ramp_time = 1.0
            gl.ramp_distance = 1.0
            gl.accel = 10.0

            # Some firmware variants may not expose these fields.
            for name, value in (("finish_on_vel", False), ("finish_on_distance", False)):
                try:
                    setattr(gl, name, value)
                except Exception:
                    pass

            try:
                gl.vel = 0.0
            except Exception:
                pass

            self._connected = True
            self._state = "Idle (LOCKIN ready)"
            return True

        except Exception as e:
            self._connected = False
            self._state = f"Error: {e}"
            return False

    def is_connected(self) -> bool:
        return self._connected

    def set_velocity_rpm(self, rpm: float) -> None:
        """
        Apply a new open-loop velocity.

        Firmware requirement:
          - Update gl.vel, then re-request LOCKIN_SPIN so the new velocity is latched.
        """
        if not self._connected or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")

        rpm = float(rpm)
        if abs(rpm) < 1e-6:
            self.stop()
            return

        turns_per_s = rpm / 60.0
        gl = self._axis.config.general_lockin
        gl.vel = float(turns_per_s)

        # Re-enter LOCKIN_SPIN so firmware applies updated velocity.
        self._axis.requested_state = self._AxisState.LOCKIN_SPIN
        self._state = f"LOCKIN_SPIN ({rpm:.0f} rpm)"

    def stop(self) -> None:
        """Safely stop motion by transitioning to IDLE."""
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
        """Traverse dotted attribute path and return the final object/value."""
        obj = root
        for part in key.split("."):
            obj = getattr(obj, part)
        return obj

    @staticmethod
    def _set_attr_path(root: Any, key: str, value: Any) -> None:
        """Traverse dotted attribute path and assign the final attribute."""
        parts = key.split(".")
        parent = root
        for part in parts[:-1]:
            parent = getattr(parent, part)
        setattr(parent, parts[-1], value)

    def read_config(self, keys: List[str]) -> Tuple[Dict[str, Any], List[str]]:
        if not self._connected or self._odrv is None:
            raise RuntimeError("ODrive not connected")

        values: Dict[str, Any] = {}
        missing: List[str] = []

        for k in keys:
            try:
                v = self._get_attr_path(self._odrv, k)
                values[k] = v
            except Exception:
                missing.append(k)

        return values, missing

    def apply_config(self, values: Dict[str, Any]) -> Tuple[bool, str, List[str]]:
        if not self._connected or self._odrv is None or self._axis is None or self._AxisState is None:
            raise RuntimeError("ODrive not connected")

        missing: List[str] = []
        try:
            # Apply config from a safe IDLE state.
            self._axis.requested_state = self._AxisState.IDLE
        except Exception:
            pass

        for k, v in values.items():
            try:
                self._set_attr_path(self._odrv, k, v)
            except Exception:
                missing.append(k)

        msg = "Config applied"
        if missing:
            msg = f"Config applied with {len(missing)} unsupported field(s)"
        return True, msg, missing

    def get_velocity_rpm(self) -> float:
        """
        Read axis0.vel_estimate and convert to RPM.

        vel_estimate is commonly turns/s on ODrive firmware; RPM = turns/s * 60.
        """
        if not self._connected or self._axis is None:
            raise RuntimeError("ODrive not connected")
        try:
            turns_per_s = float(getattr(self._axis, "vel_estimate"))
            return turns_per_s * 60.0
        except Exception:
            return 0.0


# ----------------------------
# Worker Thread for Motor Commands / ODrive I/O
# ----------------------------

class OdriveWorker(QObject):
    """
    Execute backend operations in a dedicated worker thread.

    All ODrive reads/writes are performed here to keep the UI thread responsive.
    """

    connected_changed = Signal(bool, str)   # connected, backend state string
    state_changed = Signal(str)             # backend state string
    rpm_commanded = Signal(float)           # last commanded RPM
    error = Signal(str)

    config_read = Signal(object, object)    # values: dict, missing: list
    config_applied = Signal(bool, str, object)  # ok, message, missing: list

    velocity_sample = Signal(int, float)    # ts_epoch_s, rpm

    def __init__(self, backend: OdriveInterface) -> None:
        super().__init__()
        self._backend = backend
        self._poll_timer: Optional[QTimer] = None

    @Slot()
    def connect_backend(self) -> None:
        try:
            ok = self._backend.connect()
            self.connected_changed.emit(ok, self._backend.get_state())
            self.state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Connect failed: {e}")
            self.connected_changed.emit(False, "Error")
            self.state_changed.emit("Error")

    @Slot(float)
    def set_velocity_rpm(self, rpm: float) -> None:
        try:
            self._backend.set_velocity_rpm(float(rpm))
            self.rpm_commanded.emit(float(rpm))
            self.state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Motor command failed: {e}")

    @Slot()
    def stop(self) -> None:
        try:
            self._backend.stop()
            self.rpm_commanded.emit(0.0)
            self.state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Stop failed: {e}")

    @Slot(object)
    def read_config(self, keys: List[str]) -> None:
        try:
            values, missing = self._backend.read_config(keys)
            self.config_read.emit(values, missing)
        except Exception as e:
            self.error.emit(f"Read config failed: {e}")
            self.config_read.emit({}, keys)

    @Slot(object)
    def apply_config(self, values: Dict[str, Any]) -> None:
        try:
            ok, msg, missing = self._backend.apply_config(values)
            self.config_applied.emit(ok, msg, missing)
            self.state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Apply config failed: {e}")
            self.config_applied.emit(False, str(e), list(values.keys()))

    @Slot()
    def start_velocity_polling(self) -> None:
        """
        Start 1 Hz velocity polling in the worker thread.

        A QTimer is created lazily so it is constructed in the worker thread context.
        """
        if self._poll_timer is None:
            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(VEL_POLL_MS)
            self._poll_timer.timeout.connect(self._poll_velocity)
        if not self._poll_timer.isActive():
            self._poll_timer.start()

    @Slot()
    def stop_velocity_polling(self) -> None:
        """Stop velocity polling if it is active."""
        if self._poll_timer and self._poll_timer.isActive():
            self._poll_timer.stop()

    @Slot()
    def _poll_velocity(self) -> None:
        """Read velocity from backend and emit a timestamped sample."""
        try:
            rpm = float(self._backend.get_velocity_rpm())
            self.velocity_sample.emit(epoch_s(), rpm)
        except Exception as e:
            self.error.emit(f"Velocity poll failed: {e}")


# ----------------------------
# Run Engine (QTimer-driven state machine)
# ----------------------------

class RunEngine(QObject):
    """
    Drive either:
      - Single timed run (rpm + duration)
      - Multi-step recipe run (sequence of rpm/duration steps)

    Motor commands are emitted as signals and executed by the worker thread.
    """

    status_changed = Signal(str)                 # "Idle", "Running", "Stopped"
    active_rpm_changed = Signal(float)
    remaining_changed = Signal(int)              # seconds remaining in current segment
    step_info_changed = Signal(str)              # step info for recipe mode

    request_set_rpm = Signal(float)
    request_stop = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._timer = QTimer(self)
        self._timer.setInterval(ENGINE_TICK_MS)
        self._timer.timeout.connect(self._tick)

        self._mode: str = "idle"                 # "idle" | "single" | "recipe"
        self._status: str = "Idle"
        self._active_rpm: float = 0.0

        self._single_end_t: float = 0.0

        self._recipe: Optional[Recipe] = None
        self._step_idx: int = -1
        self._step_end_t: float = 0.0

    def is_running(self) -> bool:
        return self._mode in ("single", "recipe")

    def _set_status(self, status: str) -> None:
        if status != self._status:
            self._status = status
            self.status_changed.emit(status)

    @Slot(float, int)
    def start_single(self, rpm: float, duration_s: int) -> None:
        """Start a single timed run; duration must be > 0."""
        self.stop(user_initiated=False)
        self._mode = "single"
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
        """Start a multi-step recipe run."""
        self.stop(user_initiated=False)
        self._mode = "recipe"
        self._recipe = recipe
        self._step_idx = 0

        now = time.monotonic()
        self._start_current_step(now)
        self._set_status("Running")

        self._timer.start()
        self._tick()

    def _start_current_step(self, now: float) -> None:
        """Issue motor command and timing for the current recipe step."""
        assert self._recipe is not None
        step = self._recipe.steps[self._step_idx]

        self._active_rpm = float(step.velocity_rpm)
        self.request_set_rpm.emit(self._active_rpm)
        self.active_rpm_changed.emit(self._active_rpm)

        self._step_end_t = now + max(0, int(step.duration_s))
        info = f"Step {self._step_idx + 1}/{len(self._recipe.steps)} • {step.velocity_rpm:g} rpm • {format_mmss(step.duration_s)}"
        self.step_info_changed.emit(info)

    @Slot(bool)
    def stop(self, user_initiated: bool = True) -> None:
        """
        Stop any active run.

        user_initiated=True keeps the UI status at "Stopped".
        user_initiated=False is used for internal transitions before starting a new run.
        """
        if self._timer.isActive():
            self._timer.stop()

        if self._mode != "idle":
            self.request_stop.emit()

        self._mode = "idle"
        self._recipe = None
        self._step_idx = -1
        self._active_rpm = 0.0

        self.active_rpm_changed.emit(0.0)
        self.remaining_changed.emit(0)
        self.step_info_changed.emit("")
        self._set_status("Stopped" if user_initiated else "Idle")

    @Slot()
    def _tick(self) -> None:
        """Timer tick that advances countdown and recipe steps using monotonic time."""
        now = time.monotonic()

        if self._mode == "single":
            remaining = int(round(self._single_end_t - now))
            if remaining <= 0:
                self.request_stop.emit()
                self._mode = "idle"
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
                    self._mode = "idle"
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
# Historian SQLite Store (tag compression)
# ----------------------------

class HistoryDB:
    """
    SQLite storage for time series samples.

    Compression:
      - Insert a new row only when value changes beyond a deadband OR a heartbeat interval is exceeded.
    """

    def __init__(self, db_path: Path) -> None:
        self._path = db_path
        self._conn = sqlite3.connect(str(self._path))
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
        """
        Insert a sample if it passes compression rules.

        Returns True if a row was inserted.
        """
        prev = self._last.get(tag)
        if prev is None:
            self._conn.execute("INSERT INTO samples(ts, tag, value) VALUES(?, ?, ?)", (ts, tag, float(value)))
            self._conn.commit()
            self._last[tag] = (ts, float(value))
            return True

        prev_ts, prev_val = prev
        if abs(float(value) - prev_val) >= float(deadband) or (ts - prev_ts) >= int(heartbeat_s):
            self._conn.execute("INSERT INTO samples(ts, tag, value) VALUES(?, ?, ?)", (ts, tag, float(value)))
            self._conn.commit()
            self._last[tag] = (ts, float(value))
            return True

        return False

    def query_window(self, now_ts: int, tag: str, minutes: int) -> List[Tuple[int, float]]:
        """Return samples for the given tag within the last N minutes."""
        minutes = max(1, int(minutes))
        start_ts = now_ts - minutes * 60
        cur = self._conn.execute(
            "SELECT ts, value FROM samples WHERE tag=? AND ts>=? AND ts<=? ORDER BY ts ASC",
            (tag, start_ts, now_ts),
        )
        return [(int(r[0]), float(r[1])) for r in cur.fetchall()]

    def export_window_csv(self, out_path: Path, now_ts: int, tag: str, minutes: int) -> int:
        """Export the last N minutes to CSV and return the number of exported rows."""
        rows = self.query_window(now_ts, tag, minutes)
        with out_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["ts_epoch_s", "tag", "value_rpm"])
            for ts, val in rows:
                w.writerow([ts, tag, val])
        return len(rows)


# ----------------------------
# Touch-friendly Controls
# ----------------------------

class TouchButton(QPushButton):
    """Push button with a minimum hit target suitable for finger use."""

    def __init__(self, text: str, min_h: int = 70, min_w: int = 140) -> None:
        super().__init__(text)
        self.setMinimumHeight(min_h)
        self.setMinimumWidth(min_w)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


class RpmControl(QWidget):
    """
    Numeric RPM control with +/- buttons and validation.

    Emits clamped float values.
    """
    value_changed = Signal(float)

    def __init__(self, title: str, vmin: float, vmax: float, step: float, compact: bool = False) -> None:
        super().__init__()
        self._vmin = float(vmin)
        self._vmax = float(vmax)
        self._step = float(step)

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

        self.btn_minus.clicked.connect(self._dec)
        self.btn_plus.clicked.connect(self._inc)
        self.edit.editingFinished.connect(self._emit_if_valid)

        self._emit_if_valid()

    def value(self) -> float:
        v = self._parse()
        return float(v if v is not None else self._vmin)

    def set_value(self, v: float) -> None:
        v = clamp(float(v), self._vmin, self._vmax)
        self.edit.setText(f"{v:g}")
        self.value_changed.emit(v)

    def set_enabled(self, enabled: bool) -> None:
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)
        self.edit.setEnabled(enabled)

    def _parse(self) -> Optional[float]:
        t = self.edit.text().strip()
        if not t:
            return None
        try:
            return float(t)
        except ValueError:
            return None

    @Slot()
    def _emit_if_valid(self) -> None:
        v = self._parse()
        if v is None:
            self.set_value(self._vmin)
            return
        self.set_value(v)

    @Slot()
    def _dec(self) -> None:
        self.set_value(self.value() - self._step)

    @Slot()
    def _inc(self) -> None:
        self.set_value(self.value() + self._step)


class TimeControl(QWidget):
    """
    Time control displayed as mm:ss with +/- step buttons.

    Accepts mm:ss or raw seconds; clamps to [0, TIME_MAX_S].
    """
    value_changed = Signal(int)

    def __init__(self, title: str, step_s: int, compact: bool = False) -> None:
        super().__init__()
        self._step_s = int(step_s)

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

        self.btn_minus.clicked.connect(self._dec)
        self.btn_plus.clicked.connect(self._inc)
        self.edit.editingFinished.connect(self._emit_if_valid)

        self._emit_if_valid()

    def seconds(self) -> int:
        s = self._parse()
        return int(s if s is not None else 0)

    def set_seconds(self, s: int) -> None:
        s = int(clamp(int(s), 0, TIME_MAX_S))
        self.edit.setText(format_mmss(s))
        self.value_changed.emit(s)

    def set_enabled(self, enabled: bool) -> None:
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)
        self.edit.setEnabled(enabled)

    def _parse(self) -> Optional[int]:
        s = parse_mmss(self.edit.text())
        if s is None:
            return None
        return int(clamp(int(s), 0, TIME_MAX_S))

    @Slot()
    def _emit_if_valid(self) -> None:
        s = self._parse()
        if s is None:
            self.set_seconds(0)
            return
        self.set_seconds(s)

    @Slot()
    def _dec(self) -> None:
        self.set_seconds(self.seconds() - self._step_s)

    @Slot()
    def _inc(self) -> None:
        self.set_seconds(self.seconds() + self._step_s)


# ----------------------------
# Home Screen Recipe Cards
# ----------------------------

class RecipeCard(QFrame):
    """Touch-friendly recipe card with Select and Edit actions."""

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
        """Update highlight by toggling a dynamic property used by the stylesheet."""
        self.setProperty("selected", "true" if selected else "false")
        self.style().unpolish(self)
        self.style().polish(self)


# ----------------------------
# Recipe Builder Step Row (compact, touch-usable)
# ----------------------------

class StepRow(QFrame):
    """
    Compact step editor row:
      - Header: Step label + Up/Down/Remove
      - Controls: Velocity and Duration on one line
    """

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

        btn_h = 54
        btn_w = 120
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
        self.vel.set_value(step.velocity_rpm)

        self.tim = TimeControl("Time", TIME_STEP_S, compact=True)
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


# ----------------------------
# Recipe Builder Screen
# ----------------------------

class RecipeBuilderScreen(QWidget):
    """Screen to create or edit recipes with validation and JSON persistence."""

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

    def load_recipe(self, recipe: Optional[Recipe]) -> None:
        """Load an existing recipe for editing or start a new recipe when recipe is None."""
        self._clear_rows()

        if recipe is None:
            self._editing_id = None
            self.title.setText("Recipe Builder (New)")
            self.name_edit.setText("")
            self._add_row(Step(velocity_rpm=120.0, duration_s=60))
            return

        self._editing_id = recipe.id
        self.title.setText("Recipe Builder (Edit)")
        self.name_edit.setText(recipe.name)
        for s in recipe.steps:
            self._add_row(Step(velocity_rpm=float(s.velocity_rpm), duration_s=int(s.duration_s)))

    def _rows(self) -> List[StepRow]:
        rows: List[StepRow] = []
        for i in range(self.steps_layout.count() - 1):
            w = self.steps_layout.itemAt(i).widget()
            if isinstance(w, StepRow):
                rows.append(w)
        return rows

    def _clear_rows(self) -> None:
        """Remove all step row widgets while preserving the stretch."""
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

    @Slot()
    def _on_add_step(self) -> None:
        self._add_row(Step(velocity_rpm=120.0, duration_s=60))
        self._reindex()

    @Slot(int)
    def _on_remove(self, idx: int) -> None:
        rows = self._rows()
        if idx < 0 or idx >= len(rows):
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
        if idx < 0 or idx >= len(rows) - 1:
            return
        w = rows[idx]
        self.steps_layout.removeWidget(w)
        self.steps_layout.insertWidget(idx + 1, w)
        self._reindex()

    @Slot()
    def _on_save(self) -> None:
        """Validate fields and emit a fully populated Recipe."""
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
    """A single row in the ODrive Configuration screen."""

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

        self.editor: QWidget
        if spec.kind == "bool":
            cb = QComboBox()
            cb.setMinimumHeight(58)
            cb.addItems(["False", "True"])
            self.editor = cb
        else:
            le = QLineEdit()
            le.setMinimumHeight(58)
            le.setAlignment(Qt.AlignCenter)
            dv = QDoubleValidator(le)
            if spec.min_value is not None:
                dv.setBottom(float(spec.min_value))
            if spec.max_value is not None:
                dv.setTop(float(spec.max_value))
            dv.setDecimals(4)
            le.setValidator(dv)
            self.editor = le

        self.note = QLabel("")
        self.note.setObjectName("ConfigNote")
        self.note.setMinimumWidth(220)
        self.note.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

        root.addWidget(self.lbl, 2)
        root.addWidget(self.editor, 1)
        root.addWidget(self.note, 1)

    def set_value(self, value: Any) -> None:
        """Set the editor to display the given value."""
        self.note.setText("")
        self.set_enabled(True)

        if self.spec.kind == "bool":
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            cb.setCurrentIndex(1 if bool(value) else 0)
        else:
            le = self.editor  # type: ignore[assignment]
            assert isinstance(le, QLineEdit)
            le.setText(f"{float(value):g}")

    def set_missing(self, missing: bool) -> None:
        """Disable editing and show an unsupported note."""
        if missing:
            self.note.setText("Not supported")
            self.set_enabled(False)
        else:
            self.note.setText("")
            self.set_enabled(True)

    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable user interaction for this setting."""
        self.editor.setEnabled(enabled)

    def get_value(self) -> Any:
        """Return the editor value converted to the appropriate type."""
        if self.spec.kind == "bool":
            cb = self.editor  # type: ignore[assignment]
            assert isinstance(cb, QComboBox)
            return bool(cb.currentIndex() == 1)

        le = self.editor  # type: ignore[assignment]
        assert isinstance(le, QLineEdit)
        txt = le.text().strip()
        if not txt:
            return float(self.spec.default)
        return float(txt)


class ODriveConfigScreen(QWidget):
    """Screen to view/edit important ODrive settings and export configuration."""

    back_clicked = Signal()
    save_clicked = Signal(object)     # dict key->value
    request_refresh = Signal()
    export_clicked = Signal(object)   # dict key->value
    restore_defaults_clicked = Signal()

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

        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)

        self.btn_refresh = TouchButton("Refresh", min_h=66, min_w=170)
        top.addWidget(self.btn_refresh)

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

        # Build rows from schema once.
        for spec in IMPORTANT_SETTINGS:
            row = SettingRowWidget(spec)
            self._rows[spec.key] = row
            self.vbox.insertWidget(self.vbox.count() - 1, row)

        actions = QHBoxLayout()
        actions.setSpacing(12)

        self.btn_restore = TouchButton("Restore Defaults", min_h=66, min_w=230)
        self.btn_export = TouchButton("Export JSON", min_h=66, min_w=200)
        self.btn_cancel = TouchButton("Cancel", min_h=66, min_w=180)
        self.btn_save = TouchButton("Save", min_h=66, min_w=180)

        actions.addWidget(self.btn_restore, 1)
        actions.addWidget(self.btn_export, 1)
        actions.addWidget(self.btn_cancel, 1)
        actions.addWidget(self.btn_save, 1)

        root.addLayout(actions)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_cancel.clicked.connect(self.back_clicked.emit)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_export.clicked.connect(self._on_export)
        self.btn_restore.clicked.connect(self.restore_defaults_clicked.emit)
        self.btn_refresh.clicked.connect(self.request_refresh.emit)

    def set_loading(self, text: str) -> None:
        """Update the status label to indicate a background operation."""
        self.status.setText(text)

    def apply_values(self, values: Dict[str, Any], missing: List[str]) -> None:
        """Populate UI rows with values and mark missing fields as unsupported."""
        missing_set = set(missing)
        for key, row in self._rows.items():
            if key in values:
                row.set_value(values[key])
                row.set_missing(key in missing_set)
            else:
                row.set_value(DEFAULT_CONFIG_MAP.get(key, row.spec.default))
                row.set_missing(key in missing_set)
        if missing:
            self.status.setText(f"Loaded with {len(missing)} unsupported field(s) skipped")
        else:
            self.status.setText("Loaded")

    def set_defaults(self) -> None:
        """Set the UI to hard-coded defaults without writing to device."""
        for key, row in self._rows.items():
            row.set_value(DEFAULT_CONFIG_MAP.get(key, row.spec.default))
            row.set_missing(False)
        self.status.setText("Defaults restored (not saved)")

    def gather_values(self) -> Dict[str, Any]:
        """Collect editable values from the UI, skipping disabled (unsupported) fields."""
        out: Dict[str, Any] = {}
        for k, row in self._rows.items():
            if row.editor.isEnabled():
                out[k] = row.get_value()
        return out

    @Slot()
    def _on_save(self) -> None:
        self.save_clicked.emit(self.gather_values())

    @Slot()
    def _on_export(self) -> None:
        self.export_clicked.emit(self.gather_values())


# ----------------------------
# Velocity History Screen
# ----------------------------

class VelocityHistoryScreen(QWidget):
    """Screen to view and export velocity history stored in SQLite."""

    back_clicked = Signal()
    export_clicked = Signal(int)            # minutes window
    window_changed = Signal(int)            # minutes window
    screen_visible = Signal(bool)           # True on enter, False on leave

    def __init__(self) -> None:
        super().__init__()
        self._minutes = 10

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        top = QHBoxLayout()
        top.setSpacing(12)

        self.btn_back = TouchButton("Back", min_h=66, min_w=170)
        self.title = QLabel("Velocity History")
        self.title.setObjectName("ScreenTitle")

        self.btn_export = TouchButton("Export CSV", min_h=66, min_w=200)

        top.addWidget(self.btn_back)
        top.addWidget(self.title, 1)
        top.addWidget(self.btn_export)
        root.addLayout(top)

        controls = QHBoxLayout()
        controls.setSpacing(12)

        lbl = QLabel("Window (min)")
        lbl.setObjectName("FieldLabel")
        lbl.setMinimumWidth(180)

        self.edit_min = QLineEdit()
        self.edit_min.setMinimumHeight(66)
        self.edit_min.setAlignment(Qt.AlignCenter)
        self.edit_min.setValidator(QIntValidator(1, 24 * 60, self.edit_min))
        self.edit_min.setText(str(self._minutes))

        self.btn_apply = TouchButton("Apply", min_h=66, min_w=170)

        self.lbl_latest = QLabel("Latest: — rpm")
        self.lbl_latest.setObjectName("InfoLabel")

        self.lbl_logging = QLabel("Logging: OFF")
        self.lbl_logging.setObjectName("InfoLabel")

        controls.addWidget(lbl)
        controls.addWidget(self.edit_min)
        controls.addWidget(self.btn_apply)
        controls.addStretch(1)
        controls.addWidget(self.lbl_latest)
        controls.addWidget(self.lbl_logging)

        root.addLayout(controls)

        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Time", "Velocity (rpm)"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setObjectName("HistoryTable")
        self.table.horizontalHeader().setStretchLastSection(True)

        root.addWidget(self.table, 1)

        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_export.clicked.connect(self._on_export)
        self.btn_apply.clicked.connect(self._on_apply)
        self.edit_min.editingFinished.connect(self._on_apply)

    def set_logging_active(self, active: bool) -> None:
        """Update status label when polling/logging is active."""
        self.lbl_logging.setText("Logging: ON" if active else "Logging: OFF")

    def set_latest_rpm(self, rpm: float) -> None:
        """Update the latest value label with the most recently polled RPM."""
        self.lbl_latest.setText(f"Latest: {rpm:.1f} rpm")

    def minutes_window(self) -> int:
        """Return current minutes window."""
        try:
            return max(1, int(self.edit_min.text()))
        except Exception:
            return 10

    def load_rows(self, rows: List[Tuple[int, float]]) -> None:
        """Replace the table contents with the provided rows."""
        self.table.setRowCount(0)
        for ts, val in rows:
            self._append_row(ts, val)
        self._scroll_to_bottom()

    def append_row(self, ts: int, val: float) -> None:
        """Append a row to the table."""
        self._append_row(ts, val)
        self._scroll_to_bottom()

    def prune_older_than(self, cutoff_ts: int) -> None:
        """Remove table rows older than cutoff_ts."""
        # Rows are sorted by time ascending; remove from top until within window.
        while self.table.rowCount() > 0:
            item = self.table.item(0, 0)
            if item is None:
                break
            ts = int(item.data(Qt.UserRole) or 0)
            if ts >= cutoff_ts:
                break
            self.table.removeRow(0)

    def _append_row(self, ts: int, val: float) -> None:
        r = self.table.rowCount()
        self.table.insertRow(r)

        t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts))
        it0 = QTableWidgetItem(t_str)
        it0.setData(Qt.UserRole, ts)

        it1 = QTableWidgetItem(f"{val:.2f}")

        self.table.setItem(r, 0, it0)
        self.table.setItem(r, 1, it1)

    def _scroll_to_bottom(self) -> None:
        if self.table.rowCount() > 0:
            self.table.scrollToBottom()

    @Slot()
    def _on_apply(self) -> None:
        mins = self.minutes_window()
        self.window_changed.emit(mins)

    @Slot()
    def _on_export(self) -> None:
        self.export_clicked.emit(self.minutes_window())


# ----------------------------
# Home Screen
# ----------------------------

class HomeScreen(QWidget):
    """Main screen: status bar, manual controls, recipe list, and navigation/actions."""

    start_clicked = Signal()
    stop_clicked = Signal()
    open_builder_clicked = Signal()
    clear_selection_clicked = Signal()
    recipe_select_clicked = Signal(str)
    recipe_edit_clicked = Signal(str)
    open_config_clicked = Signal()
    open_history_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(12)

        self.status_bar = QFrame()
        self.status_bar.setObjectName("StatusBar")
        sb = QHBoxLayout(self.status_bar)
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

        root.addWidget(self.status_bar)

        mid = QHBoxLayout()
        mid.setSpacing(14)

        left = QVBoxLayout()
        left.setSpacing(12)

        self.rpm_ctl = RpmControl("Velocity", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM, compact=False)
        self.time_ctl = TimeControl("Timer", TIME_STEP_S, compact=False)

        self.btn_builder = TouchButton("Recipe Builder", min_h=70, min_w=240)
        self.btn_config = TouchButton("ODrive Configuration", min_h=70, min_w=240)
        self.btn_history = TouchButton("Velocity History", min_h=70, min_w=240)
        self.btn_clear = TouchButton("Clear Selection", min_h=70, min_w=240)

        left.addWidget(self.rpm_ctl)
        left.addWidget(self.time_ctl)
        left.addWidget(self.btn_builder)
        left.addWidget(self.btn_config)
        left.addWidget(self.btn_history)
        left.addWidget(self.btn_clear)
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

        bottom = QHBoxLayout()
        bottom.setSpacing(14)

        self.btn_start = TouchButton("Start", min_h=76, min_w=240)
        self.btn_stop = TouchButton("Stop", min_h=76, min_w=240)
        self.btn_start.setObjectName("StartButton")
        self.btn_stop.setObjectName("StopButton")

        bottom.addWidget(self.btn_start, 1)
        bottom.addWidget(self.btn_stop, 1)
        root.addLayout(bottom)

        self.btn_start.clicked.connect(self.start_clicked.emit)
        self.btn_stop.clicked.connect(self.stop_clicked.emit)
        self.btn_builder.clicked.connect(self.open_builder_clicked.emit)
        self.btn_clear.clicked.connect(self.clear_selection_clicked.emit)
        self.btn_config.clicked.connect(self.open_config_clicked.emit)
        self.btn_history.clicked.connect(self.open_history_clicked.emit)

    def set_recipe_cards(self, recipes: List[Recipe], selected_id: Optional[str]) -> None:
        """Rebuild the recipe card list."""
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

    def set_running_ui(self, running: bool) -> None:
        """
        Enforce key home-screen behaviors:
          - Start disabled while running
          - Stop enabled only while running
          - Lock inputs and navigation while running
        """
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)

        self.rpm_ctl.set_enabled(not running)
        self.time_ctl.set_enabled(not running)

        self.btn_builder.setEnabled(not running)
        self.btn_config.setEnabled(not running)
        self.btn_history.setEnabled(not running)
        self.btn_clear.setEnabled(not running)
        self.recipes_area.setEnabled(not running)

    def set_timer_enabled(self, enabled: bool) -> None:
        """Disable the manual timer input when a recipe is selected."""
        self.time_ctl.set_enabled(enabled)

    def set_status(
        self,
        ui_state: str,
        rpm: float,
        remaining_s: int,
        recipe_name: str,
        step_info: str,
        backend_state: str,
    ) -> None:
        self.lbl_state.setText(f"State: {ui_state}")
        self.lbl_vel.setText(f"Velocity: {rpm:g} rpm")
        self.lbl_rem.setText(f"Remaining: {format_mmss(remaining_s)}")
        self.lbl_recipe.setText(f"Recipe: {recipe_name}")
        self.lbl_step.setText(step_info)
        self.lbl_backend.setText(backend_state)


# ----------------------------
# Controller (wires UI, engine, storage, worker)
# ----------------------------

class AppController(QObject):
    """
    Central controller:
      - Manages selection state and start/stop rules
      - Persists recipes via RecipeStore
      - Drives RunEngine and sends motor commands to OdriveWorker
      - Manages additional screens (ODrive config and velocity historian)
      - Ensures safe shutdown by stopping the motor when exiting
    """

    request_connect = Signal()
    request_read_config = Signal(object)     # list[str]
    request_apply_config = Signal(object)    # dict
    request_start_poll = Signal()
    request_stop_poll = Signal()

    def __init__(
        self,
        stack: QStackedWidget,
        home: HomeScreen,
        builder: RecipeBuilderScreen,
        cfg: ODriveConfigScreen,
        hist: VelocityHistoryScreen,
    ) -> None:
        super().__init__()
        self.stack = stack
        self.home = home
        self.builder = builder
        self.cfg = cfg
        self.hist = hist

        self.store = RecipeStore()
        self.store.load()

        self.selected_recipe_id: Optional[str] = None
        self.connected: bool = False
        self.backend_state: str = "Disconnected"

        self._ui_state = "Idle"
        self._ui_rpm = 0.0
        self._ui_remaining = 0
        self._ui_step_info = ""

        # Historian store (SQLite in config dir).
        self.history_db = HistoryDB(app_config_dir() / "history.sqlite")
        self._history_window_min = 10
        self._history_active = False

        # Backend + worker thread.
        backend: OdriveInterface = SimulatedOdrive() if BACKEND == "sim" else RealOdrive()
        self.worker_thread = QThread()
        self.worker = OdriveWorker(backend)
        self.worker.moveToThread(self.worker_thread)

        # Run engine.
        self.engine = RunEngine()

        # Engine -> worker.
        self.engine.request_set_rpm.connect(self.worker.set_velocity_rpm)
        self.engine.request_stop.connect(self.worker.stop)

        # Controller -> worker (queued).
        self.request_connect.connect(self.worker.connect_backend)
        self.request_read_config.connect(self.worker.read_config)
        self.request_apply_config.connect(self.worker.apply_config)
        self.request_start_poll.connect(self.worker.start_velocity_polling)
        self.request_stop_poll.connect(self.worker.stop_velocity_polling)

        # Worker -> controller/UI.
        self.worker.connected_changed.connect(self._on_connected)
        self.worker.state_changed.connect(self._on_backend_state)
        self.worker.error.connect(self._on_worker_error)
        self.worker.config_read.connect(self._on_config_read)
        self.worker.config_applied.connect(self._on_config_applied)
        self.worker.velocity_sample.connect(self._on_velocity_sample)

        # Engine -> UI.
        self.engine.status_changed.connect(self._on_engine_status)
        self.engine.active_rpm_changed.connect(self._on_engine_rpm)
        self.engine.remaining_changed.connect(self._on_engine_remaining)
        self.engine.step_info_changed.connect(self._on_step_info)

        # Home UI -> controller.
        self.home.start_clicked.connect(self.on_start)
        self.home.stop_clicked.connect(self.on_stop)
        self.home.open_builder_clicked.connect(self.on_open_builder_new)
        self.home.clear_selection_clicked.connect(self.on_clear_selection)
        self.home.recipe_select_clicked.connect(self.on_select_recipe)
        self.home.recipe_edit_clicked.connect(self.on_edit_recipe)
        self.home.open_config_clicked.connect(self.on_open_config)
        self.home.open_history_clicked.connect(self.on_open_history)

        # Builder UI -> controller.
        self.builder.back_clicked.connect(self.on_back_home)
        self.builder.cancel_clicked.connect(self.on_back_home)
        self.builder.save_clicked.connect(self.on_builder_save)

        # Config UI -> controller.
        self.cfg.back_clicked.connect(self.on_back_home)
        self.cfg.request_refresh.connect(self._refresh_odrive_config)
        self.cfg.save_clicked.connect(self._save_odrive_config)
        self.cfg.restore_defaults_clicked.connect(self._restore_odrive_defaults)
        self.cfg.export_clicked.connect(self._export_odrive_config)

        # History UI -> controller.
        self.hist.back_clicked.connect(self.on_back_home)
        self.hist.window_changed.connect(self._on_history_window_changed)
        self.hist.export_clicked.connect(self._export_history_csv)

        # Track screen changes to start/stop polling automatically.
        self.stack.currentChanged.connect(self._on_screen_changed)

        # Start worker thread and connect backend.
        self.worker_thread.start()
        self.request_connect.emit()

        # Initial UI build.
        self._refresh_recipes()
        self._refresh_home()

    def shutdown(self) -> None:
        """
        Stop the motor safely if running, stop historian polling, and shut down the worker thread.
        """
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

    # ---- Internal helpers ----

    def _recipes(self) -> List[Recipe]:
        return self.store.recipes()

    def _selected_recipe(self) -> Optional[Recipe]:
        if not self.selected_recipe_id:
            return None
        for r in self._recipes():
            if r.id == self.selected_recipe_id:
                return r
        return None

    def _refresh_recipes(self) -> None:
        self.home.set_recipe_cards(self._recipes(), self.selected_recipe_id)
        self.home.set_timer_enabled(enabled=(self._selected_recipe() is None))

    def _refresh_home(self) -> None:
        selected = self._selected_recipe()
        recipe_name = selected.name if selected else "(none)"
        self.home.set_status(
            ui_state=self._ui_state,
            rpm=self._ui_rpm,
            remaining_s=self._ui_remaining,
            recipe_name=recipe_name,
            step_info=self._ui_step_info,
            backend_state=self.backend_state,
        )
        self.home.set_running_ui(self.engine.is_running())

    # ---- Worker callbacks ----

    @Slot(bool, str)
    def _on_connected(self, connected: bool, state: str) -> None:
        self.connected = connected
        self.backend_state = state
        if BACKEND == "real" and not connected:
            QMessageBox.critical(
                self.home,
                "ODrive Connection",
                "ODrive is not connected.\n\n"
                "Check power/USB and ensure the odrive Python package is installed.",
            )
        self._refresh_home()

    @Slot(str)
    def _on_backend_state(self, state: str) -> None:
        self.backend_state = state
        self._refresh_home()

    @Slot(str)
    def _on_worker_error(self, msg: str) -> None:
        """
        Any worker error is surfaced to the user. If a run is active, stop the motor safely.
        """
        if self.engine.is_running():
            self.engine.stop(user_initiated=True)
        QMessageBox.critical(self.stack.currentWidget(), "ODrive Error", msg)
        self._refresh_home()

    @Slot(object, object)
    def _on_config_read(self, values_obj: Any, missing_obj: Any) -> None:
        values = dict(values_obj or {})
        missing = list(missing_obj or [])
        if self.stack.currentWidget() is self.cfg:
            self.cfg.apply_values(values, missing)

    @Slot(bool, str, object)
    def _on_config_applied(self, ok: bool, message: str, missing_obj: Any) -> None:
        missing = list(missing_obj or [])
        if ok:
            QMessageBox.information(
                self.cfg,
                "Configuration",
                f"{message}\n\nUnsupported skipped: {len(missing)}",
            )
            self.on_back_home()
        else:
            QMessageBox.critical(self.cfg, "Configuration", message)

    @Slot(int, float)
    def _on_velocity_sample(self, ts: int, rpm: float) -> None:
        """
        Handle polled velocity:
          - Update live label
          - Store in SQLite using compression
          - Update table when a stored point is added
        """
        if self.stack.currentWidget() is self.hist:
            self.hist.set_latest_rpm(rpm)

        inserted = False
        try:
            inserted = self.history_db.insert_if_needed(
                ts=ts,
                tag=HIST_TAG,
                value=rpm,
                deadband=HIST_DEADBAND_RPM,
                heartbeat_s=HIST_HEARTBEAT_S,
            )
        except Exception as e:
            QMessageBox.critical(self.hist, "History DB", f"Failed to write sample: {e}")
            return

        if self.stack.currentWidget() is self.hist and inserted:
            cutoff = ts - self._history_window_min * 60
            self.hist.append_row(ts, rpm)
            self.hist.prune_older_than(cutoff)

    # ---- Engine callbacks ----

    @Slot(str)
    def _on_engine_status(self, status: str) -> None:
        self._ui_state = status
        self._refresh_home()

    @Slot(float)
    def _on_engine_rpm(self, rpm: float) -> None:
        self._ui_rpm = float(rpm)
        self._refresh_home()

    @Slot(int)
    def _on_engine_remaining(self, seconds: int) -> None:
        self._ui_remaining = int(seconds)
        self._refresh_home()

    @Slot(str)
    def _on_step_info(self, info: str) -> None:
        self._ui_step_info = info
        self._refresh_home()

    # ---- Screen change handling ----

    @Slot(int)
    def _on_screen_changed(self, _idx: int) -> None:
        """
        Start/stop velocity polling based on whether the history screen is active.
        """
        if self.stack.currentWidget() is self.hist:
            self._start_history_logging()
        else:
            self._stop_history_logging()

    def _start_history_logging(self) -> None:
        if self._history_active:
            return
        self._history_active = True
        self.hist.set_logging_active(True)
        self.request_start_poll.emit()

        # Load current window from DB into the table.
        self._history_window_min = self.hist.minutes_window()
        try:
            rows = self.history_db.query_window(epoch_s(), HIST_TAG, self._history_window_min)
        except Exception as e:
            QMessageBox.critical(self.hist, "History DB", f"Failed to load samples: {e}")
            rows = []
        self.hist.load_rows(rows)

    def _stop_history_logging(self) -> None:
        if not self._history_active:
            return
        self._history_active = False
        self.hist.set_logging_active(False)
        self.request_stop_poll.emit()

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
    def on_open_history(self) -> None:
        if self.engine.is_running():
            return
        self.stack.setCurrentWidget(self.hist)

    @Slot()
    def on_back_home(self) -> None:
        self.stack.setCurrentWidget(self.home)
        self._refresh_recipes()
        self._refresh_home()

    @Slot()
    def on_clear_selection(self) -> None:
        if self.engine.is_running():
            return
        self.selected_recipe_id = None
        self._refresh_recipes()
        self._refresh_home()

    @Slot(str)
    def on_select_recipe(self, recipe_id: str) -> None:
        if self.engine.is_running():
            return
        self.selected_recipe_id = None if self.selected_recipe_id == recipe_id else recipe_id
        self._refresh_recipes()
        self._refresh_home()

    @Slot(str)
    def on_edit_recipe(self, recipe_id: str) -> None:
        if self.engine.is_running():
            return
        r = next((x for x in self._recipes() if x.id == recipe_id), None)
        if r is None:
            return
        self.builder.load_recipe(r)
        self.stack.setCurrentWidget(self.builder)

    @Slot(Recipe)
    def on_builder_save(self, recipe: Recipe) -> None:
        self.store.upsert(recipe)
        self.stack.setCurrentWidget(self.home)
        self._refresh_recipes()
        self._refresh_home()

    # ---- ODrive Config actions ----

    def _refresh_odrive_config(self) -> None:
        """Request a config read in the worker thread and populate the config screen."""
        keys = [s.key for s in IMPORTANT_SETTINGS]
        self.cfg.set_loading("Loading from device…")
        # In sim mode, allow refresh without needing connection state.
        if BACKEND == "real" and not self.connected:
            self.cfg.apply_values(DEFAULT_CONFIG_MAP, keys)
            self.cfg.set_loading("Not connected (showing defaults)")
            return
        self.request_read_config.emit(keys)

    @Slot()
    def _restore_odrive_defaults(self) -> None:
        self.cfg.set_defaults()

    @Slot(object)
    def _save_odrive_config(self, values_obj: Any) -> None:
        values = dict(values_obj or {})
        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.cfg, "Configuration", "Cannot save: ODrive is not connected.")
            return
        self.cfg.set_loading("Applying config…")
        self.request_apply_config.emit(values)

    @Slot(object)
    def _export_odrive_config(self, values_obj: Any) -> None:
        values = dict(values_obj or {})
        out = downloads_dir() / f"odrive_config_{time.strftime('%Y%m%d_%H%M%S')}.json"
        payload = {
            "exported_at_epoch_s": epoch_s(),
            "backend": BACKEND,
            "values": values,
        }
        try:
            out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            QMessageBox.information(self.cfg, "Export", f"Exported to:\n{out}")
        except Exception as e:
            QMessageBox.critical(self.cfg, "Export", f"Failed to export JSON:\n{e}")

    # ---- Velocity History actions ----

    @Slot(int)
    def _on_history_window_changed(self, minutes: int) -> None:
        self._history_window_min = max(1, int(minutes))
        if self.stack.currentWidget() is not self.hist:
            return
        try:
            rows = self.history_db.query_window(epoch_s(), HIST_TAG, self._history_window_min)
            self.hist.load_rows(rows)
        except Exception as e:
            QMessageBox.critical(self.hist, "History DB", f"Failed to load window:\n{e}")

    @Slot(int)
    def _export_history_csv(self, minutes: int) -> None:
        minutes = max(1, int(minutes))
        out = downloads_dir() / f"velocity_history_{minutes}min_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            n = self.history_db.export_window_csv(out, epoch_s(), HIST_TAG, minutes)
            QMessageBox.information(self.hist, "Export", f"Exported {n} row(s) to:\n{out}")
        except Exception as e:
            QMessageBox.critical(self.hist, "Export", f"Failed to export CSV:\n{e}")


# ----------------------------
# Main Window
# ----------------------------

class MainWindow(QWidget):
    """Fixed-size window hosting all screens."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Reactor Agitator HMI")
        self.setFixedSize(WINDOW_W, WINDOW_H)

        self.stack = QStackedWidget()

        self.home = HomeScreen()
        self.builder = RecipeBuilderScreen()
        self.cfg = ODriveConfigScreen()
        self.hist = VelocityHistoryScreen()

        self.stack.addWidget(self.home)
        self.stack.addWidget(self.builder)
        self.stack.addWidget(self.cfg)
        self.stack.addWidget(self.hist)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.addWidget(self.stack)

        self.controller = AppController(self.stack, self.home, self.builder, self.cfg, self.hist)

    def closeEvent(self, event) -> None:
        self.controller.shutdown()
        event.accept()


# ----------------------------
# Styling
# ----------------------------

def apply_style(app: QApplication) -> None:
    """Apply a high-contrast, large-font stylesheet suitable for touchscreens."""
    base_font = QFont()
    base_font.setPointSize(12)
    app.setFont(base_font)

    app.setStyleSheet(
        """
        QWidget {
            background: #101317;
            color: #F2F5F7;
            font-size: 20px;
        }

        QLabel#ScreenTitle {
            font-size: 28px;
            font-weight: 700;
        }

        QLabel#SectionTitle {
            font-size: 24px;
            font-weight: 650;
        }

        QLabel#SectionTitleCompact {
            font-size: 20px;
            font-weight: 650;
        }

        QLabel#FieldLabel {
            font-size: 22px;
            font-weight: 650;
        }

        QLabel#UnitsLabel {
            font-size: 18px;
            color: #D0D7DE;
        }

        QLabel#InfoLabel {
            font-size: 18px;
            color: #D0D7DE;
        }

        QFrame#StatusBar {
            background: #161B22;
            border: 2px solid #2A313A;
            border-radius: 14px;
        }

        QLabel#StatusLabel {
            font-size: 20px;
            font-weight: 650;
        }

        QLabel#StatusStep {
            font-size: 18px;
            color: #D0D7DE;
        }

        QLabel#StatusBackend {
            font-size: 16px;
            color: #9FB0C0;
        }

        QLineEdit {
            background: #0B0F14;
            border: 2px solid #2A313A;
            border-radius: 14px;
            padding: 8px 12px;
            font-size: 26px;
        }

        QComboBox {
            background: #0B0F14;
            border: 2px solid #2A313A;
            border-radius: 14px;
            padding: 8px 12px;
            font-size: 22px;
            min-height: 58px;
        }

        QPushButton {
            background: #2B3440;
            border: 2px solid #3A4656;
            border-radius: 16px;
            padding: 10px 14px;
            font-size: 24px;
            font-weight: 650;
        }

        QPushButton:hover { background: #354152; }
        QPushButton:pressed { background: #202733; }

        QPushButton:disabled {
            background: #1C222B;
            border: 2px solid #2A313A;
            color: #7A8794;
        }

        QPushButton#StartButton {
            background: #1E5F3C;
            border: 2px solid #2D7B52;
        }
        QPushButton#StartButton:hover { background: #21704A; }

        QPushButton#StopButton {
            background: #6B1E1E;
            border: 2px solid #8B2A2A;
        }
        QPushButton#StopButton:hover { background: #7A2424; }

        QFrame#RecipeCard {
            background: #161B22;
            border: 2px solid #2A313A;
            border-radius: 16px;
        }

        QFrame#RecipeCard[selected="true"] {
            border: 4px solid #2F81F7;
        }

        QLabel#RecipeName {
            font-size: 24px;
            font-weight: 700;
        }

        QLabel#RecipeSummary {
            font-size: 18px;
            color: #D0D7DE;
        }

        QFrame#StepRow {
            background: #161B22;
            border: 2px solid #2A313A;
            border-radius: 16px;
        }

        QLabel#StepIndex {
            font-size: 20px;
            font-weight: 700;
        }

        QFrame#ConfigRow {
            background: #161B22;
            border: 2px solid #2A313A;
            border-radius: 16px;
        }

        QLabel#ConfigLabel {
            font-size: 20px;
            font-weight: 650;
        }

        QLabel#ConfigNote {
            font-size: 18px;
            color: #9FB0C0;
        }

        QTableWidget#HistoryTable {
            background: #0B0F14;
            border: 2px solid #2A313A;
            border-radius: 14px;
            font-size: 18px;
            gridline-color: #2A313A;
        }

        QHeaderView::section {
            background: #161B22;
            border: 1px solid #2A313A;
            padding: 8px;
            font-size: 18px;
            font-weight: 650;
        }

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
