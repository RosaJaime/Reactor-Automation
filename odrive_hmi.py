"""
Touch-Friendly ODrive Reactor Agitator HMI (PySide6)

How to run (simulation default):
  pip install PySide6
  python odrive_hmi.py

How to switch to real ODrive later:
  1) Set BACKEND = "real" near the top of this file.
  2) Implement RealOdrive.connect(), set_velocity_rpm(), stop() using the odrive library calls
     shown as comments inside RealOdrive (stub).
  3) Ensure any ODrive configuration (control mode, input mode, etc.) is applied in connect().

Design goals:
  - Fixed 1280x800 layout, large fonts, large buttons, wide spacing for finger use.
  - Recipes persisted to a JSON file in a per-user config directory.
  - Non-blocking UI: motor commands run in a dedicated worker thread; timing uses a QTimer state machine.
"""

from __future__ import annotations

import json
import os
import sys
import time
import uuid
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Tuple

from PySide6.QtCore import (
    QObject,
    Qt,
    QThread,
    QTimer,
    QStandardPaths,
    Signal,
    Slot,
    QMetaObject,
)
from PySide6.QtGui import QDoubleValidator, QFont, QIntValidator
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)


# ----------------------------
# Configuration / Safety Limits
# ----------------------------

BACKEND = "sim"  # "sim" (default) or "real"

WINDOW_W = 1280
WINDOW_H = 800

# UI uses RPM consistently.
VEL_MIN_RPM = 0.0
VEL_MAX_RPM = 2000.0
VEL_STEP_RPM = 1.0

# Timer UI uses mm:ss, stored internally as seconds.
TIME_STEP_S = 10
TIME_MAX_S = 24 * 60 * 60  # 24h cap to avoid absurd values

# QTimer tick rate for run engine. The display updates about 5 Hz; countdown rounds to seconds.
ENGINE_TICK_MS = 200


# ----------------------------
# Utilities
# ----------------------------

def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp a numeric value to [lo, hi]."""
    return max(lo, min(hi, v))


def format_mmss(seconds: int) -> str:
    """Format seconds as mm:ss (minutes can exceed 59 for long runs)."""
    if seconds < 0:
        seconds = 0
    m = seconds // 60
    s = seconds % 60
    return f"{m:02d}:{s:02d}"


def parse_mmss(text: str) -> Optional[int]:
    """
    Parse "mm:ss" into seconds.
    Accepts:
      - "m:ss" or "mm:ss" or "mmm:ss"
      - "ss" (treated as seconds)
    Returns None on invalid input.
    """
    t = text.strip()
    if not t:
        return None

    if ":" not in t:
        # Pure seconds input (e.g., "90").
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
    """Compute total duration of a recipe in seconds."""
    return sum(max(0, s.duration_s) for s in r.steps)


def recipe_summary(r: Recipe) -> str:
    """Generate a user-facing one-line recipe summary."""
    n = len(r.steps)
    total = recipe_total_seconds(r)
    return f"{n} step{'s' if n != 1 else ''} • {format_mmss(total)} total"


# ----------------------------
# Recipe Storage (JSON, per-user config directory)
# ----------------------------

class RecipeStore:
    """
    Loads/saves recipes to JSON in a per-user config directory.

    Storage format is intentionally simple:
      {
        "recipes": [
          {"id": "...", "name": "...", "steps": [{"velocity_rpm": 100.0, "duration_s": 30}, ...]},
          ...
        ]
      }
    """

    def __init__(self) -> None:
        self._recipes: List[Recipe] = []
        self._path = self._default_path()

    @staticmethod
    def _default_path() -> Path:
        base = QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation)
        if not base:
            base = str(Path.home() / ".config")
        d = Path(base) / "PolyJouleOdriveHMI"
        d.mkdir(parents=True, exist_ok=True)
        return d / "recipes.json"

    @property
    def path(self) -> Path:
        return self._path

    def recipes(self) -> List[Recipe]:
        # Return a shallow copy to avoid accidental in-place mutation without saving.
        return list(self._recipes)

    def load(self) -> None:
        if not self._path.exists():
            self._recipes = self._default_seed_recipes()
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
            # If the file is corrupted, preserve it and start fresh.
            bad = self._path.with_suffix(".corrupt.json")
            try:
                self._path.replace(bad)
            except Exception:
                pass
            self._recipes = self._default_seed_recipes()
            self.save()

    def save(self) -> None:
        obj = {
            "recipes": [
                {"id": r.id, "name": r.name, "steps": [asdict(s) for s in r.steps]}
                for r in self._recipes
            ]
        }
        self._path.write_text(json.dumps(obj, indent=2), encoding="utf-8")

    def upsert(self, recipe: Recipe) -> None:
        for i, r in enumerate(self._recipes):
            if r.id == recipe.id:
                self._recipes[i] = recipe
                self.save()
                return
        self._recipes.append(recipe)
        self.save()

    def delete(self, recipe_id: str) -> None:
        self._recipes = [r for r in self._recipes if r.id != recipe_id]
        self.save()

    @staticmethod
    def _default_seed_recipes() -> List[Recipe]:
        # A small default recipe so the UI isn't empty on first boot.
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
# ODrive Interface and Implementations
# ----------------------------

class OdriveInterface:
    """
    Abstract ODrive interface used by the worker thread.

    The UI speaks in RPM. The backend may convert to turns/s depending on actual ODrive API usage.
    """

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


class SimulatedOdrive(OdriveInterface):
    """
    Simulation backend that never blocks and provides deterministic behavior.

    This class keeps an internal velocity and a simple state string suitable for UI display.
    """

    def __init__(self) -> None:
        self._connected = False
        self._rpm = 0.0
        self._state = "Disconnected"

    def connect(self) -> bool:
        self._connected = True
        self._state = "Idle"
        return True

    def is_connected(self) -> bool:
        return self._connected

    def set_velocity_rpm(self, rpm: float) -> None:
        self._rpm = float(rpm)
        self._state = "Running" if abs(self._rpm) > 1e-6 else "Idle"

    def stop(self) -> None:
        self._rpm = 0.0
        self._state = "Idle"

    def get_state(self) -> str:
        return self._state


class RealOdrive(OdriveInterface):
    """
    Real backend stub.

    Implementations must avoid long blocking calls in set_velocity_rpm()/stop().
    All methods run in the OdriveWorker thread, not the UI thread.

    Suggested ODrive concepts (for later implementation):
      - odrive.find_any() to connect
      - Configure axis controller for velocity control
      - Convert RPM -> turns/s: turns_per_sec = rpm / 60.0
      - Set input velocity and ensure safe stop on errors
    """

    def __init__(self) -> None:
        self._connected = False
        self._state = "Disconnected"
        self._rpm = 0.0
        self._odrv = None

    def connect(self) -> bool:
        # Example outline (do not use as-is, implement for your hardware and firmware):
        #   import odrive
        #   from odrive.enums import AxisState, ControlMode, InputMode
        #   self._odrv = odrive.find_any(timeout=5)
        #   axis = self._odrv.axis0
        #   axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        #   axis.controller.config.input_mode = InputMode.PASSTHROUGH
        #   axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        #   self._connected = True
        #   self._state = "Idle"
        #   return True
        self._state = "Error: Real backend not implemented"
        self._connected = False
        return False

    def is_connected(self) -> bool:
        return self._connected

    def set_velocity_rpm(self, rpm: float) -> None:
        # Example outline:
        #   axis = self._odrv.axis0
        #   axis.controller.input_vel = float(rpm) / 60.0  # turns/s
        self._rpm = float(rpm)
        if not self._connected:
            raise RuntimeError("ODrive not connected")

    def stop(self) -> None:
        # Example outline:
        #   axis = self._odrv.axis0
        #   axis.controller.input_vel = 0.0
        #   axis.requested_state = AxisState.IDLE
        self._rpm = 0.0
        if not self._connected:
            return

    def get_state(self) -> str:
        return self._state


# ----------------------------
# ODrive Worker (threaded motor command execution)
# ----------------------------

class OdriveWorker(QObject):
    """
    Runs ODrive backend calls in its own thread.

    The UI and engine communicate with this worker via queued signals/slots, ensuring
    that any real hardware calls do not block the UI thread.
    """

    connected_changed = Signal(bool, str)  # connected, backend_state
    rpm_changed = Signal(float)            # last commanded rpm
    backend_state_changed = Signal(str)    # backend-provided state string
    error = Signal(str)

    def __init__(self, backend: OdriveInterface) -> None:
        super().__init__()
        self._backend = backend

    @Slot()
    def connect_backend(self) -> None:
        try:
            ok = self._backend.connect()
            self.connected_changed.emit(ok, self._backend.get_state())
            self.backend_state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Connect failed: {e}")
            self.connected_changed.emit(False, "Error")
            self.backend_state_changed.emit("Error")

    @Slot(float)
    def set_velocity_rpm(self, rpm: float) -> None:
        try:
            self._backend.set_velocity_rpm(rpm)
            self.rpm_changed.emit(float(rpm))
            self.backend_state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Motor command failed: {e}")

    @Slot()
    def stop(self) -> None:
        try:
            self._backend.stop()
            self.rpm_changed.emit(0.0)
            self.backend_state_changed.emit(self._backend.get_state())
        except Exception as e:
            self.error.emit(f"Stop failed: {e}")

    def is_connected(self) -> bool:
        # This method is read-only and used only for cached state checks.
        # For real backends, prefer emitting connected_changed from connect_backend() and tracking it in the controller.
        return self._backend.is_connected()


# ----------------------------
# Run Engine (QTimer-driven state machine)
# ----------------------------

class RunEngine(QObject):
    """
    Drives timed single runs and multi-step recipe runs.

    The engine tracks its own state and emits signals for UI updates.
    Motor commands are sent via signals to the OdriveWorker (thread-safe queued delivery).
    """

    status_changed = Signal(str)  # "Idle", "Running", "Stopped", "Error"
    active_rpm_changed = Signal(float)
    recipe_step_changed = Signal(str)      # user-facing step info
    time_remaining_changed = Signal(int)   # seconds remaining for current mode
    total_remaining_changed = Signal(int)  # seconds remaining overall (recipe)

    request_set_rpm = Signal(float)
    request_stop = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._timer = QTimer(self)
        self._timer.setInterval(ENGINE_TICK_MS)
        self._timer.timeout.connect(self._tick)

        self._mode: str = "idle"  # "idle" | "single" | "recipe"
        self._status: str = "Idle"

        self._active_rpm: float = 0.0

        # Single run fields
        self._single_end_t: float = 0.0

        # Recipe run fields
        self._recipe: Optional[Recipe] = None
        self._step_idx: int = -1
        self._step_end_t: float = 0.0
        self._recipe_end_t: float = 0.0

    def is_running(self) -> bool:
        return self._mode in ("single", "recipe")

    def status(self) -> str:
        return self._status

    def _set_status(self, status: str) -> None:
        if status != self._status:
            self._status = status
            self.status_changed.emit(status)

    @Slot(float, int)
    def start_single(self, rpm: float, duration_s: int) -> None:
        """
        Start a single timed run. Duration must be > 0.
        """
        self.stop()  # Ensures a clean state transition.
        self._mode = "single"
        self._active_rpm = float(rpm)
        now = time.monotonic()
        self._single_end_t = now + max(0, int(duration_s))

        self.request_set_rpm.emit(self._active_rpm)
        self.active_rpm_changed.emit(self._active_rpm)

        self.recipe_step_changed.emit("")
        self.total_remaining_changed.emit(int(duration_s))
        self._set_status("Running")

        self._timer.start()
        self._tick()

    @Slot(object)
    def start_recipe(self, recipe: Recipe) -> None:
        """
        Start a multi-step recipe run.
        """
        self.stop()
        self._mode = "recipe"
        self._recipe = recipe
        self._step_idx = 0

        now = time.monotonic()
        total_s = recipe_total_seconds(recipe)
        self._recipe_end_t = now + total_s

        # Prime first step.
        self._start_current_step(now)
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

        step_label = f"Step {self._step_idx + 1}/{len(self._recipe.steps)} • {step.velocity_rpm:g} rpm • {format_mmss(step.duration_s)}"
        self.recipe_step_changed.emit(step_label)

    @Slot()
    def stop(self) -> None:
        """
        Stop any running mode immediately and return to Idle.
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
        self.recipe_step_changed.emit("")
        self.time_remaining_changed.emit(0)
        self.total_remaining_changed.emit(0)
        self._set_status("Idle")

    @Slot()
    def _tick(self) -> None:
        """
        Periodic tick that updates countdowns and advances recipe steps.

        The engine computes remaining time using monotonic time and rounds up/down to
        whole seconds for display.
        """
        now = time.monotonic()

        if self._mode == "single":
            remaining = int(round(self._single_end_t - now))
            if remaining <= 0:
                self.request_stop.emit()
                self._mode = "idle"
                self._active_rpm = 0.0
                self.active_rpm_changed.emit(0.0)
                self.time_remaining_changed.emit(0)
                self.total_remaining_changed.emit(0)
                self._set_status("Idle")
                self._timer.stop()
                return

            self.time_remaining_changed.emit(remaining)
            self.total_remaining_changed.emit(remaining)
            return

        if self._mode == "recipe":
            if not self._recipe:
                self.stop()
                return

            step_remaining = int(round(self._step_end_t - now))
            total_remaining = int(round(self._recipe_end_t - now))

            if step_remaining <= 0:
                # Advance to next step or finish.
                self._step_idx += 1
                if self._step_idx >= len(self._recipe.steps):
                    self.request_stop.emit()
                    self._mode = "idle"
                    self._active_rpm = 0.0
                    self.active_rpm_changed.emit(0.0)
                    self.recipe_step_changed.emit("")
                    self.time_remaining_changed.emit(0)
                    self.total_remaining_changed.emit(0)
                    self._set_status("Idle")
                    self._timer.stop()
                    return

                self._start_current_step(now)
                step_remaining = int(round(self._step_end_t - now))

            # Clamp negatives that can appear due to rounding.
            self.time_remaining_changed.emit(max(0, step_remaining))
            self.total_remaining_changed.emit(max(0, total_remaining))
            return

        # Idle mode
        self.time_remaining_changed.emit(0)
        self.total_remaining_changed.emit(0)


# ----------------------------
# Touch-friendly input widgets
# ----------------------------

class BigButton(QPushButton):
    """A QPushButton with touch-friendly sizing."""
    def __init__(self, text: str) -> None:
        super().__init__(text)
        self.setMinimumHeight(70)
        self.setMinimumWidth(160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)


class VelocityAdjust(QWidget):
    """
    Touch-friendly RPM control with +/- buttons and a numeric entry.

    This widget stores and emits float values, while restricting input to safe bounds.
    """
    value_changed = Signal(float)

    def __init__(self, label: str, units: str, vmin: float, vmax: float, step: float) -> None:
        super().__init__()
        self._vmin = float(vmin)
        self._vmax = float(vmax)
        self._step = float(step)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        title = QLabel(label)
        title.setObjectName("SectionTitle")
        root.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(12)

        self.btn_minus = BigButton("−")
        self.btn_minus.setMinimumWidth(110)
        self.btn_plus = BigButton("+")
        self.btn_plus.setMinimumWidth(110)

        self.edit = QLineEdit()
        self.edit.setMinimumHeight(70)
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText("120")
        self.edit.setValidator(QDoubleValidator(self._vmin, self._vmax, 2, self.edit))

        units_lbl = QLabel(units)
        units_lbl.setMinimumWidth(90)
        units_lbl.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

        row.addWidget(self.btn_minus)
        row.addWidget(self.edit, 1)
        row.addWidget(units_lbl)
        row.addWidget(self.btn_plus)

        root.addLayout(row)

        self.btn_minus.clicked.connect(self._dec)
        self.btn_plus.clicked.connect(self._inc)
        self.edit.editingFinished.connect(self._emit_if_valid)

        # Emit initial value so the controller has a known starting state.
        self._emit_if_valid()

    def value(self) -> float:
        v = self._parse()
        return float(v if v is not None else self._vmin)

    def set_value(self, v: float) -> None:
        v = clamp(float(v), self._vmin, self._vmax)
        self.edit.setText(f"{v:g}")
        self.value_changed.emit(v)

    def set_enabled_adjustment(self, enabled: bool) -> None:
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


class TimeAdjust(QWidget):
    """
    Touch-friendly time control displayed as mm:ss with +/- step buttons.

    The line edit accepts mm:ss or a raw seconds integer. The widget clamps to [0, TIME_MAX_S].
    """
    value_changed = Signal(int)

    def __init__(self, label: str, step_s: int) -> None:
        super().__init__()
        self._step_s = int(step_s)

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        title = QLabel(label)
        title.setObjectName("SectionTitle")
        root.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(12)

        self.btn_minus = BigButton("−")
        self.btn_minus.setMinimumWidth(110)
        self.btn_plus = BigButton("+")
        self.btn_plus.setMinimumWidth(110)

        self.edit = QLineEdit()
        self.edit.setMinimumHeight(70)
        self.edit.setAlignment(Qt.AlignCenter)
        self.edit.setText("00:00")

        hint = QLabel("mm:ss")
        hint.setMinimumWidth(90)
        hint.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

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

    def set_enabled_adjustment(self, enabled: bool) -> None:
        self.btn_minus.setEnabled(enabled)
        self.btn_plus.setEnabled(enabled)
        self.edit.setEnabled(enabled)

    def _parse(self) -> Optional[int]:
        t = self.edit.text().strip()
        s = parse_mmss(t)
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
# Recipe Card Widgets (Home screen list)
# ----------------------------

class RecipeCard(QFrame):
    """
    Touch-friendly recipe card with Select and Edit actions.
    """
    select_clicked = Signal(str)  # recipe_id
    edit_clicked = Signal(str)    # recipe_id

    def __init__(self, recipe: Recipe) -> None:
        super().__init__()
        self.recipe = recipe
        self.setObjectName("RecipeCard")
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        self.lbl_name = QLabel(recipe.name)
        self.lbl_name.setObjectName("RecipeName")
        self.lbl_summary = QLabel(recipe_summary(recipe))
        self.lbl_summary.setObjectName("RecipeSummary")

        root.addWidget(self.lbl_name)
        root.addWidget(self.lbl_summary)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)

        self.btn_select = BigButton("Select")
        self.btn_edit = BigButton("Edit")

        btn_row.addWidget(self.btn_select, 1)
        btn_row.addWidget(self.btn_edit, 1)

        root.addLayout(btn_row)

        self.btn_select.clicked.connect(lambda: self.select_clicked.emit(self.recipe.id))
        self.btn_edit.clicked.connect(lambda: self.edit_clicked.emit(self.recipe.id))

    def set_selected(self, selected: bool) -> None:
        # The visual highlight is driven by a dynamic property used in the stylesheet.
        self.setProperty("selected", "true" if selected else "false")
        self.style().unpolish(self)
        self.style().polish(self)


# ----------------------------
# Recipe Builder Widgets
# ----------------------------

class StepRow(QFrame):
    """
    One step editor row (velocity + duration with +/- controls, remove, reorder).
    """
    remove_clicked = Signal(int)  # index
    move_up_clicked = Signal(int)
    move_down_clicked = Signal(int)
    changed = Signal()

    def __init__(self, index: int, step: Step) -> None:
        super().__init__()
        self.setObjectName("StepRow")
        self._index = index

        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)

        root = QGridLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setHorizontalSpacing(14)
        root.setVerticalSpacing(10)

        self.lbl_idx = QLabel(f"Step {index + 1}")
        self.lbl_idx.setObjectName("StepIndex")

        self.vel = VelocityAdjust("Velocity", "rpm", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM)
        self.vel.set_value(step.velocity_rpm)

        self.tim = TimeAdjust("Time", TIME_STEP_S)
        self.tim.set_seconds(step.duration_s)

        self.btn_remove = BigButton("Remove")
        self.btn_up = BigButton("Up")
        self.btn_down = BigButton("Down")

        self.btn_remove.setMinimumWidth(170)
        self.btn_up.setMinimumWidth(170)
        self.btn_down.setMinimumWidth(170)

        root.addWidget(self.lbl_idx, 0, 0, 1, 3)
        root.addWidget(self.vel, 1, 0, 1, 1)
        root.addWidget(self.tim, 1, 1, 1, 1)

        btn_col = QVBoxLayout()
        btn_col.setSpacing(12)
        btn_col.addWidget(self.btn_up)
        btn_col.addWidget(self.btn_down)
        btn_col.addWidget(self.btn_remove)
        btn_col.addStretch(1)

        root.addLayout(btn_col, 1, 2, 1, 1)

        self.btn_remove.clicked.connect(lambda: self.remove_clicked.emit(self._index))
        self.btn_up.clicked.connect(lambda: self.move_up_clicked.emit(self._index))
        self.btn_down.clicked.connect(lambda: self.move_down_clicked.emit(self._index))

        self.vel.value_changed.connect(lambda _v: self.changed.emit())
        self.tim.value_changed.connect(lambda _s: self.changed.emit())

    def set_index(self, idx: int) -> None:
        self._index = idx
        self.lbl_idx.setText(f"Step {idx + 1}")

    def get_step(self) -> Step:
        return Step(velocity_rpm=float(self.vel.value()), duration_s=int(self.tim.seconds()))


class RecipeBuilderScreen(QWidget):
    """
    Screen for creating/editing recipes.
    """
    back_clicked = Signal()
    save_clicked = Signal(Recipe)
    cancel_clicked = Signal()

    def __init__(self) -> None:
        super().__init__()
        self._editing_id: Optional[str] = None

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(16)

        # Top bar
        top = QHBoxLayout()
        top.setSpacing(14)

        self.btn_back = BigButton("Back")
        self.btn_back.setMinimumWidth(180)

        self.lbl_title = QLabel("Recipe Builder")
        self.lbl_title.setObjectName("ScreenTitle")

        top.addWidget(self.btn_back)
        top.addWidget(self.lbl_title, 1)
        root.addLayout(top)

        # Name entry
        name_row = QHBoxLayout()
        name_row.setSpacing(14)

        name_lbl = QLabel("Recipe Name")
        name_lbl.setMinimumWidth(180)
        name_lbl.setObjectName("FieldLabel")

        self.name_edit = QLineEdit()
        self.name_edit.setMinimumHeight(70)
        self.name_edit.setPlaceholderText("Enter recipe name")
        self.name_edit.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        name_row.addWidget(name_lbl)
        name_row.addWidget(self.name_edit, 1)
        root.addLayout(name_row)

        # Steps list (scrollable)
        self.steps_area = QScrollArea()
        self.steps_area.setWidgetResizable(True)
        self.steps_area.setFrameShape(QFrame.NoFrame)

        self.steps_container = QWidget()
        self.steps_layout = QVBoxLayout(self.steps_container)
        self.steps_layout.setContentsMargins(0, 0, 0, 0)
        self.steps_layout.setSpacing(12)
        self.steps_layout.addStretch(1)

        self.steps_area.setWidget(self.steps_container)
        root.addWidget(self.steps_area, 1)

        # Add step
        self.btn_add_step = BigButton("Add Step")
        root.addWidget(self.btn_add_step)

        # Bottom actions
        bottom = QHBoxLayout()
        bottom.setSpacing(14)

        self.btn_cancel = BigButton("Cancel")
        self.btn_save = BigButton("Save")

        bottom.addWidget(self.btn_cancel, 1)
        bottom.addWidget(self.btn_save, 1)
        root.addLayout(bottom)

        # Signals
        self.btn_back.clicked.connect(self.back_clicked.emit)
        self.btn_cancel.clicked.connect(self.cancel_clicked.emit)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_add_step.clicked.connect(self._on_add_step)

    def load_recipe(self, recipe: Optional[Recipe]) -> None:
        """
        Load an existing recipe for edit, or None for a new recipe.
        """
        self._clear_steps()

        if recipe is None:
            self._editing_id = None
            self.name_edit.setText("")
            self._add_step(Step(velocity_rpm=120.0, duration_s=60))
            self.lbl_title.setText("Recipe Builder (New)")
            return

        self._editing_id = recipe.id
        self.name_edit.setText(recipe.name)
        for s in recipe.steps:
            self._add_step(Step(velocity_rpm=float(s.velocity_rpm), duration_s=int(s.duration_s)))
        self.lbl_title.setText("Recipe Builder (Edit)")

    def _clear_steps(self) -> None:
        # Remove all StepRow widgets while preserving the stretch at the bottom.
        while self.steps_layout.count() > 1:
            item = self.steps_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()

    def _step_rows(self) -> List[StepRow]:
        rows: List[StepRow] = []
        for i in range(self.steps_layout.count() - 1):
            w = self.steps_layout.itemAt(i).widget()
            if isinstance(w, StepRow):
                rows.append(w)
        return rows

    def _add_step(self, step: Step) -> None:
        idx = len(self._step_rows())
        row = StepRow(idx, step)
        row.remove_clicked.connect(self._on_remove_step)
        row.move_up_clicked.connect(self._on_move_up)
        row.move_down_clicked.connect(self._on_move_down)

        # Insert above the stretch.
        self.steps_layout.insertWidget(self.steps_layout.count() - 1, row)

    @Slot()
    def _on_add_step(self) -> None:
        self._add_step(Step(velocity_rpm=120.0, duration_s=60))
        self._reindex()

    @Slot(int)
    def _on_remove_step(self, idx: int) -> None:
        rows = self._step_rows()
        if idx < 0 or idx >= len(rows):
            return
        rows[idx].deleteLater()
        self.steps_layout.takeAt(idx)
        self._reindex()

    @Slot(int)
    def _on_move_up(self, idx: int) -> None:
        if idx <= 0:
            return
        rows = self._step_rows()
        if idx >= len(rows):
            return
        w = rows[idx]
        self.steps_layout.removeWidget(w)
        self.steps_layout.insertWidget(idx - 1, w)
        self._reindex()

    @Slot(int)
    def _on_move_down(self, idx: int) -> None:
        rows = self._step_rows()
        if idx < 0 or idx >= len(rows) - 1:
            return
        w = rows[idx]
        self.steps_layout.removeWidget(w)
        self.steps_layout.insertWidget(idx + 1, w)
        self._reindex()

    def _reindex(self) -> None:
        for i, r in enumerate(self._step_rows()):
            r.set_index(i)

    @Slot()
    def _on_save(self) -> None:
        name = self.name_edit.text().strip()
        rows = self._step_rows()
        steps = [r.get_step() for r in rows]

        # Validation per requirements.
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
            if not (VEL_MIN_RPM <= s.velocity_rpm <= VEL_MAX_RPM):
                QMessageBox.warning(self, "Validation", f"Step {i}: velocity out of range.")
                return

        rid = self._editing_id or str(uuid.uuid4())
        self.save_clicked.emit(Recipe(id=rid, name=name, steps=steps))


# ----------------------------
# Home Screen
# ----------------------------

class HomeScreen(QWidget):
    """
    Main screen with status bar, controls, recipe list, and start/stop actions.
    """

    start_clicked = Signal()
    stop_clicked = Signal()
    open_builder_clicked = Signal()
    clear_selection_clicked = Signal()
    recipe_select_clicked = Signal(str)
    recipe_edit_clicked = Signal(str)

    def __init__(self) -> None:
        super().__init__()

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 16, 18, 16)
        root.setSpacing(14)

        # Status bar
        self.status_bar = QFrame()
        self.status_bar.setObjectName("StatusBar")
        sb = QHBoxLayout(self.status_bar)
        sb.setContentsMargins(16, 12, 16, 12)
        sb.setSpacing(18)

        self.lbl_state = QLabel("State: Idle")
        self.lbl_vel = QLabel("Velocity: 0 rpm")
        self.lbl_rem = QLabel("Remaining: 00:00")
        self.lbl_recipe = QLabel("Recipe: (none)")
        self.lbl_step = QLabel("")  # visible only during recipe runs

        for lbl in (self.lbl_state, self.lbl_vel, self.lbl_rem, self.lbl_recipe):
            lbl.setObjectName("StatusLabel")

        self.lbl_step.setObjectName("StatusStep")
        self.lbl_step.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        sb.addWidget(self.lbl_state)
        sb.addWidget(self.lbl_vel)
        sb.addWidget(self.lbl_rem)
        sb.addWidget(self.lbl_recipe, 1)
        sb.addWidget(self.lbl_step, 2)

        root.addWidget(self.status_bar)

        # Main controls + recipes area (split)
        mid = QHBoxLayout()
        mid.setSpacing(16)

        # Left controls column
        left = QVBoxLayout()
        left.setSpacing(16)

        self.vel_ctl = VelocityAdjust("Velocity", "rpm", VEL_MIN_RPM, VEL_MAX_RPM, VEL_STEP_RPM)
        self.tim_ctl = TimeAdjust("Timer", TIME_STEP_S)

        left.addWidget(self.vel_ctl)
        left.addWidget(self.tim_ctl)

        self.btn_builder = BigButton("Recipe Builder")
        self.btn_clear = BigButton("Clear Selection")

        left.addWidget(self.btn_builder)
        left.addWidget(self.btn_clear)

        left.addStretch(1)
        mid.addLayout(left, 1)

        # Recipe list area
        recipes_col = QVBoxLayout()
        recipes_col.setSpacing(10)

        title = QLabel("Recipes")
        title.setObjectName("SectionTitle")
        recipes_col.addWidget(title)

        self.recipes_area = QScrollArea()
        self.recipes_area.setWidgetResizable(True)
        self.recipes_area.setFrameShape(QFrame.NoFrame)

        self.recipes_container = QWidget()
        self.recipes_layout = QVBoxLayout(self.recipes_container)
        self.recipes_layout.setContentsMargins(0, 0, 0, 0)
        self.recipes_layout.setSpacing(12)
        self.recipes_layout.addStretch(1)

        self.recipes_area.setWidget(self.recipes_container)
        recipes_col.addWidget(self.recipes_area, 1)

        mid.addLayout(recipes_col, 2)
        root.addLayout(mid, 1)

        # Bottom action bar
        bottom = QHBoxLayout()
        bottom.setSpacing(16)

        self.btn_start = BigButton("Start")
        self.btn_stop = BigButton("Stop")

        bottom.addWidget(self.btn_start, 1)
        bottom.addWidget(self.btn_stop, 1)
        root.addLayout(bottom)

        # Signals
        self.btn_start.clicked.connect(self.start_clicked.emit)
        self.btn_stop.clicked.connect(self.stop_clicked.emit)
        self.btn_builder.clicked.connect(self.open_builder_clicked.emit)
        self.btn_clear.clicked.connect(self.clear_selection_clicked.emit)

    def set_recipe_cards(self, recipes: List[Recipe], selected_id: Optional[str]) -> None:
        """
        Rebuild the recipe list with large card widgets.

        The list is rebuilt (rather than incrementally diffed) to keep behavior simple and robust.
        """
        # Remove all cards while preserving the stretch at the bottom.
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
        Enforce UI rules:
          - Start disabled while running
          - Stop enabled while running (may be disabled when idle)
        """
        self.btn_start.setEnabled(not running)
        self.btn_stop.setEnabled(running)

        # While running, lock the controls to avoid changing the active run parameters mid-run.
        self.vel_ctl.set_enabled_adjustment(not running)
        self.tim_ctl.set_enabled_adjustment(not running)

        # Recipe selection should not change while running.
        self.btn_builder.setEnabled(not running)
        self.btn_clear.setEnabled(not running)
        self.recipes_area.setEnabled(not running)

    def set_timer_enabled(self, enabled: bool) -> None:
        """
        Disable the main timer control when a recipe is selected (per requirements).
        """
        self.tim_ctl.set_enabled_adjustment(enabled)

    def set_status(
        self,
        state_text: str,
        active_rpm: float,
        remaining_s: int,
        selected_recipe_name: str,
        step_text: str,
    ) -> None:
        self.lbl_state.setText(f"State: {state_text}")
        self.lbl_vel.setText(f"Velocity: {active_rpm:g} rpm")
        self.lbl_rem.setText(f"Remaining: {format_mmss(remaining_s)}")
        self.lbl_recipe.setText(f"Recipe: {selected_recipe_name}")
        self.lbl_step.setText(step_text)


# ----------------------------
# Controller (connects UI, engine, storage, worker)
# ----------------------------

class AppController(QObject):
    """
    Central controller with a simple MVC-ish separation:
      - Views: HomeScreen, RecipeBuilderScreen
      - Model/Storage: RecipeStore, Recipe/Step dataclasses
      - Logic: RunEngine, OdriveWorker

    The controller is responsible for:
      - validating start conditions
      - maintaining selected recipe state
      - updating UI based on engine signals
      - stopping motor safely on app exit
    """

    def __init__(self, stack: QStackedWidget, home: HomeScreen, builder: RecipeBuilderScreen) -> None:
        super().__init__()
        self.stack = stack
        self.home = home
        self.builder = builder

        self.store = RecipeStore()
        self.store.load()

        self.selected_recipe_id: Optional[str] = None
        self.connected: bool = False
        self.backend_state: str = "Disconnected"

        # Create motor backend.
        backend: OdriveInterface = SimulatedOdrive() if BACKEND == "sim" else RealOdrive()

        # Worker thread for motor commands.
        self.worker_thread = QThread()
        self.worker = OdriveWorker(backend)
        self.worker.moveToThread(self.worker_thread)

        # Engine (timer-driven state machine).
        self.engine = RunEngine()

        # Wire engine -> worker (queued across threads).
        self.engine.request_set_rpm.connect(self.worker.set_velocity_rpm)
        self.engine.request_stop.connect(self.worker.stop)

        # Wire worker -> controller.
        self.worker.connected_changed.connect(self._on_connected_changed)
        self.worker.backend_state_changed.connect(self._on_backend_state)
        self.worker.error.connect(self._on_worker_error)

        # Wire engine -> controller/UI.
        self.engine.status_changed.connect(self._on_engine_status)
        self.engine.active_rpm_changed.connect(self._on_engine_rpm)
        self.engine.time_remaining_changed.connect(self._on_engine_remaining)
        self.engine.total_remaining_changed.connect(self._on_engine_total_remaining)
        self.engine.recipe_step_changed.connect(self._on_engine_step)

        # Wire UI -> controller.
        self.home.start_clicked.connect(self.on_start)
        self.home.stop_clicked.connect(self.on_stop)
        self.home.open_builder_clicked.connect(self.on_open_builder_new)
        self.home.clear_selection_clicked.connect(self.on_clear_selection)
        self.home.recipe_select_clicked.connect(self.on_select_recipe)
        self.home.recipe_edit_clicked.connect(self.on_edit_recipe)

        self.builder.back_clicked.connect(self.on_back_to_home)
        self.builder.cancel_clicked.connect(self.on_back_to_home)
        self.builder.save_clicked.connect(self.on_builder_save)

        # Cached display fields.
        self._ui_state_text = "Idle"
        self._ui_active_rpm = 0.0
        self._ui_remaining_s = 0
        self._ui_step_text = ""

        # Start worker thread and connect backend.
        self.worker_thread.start()
        QMetaObject.invokeMethod(self.worker, "connect_backend", Qt.QueuedConnection)

        # Initial UI build.
        self._refresh_recipe_list()
        self._refresh_home_status()

    def shutdown(self) -> None:
        """
        Safe shutdown:
          - Stop engine (and therefore motor) if running.
          - Ensure the worker executes stop() before the thread exits.
        """
        try:
            if self.engine.is_running():
                self.engine.stop()
                # BlockingQueuedConnection ensures stop() runs in the worker thread before proceeding.
                QMetaObject.invokeMethod(self.worker, "stop", Qt.BlockingQueuedConnection)
        except Exception:
            pass

        self.worker_thread.quit()
        self.worker_thread.wait(1500)

    def _recipes(self) -> List[Recipe]:
        return self.store.recipes()

    def _get_selected_recipe(self) -> Optional[Recipe]:
        if not self.selected_recipe_id:
            return None
        for r in self._recipes():
            if r.id == self.selected_recipe_id:
                return r
        return None

    def _refresh_recipe_list(self) -> None:
        self.home.set_recipe_cards(self._recipes(), self.selected_recipe_id)
        selected = self._get_selected_recipe()
        # Disable timer control if a recipe is selected (per execution rules).
        self.home.set_timer_enabled(enabled=(selected is None))

    def _refresh_home_status(self) -> None:
        selected = self._get_selected_recipe()
        selected_name = selected.name if selected else "(none)"

        self.home.set_status(
            state_text=self._ui_state_text,
            active_rpm=self._ui_active_rpm,
            remaining_s=self._ui_remaining_s,
            selected_recipe_name=selected_name,
            step_text=self._ui_step_text,
        )
        self.home.set_running_ui(self.engine.is_running())

    @Slot(bool, str)
    def _on_connected_changed(self, connected: bool, backend_state: str) -> None:
        self.connected = connected
        self.backend_state = backend_state
        if BACKEND == "real" and not connected:
            QMessageBox.critical(
                self.home,
                "ODrive Connection",
                "ODrive is not connected.\n\n"
                "Switch to simulation mode or implement RealOdrive.connect() for your hardware.",
            )

    @Slot(str)
    def _on_backend_state(self, state: str) -> None:
        self.backend_state = state

    @Slot(str)
    def _on_worker_error(self, msg: str) -> None:
        # On worker errors during a run, stop the motor and return to Idle.
        if self.engine.is_running():
            self.engine.stop()
        QMessageBox.critical(self.home, "Motor Error", msg)
        self._refresh_home_status()

    @Slot(str)
    def _on_engine_status(self, status: str) -> None:
        self._ui_state_text = status
        self._refresh_home_status()

    @Slot(float)
    def _on_engine_rpm(self, rpm: float) -> None:
        self._ui_active_rpm = float(rpm)
        self._refresh_home_status()

    @Slot(int)
    def _on_engine_remaining(self, seconds: int) -> None:
        # This is step remaining for recipe mode, or total remaining for single mode.
        self._ui_remaining_s = int(seconds)
        self._refresh_home_status()

    @Slot(int)
    def _on_engine_total_remaining(self, _seconds: int) -> None:
        # The home screen displays a single remaining time value. For a richer UI, this can be split.
        pass

    @Slot(str)
    def _on_engine_step(self, step_text: str) -> None:
        self._ui_step_text = step_text
        self._refresh_home_status()

    @Slot()
    def on_start(self) -> None:
        if self.engine.is_running():
            return

        if BACKEND == "real" and not self.connected:
            QMessageBox.critical(self.home, "ODrive Not Connected", "Cannot start: ODrive is not connected.")
            return

        rpm = clamp(self.home.vel_ctl.value(), VEL_MIN_RPM, VEL_MAX_RPM)
        selected = self._get_selected_recipe()

        if selected is None:
            duration = self.home.tim_ctl.seconds()
            if duration <= 0:
                QMessageBox.information(self.home, "Start", "Set a timer or select a recipe.")
                return
            self.engine.start_single(rpm, duration)
            return

        # Recipe selected: ignore the main timer and run the recipe.
        if len(selected.steps) < 1:
            QMessageBox.warning(self.home, "Start", "Selected recipe has no steps.")
            return
        self.engine.start_recipe(selected)

    @Slot()
    def on_stop(self) -> None:
        self.engine.stop()

    @Slot()
    def on_open_builder_new(self) -> None:
        if self.engine.is_running():
            return
        self.builder.load_recipe(None)
        self.stack.setCurrentWidget(self.builder)

    @Slot()
    def on_back_to_home(self) -> None:
        self.stack.setCurrentWidget(self.home)
        self._refresh_recipe_list()
        self._refresh_home_status()

    @Slot()
    def on_clear_selection(self) -> None:
        if self.engine.is_running():
            return
        self.selected_recipe_id = None
        self._refresh_recipe_list()
        self._refresh_home_status()

    @Slot(str)
    def on_select_recipe(self, recipe_id: str) -> None:
        if self.engine.is_running():
            return
        # Toggle selection: selecting the same recipe again unselects it.
        self.selected_recipe_id = None if self.selected_recipe_id == recipe_id else recipe_id
        self._refresh_recipe_list()
        self._refresh_home_status()

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
        # Keep the selection stable if the edited recipe was selected.
        if self.selected_recipe_id == recipe.id:
            self.selected_recipe_id = recipe.id
        self.stack.setCurrentWidget(self.home)
        self._refresh_recipe_list()
        self._refresh_home_status()


# ----------------------------
# Main Window Shell
# ----------------------------

class MainWindow(QWidget):
    """
    Simple fixed-size main window that hosts a QStackedWidget (Home / Builder).

    closeEvent enforces safe stop behavior when the application is closed mid-run.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Reactor Agitator HMI")
        self.setFixedSize(WINDOW_W, WINDOW_H)

        self.stack = QStackedWidget()
        self.home = HomeScreen()
        self.builder = RecipeBuilderScreen()
        self.stack.addWidget(self.home)
        self.stack.addWidget(self.builder)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.stack)

        self.controller = AppController(self.stack, self.home, self.builder)

    def closeEvent(self, event) -> None:
        self.controller.shutdown()
        event.accept()


# ----------------------------
# Styling
# ----------------------------

def apply_touch_style(app: QApplication) -> None:
    """
    Apply a high-contrast, large-font stylesheet.

    The stylesheet avoids small hit targets by enforcing minimum heights in code and
    uses padding and border radius for clear separation.
    """
    base_font = QFont()
    base_font.setPointSize(12)  # Combined with stylesheet pixel sizes for consistent touch readability.
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

        QLabel#FieldLabel {
            font-size: 22px;
            font-weight: 600;
        }

        QFrame#StatusBar {
            background: #161B22;
            border: 2px solid #2A313A;
            border-radius: 14px;
        }

        QLabel#StatusLabel {
            font-size: 20px;
            font-weight: 600;
        }

        QLabel#StatusStep {
            font-size: 18px;
            color: #D0D7DE;
        }

        QLineEdit {
            background: #0B0F14;
            border: 2px solid #2A313A;
            border-radius: 14px;
            padding: 10px 14px;
            font-size: 26px;
        }

        QPushButton {
            background: #2B3440;
            border: 2px solid #3A4656;
            border-radius: 16px;
            padding: 12px 18px;
            font-size: 26px;
            font-weight: 650;
        }

        QPushButton:hover {
            background: #354152;
        }

        QPushButton:pressed {
            background: #202733;
        }

        QPushButton:disabled {
            background: #1C222B;
            border: 2px solid #2A313A;
            color: #7A8794;
        }

        QPushButton[text="Start"] {
            background: #1E5F3C;
            border: 2px solid #2D7B52;
        }

        QPushButton[text="Start"]:hover {
            background: #21704A;
        }

        QPushButton[text="Stop"] {
            background: #6B1E1E;
            border: 2px solid #8B2A2A;
        }

        QPushButton[text="Stop"]:hover {
            background: #7A2424;
        }

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
            font-size: 22px;
            font-weight: 700;
        }

        QScrollArea {
            border: none;
        }
        """
    )


# ----------------------------
# Entry point
# ----------------------------

def main() -> int:
    app = QApplication(sys.argv)
    apply_touch_style(app)

    w = MainWindow()
    w.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
