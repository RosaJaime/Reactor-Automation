"""Shared ODrive configuration defaults for scripts and the HMI.

This module keeps the project defaults in one place so they can be:
- applied from helper scripts (e.g. odrivetool / standalone setup scripts)
- imported by the HMI to populate the ODrive Configuration page defaults
"""

from __future__ import annotations

import math
from typing import Any, Dict


# Settings represented as ODrive attribute paths used by the HMI config page.
HMI_DEFAULT_CONFIG_VALUES: Dict[str, Any] = {
    "config.dc_bus_overvoltage_trip_level": 25.0,
    "config.dc_bus_undervoltage_trip_level": 10.5,
    "config.dc_max_positive_current": 25.0,
    "config.dc_max_negative_current": -2.0,
    "config.enable_brake_resistor": True,
    "config.brake_resistance": 2.0,
    "axis0.config.startup_motor_calibration": False,
    "axis0.config.startup_encoder_offset_calibration": False,
    "axis0.config.startup_encoder_index_search": False,
    "axis0.config.startup_closed_loop_control": False,
    "axis0.config.motor.pole_pairs": 20,
    "axis0.config.motor.torque_constant": 0.0827,
    "axis0.config.motor.current_soft_max": 20.0,
    "axis0.config.motor.current_hard_max": 36.0,
    "axis0.config.enable_watchdog": False,
    "axis0.config.watchdog_timeout": 1.0,
}


# ODrive shop motor presets derived from:
# https://docs.odriverobotics.com/v/latest/hardware/odrive-motors.html
# These presets only populate the fields that exist on the HMI config page.
ODRIVE_MOTOR_PRESETS: Dict[str, Dict[str, Any]] = {
    "M8325s 100KV": {
        "docs_speed_constant_rpm_per_v": 100.0,
        "docs_torque_constant_nm_per_a": 0.083,
        "docs_pole_pairs": 20,
        "docs_continuous_current_a_free_air": 40.0,
        "docs_continuous_current_a_forced_air": 60.0,
        "docs_peak_current_a": 80.0,
        "hmi_values": {
            "axis0.config.motor.pole_pairs": 20,
            "axis0.config.motor.torque_constant": 0.083,
            "axis0.config.motor.current_soft_max": 40.0,
            "axis0.config.motor.current_hard_max": 80.0,
        },
    },
    "D6374 150KV": {
        "docs_speed_constant_rpm_per_v": 150.0,
        "docs_torque_constant_nm_per_a": 0.055,
        "docs_pole_pairs": 7,
        "docs_continuous_current_a_free_air": 50.0,
        "docs_continuous_current_a_forced_air": 70.0,
        "docs_peak_current_a": 90.0,
        "hmi_values": {
            "axis0.config.motor.pole_pairs": 7,
            "axis0.config.motor.torque_constant": 0.055,
            "axis0.config.motor.current_soft_max": 50.0,
            "axis0.config.motor.current_hard_max": 90.0,
        },
    },
    "D5065 270KV": {
        "docs_speed_constant_rpm_per_v": 270.0,
        "docs_torque_constant_nm_per_a": 0.031,
        "docs_pole_pairs": 7,
        "docs_continuous_current_a_free_air": 45.0,
        "docs_continuous_current_a_forced_air": 65.0,
        "docs_peak_current_a": 85.0,
        "hmi_values": {
            "axis0.config.motor.pole_pairs": 7,
            "axis0.config.motor.torque_constant": 0.031,
            "axis0.config.motor.current_soft_max": 45.0,
            "axis0.config.motor.current_hard_max": 85.0,
        },
    },
    "D5312s 330KV": {
        "docs_speed_constant_rpm_per_v": 330.0,
        "docs_torque_constant_nm_per_a": 0.025,
        "docs_pole_pairs": 7,
        "docs_continuous_current_a_free_air": 30.0,
        "docs_continuous_current_a_forced_air": 50.0,
        "docs_peak_current_a": 60.0,
        "hmi_values": {
            "axis0.config.motor.pole_pairs": 7,
            "axis0.config.motor.torque_constant": 0.025,
            "axis0.config.motor.current_soft_max": 30.0,
            "axis0.config.motor.current_hard_max": 60.0,
        },
    },
    "NEMA 34 Servomotor (16384 CPR Abs)": {
        "docs_speed_constant_rpm_per_v": 87.0,
        "docs_torque_constant_nm_per_a": 0.095,
        "docs_pole_pairs": 4,
        "docs_continuous_current_a_free_air": 20.0,
        "docs_peak_current_a": 40.0,
        "hmi_values": {
            "axis0.config.motor.pole_pairs": 4,
            "axis0.config.motor.torque_constant": 0.095,
            "axis0.config.motor.current_soft_max": 20.0,
            "axis0.config.motor.current_hard_max": 40.0,
        },
    },
    "ODrive Botwheel": {
        "docs_speed_constant_rpm_per_v": 8.7,
        "docs_torque_constant_nm_per_a": 0.951,
        "docs_pole_pairs": 15,
        "docs_continuous_current_a_free_air": 5.0,
        "docs_peak_current_a": 15.0,
        "hmi_values": {
            "axis0.config.motor.pole_pairs": 15,
            "axis0.config.motor.torque_constant": 0.951,
            "axis0.config.motor.current_soft_max": 5.0,
            "axis0.config.motor.current_hard_max": 15.0,
            "inc_encoder0.config.cpr": 3200,
        },
    },
}


# Script-style defaults represented as literal assignments (for reuse by setup scripts).
SCRIPT_DEFAULT_ASSIGNMENTS = [
    ("config.dc_bus_overvoltage_trip_level", 25.0),
    ("config.dc_bus_undervoltage_trip_level", 10.5),
    ("config.dc_max_positive_current", math.inf),
    ("config.dc_max_negative_current", -math.inf),
    ("config.brake_resistor0.enable", True),
    ("config.brake_resistor0.resistance", 2.0),
    ("axis0.config.motor.current_soft_max", 20.0),
    ("axis0.config.motor.current_hard_max", 36.0),
    ("axis0.config.motor.calibration_current", 10.0),
    ("axis0.config.motor.resistance_calib_max_voltage", 2.0),
    ("axis0.config.calibration_lockin.current", 10.0),
    ("axis0.config.enable_watchdog", False),
    ("axis0.controller.config.vel_limit", 30.0),
    ("axis0.controller.config.vel_limit_tolerance", 1.0666666666666667),
    ("axis0.controller.config.vel_ramp_rate", 10.0),
]


def _set_attr_path(root: Any, key: str, value: Any) -> None:
    obj = root
    parts = key.split(".")
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)


def apply_script_defaults(odrv: Any) -> None:
    """Apply the shared script defaults to an ODrive object."""
    for key, value in SCRIPT_DEFAULT_ASSIGNMENTS:
        _set_attr_path(odrv, key, value)
