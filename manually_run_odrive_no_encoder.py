# Open-loop spin sequence for ODrive S1 using LOCKIN_SPIN (no motor/encoder calibration).
# NOTE: This firmware requires re-entering LOCKIN_SPIN after changing gl.vel.
#
# Sequence:
#   1) vel = 3  for 10 seconds
#   2) vel = 10 for 5 seconds
#   3) vel = 1  for 1 second
# Then go IDLE.
#
# Paste this whole block into odrivetool.

import time
from odrive.enums import *

# ----------------------------
# Helper: apply a lock-in velocity for a fixed duration.
# This firmware only latches the new gl.vel when LOCKIN_SPIN is requested again.
# ----------------------------
def run_lockin_segment(vel_turns_per_s: float, duration_s: float) -> None:
    # Set the target open-loop velocity for the lock-in commutation.
    gl.vel = float(vel_turns_per_s)

    # Re-enter the lock-in state so the firmware applies the updated velocity.
    odrv0.axis0.requested_state = AxisState.LOCKIN_SPIN

    # Hold this velocity for the requested duration.
    time.sleep(float(duration_s))

# ----------------------------
# Safety / baseline state
# ----------------------------
odrv0.axis0.requested_state = AxisState.IDLE

# ----------------------------
# Disable automatic startup procedures that would attempt calibration or closed-loop.
# ----------------------------
odrv0.axis0.config.startup_motor_calibration = False
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_encoder_index_search = False
odrv0.axis0.config.startup_closed_loop_control = False

# ----------------------------
# Conservative current limits (protects motor + driver during open-loop testing).
# ----------------------------
odrv0.axis0.config.motor.current_soft_max = 5.0   # A: normal operating ceiling
odrv0.axis0.config.motor.current_hard_max = 8.0   # A: hard clamp for fault protection

# ----------------------------
# Configure open-loop lock-in profile.
# ----------------------------
gl = odrv0.axis0.config.general_lockin
gl.current = 2.0               # A: lock-in drive current; increase slowly if it won't start
gl.ramp_time = 1.0             # s: time to ramp up the lock-in motion
gl.ramp_distance = 1.0         # turns: distance covered during ramp
gl.accel = 10.0                # turns/s^2: acceleration limit during lock-in
gl.finish_on_vel = False       # do not auto-finish when velocity reached
gl.finish_on_distance = False  # do not auto-finish when distance/duration reached

# ----------------------------
# Run velocity schedule (re-request LOCKIN_SPIN each time vel changes).
# ----------------------------
run_lockin_segment(vel_turns_per_s=3.0,  duration_s=10.0)
run_lockin_segment(vel_turns_per_s=10.0, duration_s=5.0)
run_lockin_segment(vel_turns_per_s=1.0,  duration_s=1.0)

# ----------------------------
# Stop (go idle)
# ----------------------------
odrv0.axis0.requested_state = AxisState.IDLE

# ----------------------------
# Optional: report final status
# ----------------------------
print("final state:", odrv0.axis0.current_state)
print("armed:", odrv0.axis0.is_armed)
print("errors:", odrv0.axis0.active_errors)
