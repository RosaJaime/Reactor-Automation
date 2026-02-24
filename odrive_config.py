import math
import odrive
from odrive.enums import MotorType, ControlMode, InputMode, Protocol, EncoderId
from odrive_configuration import apply_script_defaults

odrv = odrv0
odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
odrv.axis0.config.motor.pole_pairs = 20
odrv.axis0.config.motor.torque_constant = 0.0827
odrv.axis0.config.motor.current_soft_max = 20
odrv.axis0.config.motor.current_hard_max = 36
odrv.axis0.config.motor.calibration_current = 10
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 10
odrv.axis0.motor.motor_thermistor.config.enabled = False
odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv.axis0.controller.config.vel_limit = 30
odrv.axis0.controller.config.vel_limit_tolerance = 1.0666666666666667
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.trap_traj.config.accel_limit = 10
odrv.axis0.controller.config.vel_ramp_rate = 10
odrv.can.config.protocol = Protocol.NONE
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
odrv.config.enable_uart_a = False
apply_script_defaults(odrv)
