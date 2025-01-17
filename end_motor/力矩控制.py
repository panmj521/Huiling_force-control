import odrive
import time

odrv0 = odrive.find_any()

odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.TORQUE_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.PASSTHROUGH

# odrv0.axis0.controller.config.torque_ramp_rate = 0.1

odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL

#力矩常数
odrv0.axis0.motor.config.torque_constant = 8.23/12.3

#力矩模式限速
odrv0.axis0.controller.config.enable_torque_mode_vel_limit = 1
odrv0.axis0.controller.config.vel_limit = 30 

odrv0.axis0.controller.input_torque = 1

# 打印 ODrive 设备中的所有错误信息
odrive.utils.dump_errors(odrv0)

time.sleep(5)

odrv0.axis0.controller.input_torque = 0