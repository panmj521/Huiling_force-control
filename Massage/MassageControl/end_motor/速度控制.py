import odrive
import time

odrv0 = odrive.find_any()
# 设置轴0（axis0）的控制模式为速度控制模式 (Velocity Control)
odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.VELOCITY_CONTROL

# 设置 input_mode 为 VEL_RAMP，表示我们给电机输入的目标速度会经过一个“速度渐变（Ramping）”过程，避免瞬间给电机一个高速度指令而产生过大的电流或冲击
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.VEL_RAMP

# 表示每秒增加或减少的速度值（数值大小具体含义可能依赖你的 ODrive 配置，比如编码器 PPR、轴配置等）
odrv0.axis0.controller.config.vel_ramp_rate = 3

# odrv0.axis0.motor.config.current_lim = 10

# 在闭环控制下，ODrive 会根据设定的控制模式（这里是 VELOCITY_CONTROL）和输入指令（input_vel）进行实际的电机驱动。如果没有先进行校准（MOTOR_CALIBRATION 和 ENCODER_OFFSET_CALIBRATION）或者 pre_calibrated，没有正确的闭环控制，就会报错或者无法正常控制。一般在使用前需确保电机和编码器都已校准
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL

# 设置轴 0 的目标速度（input_vel）为 15
odrv0.axis0.controller.input_vel=15

# 打印 ODrive 设备中的所有错误信息
odrive.utils.dump_errors(odrv0)

time.sleep(5)

# 将目标速度设为 0，电机会在 vel_ramp_rate 设定的斜率下减速到 0。此时电机会停止或进入零速维持
odrv0.axis0.controller.input_vel=0