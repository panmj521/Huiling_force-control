import odrive
import time
# import fibre.libfibre


# 1. 寻找并连接到任意可用的 ODrive 设备
odrv0 = odrive.find_any()

# 2. 打印当前 ODrive 上所有的错误信息，用于排查故障
odrive.utils.dump_errors(odrv0)

# 3. 清除当前已经存在的所有错误状态
odrv0.clear_errors()

# 4. 让电机执行 “电机校准(Motor Calibration)” 流程
odrv0.axis0.requested_state = odrive.utils.AxisState.MOTOR_CALIBRATION
time.sleep(5)  # 等待5秒，给电机一定时间完成校准

# 5. 等待校准完成：ODrive 设备的状态变为 “IDLE(空闲)” 才说明校准结束
#   注：ODrive 中，AxisState.IDLE 的值通常为 1
while odrv0.axis0.current_state != 1:
    time.sleep(0.5)  # 0.5 秒检查一次状态

# 6. 校准完成后再次查看是否存在错误
odrive.utils.dump_errors(odrv0)

# 7. 让电机执行 “编码器偏移校准(Encoder Offset Calibration)”
odrv0.axis0.requested_state = odrive.utils.AxisState.ENCODER_OFFSET_CALIBRATION
time.sleep(6)  # 给编码器校准预留时间，一般需要几秒到十几秒不等

# 8. 同样等待校准完成，直到状态回到 “IDLE”
while odrv0.axis0.current_state != 1:
    time.sleep(0.5)

# 9. 再次查看是否存在错误
odrive.utils.dump_errors(odrv0)

# 10. 将电机和编码器的 pre_calibrated 标记为 True，
#     表示以后上电无需再次做校准流程 (若硬件环境无较大变化)
odrv0.axis0.motor.config.pre_calibrated = 1
odrv0.axis0.encoder.config.pre_calibrated = 1

odrv0.save_configuration()

# 11. 将以上配置保存到 ODrive 的闪存中,跳过了一个断连报错
# try:
#     odrv0.save_configuration()
# except fibre.libfibre.ObjectLostError:
#     pass # Saving configuration makes the device reboot