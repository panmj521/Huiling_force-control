import odrive
import time
odrv0 = odrive.find_any()
odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.TRAP_TRAJ
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
#减速比 8：1 输入8旋转360°，负数反转
#反
odrv0.axis0.controller.input_pos = -0.5
# time.sleep(7)
# odrv0.axis0.controller.input_pos = -0.5

#正
# odrv0.axis0.controller.input_pos = -0.5

