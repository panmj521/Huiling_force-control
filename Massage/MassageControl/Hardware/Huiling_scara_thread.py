# sub_thread.py
import time
import threading
from Huiling.Hardware.Huiling_Scara import Huilin  
from Huiling_message import *
import queue

# 子线程类
class Huilingscara:

    def power_off(self):
        self.robot = Huilin.global_robot()
        self.robot.close_server()
        print("机器人电源关闭")

    def check_arm_status(self, interval,status_queue,stop_event):
        while not stop_event.is_set():  # 检查事件是否被设置
            # 获取各项状态
            joint1_state_code = self.robot.get_joint_state(1)
            joint2_state_code = self.robot.get_joint_state(2)
            joint3_state_code = self.robot.get_joint_state(3)
            joint4_state_code = self.robot.get_joint_state(4)
            collision_state = self.robot.is_collision()
            connect_state = self.robot.is_connect()
            emergency_state = self.robot.get_hard_emergency_stop_state()

            has_abnormal = False

            # 检查各个状态是否正常
            if joint1_state_code != 1:
                err_msg = joint_error_codes.get(joint1_state_code, "未知关节错误码")
                print(f"关节 1 异常: {err_msg}")
                has_abnormal = True

            if joint2_state_code != 1:
                err_msg = joint_error_codes.get(joint2_state_code, "未知关节错误码")
                print(f"关节 2 异常: {err_msg}")
                has_abnormal = True

            if joint3_state_code != 1:
                err_msg = joint_error_codes.get(joint3_state_code, "未知关节错误码")
                print(f"关节 3 异常: {err_msg}")
                has_abnormal = True

            if joint4_state_code != 1:
                err_msg = joint_error_codes.get(joint4_state_code, "未知关节错误码")
                print(f"关节 4 异常: {err_msg}")
                has_abnormal = True

            if collision_state == True:
                err_msg = collision_error.get(collision_state, "未知碰撞状态")
                print(f"碰撞异常: {err_msg}")
                has_abnormal = True

            if connect_state == False:
                err_msg = connect_error.get(connect_state, "未知连接状态")
                print(f"连接异常: {err_msg}")
                has_abnormal = True

            if emergency_state != 0:
                err_msg = emergency_stop_error_codes.get(emergency_state, "未知急停错误码")
                print(f"急停异常: {err_msg}")
                has_abnormal = True

            if has_abnormal:
                self.power_off()
                status_queue.put(False)
                return False 
            
            time.sleep(interval)

            # 如果没有异常，通知主线程状态正常
            status_queue.put(True)
        
    # 允许其他文件导入该模块
    def start_monitoring(interval,status_queue):
        stop_event = threading.Event()  # 创建 Event 对象来控制子线程退出
        huilingscara = Huilingscara()
        
        # 启动子线程进行检测
        monitor_thread = threading.Thread(target=huilingscara.check_arm_status, args=(interval, status_queue, stop_event), daemon=True)
        monitor_thread.start()
        return monitor_thread, stop_event
