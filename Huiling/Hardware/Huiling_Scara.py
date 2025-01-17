import sys
import os
# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取项目根目录的绝对路径
project_root = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(project_root)

"""
机械臂上电与断开

步骤：
第一步 连接机器服务器net_port_initial
第二步 机器初始化initial
第三部 机器人断开close_server

"""
# from HitbotInterface import HitbotInterface
from tools.HitbotInterface import HitbotInterface
# sys.path.append(os.path.join(project_root, 'tools'))s
from tools.log import CustomLogger
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve
import time
# import atexit
import signal
from Hardware.Huiling_message import *
import requests
import numpy as np
import queue
import threading
# import math
import pysoem
import odrive

class Huilin():
    def __init__(self,arm_ip = 114):
        # 创建日志目录
        self.logger = CustomLogger('huilin',True)
        self.robot = HitbotInterface(arm_ip)
        self.last_joint = np.zeros(3)
        # 服务器启动
        ret = self.robot.net_port_initial()
        if ret == 1:
            print("Robot client connected successfully!")
            #屏蔽关节3
            self.robot.check_joint(3,False)
            #解除示教状态
            ret = self.robot.set_drag_teach(False)
            #解除急停状态
            if self.robot.get_hard_emergency_stop_state() == 1:
                net = self.clear_hard_emergency_stop()
                time.sleep(0.5)
                if net == 1:
                    print("解除急停状态成功")
                else:
                    self.logger.log_error(f"解除急停状态失败，错误码:{net}")
                    sys.exit(0)
            robot_connect_state, robot_collision_state = self.getModeType()
            if robot_connect_state == 1 and robot_collision_state == 0:
                state = self.start_up()
                if state == True:
                    self.logger.log_info(f"初始化慧灵机械臂{arm_ip}")
                    #调节碰撞检测的灵敏度    
                    self.robot.set_robot_joint_torque_value(1,4000)
                    self.robot.set_robot_joint_torque_value(2,4000)    
                    self.robot.set_robot_joint_torque_value(4,4000)
                    #机械臂检测线程
                    self.monitor_arm_status()  
                    """
                    初始化Z轴电机
                    """
                    master = pysoem.Master()
                    # 设置要使用的网卡名称，比如 'eth0'
                    master.open(r'enp3s0')
                    # 扫描从站，识别设备数量
                    if master.config_init() > 0:
                        print(f"Found {len(master.slaves)} slave(s).")
                    else:
                        print("No slaves found!")
                        return
                    # 映射 PDO
                    master.config_map()
                    # 配置 DC（分布式时钟），如果伺服需要同步时钟，可以在这里配置
                    master.config_dc()
                    self.Z_motor = master.slaves[0]
                    #故障恢复
                    self.Z_motor.sdo_write(0x230A,0x00,(0x01).to_bytes(2, 'little'))
                    #初始速度为0
                    self.Z_motor.sdo_write(0x2600, 0x00, int(0).to_bytes(4, 'little', signed=True))                    
                    # [0]位置模式 [1]速度模式 [2]力矩模式 [3]电压模式 [4]电流模式
                    self.Z_motor.sdo_write(0x2101, 0x00, (0x01).to_bytes(2, 'little'))
                    #电机使能1开启0关闭
                    self.Z_motor.sdo_write(0x2100, 0x00, (0x00).to_bytes(2, 'little'))
                    # """
                    # 初始化末端电机
                    # """ 
                    # self.odrv0 = odrive.find_any()
                    # self.odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
                    # self.odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
                    # self.odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
                else:
                    self.logger.log_error("机械臂初始化失败,存在某个关节失效")
                    self.power_off()
                    sys.exit(0)
            if robot_collision_state == 1:
                self.logger.log_error("机器人发生碰撞，无法初始化")
                self.power_off()
                sys.exit(0)
        else:
            print("fali:40000端口号被占用")
            return -1
        #退出任务,程序正常结束执行清理任务
        #atexit.register(self.exit_task)
        #初始化以及关机位置，待更改
        #R1 R2 Z R4
        self.init_pos = [30,240,108]
        #Rot_angle
        self.init_pos_Rotagl = [30,60,30]
        # self.off_pos = [0,180,108]
        #连杆长度
        self.L1 = 325
        self.L2 = 275
        self.L3 = 248
        #关节1、2的转角范围°
        self.theta1_range = (-95.9152, 96.1697)
        self.theta2_range=(9.18, 351.57)
        self.is_exit = False
        self.move_joint(self.init_pos,0)
        self.logger.log_info("已到达机械臂运动起始位置")     


    def monitor_arm_status(self):
        """启动状态监控线程"""

        if hasattr(self, 'monitor_thread') and self.monitor_thread.is_alive():
            self.logger.log_warning("监控线程已在运行")
            return
        self.status_queue = queue.Queue(maxsize=100)  
        self.stop_event = threading.Event()
        # 创建启动监控线程
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name="Arm_Monitor_Thread"
        )
        self.monitor_thread.start()
        self.logger.log_info("监控线程已启动")

    def _monitor_loop(self):
        """监控循环"""
        try:
            while not self.stop_event.is_set():
                # 检查状态
                status = self._check_status()
                if not status:
                    # 异常处理
                    self.logger.log_error("检测到异常，正在退出...")
                    self.stop_event.set()
                    self.robot.emergency_stop()
                    self.power_off()
                    os._exit(1) 
                self.status_queue.put(status)
                time.sleep(0.05) 
                
        except Exception as e:
            self.logger.log_error(f"监控线程异常: {e}")
            self.stop_event.set()
            self.power_off()
            os._exit(1)

    def _check_status(self):
        """执行一次状态检查"""
        try:
            states = {
                1: self.robot.get_joint_state(1),
                2: self.robot.get_joint_state(2),
                #3: self.robot.get_joint_state(3),
                4: self.robot.get_joint_state(4)
            }

            collision_state = self.robot.is_collision()
            connect_state = self.robot.is_connect()
            emergency_state = self.robot.get_hard_emergency_stop_state()

            if collision_state:
                err_msg = collision_error.get(collision_state, "未知碰撞状态")
                self.logger.log_error(f"碰撞异常: {err_msg}")
                return False
            if emergency_state != 0:
                err_msg = emergency_stop_error_codes.get(emergency_state, "未知急停错误码")
                self.logger.log_error(f"急停异常: {err_msg}")
                return False
            # 检查各个状态是否正常
            for joint, code in states.items():
                if code != 1 and code != 3:
                    err_msg = joint_error_codes.get(code, "未知关节错误码")
                    self.logger.log_warning(f"关节 {joint} 异常: {err_msg}")
                    return False
                # if code == 3 :
                #     self.logger.log_warning(f"关节{joint} 未初始化")
            if not connect_state:
                err_msg = connect_error.get(connect_state, "未知连接状态")
                self.logger.log_error(f"连接异常: {err_msg}")
                return False
            return True
        except Exception as e:
            self.logger.log_error(f"状态检查异常: {e}")
            return False
        
    def init(self):
        pass

    # def exit_task(self):
    #     self.is_exit = True
    #     self.logger.log_yellow("退出任务")
    #     self.arms_home()
        # instruction = {
        #     "is_massaging": False,
        #     "massage_service_started": False
        # }
        # requests.post(
        #     "http://127.0.0.1:5000/update_massage_status", data=instruction
    #     # )
    # @staticmethod
    # # 通过静态方法访问robot实例
    # def get_robot():
    #     return Huilin.robot  

    def getModeType(self):
        #连接状态
        robot_connect_state = self.robot.is_connect()
        print("机器人连接状态:", robot_connect_state)
        print(connect_error.get(robot_connect_state, "机械臂连接状态未知"))
        #碰撞状态
        robot_collision_state = self.robot.is_collision()
        print("机器人碰撞状态:", robot_collision_state)
        print(collision_error.get(robot_collision_state, "机械臂碰撞状态未知"))
        return robot_connect_state, robot_collision_state


    def start_up(self):
        max_retries = 3
        retry_count = 0
        while retry_count < max_retries:
            ret = self.robot.initial(1, 210)
            if ret == 1:
                print("Robot init success!")
                return True
            if ret == 0:
                retry_count += 1
                self.logger.log_error(f"机械臂不在线，重试 {retry_count}/{max_retries}")
                self.robot.net_port_initial()
                time.sleep(1)  # 等待1秒后重试
                continue
            if ret == 3:
                self.arms_home()
                time.sleep(10)
                continue
            if ret == 105:
                self.logger.log_error("存在某一个关节失效")
            print(initial_error_codes.get(ret, "机械臂初始化状态未知"))
            return False, ret
        self.logger.log_error("机械臂初始化失败，超过最大重试次数")
        return False, 0  # 返回0表示重试失败
        

    #机械臂全部回正
    def arms_home(self):
        #回正操作可能会导致连杆四碰撞，可先让它升到一个安全让连杆四回位的位置再回正
        self.move_joint([0,180,108],0)
        self._move_z_axis(0)
        # self.move_joint([30,240,198],0)
        # state1 = self.robot.joint_home(4)
        # if state1 == 1:
        #     time.sleep(6)
        #     state2 = self.robot.joint_home(2)
        #     if state2 == 1:
        #         state3 = self.robot.joint_home(1)
        #         if state3 == 1:
        #             print("机械臂回正成功")
        #         else:
        #             print(joint_home_error_codes.get(state3, "机械臂4关节回正状态未知"))
        #     else:
        #         print(joint_home_error_codes.get(state2, "机械臂2关节回正状态未知"))
        # else:
        #     print(joint_home_error_codes.get(state1, "机械臂1关节回正状态未知"))
        #机械臂回正后，默认初始化状态置0，当选择为1时，重新初始化
        # if state == 1:
        #     self.start_up()
        #     return 0

    # def send_command(self,arm_position_command, arm_orientation_command):
    #     rot_euler = R.from_quat(arm_orientation_command).as_euler('z', degrees=False)
    #     rot_euler = np.clip(rot_euler,-3.1415,3.1415)
    #     pose_command = np.concatenate([arm_position_command, rot_euler])

    #     if not self.is_exit:
    #         self.robot.get_scara_param()
    #         cur = self.robot.angle1

    #断电，关闭服务器
    def power_off(self):
        self.robot.close_server()
        print(1)

    #末端TCP位置
    def get_arm_position(self):
        self.robot.get_scara_param()
        arm_x = self.robot.x
        arm_y = self.robot.y
        #z轴方向替换成Z轴电机的位置
        z_value = self.Z_motor.sdo_read(0x6064,0x00)
        z_value = int.from_bytes(z_value, byteorder='little', signed=True)
        arm_z = (z_value +149000)/15400
        if abs(arm_z) <= 0.1:
            arm_z = 0
        arm_r = self.robot.r
        arm_x = arm_x + self.L3 * np.cos((arm_r-108)*np.pi/180)
        arm_y = arm_y + self.L3 * np.sin((arm_r-108)*np.pi/180)

        position = np.array([arm_x, arm_y, arm_z])
        empty_quat = R.from_quat([0, 0, 0, 1])
        return position, empty_quat
    
    #获取编码器位置
    def get_encoder_position(self):
        self.robot.get_encoder_coor()
        encoder_x = self.robot.encoder_x
        encoder_y = self.robot.encoder_y
        encoder_z = self.robot.encoder_z
        encoder_r = self.robot.encoder_r
        encoder_angle1 = self.robot.encoder_angle1
        encoder_angle2 = self.robot.encoder_angle2
        encoder_position = np.array([encoder_x, encoder_y, encoder_z])
        encoder_quat_rot = R.from_euler('z', encoder_r, degrees=True).as_quat()
        return encoder_position, encoder_quat_rot
    
    #逆运动学
    def inverse_kinematic(self, cur_angle, position, use_numerical=True, lr=-1):
        self.last_joint = cur_angle
        if use_numerical:
            max_iter = 500
            alpha = 0.3 #学习率
            tolerance = 1e-2
            solutions = []
            joint_diff = np.zeros(3)  
            joint_diff_thresholds = np.array([10, 20, 20])
            initial_conditions = [
                #当前机械臂实际位置(转角rad)
                np.array([cur_angle[0], cur_angle[1] - 180, cur_angle[2] - 108])* np.pi/180,
                #初始位置(转角rad)
                np.array(self.init_pos_Rotagl)* np.pi/180
                ]
            for theta in initial_conditions:
                for i in range(max_iter):
                    # 正向运动学计算当前位置
                    x_current = self.L1 * np.cos(theta[0]) - self.L2 * np.cos(theta[0] + theta[1]) + \
                                self.L3 * np.cos(theta[2])
                    y_current = self.L1 * np.sin(theta[0]) - self.L2 * np.sin(theta[0] + theta[1]) + \
                                self.L3 * np.sin(theta[2])
                    # 检查收敛
                    error = np.array([position[0] - x_current, position[1] - y_current])
                    if np.linalg.norm(error) < tolerance:
                        # 计算总旋转角度
                        total_rotation = np.sum(np.abs(theta))
                        solutions.append((theta.copy(), total_rotation))
                        print(solutions)
                        break
                    # 雅可比矩阵
                    J = np.array([
                        [-self.L1 * np.sin(theta[0]) + self.L2 * np.sin(theta[0] + theta[1]),
                            self.L2 * np.sin(theta[0] + theta[1]),
                            -self.L3 * np.sin(theta[2])],
                        [self.L1 * np.cos(theta[0]) - self.L2 * np.cos(theta[0] + theta[1]),
                            -self.L2 * np.cos(theta[0] + theta[1]),
                            self.L3 * np.cos(theta[2])]
                    ])
                    # 计算伪逆并更新关节角度
                    try:
                        J_pinv = np.linalg.pinv(J)
                        delta_theta = J_pinv @ error
                        theta += alpha * delta_theta
                    except np.linalg.LinAlgError:
                        return None,3
                    # 限制角度变化幅度（最大变化5度/次）
                    max_delta = 5 * np.pi/180
                    delta_theta = np.clip(delta_theta, -max_delta, max_delta)      
                    # 更新角度并限制在允许范围内
                    theta += delta_theta
                    theta[0] = np.clip(theta[0], -60 * np.pi/180, 60 * np.pi/180)
                    theta[1] = np.clip(theta[1], -90 * np.pi/180, 90 * np.pi/180)
                    theta[2] = np.clip(theta[2], -90 * np.pi/180, 90 * np.pi/180)
            if not solutions:
                return None, 1             
            #选择总旋转角度最小的解
            print("solutions:",solutions)
            min_solution = min(solutions, key=lambda x: x[1])[0] * 180/np.pi
            desire_joint = np.array([
                0 + min_solution[0],  
                180 + min_solution[1],  
                108 + min_solution[2]  
            ])
            joint_diff = desire_joint - self.last_joint
            if np.any(np.abs(joint_diff) >= joint_diff_thresholds):
                return desire_joint, 2
            return desire_joint, 0

            
    def get_inverse_kinematics_error_message(self, code,cur,desire_joint):
        if code == 0:
            self.logger.log_info(f"InverseKinematics code: {code}," + inverse_kinematics_error_message.get(code, "未知错误码"))
        elif code != 0:
            self.logger.log_error(f"InverseKinematics fail with code: {code}," + inverse_kinematics_error_message.get(code, "未知错误码"))
            self.logger.log_error(f'Current joint position: {cur}')
            self.logger.log_error(f'desire_joint: {desire_joint}')
            self.power_off()

    def _move_z_axis(self, target_position, stop_event =None,target_speed = 2000, error=1):
        if target_speed >= 0:
            if 0 <= target_position <= 570:
                # 读取实际位置
                position_value = self.Z_motor.sdo_read(0x6064, 0x00)
                position_value = int.from_bytes(position_value, byteorder='little', signed=True)
                real_position = (position_value + 149000) / 15400
                while abs(real_position - target_position) > error:
                    # 读取实际位置
                    position_value = self.Z_motor.sdo_read(0x6064, 0x00)
                    position_value = int.from_bytes(position_value, byteorder='little', signed=True)
                    real_position = (position_value + 149000) / 15400
                    if target_position > real_position:
                        #电机使能1开启0关闭
                        self.Z_motor.sdo_write(0x2100, 0x00, (0x01).to_bytes(2, 'little'))
                        # 向上运动正转
                        self.Z_motor.sdo_write(0x2600, 0x00, target_speed.to_bytes(4, 'little', signed=True))
                        
                    else:
                        # 向下运动反转
                        self.Z_motor.sdo_write(0x2100, 0x00, (0x01).to_bytes(2, 'little'))
                        self.Z_motor.sdo_write(0x2600, 0x00, (-1*target_speed).to_bytes(4, 'little', signed=True))
                # 完成任务速度置零
                self.Z_motor.sdo_write(0x2600, 0x00, int(0).to_bytes(4, 'little', signed=True))
                self.Z_motor.sdo_write(0x2100, 0x00, (0x00).to_bytes(2, 'little'))
                self.logger.log_info("z轴到达目标点位")
                return 0
            else:
                self.logger.log_error("位置超出限制，可输入范围为[0,570]")
                if stop_event:
                    stop_event.set()
                return 1
        else:
            self.logger.log_error("速度不能小于0")
            if stop_event:
                stop_event.set()
            return 1
        
    

    # #末端电机5
    # #减速比 8：1 输入8旋转360°，负数反转    
    # def move_joint_5(self,num):
    #     self.odrv0.axis0.controller.input_pos = num
    #机械臂关节运动
    def move_joint(self, joint, mode = 1,mod0_v = 20):
        #模式1为小角度输入，适用于一段段的发送位置差距较小的移动
        if mode == 1:
            self.robot.hi_position_send(joint[0],joint[1],0,joint[2])
            self.robot.wait_stop()
            return 0
        #模式0为直接移动，用于初始工作位置移动到指定的地方获得其他较大角度的一的移动
        else:
            code = self.robot.new_movej_angle(joint[0],joint[1],0,joint[2],mod0_v,0)
            #待优化
            self.get_movej_error_message(code)
            self.robot.wait_stop()
            return 0
    # def move_pose(self, pos, deg,speed =50,roughly = 1, lr= 1, wait = False):
    #     code = self.robot.new_movej_xyz_lr(pos[0], pos[1], pos[2], deg[2], speed, roughly, lr)
    #     if code == 1:
    #         print("本次指令生效，机械臂开始运动")
    #         return 0
    #     else:
    #         print(move_error_codes.get(code, "机械臂移动状态未知"))
    #         self.robot.new_stop_move()
    #         self.power_off()
    #         return -1
    #      
    def send_command(self, arm_position_command, arm_orientation_command):
        # rot_euler = R.from_quat(arm_orientation_command).as_euler('z', degrees=False)
        # rot_euler = np.clip(rot_euler,-3.1415,3.1415)
        pose_command = arm_position_command[:2]
        print("send_command中的", pose_command)
        z_command = arm_position_command[2]
        #获得当前关节角度
        self.robot.get_scara_param()
        cur = np.array([self.robot.angle1, self.robot.angle2, self.robot.r])
        #逆运动学解算
        desire_joint, code = self.inverse_kinematic(cur, pose_command)
        self.get_inverse_kinematics_error_message(code, cur, desire_joint)

        if code == 0:
            #单开一个Z轴移动的线程，确保不会阻塞
            stop_event = threading.Event()
            z_thread = threading.Thread(
                target=self._move_z_axis,
                args=(z_command,stop_event,1000),
                daemon=True
            )                  
            z_thread.start()
            delta_joint = desire_joint - cur
            steps = 30
            step_size = delta_joint / steps
            for i in range(steps):
                if stop_event.is_set():
                    self.logger.log_error("Z-axis movement failed")
                    return -1
                target_joint = cur + (i+1)*step_size
                self.move_joint(target_joint)
            # Wait for Z-axis thread to complete
            # # z_thread.join(timeout=5.0)
            # if z_thread.is_alive():
            #     self.logger.log_warning("Z-axis movement timed out")
        else:
            print("inverse_error_arm_is_exit")
            self.power_off()
            return -1
        

    def get_movej_error_message(self,code):
        if code == 1:
            self.logger.log_info(f'moveJ code: {code},' + movej_error_codes.get(code, "未知错误码"))
            return 0
        else:
            #发生碰撞则直接停机
            if code == 102:
                self.robot.emergency_stop()
                self.power_off()
                self.logger.log_error(f"moveJ failed with code: {code}, "+movej_error_codes.get(code, "未知错误码"))
                return -1
            self.logger.log_error(f"moveJ failed with code: {code}, "+movej_error_codes.get(code, "未知错误码"))
            return -1
           
    def wait_stop(self):
        state = self.robot.wait_stop()
        if state:
            print("机器人已停止")
            return True
        else:
            print("机器人未停止")
            return False
    
    def emergency_stop(self):
        self.robot.emergency_stop()

    #发送清除急停状态，进入正常模式
    def clear_hard_emergency_stop(self):
        state = self.robot.clear_hard_emergency_stop()
        if state == 1:
            print("清除急停状态成功")
            return 1
        else:
            print(clear_hard_emergency_stop_error_codes.get(state, "清除急停状态未知"))
            return state
    

if __name__ == "__main__":
    Huiling = Huilin()
    # Huiling._move_z_axis(100)
    # Huiling.arms_home()
    # Huiling.Z_motor.sdo_write(0x2600,0x00,int(1000).to_bytes(4, 'little', signed=True))
    # # time.sleep(4)
    # pose,_ = Huiling.get_arm_position()
    # print("坐标:",pose)
    # # Huiling.Z_motor.sdo_write(0x2600,0x00,int(0).to_bytes(4, 'little', signed=True))
    # Huiling.Z_motor.sdo_write(0x2504,0x00,int(1).to_bytes(2, 'little', signed=True))
    # #读取当前模式
    # current_mode = int.from_bytes(
    #     Huiling.Z_motor.sdo_read(0x2504, 0x00),
    #     byteorder='little',
    #     signed=False
    # )
    # if current_mode != 1:
    #     raise Exception(f'Failed to set operation mode. Expected: {1}, Actual: {current_mode}')
    
    # print(f"Operation mode successfully set to: {current_mode}")
    # Huiling.power_off()
    # Huiling.arms_home()
