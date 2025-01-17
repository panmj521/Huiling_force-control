from functools import wraps
import sys
import os
import threading
import time
from typing import List, Literal, Union
import numpy as np
import atexit
from scipy.spatial.transform import Rotation as R
import requests
from scipy.interpolate import interp1d
import copy

# 导入算法
try:
    from .algorithms.arm_state import ArmState
    from .algorithms.controller_manager import ControllerManager
    from .algorithms.admittance_controller import AdmittanceController
    from .algorithms.hybrid_controller import HybridController
    from .algorithms.admithybrid_controller import AdmitHybridController
    from .algorithms.position_controller import PositionController
    from .algorithms.hybridPid_controller import HybridPidController
    from .algorithms.interpolation import linear_interpolate,spline_interpolate,circle_trajectory,cloud_point_interpolate
    # 导入硬件
    # from .hardware.force_sensor_aubo import XjcSensor
    from hardware.force_sensor import XjcSensor
    from .hardware.realman_RM63 import RealmanRM63
    from .hardware.thermotherapy import Thermotherapy
    from .hardware.shockwave import Shockwave

    # 导入工具
    from .tools.log import CustomLogger
    from .tools.Rate import Rate
    from .tools.yaml_operator import read_yaml
    from .tools.decorator_tools import custom_decorator,apply_decorators
    from .tools.serial_tools import find_serial_by_location
except:
    #导入算法
    from algorithms.arm_state import ArmState
    from algorithms.controller_manager import ControllerManager
    from algorithms.admittance_controller import AdmittanceController
    from algorithms.hybrid_controller import HybridController
    from algorithms.admithybrid_controller import AdmitHybridController
    from algorithms.position_controller import PositionController
    from algorithms.hybridPid_controller import HybridPidController
    from algorithms.interpolation import linear_interpolate,spline_interpolate,circle_trajectory

    # 导入硬件
    # from hardware.force_sensor_aubo import XjcSensor
    from hardware.force_sensor import XjcSensor
    from hardware.realman_RM63 import RealmanRM63
    from hardware.thermotherapy import Thermotherapy
    from hardware.shockwave import Shockwave



    # 导入工具
    from tools.log import CustomLogger
    from tools.Rate import Rate
    from tools.yaml_operator import read_yaml
    from tools.decorator_tools import custom_decorator,apply_decorators
    from tools.serial_tools import find_serial_by_location





import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation  

"""
在 Python 中，atexit.register 用于注册程序退出时要执行的函数。atexit 模块提供了一个简单的接口，用于在程序正常终止时执行清理操作。
当你注册多个函数时，它们的执行顺序遵循先进后出的顺序（LIFO：Last In, First Out）。也就是说，最后注册的函数会最先执行。
"""

def track_function(function_name,log = False):
    def before(self, *args, **kwargs):
        self.current_function = function_name
        if log:
            self.logger.log_info(f"Entering function: {function_name}")
    def after(self, result, *args, **kwargs):
        self.current_function = None
        if log:
            self.logger.log_info(f"Exiting function: {function_name},code: {result}")
    return custom_decorator(before=before, after=after)


@apply_decorators
class MassageRobot:
    def __init__(self,arm_config_path,is_log = False) -> None:
        # 日志
        self.logger = CustomLogger()
        if is_log:
            self.logger_data = CustomLogger()
            # 日志线程
            threading.Thread(target=self.log_thread,daemon=True).start()

        
        # 当前执行的函数
        self.current_function = None
        # 机械臂状态(机械臂坐标系下) 
        self.arm_state = ArmState()

        arm_config = read_yaml(arm_config_path)
        
        self.arm = RealmanRM63(arm_ip=arm_config['arm_ip'])
        # self.force_sensor = XjcSensor(arm_ip=arm_config['arm_ip'])
        try:
            self.force_sensor = XjcSensor('/dev/ttyUSB0', 115200)
        except:
            self.force_sensor = XjcSensor('/dev/ttyUSB1', 115200)
        # self.force_sensor.connect()

        # #########
        # max_retries = 5
        # #no_head_wrong=np.array([-80.01506042, 116.34187317, -87.65788269, -0.32910481, -0.65792173, -0.61110526])#旧版传感器）
        # head_set0=np.array([0, 0, 0, 0, 0, 0])#新版传感器）
        # min_tolerance=0.001
        # retry_count = 0
        # self.force_sensor.disable_active_transmission()
        # # self.force_sensor.enable_active_transmission()
        # time.sleep(0.5)
        # while retry_count < max_retries:
        #     # self.force_sensor.disable_active_transmission()
        #     # 调用读取数据的函数，假设返回值是一个 numpy 数组
        #     data = self.force_sensor.read_data_f32()
        #     # data = self.force_sensor.read()

        #     # 检查返回值是否为 np.array 类型
        #     if isinstance(data, np.ndarray):
        #         # 判断数据是否接近目标值

        #         if np.allclose(data, head_set0, atol=min_tolerance):
        #             print(f"检测到力传感器已置零：{data}") 
        #             break
        #         else:

        #             # 禁用传感器传输
        #             self.force_sensor.disable_active_transmission()
        #             self.force_sensor.disable_active_transmission()
        #             time.sleep(0.5)

        #             # 尝试设置传感器清零
        #             code = self.force_sensor.set_zero()
        #             code = self.force_sensor.set_zero()
        #             max_try = 3

        #             while code != 0 and max_try > 0:
        #                 self.logger.log_error("Force sensor set zero failed, retrying...")
        #                 self.force_sensor.disable_active_transmission()
        #                 self.force_sensor.disable_active_transmission()
        #                 time.sleep(0.5)
        #                 code = self.force_sensor.set_zero()
        #                 code = self.force_sensor.set_zero()
        #                 max_try -= 1
        #                 time.sleep(0.5)

        #             # 如果多次尝试后失败
        #             if max_try == 0:
        #                 self.logger.log_error("Force sensor set zero failed, exiting...")
        #                 requests.post(
        #                     "http://127.0.0.1:5000/on_message", data={"message": "传感器初始化失败"}
        #                 )
        #                 requests.post(
        #                     "http://127.0.0.1:5000/update_massage_status", data={"massage_service_started": False}
        #                 )
        #                 sys.exit(0)

        #             # 成功后跳出主循环
        #             break

        #     else:
        #         self.logger.log_error("读取数据失败或格式错误，尝试重试...")

        #     # 增加重试次数
        #     retry_count += 1
        #     self.force_sensor.disable_active_transmission()
        #     self.force_sensor.disconnect()
        #     time.sleep(0.5)
        #     self.force_sensor.connect()
        #     time.sleep(1)  # 每次重试之间添加一个短暂的延迟

        # if retry_count == max_retries:
        #     self.logger.log_error(f"超过最大重试次数 ({max_retries})，操作失败，退出程序...")
        #     requests.post(
        #         "http://127.0.0.1:5000/on_message", data={"message": "传感器初始化失败"}
        #     )
        #     requests.post(
        #         "http://127.0.0.1:5000/update_massage_status", data={"massage_service_started": False}
        #     )
        #     sys.exit(0)  # 达到最大重试次数后退出
        
        # ##########
        # self.arm.init()
        self.thermotherapy = None
        self.shockwave = None
        # self.thermotherapy = Thermotherapy(arm_ip=arm_config['arm_ip'])

        # 控制器,初始为导纳控制
        self.controller_manager = ControllerManager(self.arm_state)
        self.controller_manager.add_controller(AdmittanceController,'admittance',arm_config['controller'][0])
        self.controller_manager.add_controller(HybridController,'hybrid',arm_config['controller'][1])
        self.controller_manager.add_controller(PositionController,'position',arm_config['controller'][2])
        self.controller_manager.add_controller(AdmitHybridController,'admithybrid',arm_config['controller'][3])
        self.controller_manager.add_controller(HybridPidController,'hybridPid',arm_config['controller'][4])
        self.controller_manager.switch_controller('admittance')


        # 频率
        self.control_rate = Rate(arm_config['control_rate'])
        self.sensor_rate = Rate(arm_config['sensor_rate'])
        self.command_rate = Rate(arm_config['command_rate'])

        # 低通滤波
        self.cutoff_freq = 80.0
        

        # 停止标志位
        self.exit_event = threading.Event()
        self.exit_event.set()
        self.interrupt_envet = threading.Event()
        self.interrupt_envet.clear()
        # 调整
        self.adjust_wrench_envent = threading.Event()
        self.adjust_wrench_envent.clear()
        self.adjust_pos_envent = threading.Event()
        self.adjust_pos_envent.clear()

        self.pos_increment = np.zeros(3,dtype=np.float64)
        self.adjust_wrench = np.zeros(6,dtype=np.float64)

        # 加载按摩头配置文件,初始为不搭载按摩头
        massage_head_dir = arm_config['massage_head_dir']
        all_items = os.listdir(massage_head_dir)
        head_config_files = [f for f in all_items if os.path.isfile(os.path.join(massage_head_dir, f))]
        self.playload_dict = {}
        for file in head_config_files:
            file_address = massage_head_dir + '/' + file
            play_load = read_yaml(file_address)
            self.playload_dict[play_load['name']] = play_load
        self.current_head = 'none'

        self.is_waitting = False

        self.last_print_time = 0
        self.last_record_time = 0
        self.last_command_time = 0
        self.move_to_point_count = 0
        

        self.x_base = np.zeros(6)
        self.P_base  = np.eye(6)
        # 过程噪声协方差矩阵
        self.Q_base = np.eye(6) * 0.01
        # 测量噪声协方差矩阵
        self.R_base = np.eye(6) * 0.1

        self.x_tcp = np.zeros(6)
        self.P_tcp  = np.eye(6)
        # 过程噪声协方差矩阵
        self.Q_tcp = np.eye(6) * 0.01
        # 测量噪声协方差矩阵
        self.R_tcp = np.eye(6) * 0.1

    # 预测步骤
    def kalman_predict(self,x, P, Q):
        # 预测状态（这里假设状态不变）
        x_predict = x
        # 预测误差协方差
        P_predict = P + Q
        return x_predict, P_predict

    # 更新步骤
    def kalman_update(self,x_predict, P_predict, z, R):
        # 卡尔曼增益
        K = P_predict @ np.linalg.inv(P_predict + R)
        # 更新状态
        x_update = x_predict + K @ (z - x_predict)
        # 更新误差协方差
        P_update = (np.eye(len(K)) - K) @ P_predict
        return x_update, P_update


    # TODO 硬件处理，还需要按摩头的初始化
    #####################################################################################################
    def init_hardwares(self,ready_pose):
        
        # 位置模式
        
        self.ready_pose = np.array(ready_pose)
        self.switch_payload(self.current_head)
        self.arm_state.desired_position = self.ready_pose[:3]
        euler_angles = self.ready_pose[3:]
        self.arm_state.desired_orientation = R.from_euler('xyz', euler_angles).as_quat()
        print(self.arm.get_arm_position())
        # 伺服模式
        # self.arm.enable_servo()
        time.sleep(1)
        # code  = self.force_sensor.enable_active_transmission()
        # max_try = 3
        # while code!= 0 and max_try > 0:
        #     self.logger.log_error("Force sensor enable_active_transmission failed,retrying...")
        #     code  = self.force_sensor.enable_active_transmission()
        #     max_try -= 1
        #     time.sleep(0.1)
        # if max_try == 0:
        #     self.logger.log_error("Force sensor enable_active_transmission failed,exiting...")
        #     requests.post(
        #         "http://127.0.0.1:5000/on_message", data={"message": "传感器初始化失败"}
        #     )
        #     requests.post(
        #         "http://127.0.0.1:5000/update_massage_status", data={"massage_service_started": False}
        #     )
        #     sys.exit(0)

    def sensor_set_zero(self):
        #########
        max_retries = 5
        #no_head_wrong=np.array([-80.01506042, 116.34187317, -87.65788269, -0.32910481, -0.65792173, -0.61110526])#旧版传感器）
        head_set0=np.array([0, 0, 0, 0, 0, 0])#新版传感器）
        min_tolerance=0.001
        retry_count = 0
        self.force_sensor.disable_active_transmission()
        # self.force_sensor.enable_active_transmission()
        time.sleep(0.5)
        while retry_count < max_retries:
            # self.force_sensor.disable_active_transmission()
            # 调用读取数据的函数，假设返回值是一个 numpy 数组
            data = self.force_sensor.read_data_f32()
            # data = self.force_sensor.read()

            # 检查返回值是否为 np.array 类型
            if isinstance(data, np.ndarray):
                # 判断数据是否接近目标值

                if np.allclose(data, head_set0, atol=min_tolerance):
                    print(f"检测到力传感器已置零：{data}") 
                    break
                else:
                    print(f"检测到力传感器未置零：{data}") 

                    # 禁用传感器传输
                    self.force_sensor.disable_active_transmission()
                    self.force_sensor.disable_active_transmission()
                    time.sleep(0.5)

                    # 尝试设置传感器清零
                    code = self.force_sensor.set_zero()
                    code = self.force_sensor.set_zero()
                    max_try = 3

                    while code != 0 and max_try > 0:
                        self.logger.log_error("Force sensor set zero failed, retrying...")
                        self.force_sensor.disable_active_transmission()
                        self.force_sensor.disable_active_transmission()
                        time.sleep(0.5)
                        code = self.force_sensor.set_zero()
                        code = self.force_sensor.set_zero()
                        max_try -= 1
                        time.sleep(0.5)

                    # 如果多次尝试后失败
                    if max_try == 0:
                        self.logger.log_error("Force sensor set zero failed, exiting...")
                        requests.post(
                            "http://127.0.0.1:5000/on_message", data={"message": "传感器初始化失败"}
                        )
                        requests.post(
                            "http://127.0.0.1:5000/update_massage_status", data={"massage_service_started": False}
                        )
                        sys.exit(0)

                    # 成功后跳出主循环
                    break

            else:
                self.logger.log_error("读取数据失败或格式错误，尝试重试...")

            # 增加重试次数
            retry_count += 1
            self.force_sensor.disable_active_transmission()
            self.force_sensor.disconnect()
            time.sleep(0.5)
            self.force_sensor.connect()
            time.sleep(1)  # 每次重试之间添加一个短暂的延迟

        if retry_count == max_retries:
            self.logger.log_error(f"超过最大重试次数 ({max_retries})，操作失败，退出程序...")
            requests.post(
                "http://127.0.0.1:5000/on_message", data={"message": "传感器初始化失败"}
            )
            requests.post(
                "http://127.0.0.1:5000/update_massage_status", data={"massage_service_started": False}
            )
            sys.exit(0)  # 达到最大重试次数后退出

    def sensor_enable(self):
        code  = self.force_sensor.enable_active_transmission()
        max_try = 3
        while code!= 0 and max_try > 0:
            self.logger.log_error("Force sensor enable_active_transmission failed,retrying...")
            code  = self.force_sensor.enable_active_transmission()
            max_try -= 1
            time.sleep(0.1)
        if max_try == 0:
            self.logger.log_error("Force sensor enable_active_transmission failed,exiting...")
            requests.post(
                "http://127.0.0.1:5000/on_message", data={"message": "传感器初始化失败"}
            )
            requests.post(
                "http://127.0.0.1:5000/update_massage_status", data={"massage_service_started": False}
            )
            sys.exit(0)
            
    def update_wrench(self):
        compensation_config = self.playload_dict[self.current_head]

        # 读取数据
        gravity_base = np.array(compensation_config['gravity_base'])
        force_zero = np.array(compensation_config['force_zero'])
        torque_zero = np.array(compensation_config['torque_zero'])
        tool_position = np.array(compensation_config['tcp_offset'])
        mass_center_position = np.array(compensation_config['mass_center_position'])
        s_tf_matrix_t = self.get_tf_matrix(tool_position[:3], R.from_euler('xyz',tool_position[3:],degrees=False).as_quat())

        b_rotation_s = R.from_quat(self.arm_state.arm_orientation).as_matrix()

        arm_orientation_set0 = np.array([150,0,180])
        b_rotation_s_set0 = R.from_euler('xyz',arm_orientation_set0,degrees=True).as_matrix()
        # 读取数据

        sensor_data = self.force_sensor.read()

        if sensor_data is None:
            self.logger.log_error("传感器数据读取失败")
            return
        
        
        # 反重力补偿
        sensor_data[:3] = sensor_data[:3] + force_zero + b_rotation_s_set0.T @ gravity_base
        sensor_data[3:] = sensor_data[3:] + torque_zero + np.cross(mass_center_position, b_rotation_s_set0.T @ gravity_base)

        # print("sensor_data:",sensor_data)
        # 重力补偿
        gravity_sensor = b_rotation_s.T @ gravity_base
        # 力的矫正
        s_force = sensor_data[:3] - force_zero - gravity_sensor
        # print(s_force)
        # 力矩的矫正
        s_torque = sensor_data[3:] - torque_zero - np.cross(mass_center_position, gravity_sensor)
        # print(s_torque)
        wrench = np.concatenate([s_force, s_torque ])
        # if time.time() - self.last_print_time > 5:
        #     print("sensor_data",sensor_data)
        #     print("wrench",wrench)
            # self.last_print_time = time.time()

        # 将力和力矩转换到tcp坐标下
        wrench = self.wrench_coordinate_conversion(s_tf_matrix_t, wrench)
        # if time.time() - self.last_print_time > 5:
        #     print("tf_wrench",wrench)
            # print("---------------------------")
            # self.last_print_time = time.time()
        # print(wrench)
        # print(self.last_print_time)
        self.arm_state.external_wrench_tcp = wrench
        # sensor frame to base frame
        self.arm_state.external_wrench_base = np.concatenate([b_rotation_s @ self.arm_state.external_wrench_tcp[:3],
                                                          b_rotation_s @ self.arm_state.external_wrench_tcp[3:]]) 
        



        # apply low-pass filter
        T_sensor = self.sensor_rate.to_sec()
        a = T_sensor / (1 / (2 * np.pi * self.cutoff_freq) + T_sensor)
        self.arm_state.external_wrench_base = (1 - a) * self.arm_state.last_external_wrench_base \
                                            + a * self.arm_state.external_wrench_base
        self.arm_state.last_external_wrench_base = self.arm_state.external_wrench_base



        self.arm_state.external_wrench_tcp = (1 - a) * self.arm_state.last_external_wrench_tcp \
                                                + a * self.arm_state.external_wrench_tcp    
        self.arm_state.last_external_wrench_tcp = self.arm_state.external_wrench_tcp    


        # # 卡尔曼滤波器平滑
        # # 对base坐标系下的外力外矩进行平滑
        # x_base_predict, P_base_predict = self.kalman_predict(x = self.x_base, P = self.P_base, Q = self.Q_base)
        # self.x_base, self.P_base = self.kalman_update(x_predict = x_base_predict, P_predict = P_base_predict, z = self.arm_state.external_wrench_base, R = self.R_base)
        # self.arm_state.external_wrench_base = self.x_base

        # self.arm_state.last_external_wrench_base = self.arm_state.external_wrench_base

        # # 对tcp坐标系下的外力外矩进行平滑
        # x_tcp_predict, P_tcp_predict = self.kalman_predict(x = self.x_tcp, P = self.P_tcp, Q = self.Q_tcp)
        # self.x_tcp, self.P_tcp = self.kalman_update(x_predict = x_tcp_predict, P_predict = P_tcp_predict, z = self.arm_state.external_wrench_tcp, R = self.R_tcp)
        # self.arm_state.external_wrench_tcp = self.x_tcp

        # self.arm_state.last_external_wrench_tcp = self.arm_state.external_wrench_tcp 


        # self.logger_data.log_info()   
        # if time.time() - self.last_print_time > 1:
        #     print("tcp_wrench:",self.arm_state.external_wrench_tcp)
        #     print("base_wrench:",self.arm_state.external_wrench_base)
        #     print("---------------------------")
        #     self.last_print_time = time.time()
    

    def switch_payload(self,name):
        if name in self.playload_dict:
            self.stop()
            self.current_head = name
            print("arm_orientation",R.from_quat(self.arm_state.arm_orientation).as_euler('xyz',degrees=False))
            print("arm_position",self.arm_state.arm_position)
            print("-------change-------")
            self.arm.change_end_effector(self.playload_dict[name])
            self.logger.log_info(f"切换到{name}按摩头")
            current_orientation = self.arm_state.arm_orientation
            print("arm_orientation",R.from_quat(self.arm_state.arm_orientation).as_euler('xyz',degrees=False))
            print("arm_position",self.arm_state.arm_position)
            # R_matrix = R.from_quat(current_orientation).as_matrix()
            R_matrix = R.from_euler('xyz',self.ready_pose[3:] ,degrees=False).as_matrix()
            ready_position = self.ready_pose[:3] + R_matrix @ self.playload_dict[self.current_head]['tcp_offset'][:3]
            ready_orientation =  R_matrix @ R.from_euler('xyz',self.playload_dict[self.current_head]['tcp_offset'][3:],degrees=False).as_matrix()
            ready_orientation_euler = R.from_matrix(ready_orientation).as_euler('xyz',degrees=False)
            print("ready_pose",self.ready_pose)
            print("desired_position", ready_position)
            # print("ready_orientation",ready_orientation)
            print("desired_orientation",ready_orientation_euler)
            # TODO 
            self.arm_state.desired_position = ready_position 
            self.arm_state.desired_orientation = R.from_euler('xyz',ready_orientation_euler,degrees=False).as_quat()
            # if name != "wash_head":
            #     self.controller_manager.switch_controller('admittance')
            # else:
            #     self.controller_manager.switch_controller('hybrid')
            #     # self.controller_manager.switch_controller('admittance')
            self.controller_manager.switch_controller('position')

        else:
            self.logger.log_error(f"未找到{name}按摩头")

    #####################################################################################################
    def step(self,dt):
        self.controller_manager.step(dt)

    def arm_measure_loop(self):
        self.logger.log_info("机械臂测量线程启动")
        # while (not self.arm.is_exit) and (not self.exit_event.is_set()):
        while not self.exit_event.is_set():
            try:
                if not self.is_waitting:
                    # 机械臂测量
                    self.arm_state.arm_position,self.arm_state.arm_orientation = self.arm.get_arm_position()
                    # 力传感器测量
                    self.update_wrench()
            except Exception as e:
                self.logger.log_error(f"机械臂或传感器数据读取失败:{e}")
                self.exit_event.set()
            self.sensor_rate.sleep()

    def arm_command_loop(self):
        self.logger.log_info("机械臂控制线程启动")
        # while (not self.arm.is_exit) and (not self.exit_event.is_set()):
        while not self.exit_event.is_set():
            try:
                if not self.is_waitting:
                    self.step(self.control_rate.to_sec())
                    # print(self.arm_state.arm_position_command)
                    self.last_command_time += 1

                    code = self.arm.send_command(self.arm_state.arm_position_command,self.arm_state.arm_orientation_command)

                    # if self.last_command_time > 10:
                    #     print("commandTime:",(time.time()-self.last_record_time)/self.last_command_time)
                    #     self.last_record_time = time.time()
                    #     self.last_command_time = 0

                    if code == -1:
                        self.logger.log_error("机械臂急停")
                        self.stop()
                        break
            except Exception as e:
                self.logger.log_error(f"机械臂控制失败:{e}")
                self.exit_event.set()
            self.control_rate.sleep()

    def start(self):
        if self.exit_event.is_set():
            self.exit_event.clear()
            self.arm_measure_thread = threading.Thread(target=self.arm_measure_loop)
            self.arm_control_thread = threading.Thread(target=self.arm_command_loop)
            # 线程开始
            self.arm_measure_thread.start()
            
            poistion ,quat_rot  = self.arm.get_arm_position()
            self.arm_state.desired_position = poistion
            self.arm_state.arm_position_command = poistion
            self.arm_state.desired_orientation = quat_rot
            self.arm_state.arm_orientation_command = quat_rot
            for i in range(20):
                self.step(self.control_rate.to_sec())
                # self.logger.log_blue(f"position command: {self.arm_state.arm_position_command}")
                # self.logger.log_blue(f"orientation command: {R.from_quat(self.arm_state.arm_orientation_command).as_euler('xyz',degrees=False)}")
                poistion ,quat_rot  = self.arm.get_arm_position()
                # self.logger.log_blue(f"position current: {poistion}")
                # self.logger.log_blue(f"orientation current: {R.from_quat(quat_rot).as_euler('xyz',degrees=False)}")
                self.command_rate.sleep()
            # self.arm.enable_servo()
            poistion ,quat_rot  = self.arm.get_arm_position()
            self.arm_state.desired_position = poistion
            self.arm_state.arm_position_command = poistion
            self.arm_state.desired_orientation = quat_rot
            self.arm_state.arm_orientation_command = quat_rot
            self.arm_control_thread.start()
            self.logger.log_info("MassageRobot启动")
            time.sleep(1)

    def stop(self):
        if not self.exit_event.is_set():
            self.exit_event.set()
            # self.arm.move_joint(self.arm.init_pos,wait=True)
            self.arm_control_thread.join()
            self.arm_measure_thread.join()
            # self.switch_payload('none')
            self.arm.disable_servo()
            self.logger.log_info("MassageRobot停止")
   
    @track_function("move_to_point",True)
    def move_to_point(self, t, pose, is_interrupt=True, wait=True, timeout=0.5, interpolation:Literal["linear","circle","cloud_point"] ='linear', algorithm:Literal["admittance","position","hybrid","admithybrid","hybridPid"] = "admittance",is_switch_controller = False):
        """
        移动到指定的点
        param:
            t: 时间或时间序列
            pose(nx6): 世界坐标系下的位姿或位姿序列,当输入为None时,表示保持当前位姿
            is_interrupt: 是否允许中断
            wait: 是否等待运动完成
            timeout: 超时时间
            interpolation: 插值方式
            algorithm: 控制算法
        return:
            code: 0表示成功
                1表示超时
                2表示用户打断
                3表示硬件错误
                4表示参数错误
        """
        self.move_to_point_count += 1
        self.logger.log_blue(f"move_to_point_count: {self.move_to_point_count}")
        self.logger.log_yellow(f"move_to_point: {pose}")
        # 在函数内部直接规划轨迹
        traj = self.traj_generate(t, pose, None, interpolation=interpolation)
        if traj is None:
            self.logger.log_error("轨迹未生成或生成错误")
            return 4

        self.logger.log_yellow(f"traj_quat: {R.from_quat(traj['quat'][-1]).as_euler('xyz',degrees=False)}")
        self.logger.log_yellow(f"traj_pos: {traj['pos'][-1]}")

        time_seq = traj['t']
        pos_traj = traj['pos']
        quat_traj = traj['quat']

        if is_switch_controller == True:
            try:
                self.controller_manager.switch_controller(algorithm)
            except ValueError as error:
                self.logger.log_error(error)
                return 4
            self.logger.log_info("切换到%s控制器" % algorithm)

        # self.logger.log_info("当前为%s控制器" % algorithm)

        
        self.arm.enable_servo()
        
        for i in range(len(time_seq)):
            if self.interrupt_envet.is_set():
                self.interrupt_envet.clear()
                self.logger.log_error("self.interrupt_envet.clear()")
                return 2
            if self.arm.is_exit or self.exit_event.is_set():
                print("robot_hardware_error:",self.arm.is_exit,self.exit_event.is_set())
                return 3
            if self.adjust_pos_envent.is_set():
                if is_interrupt:
                    self.adjust_pos_envent.clear()
                    return 2
                else:
                    # if pos_traj is not None: 
                    #     pos_traj[i:] = pos_traj[i:] + self.pos_increment
                    self.adjust_pos_envent.clear()
            if self.adjust_wrench_envent.is_set():
                if self.adjust_wrench is not None:
                    self.arm_state.desired_wrench = self.adjust_wrench
                self.adjust_wrench_envent.clear()

            self.arm_state.desired_position = pos_traj[i]
            self.arm_state.desired_orientation = quat_traj[i]
            # if time.time() - self.last_print_time > 0.5:
            #     euler = R.from_quat(self.arm_state.desired_orientation).as_euler('xyz',degrees=False)
            #     self.logger.log_blue(f'{self.arm_state.desired_position} {euler}')
            #     self.last_print_time = time.time()
            # print("----")
            # print(self.arm_state.desired_position,R.from_quat(self.arm_state.desired_orientation).as_euler('xyz',degrees=False))
            self.command_rate.sleep()

        start = time.time()

        while wait:
            if self.adjust_pos_envent.is_set():
                if is_interrupt:
                    self.adjust_pos_envent.clear()
                    return 2
                else:
                    # if pos_traj is not None: 
                    #     pos_traj[-1] = pos_traj[-1] + self.pos_increment
                    self.adjust_pos_envent.clear()

            if self.interrupt_envet.is_set():
                self.interrupt_envet.clear()
                return 2
            if self.arm.is_exit or self.exit_event.is_set():
                print("robot_hardware_error:",self.arm.is_exit,self.exit_event.is_set())
                return 3
            if self.is_arrivated():
                return 0
            if time.time() - start > timeout:
                return 1
            self.command_rate.sleep()

        return 0
    
    @track_function("generate_prolate_cycloid_cloudPoint",True)
    def generate_prolate_cycloid_cloudPoint(self, start, end, a_prolate=4.5, radius=30.0, step_degree=np.pi/12,factor = 1.0,num_cycle_factor = 1):

        temp_radius = radius * factor
        if temp_radius > radius:
            temp_radius = radius
        distance = np.linalg.norm(np.array(end) - np.array(start))
        angle = np.arctan2(end[1] - start[1], end[0] - start[0])
        
        # Calculate cycloid parameters
        r_cycloid = radius / (2 * a_prolate)
        num_cycles = round(num_cycle_factor*1.5*(distance / (temp_radius / (2 * a_prolate))- 2 * a_prolate - np.pi) / (2 * np.pi))
        temp_k = distance / (r_cycloid * (np.pi + 2 * a_prolate + 2 * np.pi * num_cycles))

        # Generate the prolate cycloid in its local coordinates
        theta_prolate = np.linspace(0.5 * np.pi, num_cycles * 2 * np.pi + 1.5 * np.pi,
                                    round((num_cycles * 2 * np.pi + 1.5 * np.pi) / step_degree))

        x_prolate = r_cycloid * (theta_prolate - a_prolate * np.sin(theta_prolate)) *temp_k
        y_prolate = r_cycloid * (1 - a_prolate * np.cos(theta_prolate))
        for i in range(len(y_prolate)-1):
            y_prolate[i] = y_prolate[i] + y_prolate[i] * np.abs((x_prolate[i]-x_prolate[0])/(x_prolate[-1]-x_prolate[0]))*(factor-1.0)

        # Rotate the cycloid to align with the angle
        x_rotated = x_prolate * np.cos(angle) - y_prolate * np.sin(angle)
        y_rotated = x_prolate * np.sin(angle) + y_prolate * np.cos(angle)

        # Translate the cycloid to start from the starting point
        x_final = x_rotated + start[0] - x_rotated[0]
        y_final = y_rotated + start[1] - y_rotated[0]

        # Optionally, convert to integers if required for specific use case
        x_final_int = np.round(x_final).astype(int).reshape(-1, 1)
        y_final_int = np.round(y_final).astype(int).reshape(-1, 1)

        # Combine x and y into a single array and add the endpoint
        result = np.hstack((x_final_int, y_final_int))

        return result
    
    @track_function("generate_circle_cloudPoint",True)
    def generate_circle_cloudPoint(self, start_point, center, step_degree=np.pi/12, num_cycle_factor = 3):
        """
        center: 圆心坐标，形如 (x_c, y_c)
        start_point: 起始点坐标，形如 (x_0, y_0)
        radius: 圆的半径
        delta_theta: 每次插补的角度增量
        num_turns: 绕圈的次数
        """
        # 确定总共需要生成的插补点数
        num_points = int((2 * np.pi * num_cycle_factor) / step_degree)
        
        # 圆心
        x_c, y_c = center

        radius =  np.linalg.norm(np.array(start_point) - np.array(center))  # 半径
        
        # 计算起始点的初始角度
        x_0, y_0 = start_point
        theta_0 = np.arctan2(y_0 - y_c, x_0 - x_c)
        
        # 初始化存储插补点的列表
        circle_points = []
        
        # 生成插补点
        for i in range(num_points):
            # 当前角度
            theta_i = theta_0 + i * step_degree
            
            # 计算插补点的坐标
            x_i = x_c + radius * np.cos(theta_i)
            y_i = y_c + radius * np.sin(theta_i)
            
            # 将点添加到列表中

            circle_points.append((np.round(x_i).astype(int), np.round(y_i).astype(int)))

        circle_points.append((np.round(x_0).astype(int), np.round(y_0).astype(int)))
        
        return circle_points

    @track_function("generate_repeat_line_cloudPoint",True)
    def generate_repeat_line_cloudPoint(self, start_point, end_point, num_cycle_factor = 5):
        """
        start_point: 起始点坐标，形如 (x_0, y_0)
        end_point: 终点坐标，形如 (x_0, y_0)
        num_cycle_factor: 重复的次数
        """
        
        # 初始化存储插补点的列表
        result_points = []
        
        # 生成插补点
        for i in range(num_cycle_factor):
            # 将点添加到列表中

            result_points.append(start_point)
            result_points.append(end_point)

        return np.round(np.array(result_points)).astype(int)



    @track_function("generate_finger_circle_cloudPoint",True)
    def generate_finger_circle_cloudPoint(self, center_point, num_cycle_factor=5, radius=30,deviation=15):
        """
        生成一指禅绕球心的圆弧轨迹
        center_point: 圆心坐标 [x, y, z, roll, pitch, yaw]
        num_cycle_factor: 轨迹的重复次数
        radius: 圆的半径
        """
        result_points = []
        result_points.append(copy.deepcopy(center_point))
        deviation = deviation * np.pi/180

        totalCounts = int(360 / radius)
        shift = 1


        for i in range(num_cycle_factor):
            for countR in range(0, totalCounts):
                # 当前旋转矩阵
                temp_pose = copy.deepcopy(center_point)
                rotation = R.from_euler('xyz', temp_pose[3:], degrees=False)
                current_rotation_matrix = rotation.as_matrix()
                

                if i == 0:
                    shift = countR/(90/radius) if countR/(90/radius) < 1 else 1
                if i == num_cycle_factor-1:
                    shift = (totalCounts-countR)/(90/radius) if (totalCounts-countR)/(90/radius) < 1 else 1


                # 生成增量旋转矩阵
                increment_rotation= R.from_euler('xyz', [deviation*shift,0,countR * np.pi*2 /totalCounts], degrees=False).as_matrix()

                # 更新旋转矩阵
                updated_rotation_matrix = current_rotation_matrix @ increment_rotation

                # 将旋转矩阵转换回欧拉角
                temp_pose[3:] = R.from_matrix(updated_rotation_matrix).as_euler('xyz', degrees=False)

                # 添加当前点到结果列表
                result_points.append(copy.deepcopy(temp_pose))

        result_points.append(copy.deepcopy(center_point))
        return result_points
    
    @track_function("generate_ellipse_points",True)    
    def generate_ellipse_points(self, ellipse_center, ellipse_a, ellipse_b, num_points=100):
        # 生成椭圆离散点
        # 生成参数 t
        ellipse_t = np.linspace(0, 2 * np.pi, int(num_points))
        # 计算椭圆的 x 和 y 坐标
        ellipse_x = ellipse_center[0] + ellipse_a * np.cos(ellipse_t)
        ellipse_y = ellipse_center[1] + ellipse_b * np.sin(ellipse_t)

        return ellipse_x, ellipse_y

    @track_function("generate_uniform_points",True)    
    def generate_uniform_points(self,line_points,center_point = None,min_distance = 3,line_points_num=100):
        # 生成离散点
        # 提取 x 和 y 坐标
        line_points_x = line_points[:, 0]
        line_points_y = line_points[:, 1]


        # 计算沿曲线的累积距离
        line_points_distances = np.sqrt(np.diff(line_points_x) ** 2 + np.diff(line_points_y) ** 2)
        cumulative_distances = np.concatenate(([0], np.cumsum(line_points_distances)))

        # 生成均匀分布的采样点位置
        uniform_distances = np.linspace(0, cumulative_distances[-1], line_points_num)

        # 对 x 和 y 进行插值
        interp_x = interp1d(cumulative_distances, line_points_x)
        interp_y = interp1d(cumulative_distances, line_points_y)

        # 生成均匀分布的离散点
        uniform_x = interp_x(uniform_distances)
        uniform_y = interp_y(uniform_distances)

        # 将 x 和 y 合并为 (n, 2) 形式的数组
        uniform_points = np.column_stack((uniform_x, uniform_y))

        # # 剔除距离中心点小于 min_distance 的点
        # if center_point is not None:
        #     distances_from_center = np.sqrt((uniform_points[:, 0] - center_point[0]) ** 2 +
        #                                     (uniform_points[:, 1] - center_point[1]) ** 2)
        #     uniform_points = uniform_points[distances_from_center >= min_distance]
        return uniform_points
    
    @track_function("generate_insert_line",True)
    def generate_insert_line_cloudPoint(self,start_point,end_point,insert_distance = 30):
        # 生成均匀分布的二维点
        insert_num = max(int(np.linalg.norm(start_point - end_point)/insert_distance) + 2, 2) # 两个 mode 起始点之间插入点数 = tmp_insert_num-2
        temp_points = np.linspace(start_point, end_point, insert_num)
        return np.round(temp_points).astype(int)
    
    @track_function("generate_belly_circles_cloudPoint",True)
    def generate_belly_circles_cloudPoint(self, circles_num_turns, ellipse_a,ellipse_b, tmp_center_point):
        print("mode = circles 从内 到 外螺线运动")
        # 螺旋参数
        # circles_num_turns = 4  # 螺旋圈数

        # 生成螺旋
        circles_theta,circles_r = np.array([]),np.array([])
        for i in range(circles_num_turns):
            tmp_circle_points = int(4 * (2 + i * 1))
            circles_theta = np.concatenate([circles_theta, np.linspace(0, 2 * np.pi, tmp_circle_points)[:-1]])
            circles_r = np.concatenate([circles_r, np.linspace(0.2 + i * 0.8 / circles_num_turns, 0.2 + (i + 1) * 0.8 / circles_num_turns, tmp_circle_points)[:-1]])
        # 为了防止奇点，去除开始的 n 个点
        circles_theta = circles_theta[2:]
        circles_r = circles_r[2:]

        # 初始化有效点的列表
        valid_points_tmp_x = []
        valid_points_tmp_y = []
        circles_total_points = circles_theta.shape[0]
        for i in range(circles_total_points):
            tmp_r, tmp_theta = circles_r[i], circles_theta[i]
            tmp_x = (ellipse_a * tmp_r * np.cos(tmp_theta)) + tmp_center_point[0]
            tmp_y = (ellipse_b * tmp_r * np.sin(tmp_theta)) + tmp_center_point[1]
            # # 检查每个点是否在避开的圆内
            # distance = np.sqrt((tmp_x - tmp_center_point[0]) ** 2 + (tmp_y - tmp_center_point[1]) ** 2)
            # if distance > min_distance:  # 仅保留在外部的点
            valid_points_tmp_x.append(tmp_x)
            valid_points_tmp_y.append(tmp_y)
        # 将有效点保存在 temp_points 中 
        temp_points = np.column_stack((valid_points_tmp_x, valid_points_tmp_y))
        return np.round(temp_points).astype(int)
    
    @track_function("generate_max_ellipse_spiral_cloudPoint",True)
    def generate_max_ellipse_spiral_cloudPoint(self, ellipse_a, ellipse_b, tmp_center_point):

        print("mode = max_ellipse_spiral  围绕外围螺旋线运动")
        # spiral_num最小应>10（建议20<n<50)，否则轨迹为梅花形或效率过低
        spiral_r,spiral_num = 0.15*ellipse_a,40
        
        # 防止超过最大边界
        ellipse_a -= spiral_r
        ellipse_b -= spiral_r
        ellipse_x, ellipse_y = self.generate_ellipse_points(tmp_center_point, ellipse_a, ellipse_b)

        # 生成螺旋线
        for i in range(len(ellipse_x)):
            ellipse_x[i] += spiral_r * np.cos(spiral_num*i * 2*np.pi/len(ellipse_x))
            ellipse_y[i] += spiral_r * np.sin(spiral_num*i * 2*np.pi/len(ellipse_x))
        temp_points = np.array(list(zip(ellipse_x,ellipse_y)))

        temp_points = np.roll(temp_points, shift=int(len(ellipse_x)*1 / 4), axis=0) # 对其上个轨迹末端(lines_1)

        return np.round(temp_points).astype(int)

    @track_function("generate_lines_01_cloudPoint",True)
    def generate_lines_01_cloudPoint(self, tmp_qua_points,tmp_navel_point):    

        print("mode = lines_-01  水平直线往复运动01，左下到左上")
        # 创建离散关键点
        lines_num = 3 # 往复次数
        dim_points = 0.9 # 防止point_start,point_end过于靠外，机械臂无法到达
        line_points_num = 24
        # 左到右
        # 左边端点
        point_start,point_end = dim_points*tmp_qua_points[2] + (1-dim_points)*tmp_qua_points[3], dim_points*tmp_qua_points[0] + (1-dim_points)*tmp_qua_points[1]
        #point_start,point_end = tmp_qua_points[0], tmp_qua_points[2]
        points_left_step = (point_end - point_start) / (lines_num*2)
        tmp_line_left_points = np.array([point_start + i * points_left_step for i in range(lines_num*2 + 1)])
        points_left = tmp_line_left_points[::2]

        # 右边端点
        point_start,point_end = dim_points*tmp_qua_points[3] + (1-dim_points)*tmp_qua_points[2], dim_points*tmp_qua_points[1] + (1-dim_points)*tmp_qua_points[0]
        # point_start,point_end = tmp_qua_points[1], tmp_qua_points[3]
        points_right_step = (point_end - point_start) / (lines_num*2)
        tmp_line_right_points = np.array([point_start + i * points_right_step for i in range(lines_num*2 + 1)])
        points_right = tmp_line_right_points[1::2]

        line_points = []
        for i in range(len(points_right)):
            line_points.append(points_left[i])
            line_points.append(points_right[i])
        line_points.append(tmp_line_left_points[-1])
        line_points = np.array(line_points)
        # 去除距离肚脐过近的点
        temp_points = self.generate_uniform_points(line_points,center_point = tmp_navel_point, min_distance = 3, line_points_num=line_points_num)
        temp_points = temp_points[::-1] # 翻转points列表，轨迹变为左上-左下

        return np.round(temp_points).astype(int)
    
    @track_function("generate_lines_02_cloudPoint",True)
    def generate_lines_02_cloudPoint(self, tmp_qua_points,tmp_navel_point):
        print("mode = lines_-02  水平直线往复运动02,右下到右上")
        # 创建离散关键点
        lines_num = 3 # 往复次数
        dim_points = 0.9 # 防止point_start,point_end过于靠外，机械臂无法到达
        line_points_num = 24
        # 右到左
        # 右边端点
        point_start,point_end = dim_points*tmp_qua_points[3] + (1-dim_points)*tmp_qua_points[2], dim_points*tmp_qua_points[1] + (1-dim_points)*tmp_qua_points[0]
        points_right_step = (point_end - point_start) / (lines_num*2)
        tmp_line_right_points = np.array([point_start + i * points_right_step for i in range(lines_num*2 + 1)])
        points_right = tmp_line_right_points[::2]

        # 左边端点
        point_start,point_end = dim_points*tmp_qua_points[2] + (1-dim_points)*tmp_qua_points[3], dim_points*tmp_qua_points[0] + (1-dim_points)*tmp_qua_points[1]
        points_left_step = (point_end - point_start) / (lines_num*2)
        tmp_line_left_points = np.array([point_start + i * points_left_step for i in range(lines_num*2 + 1)])
        points_left = tmp_line_left_points[1::2]

        line_points = []
        for i in range(len(points_left)):
            line_points.append(points_right[i])
            line_points.append(points_left[i])
        line_points.append(tmp_line_right_points[-1])
        line_points = np.array(line_points)
        # 去除距离肚脐过近的点
        temp_points = self.generate_uniform_points(line_points,center_point = tmp_navel_point, min_distance = 3, line_points_num=line_points_num)
        # temp_points = temp_points[::-1]
        return np.round(temp_points).astype(int)
    
    @track_function("is_point_in_polygon",True)
    def is_point_in_polygon(self, point, vertices):
        # 判断点是否在多边形内部
        x, y = point
        n = len(vertices)

        inside = True

        min_x, min_y, max_x, max_y = np.inf, np.inf, -np.inf, -np.inf
        for v in vertices:
            min_x = min(min_x, v[0])
            min_y = min(min_y, v[1])
            max_x = max(max_x, v[0])
            max_y = max(max_y, v[1])
        if x>=max_x or x<=min_x or y>=max_y or y<=min_y:
            inside = False
        return inside

    @track_function("estimate_max_ellipse",True)
    def estimate_max_ellipse(self,tmp_qua_points,tmp_center_point):
        # 估计最大椭圆a,b
        print("估计最大椭圆")
        ellipse_a = tmp_qua_points[1][0] + tmp_qua_points[3][0] - tmp_qua_points[0][0] - tmp_qua_points[2][0]
        ellipse_b = tmp_qua_points[2][1] + tmp_qua_points[3][1] - tmp_qua_points[0][1] - tmp_qua_points[1][1]
        for _ in range(50):
            # 迭代n次，寻找最大椭圆
            ellipse_inside_flag = 1  # 为1则在内部
            # 生成椭圆离散点
            ellipse_x, ellipse_y = self.generate_ellipse_points(tmp_center_point, ellipse_a, ellipse_b)
            # 判断每个椭圆点是否在四边形内
            for i in range(len(ellipse_x)):
                tmp_point = [ellipse_x[i], ellipse_y[i]]
                is_inside = self.is_point_in_polygon(tmp_point, tmp_qua_points)

                if not is_inside:
                    # 在外部，缩小椭圆
                    ellipse_inside_flag = 0
                    ellipse_a *= 0.95
                    ellipse_b *= 0.95
                    break
            if ellipse_inside_flag:
                # 在内部，放大椭圆
                ellipse_a *= 1.01
                ellipse_b *= 1.01    
                break
        print("估计最大椭圆-完成")
        return ellipse_a,ellipse_b

        
    @track_function("generate_lemniscate_cloudPoint",True)
    def generate_lemniscate_cloudPoint(self, start, end, a_prolate=1.0,radius=30.0, step_degree=np.pi/15,factor = 1.0,num_cycle_factor = 1):
        """
        生成从start到end的8字形插补路径
        
        :param start: 起点坐标 (x0, y0)
        :param end: 终点坐标 (x1, y1)
        :param num_points: 插补点的数量
        :return: 插补路径的坐标列表
        """
        # 起点和终点的坐标
        x1, y1 = start
        x0, y0 = end

        # 计算轨迹的缩放因子和旋转角度
        distance = np.linalg.norm([x1 - x0, y1 - y0])
        temp_radius = distance * factor / 4 
        if radius > temp_radius:
            radius = temp_radius

        a = distance  / 2  # 轨迹的宽度一半
        b = radius
        angle = np.arctan2(y1 - y0, x1 - x0)

        num_points = np.abs((2*num_cycle_factor+1)*np.pi/step_degree).astype(int)
        
        # 生成标准的8字形轨迹 (Gerono卵形线)
        t_values = np.linspace(0.5*np.pi, (2*num_cycle_factor + 1.5)* np.pi, num_points)
        x_curve = a * np.sin(t_values)
        y_curve = b * np.sin(2 * t_values)
        
        # 构建旋转矩阵
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        
        # 旋转并平移轨迹
        curve_points = np.vstack((x_curve, y_curve)).T
        rotated_points = curve_points @ rotation_matrix.T
        shifted_points = rotated_points + [(x0 + x1) / 2, (y0 + y1) / 2]
        
        # 将插值结果四舍五入为整数
        return np.round(shifted_points).astype(int)

    
    @track_function("generate_line_cloudPoint",True)
    def generate_line_cloudPoint(self,start, end, step_distance = 5):
        """
        生成起点和终点之间的线性插值点

        参数:
        start: 起点 (标量或数组，例如 [x1, y1, z1])
        end: 终点 (标量或数组，例如 [x2, y2, z2])
        num_points: 插值点的数量（包括起点和终点）

        返回:
        包含线性插值点的数组
        """
        start = np.array(start)
        end = np.array(end)
        distance = np.linalg.norm(end - start)
        num_points = int(distance / step_distance)
        
        # 使用 numpy 的 linspace 在起点和终点之间生成线性插值点
        interpolated_points = np.linspace(start, end, num_points)
        
        # 将插值结果四舍五入为整数
        return np.round(interpolated_points).astype(int)
    
    @track_function("apply_force",True)
    def apply_force(self, t, wrench, interpolation="linear", algorithm:Literal["admittance","position","hybrid","admithybrid","hybridPid"] = "admittance"):
        """
        施加力
        param:
            t: 时间或时间序列
            wrench(nx6): 力矩或力矩序列,当输入为None时,表示力为[0,0,0,0,0,0]
            is_interrupt: 是否允许中断
            wait: 是否等待运动完成
            timeout: 超时时间
            interpolation: 插值方式
            algorithm: 控制算法
        return:
            code: 0表示成功
                1表示超时
                2表示用户打断
                3表示硬件错误
                4表示参数错误
        """

        if t==0:
            self.arm_state.desired_wrench = wrench
            return 0

        # 在函数内部直接规划力矩轨迹
        traj = self.traj_generate(t, None, wrench, interpolation=interpolation)
        if traj is None:
            return 4

        time_seq = traj['t']
        wrench_traj = traj['wrench']

        try:
            self.controller_manager.switch_controller(algorithm)
        except ValueError as error:
            self.logger.log_error(error)
            return 4

        self.logger.log_info("切换到%s控制器" % algorithm)

        for i in range(len(time_seq)):
            if self.interrupt_envet.is_set():
                self.interrupt_envet.clear()
                return 2
            if self.arm.is_exit or self.exit_event.is_set():
                return 3
            if self.adjust_wrench_envent.is_set():
                # if wrench_traj is not None:
                #     wrench_traj[i:] = self.adjust_wrench
                self.adjust_wrench_envent.clear()
            # print(wrench_traj[i])
            self.arm_state.desired_wrench = wrench_traj[i] if wrench_traj is not None else np.zeros(6, dtype=np.float64)
            self.command_rate.sleep()

        return 0

    
    @track_function("move_circle",True)
    def move_circle(self,t,center = None,wrench = None,omega = 0.2,radius=0.025,reversed = False,is_interrupt = True,
                    algorithm:Literal["admittance","position","hybrid"] = "admittance"):
        
        if center is None:
            center = self.read_pose()
        traj = self.traj_generate(t,pose=center,wrench=wrench,interpolation='circle',omega=omega,radius=radius,reverse=reversed)
        time_seq = traj['t']
        pos_traj = traj['pos']
        quat_traj = traj['quat']
        wrench_traj = traj['wrench']
        try:
            self.controller_manager.switch_controller(algorithm)
        except ValueError as error:
            self.logger.log_error(error)
            return 4
        self.logger.log_info("切换到%s控制器"%algorithm)
        for i in range(len(time_seq)):
            if self.interrupt_envet.is_set():
                self.interrupt_envet.clear()
                return 2
            if self.arm.is_exit or self.exit_event.is_set():
                return 3
            if self.adjust_pos_envent.is_set():
                if is_interrupt:
                    self.adjust_pos_envent.clear()
                    return 2
                else:
                    if pos_traj is not None: 
                        pos_traj[i:] = pos_traj[i:] + self.pos_increment
                    self.adjust_pos_envent.clear()
            if self.adjust_wrench_envent.is_set():
                if wrench_traj is not None:
                    wrench_traj[i:] = self.adjust_wrench
                self.adjust_wrench_envent.clear()

            if pos_traj is not None and quat_traj is not None:
                # print(pos_traj[i],quat_traj[i])
                self.arm_state.desired_position = pos_traj[i]
                self.arm_state.desired_orientation = quat_traj[i]
            # 没有输入力控指令，期望力修改为默认
            if wrench_traj is not None:
                self.arm_state.desired_wrench = wrench_traj[i]
            else:
                self.arm_state.desired_wrench = np.zeros(6,dtype=np.float64)
            self.command_rate.sleep()
        return 0
      
    def read_pose(self):
        world_pos = self.arm_state.arm_position
        world_euler = R.from_quat(self.arm_state.arm_orientation).as_euler('xyz',degrees=False)
        return np.concatenate([world_pos,world_euler])

    def user_adjust(self,pose_increment = None,force = None,temperature = None, gear = None,press = None,frequency = None):
        self.logger.log_info("用户调整")
        if pose_increment is not None:
            pass
            # self.pos_increment = np.array(pose_increment)
            # self.adjust_pos_envent.set()
        if force is not None:
            self.adjust_wrench = np.array(force)
            self.adjust_wrench_envent.set()
        if temperature is not None:
            self.thermotherapy.set_working_status(upper_temp=temperature)
        if gear is not None:
            self.thermotherapy.set_working_status(level=gear)
        if press is not None:
            self.shockwave.p_set(level=press)
        if frequency is not None:
            self.shockwave.f_set(level=frequency)

    def user_interrupt(self):
        self.logger.log_info("用户打断")
        self.interrupt_envet.set()
    
    # 工具函数
    ############################################################################################################
    def convert_to_7d(self,matrix):
        matrix = np.array(matrix)  
        positions = matrix[:, :3]  
        rpy_angles = matrix[:, 3:]  
        # 将RPY角度转换为四元数
        quaternions = R.from_euler('xyz', rpy_angles).as_quat()
        # 将位置和四元数组合成一个新的矩阵
        result_matrix = np.hstack([positions, quaternions])
        # 加入当前位置
        current_quat = self.arm_state.arm_orientation
        current_position  = self.arm_state.arm_position
        if quaternions[0].dot(current_quat) < 0:
            current_quat = -current_quat
        current_pose = np.hstack([current_position,current_quat])
        result_matrix = np.vstack([current_pose,result_matrix])
        return result_matrix

    def is_arrivated(self,position_tolerance = 0.01,orientation_tolerance = 0.05):
        # 计算位置误差
        position_error = np.linalg.norm(np.array(self.arm_state.arm_position) - np.array(self.arm_state.desired_position))
         # 计算两个四元数的差异
        q_error = R.from_quat(self.arm_state.arm_orientation).inv() * R.from_quat(self.arm_state.desired_orientation)
        # 提取误差角度
        angle = q_error.magnitude()
        # 判断是否在容忍范围内
        if position_error <= position_tolerance and angle <= orientation_tolerance:
            return True
        else:
            return False
    
    def traj_generate(self, t: Union[int, float, List[float]], pose = None, wrench = None, interpolation: Literal["linear", "cubic","circle","cloud_point"] = 'linear',**kwargs):
        """
        轨迹生成
        param:
            t: 时间或时间序列
            pose(nx6): 世界坐标系下的位姿或位姿序列,当输入为None时,表示保持当前位姿
            wrench(nx6): 力矩或力矩序列,当输入为None时,表示保持当前的力
            interpolation: 插值方式
        return:
            traj: 生成的轨迹 -> dict
        """
        # 确保时间输入是列表
        if isinstance(t, (int, float)):
            t = [t]
        elif not isinstance(t, list) or any(not isinstance(i, (int, float)) for i in t):
            self.logger.log_error("not isinstance(t, list) or any(not isinstance(i, (int, float)) for i in t)")
            return None
        # 检查时间点数量
        if len(t) < 1:
            self.logger.log_error("len(t) < 1")
            return None
        # 时间序列
        time_points = [0.0] + t
        time_points = np.array(time_points)
        dt = self.command_rate.to_sec()

        for i in range(len(time_points)):
            time_points[i] = np.round(time_points[i] / dt) * dt
        if np.abs(time_points[-1]) < dt:
            time_points[-1] = dt
        # 由于浮点数精度问题，time_step要乘0.9
        time_seq = np.arange(time_points[0], time_points[-1] + dt * 0.9, dt)
        pos_traj = None
        quat_traj = None
        wrench_traj = None
        pos_vel = None
        angular_vel = None
        # 位置轨迹与速度轨迹生成
        if pose is not None:
            if interpolation != 'cloud_point':
                pose = np.array(pose).reshape(-1, 6)
                if pose.shape[0] != len(t):
                    self.logger.log_error("pose.shape[0] != len(t)")
                    print(pose.shape[0],len(t))
                    return None
                pose_matrix = self.convert_to_7d(pose)
            else:
                pose = np.array(pose)
            try:
                # 插值
                if interpolation == 'linear':
                    pos_traj, quat_traj = linear_interpolate(positions=pose_matrix[:, :3], quaternions=pose_matrix[:, 3:], time_points=time_points, time_step=dt)
                elif interpolation == 'cubic':
                    pos_traj, quat_traj = spline_interpolate(positions=pose_matrix[:, :3], quaternions=pose_matrix[:, 3:], time_points=time_points, time_step=dt)
                elif interpolation == 'circle':
                    pos_traj = circle_trajectory(center=pose,time_points=time_points,time_step=dt,**kwargs)
                    quat_traj = np.tile(R.from_euler('xyz', np.array(pose[0][3:])).as_quat(), (pos_traj.shape[0], 1))
                elif interpolation == 'cloud_point':
                    pos_traj, quat_traj = cloud_point_interpolate(positions=pose[:, :3], quaternions=pose[:, 3:], time_points=time_points, time_step=dt)
                    # self.logger.log_yellow(f'{pos_traj[0]} {pos_traj[-1]}')
            except ValueError as error:
                self.logger.log_error(error)
                return None
        else:
            self.logger.log_error("未输入位置进行规划")

        # TODO 力轨迹生成 改为线性增加 
        if wrench is not None:
            wrench = np.array(wrench).reshape(-1, 6)
            if wrench.shape[0] != len(t):
                return None
            indices = np.searchsorted(time_points, time_seq, side='left') - 1
            indices = np.clip(indices, 0, len(wrench) - 1)
            wrench_traj = wrench[indices]
        # 构建返回数据字典
        traj = {
            't': time_seq,
            'pos': pos_traj,
            'quat': quat_traj,
            'pos_vel': pos_vel,
            'angular_vel': angular_vel,
            'wrench': wrench_traj
        }
        return traj
    
    @staticmethod
    def wrench_coordinate_conversion(tf_matrix, wrench):
        """
        Convert wrench from current frame to other frame
        """
        rot_matrix = tf_matrix[:3, :3]
        vector_p = tf_matrix[:3, 3]
        skew_matrix = np.array([[0, -vector_p[2], vector_p[1]],
                                [vector_p[2], 0, -vector_p[0]],
                                [-vector_p[1], vector_p[0], 0]])
        temp_force = wrench[:3]
        torque = wrench[3:]

        force = rot_matrix.T @ temp_force
        torque = rot_matrix.T @ np.cross(temp_force,vector_p) + rot_matrix.T @ torque
        return np.concatenate([force, torque])
    
    @staticmethod
    def get_tf_matrix(position, orientation):
        """
        Get transformation matrix from position and orientation
        :param position: np.ndarray [x,y,z]
        :param orientation: np.ndarray [qx,qy,qz,qw]
        """
        tf_matrix = np.eye(4)
        rotation_matrix = R.from_quat(orientation).as_matrix()
        tf_matrix[:3, 3] = position
        tf_matrix[:3, :3] = rotation_matrix
        return tf_matrix
    
    # 仿真
    ############################################################################################################
    def figure_update(self,frame,arrow_vec,ax,bed_corners):
        ax.clear()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Position Trajectory')

        ax.set_xlim([-0.7, 1.5])
        ax.set_ylim([-0.2, 2])
        ax.set_zlim([-2, 0.2])

        # 旋转坐标系，绕 x 轴旋转 180 度
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])

        y_offset = self.rail_pos
        current_position = self.arm_state.arm_position
        current_quaternion = self.arm_state.arm_orientation

        # 更新动态坐标系的原点
        dynamic_origin = np.array([0, -y_offset ,0])
        # 将动态坐标系中的点转换为世界坐标系
        pos = np.dot(rotation_matrix, current_position + dynamic_origin) 
        quat = current_quaternion
        rotation = R.from_quat(quat)
        rotated_vec = rotation.apply(arrow_vec)
        rotated_vec = np.dot(rotation_matrix, rotated_vec)
        # print("导轨期望位置:",self.desired_rail_pos)
        # print("导轨位置:",y_offset)
        # print("机械臂期望坐标：",self.arm_state.desired_position)
        # print("机械臂坐标：",current_position)
        # print("机械臂期望姿态:",R.from_quat(self.arm_state.desired_orientation).as_euler('xyz',degrees=False))
        # print("机械臂姿态:",R.from_quat(quat).as_euler('xyz',degrees=False))
        # print("世界坐标：",pos)
        ax.scatter(pos[0], pos[1], pos[2], color='b', s=50)
        ax.quiver(pos[0], pos[1], pos[2], rotated_vec[0], rotated_vec[1], rotated_vec[2], length=0.6, color='r')

        # 绘制动态坐标系
        x_axis = dynamic_origin + np.array([1, 0, 0])
        y_axis = dynamic_origin + np.array([0, 1, 0])
        z_axis = dynamic_origin + np.array([0, 0, 1])

        dynamic_origin_rot = np.dot(rotation_matrix, dynamic_origin)
       
        x_axis_rot = np.dot(rotation_matrix, x_axis)
        y_axis_rot = np.dot(rotation_matrix, y_axis)
        z_axis_rot = np.dot(rotation_matrix, z_axis)

        ax.quiver(*dynamic_origin_rot, *(x_axis_rot - dynamic_origin_rot), color='r', length=0.4, normalize=True, arrow_length_ratio=0.2)
        ax.quiver(*dynamic_origin_rot, *(y_axis_rot - dynamic_origin_rot), color='g', length=0.4, normalize=True, arrow_length_ratio=0.2)
        ax.quiver(*dynamic_origin_rot, *(z_axis_rot - dynamic_origin_rot), color='b', length=0.4, normalize=True, arrow_length_ratio=0.2)

        # 绘制床
        ax.plot(bed_corners[:, 0], bed_corners[:, 1], bed_corners[:, 2], color='brown')

    def simulation(self,interval=10):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        arrow_vec = np.array([0, 0, 1])
        # 定义床的角点，床是静态的
        bed_corners = np.array([
            [0, 1.8, -1],  # 左上角
            [0, 0, -1],  # 右上角
            [0.7, 0 , -1],  # 右下角
            [0.7, 1.8, -1],  # 左下角
            [0, 1.8, -1]  # 返回左上角
        ])
        bed_corners += np.array([-0.19,-0.28,0])
        ani = FuncAnimation(fig, self.figure_update, frames=200, fargs=(arrow_vec, ax, bed_corners), interval=interval)
        plt.show()

    ############################################################################################################
        
    # TODO 日志 可以关闭
    def log_thread(self):
        while True:
            self.logger_data.log_info(f"机械臂位置:{self.arm_state.arm_position},机械臂姿态:{self.arm_state.arm_orientation}",is_print=False)
            self.logger_data.log_info(f"机械臂期望位置:{self.arm_state.desired_position},机械臂真实位置:{self.arm_state.arm_position}",is_print=False)
            self.logger_data.log_info(f"机械臂期望姿态:{self.arm_state.desired_orientation},机械臂真实姿态:{self.arm_state.arm_orientation}",is_print=False)
            self.logger_data.log_info(f"机械臂期望力矩:{self.arm_state.desired_wrench},机械臂真实力矩:{self.arm_state.external_wrench_base}",is_print=False)
            self.logger_data.log_info(f"当前按摩头:{self.current_head}",is_print=False)

            time.sleep(1)



        

    

if __name__ == "__main__":

    import signal
    import copy

    def signal_handler(signum, frame):
        robot.stop()

    robot = MassageRobot('MassageControl/config/robot_config.yaml')
   
    signal.signal(signal.SIGINT, signal_handler)

    # robot.init_hardwares([ 0.1215, 0.25739308, 0.63318263, 150 * (np.pi / 180), 0, 180 * (np.pi / 180)])
    
    robot.init_hardwares([ 0.31892, 8.8e-05, 0.367243, 3.141, 0.0, 2.094])
    # robot.move_to_points(1.247456,pose=[0.247,0.1043,0.761,0,0,0],is_interrupt=False)
    # traj = robot.traj_generate(1.247456,pose=[0.247,0.1043,0.761,0,0,0],interpolation='linear')
    # print(traj)
    # print(traj)
    # robot.controller_manager.switch_controller('hybrid')
    robot.sensor_set_zero()
    robot.sensor_enable()
    # robot.switch_payload('thermotherapy_head')
    # robot.switch_payload('shockwave_head')
    robot.switch_payload('realman_test')
    # robot.switch_payload('wash_head')

    # robot.controller_manager.switch_controller('admittance')ss
    robot.controller_manager.switch_controller('hybridPid')
    robot.arm_state.desired_wrench = np.array([0,0, -5,0,0,0])
    # 创建并启动一个新线程来运行robot.start()
    robot_thread = threading.Thread(target=robot.start)
    robot_thread.start()

    # time.sleep(5)
    # robot.switch_payload('wash_head')
    # robot.is_waitting = False
    # robot.arm_state.desired_wrench = np.array([0,0,-10,0,0,0])
    robot.controller_manager.switch_controller('hybrid')
    # robot.controller_manager.switch_controller('admittance')
    # robot.switch_payload('rotate_head')
    # print("1")
    # robot.is_waitting = True
    # # robot.arm.is_standby = False
    # pose = copy.deepcopy(robot.arm.standby_pos)
    # pose[4] = -90 * (np.pi / 180)
    # time.sleep(1)
    # robot.arm.move_joint(pose,5)
    # time.sleep(1)
    # print(robot.arm.get_arm_position())
    # time.sleep(10)
    # pose = copy.deepcopy(robot.arm.standby_pos)
    # robot.arm.move_joint(pose,5)
    # time.sleep(1)
    # # robot.arm.is_standby = True
    # robot.is_waitting = False
    # print("3")
    # robot.move_circle(t=30,center=[0.2008, -0.1215, 0.7472, np.pi/4*3, 0, np.pi/2],algorithm='position')
    # while True:
    #     print(robot.arm_state.external_wrench_tcp)
    #     time.sleep(1)
    # print("4")

    # print("moveing")
    # robot.move_to_points(5,pose=[0.20330386, -0.11231608  ,0.74970386, np.pi/4*3, 0, np.pi/2],is_interrupt=False)
    # print("3",robot.arm.get_arm_position())
    robot_thread.join()