import sys
import os

# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))

# 获取项目根目录的绝对路径
project_root = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(project_root)
# import pyaubo_sdk
from tools.Rate import Rate
from scipy.spatial.transform import Rotation as R
from tools.log import CustomLogger
from tools.yaml_operator import read_yaml
import threading
from tools.bytes_transform import *
from .aubo_message import *
import socket
import numpy as np
import atexit
import time
import requests

import copy

class AuboC5():
    def __init__(self, arm_ip = "192.168.100.20", arm_port = 30004):
        self.arm_ip = arm_ip
        self.arm_port = arm_port
        self.logger = CustomLogger('aubo_C5',True)

        self.robot_rpc_client = pyaubo_sdk.RpcClient() 
        self.robot_rpc_client.setRequestTimeout(1000)  # 接口调用: 设置 RPC 超时
        self.robot_rpc_client.connect(self.arm_ip, self.arm_port)  # 连接 RPC 服务
        if self.robot_rpc_client.hasConnected():
            print("Robot rpc_client connected successfully!")
            self.robot_rpc_client.login("aubo", "123456")  # 登录机械臂
            if self.robot_rpc_client.hasLogined():
                print("Robot rpc_client logged in successfully!")
                self.robot_name = self.robot_rpc_client.getRobotNames()[0]  # 接口调用: 获取机器人的名字
                self.robot = self.robot_rpc_client.getRobotInterface(self.robot_name)
                self.robot_config = self.robot.getRobotConfig()
                # 获取状态
                robot_mode_type, safety_mode_type = self.getModelType()
                if safety_mode_type == 8:
                    requests.post(
                        "http://127.0.0.1:5000/on_message", data={"message": "机械臂处于锁定状态<br>请解除锁定后再使用"}
                    )
                    instruction = {
                        "is_massaging": False,
                        "massage_service_started": False
                    }
                    requests.post(
                        "http://127.0.0.1:5000/update_massage_status", data=instruction
                    )
                    sys.exit(0)
                if safety_mode_type == 5:
                    self.robot.getRobotManage().setUnlockProtectiveStop()    # 接口调用: 解除保护停止
                if safety_mode_type in [3,4]:
                    self.power_off()
                    time.sleep(1)
                # 启动
                self.start_up()
                # 接口调用: 设置工具中心点（TCP相对于法兰盘中心的偏移）
                tcp_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.robot_config.setTcpOffset(tcp_offset)
                self.mc = self.robot.getMotionControl()
                # 接口调用: 设置机械臂的速度比率
                self.mc.setSpeedFraction(1)
        self.robot_name = self.robot_rpc_client.getRobotNames()[0]
        self.robot_rpc_client.getRuntimeMachine().stop()
        self.robot_rpc_client.getRuntimeMachine().start()
        
        self.logger.log_info(f"连接到机械臂{arm_ip}")
        # 退出任务
        atexit.register(self.exit_task)

        self.standby_pos = [90 * (np.pi / 180), 30 * (np.pi / 180), 90 * (np.pi / 180),
                     0 * (np.pi / 180), 90 * (np.pi / 180), 0.0 * (np.pi / 180)
                    ]
        self.init_pos = [90 * (np.pi / 180), 70 * (np.pi / 180), 130 * (np.pi / 180),
                     0 * (np.pi / 180), 90 * (np.pi / 180), 0.0 * (np.pi / 180)
                    ]
        self.off_pos = [90 * (np.pi / 180), 90 * (np.pi / 180), 150 * (np.pi / 180),
                     0 * (np.pi / 180), 90 * (np.pi / 180), 0.0 * (np.pi / 180)
                    ]

    def init(self):
        self.is_exit = False
        self.traj_list = None
        self.last_pose_command = np.zeros(6)
               
               
        self.last_input_command = None
        self.tcp_offset = None
        
        time.sleep(2)
        self.is_standby = False
        self.disable_servo()
        code = self.move_joint(self.init_pos,5)
        if code == -1:
            self.logger.log_error("机械臂初始化失败")
            requests.post(
                "http://127.0.0.1:5000/on_message", data={"message": "机械臂初始化失败"}
            )
            self.is_exit = True
            sys.exit(0)
        self.is_standby = True
        
        self.last_print_time=0
        self.last_reocrd_time =0

    def exit_task(self):
        self.is_exit = True
        self.logger.log_yellow("退出任务")
        self.disable_servo()
        time.sleep(1)
        self.move_joint(self.off_pos,wait=True)
        instruction = {
            "is_massaging": False,
            "massage_service_started": False
        }
        requests.post(
            "http://127.0.0.1:5000/update_massage_status", data=instruction
        )

    def getModelType(self):
        robot_mode_type = self.robot.getRobotState().getRobotModeType()
        print("机器人的模式状态:", robot_mode_type)
        print(robot_mode_types.get(robot_mode_type.value, "未知模式"))
        # 接口调用: 获取安全模式
        safety_mode_type = self.robot.getRobotState().getSafetyModeType()
        print("安全模式:", safety_mode_type)
        print(safety_mode_types.get(safety_mode_type.value, "未知模式"))
        return robot_mode_type.value, safety_mode_type.value

    def start_up(self):
        if 0 == self.robot_rpc_client.getRobotInterface(
                self.robot_name).getRobotManage().poweron():  # 接口调用: 发起机器人上电请求
            print("The robot is requesting power-on!")
            if 0 == self.robot_rpc_client.getRobotInterface(
                    self.robot_name).getRobotManage().startup():  # 接口调用: 发起机器人启动请求
                print("The robot is requesting startup!")
                # 循环直至机械臂松刹车成功
                while 1:
                    robot_mode = self.robot_rpc_client.getRobotInterface(self.robot_name) \
                        .getRobotState().getRobotModeType()  # 接口调用: 获取机器人的模式类型
                    print("Robot current mode: %s" % (robot_mode.name))
                    if robot_mode == pyaubo_sdk.RobotModeType.Running:
                        break
                    time.sleep(1)
    
    def power_off(self):
        robot_name = self.robot_rpc_client.getRobotNames()[0]  # 接口调用: 获取机器人的名字
        if 0 == self.robot_rpc_client.getRobotInterface(
                robot_name).getRobotManage().poweroff(): # 接口调用: 机械臂断电
            print("The robot is requesting power-off!")

    def enable_servo(self):
        # self.robot_rpc_client.getRuntimeMachine().start()
        # 关闭 servo 模式
        # self.mc.setServoMode(False)
        # print("Servoj end")
        # 开启 servo 模式
        self.mc.setServoMode(True)
        i = 0
        while not self.mc.isServoModeEnabled():
            i = i + 1
            if i > 5:
                print("Servo Mode is ", self.mc.isServoModeEnabled())
                return -1
            time.sleep(0.005)
        print("Servoj Enabled")

    def disable_servo(self):
        self.mc.setServoMode(False)
        print("Servoj Disabled")

    def get_servo_error_message(self,code,success_info = False,failure_info = True):
        if code == 0 and success_info:
            self.logger.log_info(f'ServoJ code: {code},' +servo_error_codes.get(code, "未知错误码"))
        elif code != 0:
            if code != 2:
                self.logger.log_error(f"Servo code: {code} error code, "+ servo_error_codes.get(code, "未知错误码")   )
            if failure_info:
                if code == 2:
                    # self.logger.log_error(f"Servo failed with code: {code}, "+servo_error_codes.get(code, "未知错误码"))
                    pass
                else:
                    self.logger.log_error(f"Servo failed with code: {code}, "+servo_error_codes.get(code, "未知错误码"))
            if code == 1:
                self.power_off()
                self.logger.log_error("机械臂处于非 Normal、ReducedMode、Recovery 状态，机械臂已断电")
            elif code == -13:
                self.logger.log_error("Servo mode disabled, re-enable servo mode")
                self.enable_servo()
                
    def get_inverse_kinematics_error_message(self,code,cur, pose_command,success_info = False,failure_info = True):
        if code == 0 and success_info: 
            self.logger.log_info(f'InverseKinematics code: {code},'+inverse_kinematics_errors.get(code, "未知错误码"))
        elif code != 0:
            if failure_info:
                self.logger.log_error(f'InverseKinematics failed with code: {code},'+inverse_kinematics_errors.get(code, "未知错误码"))
                self.logger.log_error(f'Current joint position: {cur}')
                self.logger.log_error(f'Pose command: {pose_command}')
    
    def get_move_error_message(self,code,success_info = False,failure_info = True):
        if code == 0 and success_info:
            self.logger.log_info(f'Move code: {code},'+move_error_codes.get(code, "未知错误码"))
        elif code != 0:
            if failure_info:
                self.logger.log_error(f"Move failed with code: {code},"+move_error_codes.get(code, "未知错误码"))
            if code == 1:
                self.power_off()
                self.logger.log_error("机械臂处于非 Normal、ReducedMode、Recovery 状态，机械臂已断电")

    def send_command(self, arm_position_command, arm_orientation_command):
        rot_euler = R.from_quat(arm_orientation_command).as_euler('xyz', degrees=False)
        rot_euler = np.clip(rot_euler,-3.1415,3.1415) #将发送姿态限制在正负180度以内
        pose_command = np.concatenate([arm_position_command, rot_euler])

        # 获取机械臂的模式类型
        robot_mode_type = self.robot.getRobotState().getRobotModeType()
        # print("robot_mode_type: ", type(robot_mode_type))
        # 获取安全模式类型
        safety_mode_type = self.robot.getRobotState().getSafetyModeType()
        # print("safety_mode_type: ", type(safety_mode_type))
        # 修改判断方式以符合枚举类型
        if robot_mode_type in [pyaubo_sdk.RobotModeType.Disconnected,
                            pyaubo_sdk.RobotModeType.PowerOff,
                            pyaubo_sdk.RobotModeType.BackDrive,
                            pyaubo_sdk.RobotModeType.Error]:
            self.logger.log_error(f"机械臂模式错误: code = {robot_mode_type}")
            print(f"send_position: {pose_command}")
            pose = self.robot_rpc_client.getRobotInterface(self.robot_name).getRobotState().getTcpPose()
            print(f"current position: {pose}")
            print
            self.power_off()
            self.is_exit = True
            return -1

        if safety_mode_type in [pyaubo_sdk.SafetyModeType.Undefined,
                                pyaubo_sdk.SafetyModeType.Recovery,
                                pyaubo_sdk.SafetyModeType.Violation,
                                pyaubo_sdk.SafetyModeType.ProtectiveStop,
                                pyaubo_sdk.SafetyModeType.SafeguardStop,
                                pyaubo_sdk.SafetyModeType.SystemEmergencyStop,
                                pyaubo_sdk.SafetyModeType.RobotEmergencyStop,
                                pyaubo_sdk.SafetyModeType.Fault]:
            self.logger.log_error(f"机械臂安全模式错误: code = {safety_mode_type}")
            print(f"send_position: {pose_command}")
            pose = self.robot_rpc_client.getRobotInterface(self.robot_name).getRobotState().getTcpPose()
            print(f"current position: {pose}")
            self.power_off()
            self.is_exit = True
            return -1
        
        if not self.is_exit and self.is_standby: 
            cur = self.robot.getRobotState().getJointPositions() # 接口调用: 获取机器人的关节位置
            res,code = self.robot.getRobotAlgorithm().inverseKinematics(cur, pose_command)
            self.get_inverse_kinematics_error_message(code,cur, pose_command)
            if code == 0:
                code = self.mc.servoJoint(res, 0, 0, 0.007, 0.0, 0.0) #0.007
                self.get_servo_error_message(code,False,True)
                if code == -5:
                    self.logger.log_yellow(f"cur_joint_position: {cur}")
                    self.logger.log_yellow(f"res_joint_position: {res}")
                    self.logger.log_yellow(f"pose_command: {pose_command}")
                    position ,quat_rot = self.get_arm_position()
                    self.logger.log_yellow(f"position: {position}")
                    self.logger.log_yellow(f"rotaion: {R.from_quat(quat_rot).as_euler('xyz', degrees=False)}")
            else:
                print("inverse_error_arm_is_exit")
                self.power_off()
                self.is_exit = True
                return -1
    #获取当前TCP位姿
    def get_arm_position(self):
        pose = self.robot_rpc_client.getRobotInterface(self.robot_name).getRobotState().getTcpPose()
        x, y, z, roll, pitch, yaw = pose # x, y, z unit: m
        position = np.array([x, y, z])
        quat_rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()

        return position,quat_rot
    #工具端的位姿，不带TCP偏移
    def get_end_position(self):
        pose = self.robot_rpc_client.getRobotInterface(self.robot_name).getRobotState().getToolPose()
        x, y, z, roll, pitch, yaw = pose # x, y, z unit: m
        print(roll, pitch, yaw)
        position = np.array([x, y, z])
        quat_rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()

        return position,quat_rot
    
    def move_joint(self, q ,max_retry_count = 3, wait = False):
        cnt = 0
        while not self.is_exit or wait:
            cnt += 1
            if cnt > max_retry_count:
                print(f'Failed to arrive at the joint position: {q}')
                return -1
            self.mc.moveJoint(q,  5 * (np.pi / 180), 500 * (np.pi / 180), 0, 0)

            is_arrived = self.waitArrival()
            if  is_arrived == -1:
                self.robot_rpc_client.getRuntimeMachine().stop()
                self.robot_rpc_client.getRuntimeMachine().start()
                time.sleep(0.1)
                continue
            elif is_arrived == -2:
                print(f'Failed to arrive at the joint position: {q}')
                return -1
            else:
                print(f'Arrived at the joint position: {q}')
                return 0
        print(f'Failed to arrive at the joint position: {q}')
        

    def waitArrival(self,max_retry_count = 3):
        cnt = 0
        while self.mc.getExecId() == -1:
            cnt += 1
            if cnt > max_retry_count:
                print("Motion fail!")
                return -1
            time.sleep(0.05)
            print("getExecId: ", self.mc.getExecId())
        id = self.mc.getExecId()
        while True:
            id1 = self.mc.getExecId()
            # 获取机械臂的模式类型
            robot_mode_type = self.robot.getRobotState().getRobotModeType()
            # print("robot_mode_type: ", type(robot_mode_type))

            # 获取安全模式类型
            safety_mode_type = self.robot.getRobotState().getSafetyModeType()
            # print("safety_mode_type: ", type(safety_mode_type))

            # 假设 pyaubo_sdk.RobotModeType 和 pyaubo_sdk.SafetyModeType 是枚举类型
            # 将错误的模式类型代码列出来进行比较

            # 修改判断方式以符合枚举类型
            if robot_mode_type in [pyaubo_sdk.RobotModeType.Disconnected,
                                pyaubo_sdk.RobotModeType.PowerOff,
                                pyaubo_sdk.RobotModeType.BackDrive,
                                pyaubo_sdk.RobotModeType.Error]:
                self.logger.log_error(f"机械臂模式错误: code = {robot_mode_type}")
                self.power_off()
                return -2

            if safety_mode_type in [pyaubo_sdk.SafetyModeType.Undefined,
                                    pyaubo_sdk.SafetyModeType.Recovery,
                                    pyaubo_sdk.SafetyModeType.Violation,
                                    pyaubo_sdk.SafetyModeType.ProtectiveStop,
                                    pyaubo_sdk.SafetyModeType.SafeguardStop,
                                    pyaubo_sdk.SafetyModeType.SystemEmergencyStop,
                                    pyaubo_sdk.SafetyModeType.RobotEmergencyStop,
                                    pyaubo_sdk.SafetyModeType.Fault]:
                self.logger.log_error(f"机械臂安全模式错误: code = {safety_mode_type}")
                self.power_off()
                return -2
            if id != id1:
                break
            time.sleep(0.05)
        return 0
    
    def change_end_effector(self,end_effector_config):
        type_name = end_effector_config["tool_type"]
        # 设置tcp偏移
        tcp_offset = end_effector_config["tcp_offset"]
        self.tcp_offset = tcp_offset
        # current_pos,current_rot = self.get_arm_position()
        # R_matrix = R.from_quat(current_rot).as_matrix()
        # tcp_position =  R_matrix.T @ tcp_offset[:3]
        # tcp_orientation=  R_matrix.T @  R_matrix @  R.from_euler('xyz',tcp_offset[3:],degrees=False).as_matrix()
        # tcp_orientation_euler = R.from_matrix(tcp_orientation).as_euler('xyz',degrees=False)
        # tcp_offset = np.concatenate([[0,0,0.27],[0,0,0]]).tolist()
        print(tcp_offset)
        while self.robot_config.getTcpOffset() != tcp_offset:
            self.robot_config.setTcpOffset(tcp_offset)
            time.sleep(0.5)
        # 设置负载
        # total_mass = end_effector_config["sensor_mass"]+end_effector_config["tool_mass"]
        # mass_center_position = [element*1000 for element in end_effector_config["mass_center_position"]]
        # aom = [0.0, 0.0, 0.0]
        # inertia = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # while self.robot_config.getPayload() != (total_mass, mass_center_position, aom, inertia):
        #     result = self.robot_config.setPayload(total_mass, mass_center_position, aom, inertia)
        #     time.sleep(0.5)
        self.logger.log_info(f"修改为{end_effector_config['name']}")
    
if __name__ == '__main__':
    auboC5 = AuboC5()