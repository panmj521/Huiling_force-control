#! /usr/bin/env python
# coding=utf-8

import sys
import os
# 获取当前文件的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取项目根目录的绝对路径
project_root = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(project_root)
"""
机械臂上电与断电

步骤:
第一步: 机械臂初始化
第二步: 机械臂上电
第三步: 机械臂断电
"""
import time
from Robotic_Arm.rm_robot_interface import *
import numpy as np
from tools.log import CustomLogger
# from Language.tools.log import CustomLogger
try:
    from .realman_message import *
except:
    from realman_message import *
from scipy.spatial.transform import Rotation as R




class RealmanRM63():
    def __init__(self,arm_ip = "192.168.1.18", arm_port = 8080):
        # 初始化机械臂
        self.arm_ip = arm_ip
        self.arm_port = arm_port
        self.logger = CustomLogger('Realman',True)

        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm(self.arm_ip, self.arm_port)
        print("机械臂ID:", self.handle.id)

        # 初始化参数
        self.init_pos = [0, -0.3, 0.35, 3.14, 0.0, -1.57]
        # self.init_joint = [90, 0.0, 90, 0, 90, 0]
        # force_init = [0, 0, 90, 0, 90, 60]
        self.init_joint = [0.0, -32.8, 114.3, 0.0, 98.5, 90.0]
        # [0, 0, 90, 0, 90, 90]

        
        # self.tool_frame = {"tool_type":"test", "name":123, "tcp_offset":[0, 0, 1.1, 0, 0, 0], 
        #                    "frame_payload":0, "frame_gravity_center":[0,0,0]}
        self.arm_model = rm_robot_arm_model_e.RM_MODEL_RM_63_II_E  # RM_65机械臂
        self.force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 标准版
        # 初始化算法的机械臂及末端型号
        self.algo_handle = Algo(self.arm_model, self.force_type)

        max_retry_num, retry_time = 30,1
        cnt_num = 0
        while cnt_num < max_retry_num:
            # 检测连接并启动机械臂
            self.software_info = self.robot.rm_get_arm_software_info()
            code = self.software_info[0]
            if code == 0:
                self.check_error_state(self.robot.rm_get_current_arm_state()[0])
                self.logger.log_info(f"连接到机械臂{self.arm_ip}  ID:{self.handle.id}")
                # 启动
                tmp_power_state = self.robot.rm_get_arm_power_state()[1]
                if tmp_power_state == 0:
                # if self.check_power_state() == 0:
                    # 上电
                    self.start_up()
                    time.sleep(2)
                self.check_error_state(self.robot.rm_set_tool_voltage(3)) # 设置末端IO接口电压（0/12/24V）

                # 设置UDP端口，广播周期500ms，使能，广播端口号8089，力数据坐标系使用传感器坐标系，上报目标IP为"192.168.1.104"
                # 自定义上报项均设置，1为启用上报
                self.custom = rm_udp_custom_config_t()
                self.custom.joint_speed = 0
                self.custom.lift_state = 0
                self.custom.expand_state = 0
                self.custom.arm_current_status = 0
                self.custom.hand_state = 0
                self.custom.aloha_state = 0

                self.force_coordinate = 0 # 0：传感器坐标系；1：当前工作坐标系；2：当前工具坐标系。
                self.custom_config = rm_realtime_push_config_t(5, True, 8089, self.force_coordinate, "192.168.1.104", self.custom)
                self.check_error_state(self.robot.rm_set_realtime_push(self.custom_config))

                # 机械臂 UDP主动上报
                self.arm_state_callback = rm_realtime_arm_state_callback_ptr(self.arm_state_func)
                self.robot.rm_realtime_arm_state_call_back(self.arm_state_callback)

                print("机械臂初始化完成")
                break
            else:
                cnt_num += 1
                print("\n机械臂连接失败, Error code:", code)
                self.check_error_state(code)
                if cnt_num < max_retry_num:
                    print(f"尝试重新连接...{cnt_num}/{max_retry_num}")
                    time.sleep(retry_time)
        
        # 更新工具坐标系
        # self.change_end_effector(self.tool_frame)
    def init(self):
        pass
    
    def arm_state_func(self, data):
        # 机械臂 UDP主动上报 回调函数
        # print("Current arm ip: ", data.arm_ip)
        # print("Current arm pose: ", data.waypoint.to_dict()['position']['x'])
        # print("arm_err", data.arm_err, type(data.arm_err))
        # print("sys_err", data.sys_err)
        # print("errCode", data.errCode)
        # if data.arm_err != 0 or data.sys_err != 0 or data.errCode != 0 or data.arm_current_status != 0:
        if data.arm_err != 0 or data.sys_err != 0 or data.errCode != 0:
            # rm63.logger.log_info(f"arm_err{data.arm_err}")
            rm63.logger.log_error(f"触发急停, arm_err:{data.arm_err}, sys_err:{data.sys_err}, errCode:{data.errCode}")
            rm63.robot.rm_set_arm_stop()

    def start_up(self):
        # 机械臂上电
        self.check_error_state(self.robot.rm_set_arm_power(1))
        print("机械臂上电")
        print(power_state.get(self.robot.rm_get_arm_power_state()[1], "未知电源状态"))
    def power_off(self):
        # 机械臂断电
        self.check_error_state(self.robot.rm_set_arm_power(0))
        print("机械臂断电")
        print(power_state.get(self.robot.rm_get_arm_power_state()[1], "未知电源状态"))
    def go_home(self):
        # self.robot.rm_movej(self.init_joint, 10, 0, 0, 1)
        self.move_joint(self.init_joint, 10)

    def check_error_state(self, code):
        if code != 0:
            print('\n', error_codes.get(code, "未知错误"), '\n')
            # self.logger.log_error("机械臂急停")
            # self.robot.rm_set_arm_stop()
            return -1
        else:
            return 0
            
    def get_arm_state(self):
        code, cur_state = self.robot.rm_get_current_arm_state()
        self.check_error_state(code)
        if code == 0:
            return cur_state

    def get_arm_pose(self):
        # 获取当前位姿信息（弧度）
        cur_state = self.get_arm_state()
        return cur_state["pose"]
        # code, cur_state = self.robot.rm_get_current_arm_state()
        # if code == 0:
        #     return cur_state["pose"]
        # else:
        #     self.check_error_state(code)
    def get_arm_joint(self):
        # 获取当前6轴角度信息
        cur_state = self.get_arm_state()
        return cur_state["joint"]
        # code, cur_state = self.robot.rm_get_current_arm_state()
        # if code == 0:
        #     return cur_state["joint"]
        # else:
        #     self.check_error_state(code)

    def get_arm_position(self):
        # 获取当前位姿信息（四元数）
        cur_tool_state = self.robot.rm_get_current_tool_frame()
        cur_frame_name = cur_tool_state[1]['name']
        self.check_error_state(cur_tool_state[0])
        # 转为机械臂末端坐标系
        self.check_error_state(self.robot.rm_change_tool_frame('Arm_Tip'))
        position,quat_rot = self.get_end_position()
        # 转回原坐标系
        self.check_error_state(self.robot.rm_change_tool_frame(cur_frame_name))
        
        return position, quat_rot

    def get_end_position(self):
        # 获取工具末端位姿信息（四元数）
        pose = self.get_arm_pose()
        x, y, z, roll, pitch, yaw = pose # x, y, z unit: m
        tool_position = np.array([x, y, z])
        tool_quat_rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
        
        return tool_position, tool_quat_rot

    def set_frame(self, end_effector_config):
        frame_name = end_effector_config["tool_type"]
        frame_pose = end_effector_config["tcp_offset"]
        frame_payload = end_effector_config.get("frame_payload", 0)
        frame_gravity_center = end_effector_config.get("frame_gravity_center", [0,0,0])
        tool_frame = rm_frame_t(frame_name, frame_pose, frame_payload, frame_gravity_center[0], frame_gravity_center[1], frame_gravity_center[2])
        # 获取当前 frame 列表
        tool_frame_state = self.robot.rm_get_total_tool_frame()
        self.check_error_state(tool_frame_state['return_code'])
        if frame_name in tool_frame_state['tool_names']:
            self.check_error_state(self.robot.rm_update_tool_frame(tool_frame))
        else:
            self.check_error_state(self.robot.rm_set_manual_tool_frame(tool_frame))
        print(f"坐标系 {frame_name} 更新成功")
        # return frame_name, frame_pose, frame_payload, frame_gravity_center

    def change_end_effector(self, end_effector_config):
        type_name = end_effector_config["tool_type"]
        # 设置tcp偏移
        self.tcp_offset = end_effector_config["tcp_offset"]
        # self.set_frame(type_name, self.tcp_offset)
        self.set_frame(end_effector_config)

        # 更新工具坐标系
        self.check_error_state(self.robot.rm_change_tool_frame(type_name))
        self.check_error_state(self.robot.rm_get_current_tool_frame()[0])

        self.logger.log_info(f"修改为 {end_effector_config['name']} 工具坐标系")
    
    def move_joint(self, joints, speed = 15):
        # speed为比例
        self.check_error_state(self.robot.rm_movej(joints, speed, 0, 0, 1))
    #arm_position_command和arm_orientation_command如何获得的。
    def send_command(self, arm_position_command, arm_orientation_command, canfd_mode = False):
        rot_euler = R.from_quat(arm_orientation_command).as_euler('xyz', degrees=False)
        rot_euler = np.clip(rot_euler,-3.1415,3.1415) # 将发送姿态限制在正负180度以内
        pose_command = np.concatenate([arm_position_command, rot_euler]) # 将传入的四元数转换为 xyzRxRyRz 姿态表示


        code = self.robot.rm_movep_canfd(pose_command, canfd_mode) # 透传，输出间隔大时会很抖
        return self.check_error_state(code)

    

def RM_demo_circle(rm63):
    print("圆周运动示例")
    rm63.robot.rm_movej(rm63.init_joint, 10, 0, 0, 1)
    demo_init_pose = rm63.robot.rm_get_current_arm_state()[1]["pose"]
    # demo_cur_pose = demo_init_pose
    # demo_cur_pose[0] += 0.3
    # robot.rm_movel(demo_cur_pose, 30, 0, 0, 1)
    # time.sleep(0.2)
    # robot.rm_set_arm_slow_stop()
    # robot.rm_set_arm_stop()
    # print(robot.rm_set_arm_power(0))

    # 圆的参数
    demo_radius = 0.1  # 半径，单位为m
    demo_num_points = 5000  # 离散点的数量
    # 生成角度
    demo_theta = np.linspace(0, 2 * np.pi, demo_num_points, endpoint=False)

    demo_x_list = demo_radius * np.cos(demo_theta)
    demo_y_list = demo_radius * np.sin(demo_theta)

    demo_target_pos = demo_init_pose[::]
    demo_target_pos[0] += demo_x_list[0]
    demo_target_pos[1] += demo_y_list[0]
    rm63.robot.rm_movel(demo_target_pos, 20, 0, 0, 1)
    
    for _ in range(10):
        for i in range(1, len(demo_x_list)):
            demo_target_pos = demo_init_pose[::]
            # print(x_list[i], y_list[i])
            demo_target_pos[0] += demo_x_list[i]
            demo_target_pos[1] += demo_y_list[i]
            # print(demo_target_pos)
            # code = rm63.robot.rm_movel(demo_target_pos, 10, 0, 0, 1)
            code = rm63.robot.rm_movep_canfd(demo_target_pos, False) # 透传,频率低时会很抖
            # print("code=",code)
            rm63.check_error_state(code)
            # robot.rm_movep_follow(demo_target_pos) # 跟随

            if i==len(demo_x_list)//2:
                pass
                # print(11,robot.robot.rm_get_arm_current_trajectory())
                # 未发现明显区别
                # robot.robot.rm_set_arm_slow_stop()
                # robot.robot.rm_set_arm_pause()
                # time.sleep(2)
                # robot.robot.rm_set_arm_continue()

            # time.sleep(0.1)
            time.sleep(0.001)

def RM_demo_send_command(rm63):
    position,quat_rot = rm63.get_arm_position()
    # pose = rm63.robot.rm_get_current_arm_state()[1]["pose"]
    # print(pose)

    position[2] += 0.03
    rm63.send_command(position, quat_rot)
    time.sleep(0.1)
    position[2] -= 0.03
    rm63.send_command(position, quat_rot)

def RM_demo_get_end_positon(rm63):
    rm63 = RealmanRM63()
    # 新建工具坐标系
    rm63.robot.rm_delete_tool_frame("test")
    frame = rm_frame_t("test", [0, 0, 0.5, 0, 0, 0], 1, 0, 0, 0)
    rm63.robot.rm_set_manual_tool_frame(frame)
    print("全部坐标系信息：", rm63.robot.rm_get_total_tool_frame())
    print("工具坐标系信息：", rm63.robot.rm_get_given_tool_frame("test"))

    rm63.robot.rm_change_tool_frame('Arm_Tip')
    print("机械臂末端坐标", rm63.get_arm_position())
    print("tool末端坐标  ", rm63.get_end_position())
    print(rm63.get_arm_position())

def RM_demo_change_end_effector(rm63):
    end_effector_config = {"tool_type":"test02", "tcp_offset":[0,0,1,0,0,0],"name":123}
    rm63.change_end_effector(end_effector_config)


    print(rm63.robot.rm_get_current_tool_frame())

    print(rm63.get_arm_pose())
    print(rm63.get_arm_position())
    print(rm63.get_end_position())

def RM_demo():
    # rm63.robot.rm_set_collision_state(8)
    # print(rm63.robot.rm_get_collision_stage())
    custom = rm63.robot.rm_udp_custom_config()
    config = rm_realtime_push_config_t(100, True, 8089, 0, "192.168.1.104", custom)
    RM_demo_circle(rm63)
    # RM_demo_get_end_positon(rm63)
    # RM_demo_change_end_effector(rm63)

    # 运动至初始位置（建议使用movej）
    # movej 的速度比较快
    # self.robot.rm_movel(self.init_pos, 30, 0, 0, 1)
    # self.robot.rm_movej(self.init_joint, 10, 0, 0, 1)

def RM_demo_UDP():


    # 设置UDP端口，广播周期500ms，使能，广播端口号8089，力数据坐标系使用传感器坐标系，上报目标IP为"192.168.1.104"
    # 自定义上报项均设置关闭，用户可根据实际情况修改这些配置
    custom = rm_udp_custom_config_t()
    custom.joint_speed = 0
    custom.lift_state = 0
    custom.expand_state = 0
    custom.arm_current_status = 1
    config = rm_realtime_push_config_t(5, True, 8089, 0, "192.168.1.100", custom)
    print(rm63.robot.rm_set_realtime_push(config))
    print(rm63.robot.rm_get_realtime_push())

    # while 1:
    arm_state_callback = rm_realtime_arm_state_callback_ptr(rm63.arm_state_func)
    rm63.robot.rm_realtime_arm_state_call_back(arm_state_callback)

    # code = rm_realtime_arm_joint_state_t.arm_ip
    # print(code)
    # print(int(code))

    # ret = rm63.robot.rm_movej([0, 0, 90, 0, 90, 0], 10, 0, 0, 1)
    while 1:
        # 关节运动
        ret = rm63.robot.rm_movej([0, 0, 90, 0, 90, 0], 10, 0, 0, 1)
        print("movej: ", ret)
        ret = rm63.robot.rm_movej([90, 0, 90, 0, 90, 0], 10, 0, 0, 1)
        print("movej: ", ret)


def RM_test():

    arm_model = rm_robot_arm_model_e.RM_MODEL_RM_63_II_E  # RM_63机械臂
    force_type = rm_force_type_e.RM_MODEL_RM_B_E  # 标准版
    # 初始化算法的机械臂及末端型号
    algo_handle = Algo(arm_model, force_type)
    algo_handle.rm_algo_set_redundant_parameter_traversal_mode(True) # 设置逆解模式。true：遍历模式，冗余参数遍历的求解策略。


    j0 = [0, 0, -90, 0, -90, 0]
    # p1 = [0.186350, 0.062099, 0.200000, 3.141, 0,   1.569]
    # p1 = [0.186350, 0.062099, 0.400000, 3.141, 0,   1.569]
    # p1 = [0.186350, 0.062099, 0.500000, 3.141, 0,   1.569]
    # p1 = [0.186350, 0.062099, 0.600000, 3.141, 0,   1.569]
    p1 = [-0.491,   0,        0.516,    3.141, 0.0, 3.141]
    # rm63.move_joint(j0)
    # time.sleep(1)

    ready_pos = rm63.get_arm_pose()
    ready_pos[2] += 0.1
    cur = rm63.get_arm_joint()

    # print("cur",cur,rm63.get_arm_pose())

    
    
    # rm63.robot.rm_movel(p1, 20, 0, 0, 1)

    # 逆解从关节角度[0, 0, -90, 0, -90, 0]到目标位姿[0.186350, 0.062099, 0.200000, 3.141, 0, 1.569]。目标位姿姿态使用欧拉角表示。
    
    
    # cur = [0, 0, -90, 0, -90, 0]
    # print("*/******",type(cur))
    # ready_pos = [0.186350, 0.062099, 0.200000, 3.141, 0, 1.569]
    # print(params)
    params = rm_inverse_kinematics_params_t(cur, ready_pos, 1)
    # params = rm_inverse_kinematics_params_t([18.437000274658203, -10.675000190734863, -124.15899658203125, -0.01600000075995922, -45.132999420166016, -71.44599914550781], [-0.467046, -0.155683, 0.203799, -3.141, 0.0, -1.572], 1)
    
    
    # params = rm_inverse_kinematics_params_t(j0, p1, 1)
    
    print("ready_pose", p1)
    code, q_out = algo_handle.rm_algo_inverse_kinematics(params)

    print('\n', inverse_codes.get(code, "未知错误"), '\n')
    if code == 0:
        print("move to:", q_out)
        rm63.move_joint(q_out)


if __name__ == '__main__':
    rm63 = RealmanRM63()
    # RM_demo_UDP()
    p1 = [0.186350, 0.062099, 0.600000, 3.141, 0,   1.569]
    # rm63.robot.rm_movel(p1, 20, 0, 0, 1)
    RM_test()
    # ret = rm63.robot.rm_movej([0, 0, 90, 0, 90, 90], 20, 0, 0, 1)
    # ret = rm63.go_home()
    # RM_demo_UDP()

    # rm63.go_home()
    # print(rm63.get_arm_position())
    # print(rm63.get_arm_pose())
    # p0 = rm63.get_arm_pose()
    # p0[0] -= 0.2

    # rm63.robot.rm_movel(p0, 25, 0, 0, 1)

    # print(rm63.get_arm_joint())
    # print(rm63.get_arm_pose())
    # print(np.array(rm63.get_arm_pose()[3:])*180/3.14159)

    
    # print(rm63.robot.rm_get_realtime_push())
    
    # RM_demo_UDP()

    
    # time.sleep(3)
    # rm63.power_off()
    # time.sleep(3)
    # rm63.start_up()