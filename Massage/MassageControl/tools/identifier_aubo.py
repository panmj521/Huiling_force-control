# cython: language_level=3
import time
import numpy as np
import atexit
from scipy.spatial.transform import Rotation as R
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from Hardware.aubo_C5 import AuboC5
from tools.yaml_operator import *
from Hardware.force_sensor_aubo import XjcSensor
import os
import math

class Identifier:
    """
    reference: https://blog.csdn.net/qq_34935373/article/details/106208345
    """
    def __init__(self,arm:AuboC5,sensor:XjcSensor,config_path:str,):
        self.arm   = arm
        self.sensor = sensor
        self.config_path = config_path
        # solve for xyz: M = F @ A, A=[x, y, z, k1, k2, k3]
        self.M = None
        self.F = None
        
        # solve for F0F1F2: f = R @ B, B=[0, 0, -g, Fx0, Fy0, Fz0]
        self.R = None
        self.f = None
        
        self.len_data = 0
        
    def __add_data(self, wrench: np.ndarray, ee_pose: np.ndarray):
        force = wrench[:3].copy()
        torque = wrench[3:].copy()
        # import pdb
        # pdb.set_trace()
        if self.M is None:
            self.M = torque.reshape((3, 1))
            f_skew = self.__skew_symmetric(force)
            self.F = np.hstack((f_skew, np.eye(3)))
        else:
            self.M = np.vstack((self.M, torque.reshape((3, 1))))
            f_skew = self.__skew_symmetric(force)
            self.F = np.vstack((self.F, np.hstack((f_skew, np.eye(3)))))
        
        if self.R is None:
            self.R = np.hstack((ee_pose.T, np.eye(3)))
            # self.R = np.hstack((ee_pose[:3].T, np.eye(3)))
            self.f = force.reshape((3, 1))
        else:
            self.R = np.vstack((self.R, np.hstack((ee_pose.T, np.eye(3)))))
            # self.R = np.vstack((self.R, np.hstack((ee_pose[:3].T, np.eye(3)))))
            self.f = np.vstack((self.f, force.reshape((3, 1))))
        self.len_data += 1
        
    def __solve(self):
        A, residuals, rank, s = np.linalg.lstsq(self.F, self.M, rcond=None)
        B, residuals, rank, s = np.linalg.lstsq(self.R, self.f, rcond=None)
        
        x, y, z, k1, k2, k3 = A.flatten()
        Gx, Gy, Gz, Fx0, Fy0, Fz0 = B.flatten()
        
        mass_center = np.array([x, y, z])
        F_0 = np.array([Fx0, Fy0, Fz0])
        M_x0 = k1 - Fy0 * z + Fz0 * y
        M_y0 = k2 - Fz0 * x + Fx0 * z
        M_z0 = k3 - Fx0 * y + Fy0 * x
        M_0 = np.array([M_x0, M_y0, M_z0])
        gravity_base = np.array([Gx, Gy, Gz])
        return mass_center, F_0, M_0, gravity_base
    
    def __read_data_once(self,num):
        sum = np.zeros(6)
        i = 0
        # 读取num次数据，取平均值
        while i < num:
            # data = self.sensor.read()
            data = self.sensor.read_data_f32()
            if data is not None:
                sum += data
                i += 1 
            time.sleep(0.2)
        return  sum/i

    def __skew_symmetric(self, v: np.ndarray):
        """ 从3维向量生成对应的3x3反对称矩阵 """
        if len(v) != 3:
            raise ValueError("输入数组的长度必须为3!")
        return np.array([[0, v[2], -v[1]],
                        [-v[2], 0, v[0]],
                        [v[1], -v[0], 0]])

    def cal_R_mat(self):
        pose = self.arm.robot_rpc_client.getRobotInterface(self.arm.robot_name).getRobotState().getTcpPose()
        x, y, z, roll, pitch, yaw = pose # x, y, z unit: millimeter
        R_mat = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
        return R_mat
        
    def set_position(self,x,y,z,roll,pitch,yaw):
        x,y,z = x/1000,y/1000,z/1000
        roll,pitch,yaw =roll / (180 / np.pi) ,pitch / (180 / np.pi),yaw / (180 / np.pi)
        pose_command = np.concatenate([[x,y,z], [roll,pitch,yaw]])
        cur = self.arm.robot.getRobotState().getJointPositions() # 接口调用: 获取机器人的关节位置
        print(cur)
        res,code = self.arm.robot.getRobotAlgorithm().inverseKinematics(cur, pose_command)
        self.arm.get_inverse_kinematics_error_message(code,cur, pose_command)
        print(res)
        self.arm.move_joint(res)

    def identify_param_auto(self,ready_pos,cnt,sample_num = 5):
        if cnt < 15:
            raise ValueError("数据量太少，请重新输入，至少要有15组数据")
        control_pose = np.array(ready_pos)
        R_mat = self.cal_R_mat()
        desired_cart_pose = control_pose.copy()
        # desired_cart_pose[0:3] = desired_cart_pose[0:3] + R_mat @ np.array([0, 0, 280])
        print("desired_cart_pose:",desired_cart_pose)
        

        
        # move to the initial position
        self.set_position(x=desired_cart_pose[0],y=desired_cart_pose[1],z=desired_cart_pose[2],
                                roll=desired_cart_pose[3],pitch=desired_cart_pose[4],yaw=desired_cart_pose[5])
        
        time.sleep(2)
        # self.sensor.set_zero()
        time.sleep(1)
        delta = 60/cnt*6 
        quarter = cnt // 6
        for i in range(cnt):
            if i % 2 == 0:
                control_pose[3]+= delta * np.where((i <= quarter) | (3* quarter< i <= 5*quarter) , 1, -1)
            else:
                control_pose[4]+= delta *  np.where((i <= quarter) | (3* quarter< i <= 5*quarter), -1, 1)
            R_mat = self.cal_R_mat()
            desired_cart_pose = control_pose.copy()
            # desired_cart_pose[0:3] = desired_cart_pose[0:3] + R_mat @ np.array([0, 0, 280])
            print("desired_cart_pose:",desired_cart_pose)
            self.set_position(x=desired_cart_pose[0],
                                y=desired_cart_pose[1],
                                z=desired_cart_pose[2],
                                roll=desired_cart_pose[3],
                                pitch=desired_cart_pose[4],
                                yaw=desired_cart_pose[5])
            print("move done")
            time.sleep(1)
            wrench = self.__read_data_once(sample_num)
            print(f"wrench:{wrench}")
            print("read done")
            pose = self.arm.robot_rpc_client.getRobotInterface(self.arm.robot_name).getRobotState().getTcpPose()
            b_R_e = R.from_euler('xyz', pose[3:], degrees=False).as_matrix()
            self.__add_data(wrench, b_R_e)
            print(f"=======添加了第{i+1}组数据=======")
            time.sleep(0.5)
        mass_center, F_0, M_0, gravity_base = self.__solve()
        # output all results
        print(f"mass_center_position: {mass_center}")
        print(f"force_zero: {F_0}")
        print(f"torque_zero: {M_0}")
        print(f"gravity_base: {gravity_base}")
        dict_data = {
            'mass_center_position': mass_center.tolist(),
            'force_zero': F_0.tolist(),
            'torque_zero': M_0.tolist(),
            'gravity_base': gravity_base.tolist()
        }
        confirmation = input("是否确认更新数据？(y/n): ").strip().lower()

        if confirmation == 'y' or confirmation == 'Y' or confirmation == 'yes' or confirmation == 'YES':
            # 执行更新操作
            if not os.path.exists(self.config_path):
                # 如果文件不存在，则创建一个新的config.yaml文件
                with open(self.config_path, 'w') as file:
                    pass
            
            update_yaml(self.config_path, dict_data)
            print("更新操作已完成。")
        else:
            print("更新操作已取消。")
            
        
        
if __name__ == '__main__':
    arm = AuboC5()
    arm.init()
    # os.system('sudo chmod 777 /dev/ttyUSB0')
    sensor = XjcSensor('/dev/ttyUSB0', 115200)
    sensor.connect()
    sensor.disable_active_transmission()
    atexit.register(sensor.disconnect)
    identifier = Identifier(arm=arm,sensor=sensor,config_path="/home/jsfb/jsfb_ws/global_config/massage_head/roller_playload.yaml")
    # identifier = Identifier(arm=arm,sensor=sensor,config_path="/home/jsfb/jsf`b_ws/global_config/massage_head/none_playload.yaml")
    ready_pos = identifier.arm.robot_rpc_client.getRobotInterface(identifier.arm.robot_name).getRobotState().getTcpPose()
    ready_pos[0] = ready_pos[0]*1000
    ready_pos[1] = ready_pos[1]*1000+100
    ready_pos[2] = ready_pos[2]*1000+100
    ready_pos[3] = ready_pos[3] * (180 / np.pi)
    ready_pos[4] = ready_pos[4] * (180 / np.pi)
    ready_pos[5] = ready_pos[5] * (180 / np.pi)
    print(ready_pos)
    time.sleep(1)
    identifier.identify_param_auto(ready_pos,45)
    # identifier.identify_param_manual()
    # DATA = read_yaml("config/compensation.yaml")
    # print(DATA)


    
    
    
        