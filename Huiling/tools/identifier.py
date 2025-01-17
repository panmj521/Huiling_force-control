# cython: language_level=3
import time
import numpy as np
import atexit
from scipy.spatial.transform import Rotation as R
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from hardware.u6lite import Ufactory6Lite
from tools.yaml_operator import *
from hardware.force_sensor import XjcSensor
import os

class Identifier:
    """
    reference: https://blog.csdn.net/qq_34935373/article/details/106208345
    """
    def __init__(self,arm:Ufactory6Lite,sensor:XjcSensor,config_path:str,):
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
            self.R = np.hstack((ee_pose[:3].T, np.eye(3)))
            self.f = force.reshape((3, 1))
        else:
            self.R = np.vstack((self.R, np.hstack((ee_pose[:3].T, np.eye(3)))))
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
            data = self.sensor.read_data_f32()
            if data is not None:
                sum += data
                i += 1 
            time.sleep(0.01)
        return  sum/i

    def __skew_symmetric(self, v: np.ndarray):
        """ 从3维向量生成对应的3x3反对称矩阵 """
        if len(v) != 3:
            raise ValueError("输入数组的长度必须为3!")
        return np.array([[0, v[2], -v[1]],
                        [-v[2], 0, v[0]],
                        [v[1], -v[0], 0]])
    
    def identify_param_manual(self,sample_num = 5):
        cnt = 0
        while True: 
            key = input("=======输入w添加数据, 输入q退出=======\n")
            if key == 'w' or key == 'W':
                wrench = self.read_data_once(sample_num)
                code, pose = self.arm.get_position(is_radian=True)
                b_R_e = R.from_euler('xyz', pose[3:], degrees=False).as_matrix()
                self.__add_data(wrench, b_R_e)
                cnt += 1
                print(f"=======添加了第{cnt}组数据=======")
            elif key == 'q':
                if self.len_data < 3:
                    print("数据量太少，请继续输入")
                    continue
                else: 
                    break
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

    def cal_R_mat(self):
        code, pose = self.arm.get_position(is_radian=True)
        x, y, z, roll, pitch, yaw = pose # x, y, z unit: millimeter
        R_mat = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
        return R_mat
        

    def identify_param_auto(self,ready_pos,cnt,sample_num = 5):
        if cnt < 15:
            raise ValueError("数据量太少，请重新输入，至少要有15组数据")
        self.arm.set_mode(0)
        self.arm.set_state(0)
        control_pose = np.array(ready_pos)
        R_mat = self.cal_R_mat()
        desired_cart_pose = control_pose.copy()
        desired_cart_pose[0:3] = desired_cart_pose[0:3] + R_mat @ np.array([0, 0, 280])
        print("desired_cart_pose:",desired_cart_pose)

        
        # move to the initial position
        self.arm.set_position(x=desired_cart_pose[0],y=desired_cart_pose[1],z=desired_cart_pose[2],
                                roll=desired_cart_pose[3],pitch=desired_cart_pose[4],yaw=desired_cart_pose[5],
                                speed=30,wait=True)
        
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
            desired_cart_pose[0:3] = desired_cart_pose[0:3] + R_mat @ np.array([0, 0, 280])
            print("desired_cart_pose:",desired_cart_pose)
            self.arm.set_position(x=desired_cart_pose[0],
                                y=desired_cart_pose[1],
                                z=desired_cart_pose[2],
                                roll=desired_cart_pose[3],
                                pitch=desired_cart_pose[4],
                                yaw=desired_cart_pose[5],
                                speed=30,
                                wait=True)
            print("move done")
            time.sleep(1)
            wrench = self.__read_data_once(sample_num)
            print(f"wrench:{wrench}")
            print("read done")
            code, pose = self.arm.get_position(is_radian=True)
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
    arm_ip = '192.168.1.176'
    arm = Ufactory6Lite(arm_ip,is_radian=False)
    # os.system('sudo chmod 777 /dev/ttyUSB0')
    sensor = XjcSensor(port='/dev/ttyUSB0', baudrate=115200)
    # sensor.enable_active_transmission()
    
    atexit.register(arm.disconnect)
    atexit.register(sensor.disconnect)
    identifier = Identifier(arm=arm,sensor=sensor,config_path="MassageControl/config/massage_head/cupping_playload.yaml")
    ready_pos = [7.3, 301, 450, 0, 0, 0]
    identifier.identify_param_auto(ready_pos,30)
    # identifier.identify_param_manual()
    # DATA = read_yaml("config/compensation.yaml")
    # print(DATA)


    
    
    
        