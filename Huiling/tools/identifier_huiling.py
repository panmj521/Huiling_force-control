import time
import numpy as np
import atexit
from scipy.spatial.transform import Rotation as R
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from Hardware.Huiling_Scara import Huilin
from tools.yaml_operator import *
from Hardware.force_sensor import XjcSensor
import pysoem
import os

class Identifier:
    """
    reference: https://blog.csdn.net/qq_34935373/article/details/106208345
    """
    def __init__(self,arm:Huilin,sensor:XjcSensor,config_path:str,):
        self.arm  = arm
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
            data = self.sensor.read()
            # data = self.sensor.read_data_f32()
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
        
    # def set_position(self,x,y,z):
    #     x,y,z = x/1000,y/1000,z/1000
    #     joint = self.arm.inverse_kinematic(x,y)
    #     print(joint)
    #     self.arm.move_joint(joint)


    def identify_param_auto(self,ready_pose,cnt,sample_num = 5):
        control_pose =  list(ready_pose)


        for i in range(cnt):
            control_pose[0]+= 5
            control_pose[1]+= 5
            control_pose[2]+= 15
            self.arm.send_command(control_pose,0)
            print("move done")
            time.sleep(0.8)
            wrench = self.__read_data_once(sample_num)
            print(f"wrench:{wrench}")
            print("read done")
            pose = self.arm.get_arm_position()
            b_R_e = R.from_euler('xyz', [0,0,0], degrees=False).as_matrix()
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
        #confirmation = input("是否确认更新数据？(y/n): ").strip().lower() 有点问题
        # try:
        #     confirmation = input("是否确认更新数据？(y/n): ").strip().lower()
        # except (EOFError, OSError) as e:
        #     print(f"无法获取用户输入，默认不更新数据。错误: {str(e)}")
        #     confirmation = 'n'
        confirmation = 'y'
        if confirmation in ('y', 'yes'):
            # 执行更新操作
            if not os.path.exists(self.config_path):
                # 如果文件不存在，则创建一个新的config.yaml文件
                with open(self.config_path, 'w') as file:
                    pass
            
            update_yaml(self.config_path, dict_data)
            print("更新操作已完成。")
            # self.arm.arms_home()
        else:
            print("更新操作已取消。")
            # self.arm.arms_home()       
if __name__ == '__main__':
    arm = Huilin()
    # arm.init()
    
    # 传感器需要连接上
    os.system('sudo chmod 777 /dev/ttyUSB0')
    sensor = XjcSensor('/dev/ttyUSB0', 115200)
    # sensor.connect()
    sensor.enable_active_transmission()
    identifier = Identifier(arm=arm,sensor=sensor,config_path="/home/jsfb/jsfb_ws/HuilingScara-main/global_config/massage_head/roller_playload.yaml")
    # atexit.register(sensor.disconnect)

    ready_pose,_ = arm.get_arm_position()
    print(ready_pose)
    identifier.identify_param_auto(ready_pose,3)










    # def send_command(self,arm_position_command, arm_orientation_command):
    #     # rot_euler = R.from_quat(arm_orientation_command).as_euler('z', degrees=False)
    #     # rot_euler = np.clip(rot_euler,-3.1415,3.1415)
    #     pose_command = arm_position_command[:2]
    #     print("send_command中的",pose_command)
    #     z_command = arm_position_command[2]
    #     self.robot.get_scara_param()
    #     cur = np.array([self.robot.angle1,self.robot.angle2,self.robot.r])
    #     desire_joint, code = self.inverse_kinematic(cur, pose_command)
    #     self.get_inverse_kinematics_error_message(code,cur,desire_joint)
    #     #判断并执行机械臂关节的运动
    #     if code == 0:
    #         delta_joint = desire_joint - cur
    #         steps = 200
    #         step_size = delta_joint / steps
    #         for i in range(steps):
    #             target_joint = cur + (i+1)*step_size
    #             self.move_joint(target_joint)
    #         # 执行Z轴运动
    #         code_z = self.move_joint_3(z_command,target_speed = 1000)
    #         if code_z != 0:
    #             return -1  
                  
    #     else:
    #         print("inverse_error_arm_is_exit")
    #         self.power_off()
    #         return -1