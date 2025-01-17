import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from .arm_state import ArmState
from .base_controller import BaseController
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from tools.yaml_operator import read_yaml
import time

class HybridPidController(BaseController):
    def __init__(self, name, state: ArmState,config_path) -> None:
        super().__init__(name, state)
        self.load_config(config_path)
        self.laset_print_time = 0
    

    def load_config(self, config_path):
        config_dict = read_yaml(config_path)
        if self.name != config_dict['name']:
            raise ValueError(f"Controller name {self.name} does not match config name {config_dict['name']}")
        # 姿态调节器
        # 位控 2x2矩阵
        self.Kp_R = np.diag(np.array(config_dict['Kp_R']))
        self.Ki_R = np.diag(np.array(config_dict['Ki_R']))
        self.Kd_R = np.diag(np.array(config_dict['Kd_R']))
        # mass_rot = np.array(config_dict['mass_rot'])
        # stiff_rot = np.array(config_dict['stiff_rot'])
        # desired_xi = np.array(config_dict['desired_xi'])
        # damp_rot = np.array(config_dict['damp_rot'])
        # for i in range(3):
        #     if damp_rot[i] < 0:
        #         damp_rot[i] = 2 * desired_xi * np.sqrt(stiff_rot[i] * mass_rot[i])
        # self.M_rot = np.diag(mass_rot)
        # self.K_rot = np.diag(stiff_rot)
        # self.D_rot = np.diag(damp_rot)

        # 力控
        self.force_mass = np.array(config_dict['force_mass'])
        self.force_damp = np.array(config_dict['force_damp'])
        self.Kp_force = np.array(config_dict['Kp_force'])
        self.Kd_force = np.array(config_dict['Kd_force'])
        self.Ki_force = np.array(config_dict['Ki_force'])
        # TODO 修改控制率
        self.e_t = 0
        self.e_t1 = 0
        self.e_t2 = 0
        self.force_control_value = 0

        # 位控 2x2矩阵
        self.Kp = np.diag(np.array(config_dict['Kp']))
        self.Ki = np.diag(np.array(config_dict['Ki']))
        self.Kd = np.diag(np.array(config_dict['Kd']))

        self.pose_integral_error = np.zeros(6)
    


        
    def step(self, dt):
        # 获得当前RPY角度对应的旋转矩阵
        b_rotation_s = R.from_quat(self.state.arm_orientation).as_matrix()

        # 处理四元数的符号问题
        if self.state.desired_orientation.dot(self.state.arm_orientation) < 0:
            self.state.arm_orientation = -self.state.arm_orientation
        
        #初始化误差矩阵
        temp_pose_error = np.zeros(6)

        # 计算误差 位置直接作差，姿态误差以旋转向量表示
        #self.state.pose_error[:3] = self.state.arm_position - self.state.desired_position 
        temp_pose_error[:3] = self.state.arm_position - self.state.desired_position 

        # 获得当前笛卡尔位置切换到对应tcp点坐标系下的表达
        self.state.pose_error[:3] = R.from_quat(self.state.arm_orientation).as_matrix().T @  temp_pose_error[:3]
        
        #rot_err_mat = R.from_quat(self.state.arm_orientation).as_matrix() @ R.from_quat(self.state.desired_orientation).as_matrix().T
        rot_err_mat = R.from_quat(self.state.arm_orientation).as_matrix().T @ R.from_quat(self.state.desired_orientation).as_matrix()  
        rot_err_rotvex = R.from_matrix(rot_err_mat).as_rotvec(degrees=False)
        #self.state.pose_error[3:] = rot_err_rotvex
        self.state.pose_error[3:] = -rot_err_rotvex

        # # 姿态控制 期望世界坐标系的姿态下的力矩为0
        # wrench_err_base = self.state.external_wrench_base[3:] - self.state.desired_wrench[3:]
        # self.state.arm_desired_acc[3:] = np.linalg.inv(self.M_rot) @ (wrench_err_base - self.D_rot @ (self.state.arm_desired_twist[3:] - self.state.desired_twist[3:])\
        #                                                           -self.K_rot @ self.state.pose_error[3:])

        # 姿态控制 期望目标点坐标系的姿态下的力矩为0
        # wrench_err_tcp = self.state.external_wrench_tcp[3:] - self.state.desired_wrench[3:]
        # self.state.arm_desired_acc[3:] = np.linalg.inv(self.M_rot) @ (wrench_err_tcp - self.D_rot @ (self.state.arm_desired_twist[3:] - self.state.desired_twist[3:])\
        #                                                           -self.K_rot @ self.state.pose_error[3:])

        # rpy位置控制
        # tempKd = np.array([2,2,2])
        # tempKp = np.array([10,10,10])
        # self.state.arm_desired_acc[3:] = -tempKd @ self.state.arm_desired_twist[3:] - tempKp @ self.state.pose_error[3:]
        
        # z方向力控制
        force_err = self.state.external_wrench_tcp[2] - self.state.desired_wrench[2]
        self.e_t2 = self.e_t1
        self.e_t1 = self.e_t
        self.e_t = force_err
        self.force_control_value += self.Kp_force * (self.e_t - self.e_t1) + self.Ki_force * self.e_t + self.Kd_force * (self.e_t - 2 * self.e_t1 + self.e_t2)
        self.state.arm_desired_acc[2] = (1.0 / self.force_mass) * (self.force_control_value - self.force_damp * self.state.arm_desired_twist[2])

        self.pose_integral_error +=  self.state.pose_error * dt
        # 位控制
        self.state.arm_desired_acc[:2] = -self.Kd @ self.state.arm_desired_twist[:2] - self.Kp @ self.state.pose_error[:2] - self.Ki @ self.pose_integral_error[:2]

        # 姿态pid
        self.state.arm_desired_acc[3:] = -self.Kd_R @ self.state.arm_desired_twist[3:] - self.Kp_R @ self.state.pose_error[3:] - self.Ki_R @ self.pose_integral_error[3:]

        self.clip_command(self.state.arm_desired_acc,"acc")
        self.state.arm_desired_twist = self.state.arm_desired_acc * dt + self.state.arm_desired_twist
        self.clip_command(self.state.arm_desired_twist,"vel")

        delta_pose = self.state.arm_desired_twist * dt


        delta_pose[:3] = R.from_quat(self.state.arm_orientation).as_matrix() @ delta_pose[:3]
        #delta_pose[:3] = b_rotation_s @ delta_pose[:3] # 这里本身位置误差即为基坐标系下的误差，所以不需要再乘上旋转矩阵

        self.clip_command(delta_pose,"pose")

        delta_ori_mat = R.from_rotvec(delta_pose[3:]).as_matrix()
        #arm_ori_mat = delta_ori_mat @ R.from_quat(self.state.arm_orientation).as_matrix()
        arm_ori_mat =  R.from_quat(self.state.arm_orientation).as_matrix() @ delta_ori_mat
        
        self.state.arm_orientation_command = R.from_matrix(arm_ori_mat).as_quat()
        self.state.arm_position_command = self.state.arm_position + delta_pose[:3]

        # if time.time() - self.laset_print_time > 0.2:
        #     print("-----------------hybrid1-------------------------------")
        #     print("arm_position:",self.state.arm_position)
        #     print("desired_position:",self.state.desired_position)
        #     print("arm_orientation",R.from_quat(self.state.arm_orientation).as_euler('xyz',degrees=True))
        #     print("desired_orientation",R.from_quat(self.state.desired_orientation).as_euler('xyz',degrees=True))
        #     print("arm_position_command",self.state.arm_position_command)
        #     print("arm_orientation_command",R.from_quat(self.state.arm_orientation_command).as_euler('xyz',degrees=True))
        #     print("delta_pose:",delta_pose)
        #     self.laset_print_time = time.time()















        