import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from .base_controller import BaseController
from .arm_state import ArmState
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
import time

from tools.yaml_operator import read_yaml
class AdmittanceController(BaseController):
    def __init__(self, name, state:ArmState,config_path) -> None:
        super().__init__(name, state)
        self.load_config(config_path)
        

    def load_config(self, config_path):
        config_dict = read_yaml(config_path)
        if self.name != config_dict['name']:
            raise ValueError(f"Controller name {self.name} does not match config name {config_dict['name']}")
        mass_tran = np.array(config_dict['mass_tran'])
        mass_rot = np.array(config_dict['mass_rot'])
        stiff_tran = np.array(config_dict['stiff_tran'])
        stiff_rot = np.array(config_dict['stiff_rot'])
        desired_xi = np.array(config_dict['desired_xi'])
        damp_tran = np.array(config_dict['damp_tran'])
        damp_rot = np.array(config_dict['damp_rot'])
        self.pos_scale_factor = config_dict['pos_scale_factor']
        self.rot_scale_factor = config_dict['rot_scale_factor']
        for i in range(3):
            if damp_tran[i] < 0:
                damp_tran[i] = 2 * desired_xi * np.sqrt(stiff_tran[i] * mass_tran[i])
            if damp_rot[i] < 0:
                damp_rot[i] = 2 * desired_xi * np.sqrt(stiff_rot[i] * mass_rot[i])
        self.M = np.diag(np.concatenate([mass_tran, mass_rot]))
        self.K = np.diag(np.concatenate([stiff_tran, stiff_rot]))
        self.D = np.diag(np.concatenate([damp_tran, damp_rot]))

        self.laset_print_time = 0
        
    def step(self,dt):
        # 计算误差 位置直接作差，姿态误差以旋转向量表示

        temp_pose_error = self.state.arm_position - self.state.desired_position 
        # if time.time() - self.laset_print_time > 5:
        #     print(f'temp_pose_error: {temp_pose_error} ||| arm_position: {self.state.arm_position} ||| desired_position: {self.state.desired_position}')
        if self.state.desired_orientation.dot(self.state.arm_orientation) < 0:
            self.state.arm_orientation = -self.state.arm_orientation

        self.state.pose_error[:3] = R.from_quat(self.state.arm_orientation).as_matrix().T @ temp_pose_error
        # if time.time() - self.laset_print_time > 5:
        #     print("pose_error:",self.state.pose_error[:3])
        # 计算误差 位置直接作差，姿态误差以旋转向量表示
        #rot_err_mat = R.from_quat(self.state.arm_orientation).as_matrix() @ R.from_quat(self.state.desired_orientation).as_matrix().T
        rot_err_mat = R.from_quat(self.state.arm_orientation).as_matrix().T @ R.from_quat(self.state.desired_orientation).as_matrix()
        # print(f'rot_err_mat: {rot_err_mat} ||| arm_orientation: {R.from_quat(self.state.arm_orientation).as_euler('xyz',False)} ||| desired_orientation: {R.from_quat(self.state.desired_orientation).as_euler('xyz',False)}')
        rot_err_rotvex = R.from_matrix(rot_err_mat).as_rotvec(degrees=False)
        self.state.pose_error[3:] = -rot_err_rotvex

        #wrench_err = self.state.external_wrench_base - self.state.desired_wrench
        wrench_err = self.state.external_wrench_tcp - self.state.desired_wrench
        # if time.time() - self.laset_print_time > 5:
        #     print(f'wrench_err: {wrench_err} ||| external_wrench_tcp: {self.state.external_wrench_tcp} ||| desired_wrench: {self.state.desired_wrench}')
        self.state.arm_desired_acc = np.linalg.inv(self.M) @ (wrench_err - self.D @ (self.state.arm_desired_twist -self.state.desired_twist) - self.K @ self.state.pose_error)
        # if time.time() - self.laset_print_time > 5:
        #     print("@@@:",wrench_err - self.D @ (self.state.arm_desired_twist -self.state.desired_twist) - self.K @ self.state.pose_error)
        self.clip_command(self.state.arm_desired_acc,"acc")
        self.state.arm_desired_twist = self.state.arm_desired_acc * dt + self.state.arm_desired_twist
        self.clip_command(self.state.arm_desired_twist,"vel")
        delta_pose = self.state.arm_desired_twist * dt

        delta_pose[:3] = self.pos_scale_factor * delta_pose[:3]
        delta_pose[3:] = self.rot_scale_factor * delta_pose[3:]
        # if time.time() - self.laset_print_time > 5:
        #     print("delta_pose:",delta_pose)
        
        delta_pose[:3] = R.from_quat(self.state.arm_orientation).as_matrix() @ delta_pose[:3]

        # if time.time() - self.laset_print_time > 5:
        #     print("tf_delta_pose:",delta_pose)
        self.clip_command(delta_pose,"pose")

        # testlsy
        delta_ori_mat = R.from_rotvec(delta_pose[3:]).as_matrix()

        #arm_ori_mat = delta_ori_mat @ R.from_quat(self.state.arm_orientation).as_matrix()
        arm_ori_mat = R.from_quat(self.state.arm_orientation).as_matrix() @ delta_ori_mat 
        self.state.arm_orientation_command = R.from_matrix(arm_ori_mat).as_quat()

        # arm_ori_mat = R.from_quat(self.state.arm_orientation).as_rotvec(degrees=False) + delta_pose[3:]
        # self.state.arm_orientation_command = R.from_rotvec(arm_ori_mat).as_quat()
        
        # self.state.arm_orientation_command = R.from_matrix(arm_ori_mat).as_quat()
        self.state.arm_position_command = self.state.arm_position + delta_pose[:3]
        # if time.time() - self.laset_print_time > 0.1:
        #     print("-------------admittance_1-------------------------------")
        #     print("arm_position:",self.state.arm_position)
        #     print("desired_position:",self.state.desired_position)
        #     print("arm_orientation",R.from_quat(self.state.arm_orientation).as_euler('xyz',degrees=True))
        #     print("desired_orientation",R.from_quat(self.state.desired_orientation).as_euler('xyz',degrees=True))
        #     print("arm_position_command",self.state.arm_position_command)
        #     print("arm_orientation_command",R.from_quat(self.state.arm_orientation_command).as_euler('xyz',degrees=True))
        #     print("delta_pose:",delta_pose)
        #     self.laset_print_time = time.time()



if __name__ == "__main__":
    state = ArmState()
    controller = AdmittanceController("admittance",state,"/home/zyc/admittance_control/MassageControl/config/admittance.yaml")
    print(controller.name)
    print(controller.state.arm_position)
    state.arm_position = np.array([1,2,3])
    print(controller.state.arm_position)
    print(controller.M)
    print(controller.D)
    print(controller.K)