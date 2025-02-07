import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from .base_controller import BaseController
from .arm_state import ArmState
from pathlib import Path
import time
sys.path.append(str(Path(__file__).resolve().parent.parent))

from tools.yaml_operator import read_yaml

class PositionController(BaseController):
    def __init__(self, name, state:ArmState,config_path) -> None:
        super().__init__(name, state)
        self.load_config(config_path)

        self.laset_print_time = 0

    def load_config(self, config_path):
        config_dict = read_yaml(config_path)
        if self.name != config_dict['name']:
            raise ValueError(f"Controller name {self.name} does not match config name {config_dict['name']}")
        self.Kp = np.diag(np.array(config_dict['Kp']))
        self.Ki = np.diag(np.array(config_dict['Ki']))
        self.Kd = np.diag(np.array(config_dict['Kd']))

        self.pose_integral_error = np.zeros(6)
    
    def step(self,dt):
        # print(f"desired position: {self.state.desired_position}, desired orientation: {R.from_quat(self.state.desired_orientation).as_euler('xyz',degrees=False)}")
        # print(f"current position: {self.state.arm_position}, current orientation: {R.from_quat(self.state.arm_orientation).as_euler('xyz',degrees=False)}")
        self.state.pose_error[:3] = self.state.arm_position  - self.state.desired_position
        if self.state.desired_orientation.dot(self.state.arm_orientation) < 0:
            self.state.arm_orientation = -self.state.arm_orientation
        rot_err_mat = R.from_quat(self.state.arm_orientation).as_matrix() @ R.from_quat(self.state.desired_orientation).as_matrix().T
        rot_err_rotvex = R.from_matrix(rot_err_mat).as_rotvec(degrees=False)
        self.state.pose_error[3:] = rot_err_rotvex

        self.pose_integral_error +=  self.state.pose_error * dt

        self.state.arm_desired_acc = - self.Kd @ (self.state.arm_desired_twist - self.state.desired_twist) - self.Kp @ self.state.pose_error - self.Ki @ self.pose_integral_error
        self.clip_command(self.state.arm_desired_acc,"acc")
        self.state.arm_desired_twist = self.state.arm_desired_acc * dt + self.state.arm_desired_twist
        self.clip_command(self.state.arm_desired_twist,"vel")
        delta_pose = self.state.arm_desired_twist * dt
        self.clip_command(delta_pose,"pose")
        delta_ori_mat = R.from_rotvec(delta_pose[3:]).as_matrix()
        arm_ori_mat = delta_ori_mat @ R.from_quat(self.state.arm_orientation).as_matrix()
        
        self.state.arm_orientation_command = R.from_matrix(arm_ori_mat).as_quat()
        self.state.arm_position_command = self.state.arm_position + delta_pose[:3]

        # if time.time() - self.laset_print_time > 0.5:
        #     print("---------------positioner-------------------------------------")
        #     print("arm_position:",self.state.arm_position)
        #     print("desired_position:",self.state.desired_position)
        #     print("arm_orientation",R.from_quat(self.state.arm_orientation).as_euler('xyz',degrees=True))
        #     print("desired_orientation",R.from_quat(self.state.desired_orientation).as_euler('xyz',degrees=True))
        #     print("arm_position_command",self.state.arm_position_command)
        #     print("arm_orientation_command",R.from_quat(self.state.arm_orientation_command).as_euler('xyz',degrees=True))
        #     print("delta_pose:",delta_pose)
        #     self.laset_print_time = time.time()

        # if time.time() - self.last_print_time > 1:
        #     print(self.state.arm_position_command,R.from_quat(self.state.arm_orientation_command).as_euler('xyz',degrees=True))
        #     self.last_print_time = time.time()
