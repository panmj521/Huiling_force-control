from abc import ABC, abstractmethod
from typing import Literal
import numpy as np
from scipy.spatial.transform import Rotation as R
from .arm_state import ArmState

"""
    位置单位为m，力单位为N，力矩单位为Nm，角度单位为rad
"""
class BaseController(ABC):
    def __init__(self,name,state:ArmState) -> None:
        super().__init__()
        self.name = name
        self.state = state
       

    @abstractmethod
    def step(self,dt):
        # 算法的一次迭代
        pass

    @abstractmethod
    def load_config(self, config_path):
        # 加载配置文件
        pass

    def clip_command(self, command :np.array,type: Literal["acc", "vel", "pose"],is_print=False):
        if type == "acc":
            if np.linalg.norm(command[:3]) > self.state.max_acc_tran:
                command[:3] = command[:3] / np.linalg.norm(command[:3]) * self.state.max_acc_tran
                if is_print:
                    print(f"translational acceleration {np.linalg.norm(command[:3])}m/s exceeds maximum allowed value")
            if np.linalg.norm(command[3:]) > self.state.max_acc_rot:
                if is_print:
                    print(f"rotational acceleration {np.linalg.norm(command[3:])}rad/s exceeds maximum allowed value,")
                command[3:] = command[3:] / np.linalg.norm(command[3:]) * self.state.max_acc_rot
        elif type == "vel":
            if np.linalg.norm(command[:3]) > self.state.max_vel_tran:
                if is_print:
                    print(f"translational velocity {np.linalg.norm(command[:3])}m/s exceeds maximum allowed value,")
                command[:3] = command[:3] / np.linalg.norm(command[:3]) * self.state.max_vel_tran
            if np.linalg.norm(command[3:]) > self.state.max_vel_rot:
                command[3:] = command[3:] / np.linalg.norm(command[3:]) * self.state.max_vel_rot
                if is_print:
                    print(f"rotational velocity {np.linalg.norm(command[3:])}rad/s exceeds maximum allowed value")
        elif type == "pose":
            if np.linalg.norm(command[:3]) > self.state.max_dx:
                command[:3] = command[:3] / np.linalg.norm(command[:3]) * self.state.max_dx
                if is_print:
                    print(f"translational displacement {np.linalg.norm(command[:3])}m exceeds maximum allowed value")
            if np.linalg.norm(command[3:]) > self.state.max_dr:
                command[3:] = command[3:] / np.linalg.norm(command[3:]) * self.state.max_dr
                if is_print:
                    print(f"rotational displacement {np.linalg.norm(command[3:])}rad exceeds maximum allowed value")
        
        
    
    @staticmethod
    def rotvec_pose_add(pose, delta_pose):
        """
        Compute the pose sum between two poses, which consists if position (x, y, z) and rotation vector (rx, ry, rz).
        Update rule: x_t+1 = x_t + dx, R_t+1 = dR * R_t (rotation matrix)

        :param pose: np.ndarray (6,)
        :param delta_pose: np.ndarray (6,)
        :return: np.ndarray (6,)
        """
        assert len(pose) == 6 and len(delta_pose) == 6, "pose and delta_pose must be 6-dimensional"

        ret = np.zeros(6)
        ret[:3] = pose[:3] + delta_pose[:3]
        # 当前姿态的旋转矩阵
        pose_matrix = R.from_rotvec(pose[3:]).as_matrix()
        # 旋转矩阵的增量
        pose_delta_matrix = R.from_rotvec(delta_pose[3:]).as_matrix()
        # 更新后的旋转矩阵，然后转换为旋转向量
        ret[3:] = R.from_matrix(pose_delta_matrix @ pose_matrix).as_rotvec()
        return ret

   