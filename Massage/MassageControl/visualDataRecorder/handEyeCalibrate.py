import time
import numpy as np
import atexit
from scipy.spatial.transform import Rotation as R
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from Hardware.Huiling_Scara import Huilin
from tools.yaml_operator import *
from Hardware.force_sensor_aubo import XjcSensor
import os
import cv2
from Hardware.remote_cam import ToolCamera
import math
from scipy import optimize  
import odrive

import numpy as np
import cv2
import os

def load_T_robot_from_file(filename):
    """
    从 txt 文件读取所有保存的 4x4 变换矩阵
    :param filename: 存储的文件名，默认为 "T_E2B_data.txt"
    :return: 读取的所有 4x4 变换矩阵列表
    """

    absolute_filename = "/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/" + filename

    if not os.path.exists(absolute_filename):
        print("文件不存在，无法读取。")
        return []

    # 读取数据
    data = np.loadtxt(absolute_filename, delimiter=",")
    
    # 处理数据维度
    if data.ndim == 1:
        # 只有一行数据时，reshape 成 4x4
        return [data.reshape(4, 4)]
    else:
        # 多行数据，每 16 个值构成一个 4x4 矩阵
        return [row.reshape(4, 4) for row in data]

if __name__ == '__main__':

    T_E2B = load_T_robot_from_file("T_E2B_data.txt")
    T_O2C = load_T_robot_from_file("T_O2C_data.txt")

    # 初始化存储旋转矩阵和位移向量的列表
    R_gripper_to_base = []
    T_gripper_to_base = []
    R_target_to_camera = []
    T_target_to_camera = []

    # 处理机械臂数据
    for matrix in T_E2B:
        # 旋转矩阵 R 是前3行前3列
        R = matrix[:3, :3]
        R_gripper_to_base.append(R)
        # 位移向量 T 是前3行的第4列
        T = matrix[:3, 3]
        T_gripper_to_base.append(T)
    
    # 处理相机数据
    for matrix in T_O2C:
        R = matrix[:3, :3]
        R_target_to_camera.append(R)
        T = matrix[:3, 3]
        T_target_to_camera.append(T)
    
    # 将列表转换为 numpy 数组
    R_gripper_to_base = np.array(R_gripper_to_base)
    T_gripper_to_base = np.array(T_gripper_to_base)
    R_target_to_camera = np.array(R_target_to_camera)
    T_target_to_camera = np.array(T_target_to_camera)

    # 打印形状以确认数据格式正确
    print("R_gripper_to_base shape:", R_gripper_to_base.shape)
    print("T_gripper_to_base shape:", T_gripper_to_base.shape)
    print("R_target_to_camera shape:", R_target_to_camera.shape)
    print("T_target_to_camera shape:", T_target_to_camera.shape)
    print(R_gripper_to_base)
    print(R_target_to_camera)
    print(T_gripper_to_base)
    print(T_target_to_camera)

    # 计算手眼标定
    cali_R, cali_t = cv2.calibrateHandEye(
        R_gripper_to_base, T_gripper_to_base,
        R_target_to_camera, T_target_to_camera
    )

    print("\n标定结果：")
    print("旋转矩阵 R:")
    print(cali_R)
    print("变换向量 t:")
    print(cali_t)

    # 如果需要的话，可以将这些转换为 numpy 数组以便于进一步处理
    cali_R = np.array(cali_R)
    cali_t = np.array(cali_t).flatten()

    # 保存标定结果到文件
    np.savetxt("/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/handeye_R.txt", cali_R, delimiter=",")
    np.savetxt("/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/handeye_t.txt", cali_t, delimiter=",")