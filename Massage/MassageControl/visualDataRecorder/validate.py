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
from scipy.linalg import svd
'''
精度验证程序。

相对/绝对精度验证：计算相邻角点坐标后，计算相对距离。
'''
def euler_angles_to_rotation_matrix(rx, ry, rz):
    # 计算旋转矩阵
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    # R = Rz@Ry@Rx   #xyz
    R = Rx@Ry@Rz   #zyx

    return R

## 内参矩阵:
#  [[410.36722439   0.         316.147434  ]
#  [  0.         409.29416702 190.60068917]
#  [  0.           0.           1.        ]]
## 畸变系数:
#  [[-0.03036903  0.53345114 -0.0004762  -0.00000321 -2.361545  ]]

## T_handeye （eye in hand）
# rotation_matrix:
# [[ 1. -0. -0.]
#  [ 0.  1.  0.]
#  [ 0. -0.  1.]]
# translation_vector:
# [[ 0.09785933]
#  [ 0.04347791]
#  [-0.43874546]]

cam = ToolCamera(host='127.0.0.1')
cam.start()
time.sleep(0.1)

# 获取最新的RGB和深度图像
rgb_image, depth_image, _ = cam.get_latest_frame()

camera_matrix = np.array([[410.36722439, 0,          316.147434  ],
                [  0,         409.29416702, 190.60068917],
                [  0,           0,           1        ]])
camera_distortion = np.array([-0.03036903,0.53345114,-0.0004762,-0.00000321,-2.361545])

#获取像素坐标
px, py = [167.0,128.0]

depth_value = depth_image[int(py), int(px)]  # 深度值

# 首先对像素坐标进行去畸变
undistorted_point = cv2.undistortPoints(np.array([px, py]), camera_matrix, camera_distortion)
undistorted_px, undistorted_py = undistorted_point[0][0]

# 计算三维坐标
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

# 计算相机坐标系下的三维坐标
X_camera = (undistorted_px - cx) * depth_value / fx
Y_camera = (undistorted_py - cy) * depth_value / fy
Z_camera = depth_value

# 输出相机坐标系下的三维坐标
print(f"相机坐标系下的三维坐标: ({X_camera}, {Y_camera}, {Z_camera})")

X,Y,Z,RX,RY,RZ = [517.179869179316,-138.078831958831,0.0,-0.0,0,-1.731054708551505e-06]

R_arm2base = euler_angles_to_rotation_matrix(RX,RY,RZ)
t_arm2base = [X,Y,Z]
T_a2b = np.eye(4,4)
T_a2b[:3,:3] = R_arm2base
T_a2b[:3,3] = t_arm2base

T_eye2arm = np.array([[1,0,0,0.09785933],
                    [0,1,0,0.04347791],
                    [0,0,1,-0.43874546],
                    [0,0,0,1]])

pos_eye = np.array([X_camera,Y_camera,Z_camera,1])

pos_world = T_a2b @ T_eye2arm @ pos_eye.T

print(pos_world)

# [ 248.79670003 -300.22706186  347.56125454    1.        ]
# [ 246.55555411 -301.62023826  350.56125454    1.        ]