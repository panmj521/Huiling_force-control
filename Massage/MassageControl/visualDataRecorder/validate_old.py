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

cam = ToolCamera(host='127.0.0.1')
cam.start()
time.sleep(0.1)

# 获取最新的RGB和深度图像
rgb_image, depth_image, camera_intrics = cam.get_latest_frame()

cv2.imwrite("./visualDataRecorder/rgb_test.jpg",rgb_image)
cv2.imwrite("./visualDataRecorder/depth_test.jpg",depth_image)
cv2.imshow("see",rgb_image)
cv2.waitKey()
# camera_matrix = np.array([[418.93448339, 0,          319.08389007  ],
#                 [  0,         419.14056955, 188.40974033],
#                 [  0,           0,           1        ]])
# camera_distortion = np.array([-0.03162714,0.53027997,-0.0011312,0.00193427,-2.09467312])
# camera_matrix = camera_intrics["camera_intrics"]
camera_distortion = camera_intrics["distortion_coeffs"]
camera_matrix = np.array([[camera_intrics["fx"], 0,          camera_intrics['cx']  ],
                [  0,         camera_intrics["fy"], camera_intrics['cy']],
                [  0,           0,           1        ]])
camera_distortion = np.array([camera_distortion['k1'],camera_distortion['k2'],camera_distortion['p1'],camera_distortion['p2'],camera_distortion['k3']])
# print(camera_intrics)
#获取像素坐标
# 263 50
# 301 65
px, py = [301.0,65.0]

depth_value = depth_image[int(py), int(px)]  # 深度值
print(depth_value)
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
X_camera /= 1000
Y_camera /= 1000
Z_camera /= 1000
# 输出相机坐标系下的三维坐标
print(f"相机坐标系下的三维坐标: ({X_camera}, {Y_camera}, {Z_camera})")

X,Y,Z,RX,RY,RZ = [0.4934117008920783,-0.06501354860754606,0.0,-0.23561944901923437,0,0.17453292519943295]

R_arm2base = euler_angles_to_rotation_matrix(RX,RY,RZ)
t_arm2base = [X,Y,Z]
T_a2b = np.eye(4,4)
T_a2b[:3,:3] = R_arm2base
T_a2b[:3,3] = t_arm2base
print("t_arm2base",t_arm2base)
# T_eye2arm = np.array([[-0.95929473,0.28240684,0.00008075,-0.00982097],
#                     [0.24784548,0.84175772,0.47960041,-0.13006099],
#                     [0.13537446,0.4600981,-0.877487,-0.09190229],
#                     [0,0,0,1]])
T_eye2arm = np.array([[-0.9955406,0.09182738,-0.0216019,0.0024927],
                    [0.08717433,0.98304449,0.16132007,-0.04637209],
                    [0.03604922,0.15871755,-0.98666569,-0.05493419],
                    [0,0,0,1]])

pos_eye = np.array([X_camera,Y_camera,Z_camera,1]).T

pos_end = T_eye2arm @ pos_eye
pos_world = T_a2b @ T_eye2arm @ pos_eye
print("眼坐标:",pos_eye)
print("末端坐标:",pos_end)
print("基坐标系:",pos_world)

# 基坐标系: [ 0.73717172 -0.31291418 -0.49882502  1.        ] 301,65
# 基坐标系: [ 0.7348124  -0.31189614 -0.49420095  1.        ] 263,50
# [ 0.78357916 -0.29953172 -0.41916194  1.        ]
# [ 0.78647672 -0.3014011  -0.42317373  1.        ]
pos_1 = [0.78357916,-0.29953172,-0.41916194]
pos_2 = [0.78647672,-0.3014011,-0.42317373]
x1,y1,z1 = pos_1
x2,y2,z2 = pos_2
relative_dis = math.sqrt((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)
print(relative_dis)
cam.stop()