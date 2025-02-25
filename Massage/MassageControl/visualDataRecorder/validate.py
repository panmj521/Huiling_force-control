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
click_info = None

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
# 创建鼠标点击事件
def get_pixel_info(event, x, y, flags, param):
    global click_info
    if event == cv2.EVENT_MOUSEMOVE:  # 当鼠标移动时
        # 获取鼠标位置的像素值
        pixel_value = rgb_image[y, x]
        # 将像素值和坐标信息转换为字符串
        pixel_info = f"(x: {x}, y: {y}) - B: {pixel_value[0]}, G: {pixel_value[1]}, R: {pixel_value[2]}"
        # 在图像上显示像素信息
        image_with_info = rgb_image.copy()
        # 在鼠标位置附近显示信息
        cv2.putText(image_with_info, pixel_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('out', image_with_info)
    if event == cv2.EVENT_LBUTTONDOWN:
        click_info = [x,y]

my_Huilin = Huilin()
print("arm going home")
time.sleep(5)
coor_z = 140 # 140
input_RZ = 68.0 # 68.0
# my_Huilin._move_z_axis_p(coor_z)
# print("arm going to target")
# time.sleep(14) # 阻塞
# P2P
# my_Huilin.move_pose([249.17990112304688,-148.0781021118164,0],[0,0,input_RZ],lr=-1) # -148
# my_Huilin.robot.wait_stop()

cam = ToolCamera(host='127.0.0.1')
cam.start()
time.sleep(0.1)

# 获取最新的RGB和深度图像
rgb_image, depth_image, _ = cam.get_latest_frame()

camera_matrix = np.array([[417.41134845, 0,          320.66502839  ],
                [  0,         417.24282371, 190.14642013],
                [  0,           0,           1        ]])
camera_distortion = np.array([-0.03054865,0.62333924,-0.00001519,0.00314861,-3.13146461])
# 检查图像是否成功加载
if rgb_image is None:
    print("Error: 图像未找到或无法加载！")
else:
    # 创建窗口并绑定鼠标回调函数
    cv2.namedWindow('out')
    cv2.setMouseCallback('out', get_pixel_info)

    # 显示图像
    cv2.imshow('out', rgb_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("pixel_pos:",click_info)

#获取像素坐标
# px, py = [167.0,128.0]
px = float(click_info[0])
py = float(click_info[1])
depth_value = depth_image[int(py), int(px)]  # 深度值

# 首先对像素坐标进行去畸变
# 去畸变步结果有误
# undistorted_point = cv2.undistortPoints(np.array([px, py]), camera_matrix, camera_distortion)
# undistorted_px, undistorted_py = undistorted_point[0][0]

# 计算三维坐标
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

# 计算相机坐标系下的三维坐标
X_camera = (px - cx) * depth_value / fx
Y_camera = (py - cy) * depth_value / fy
Z_camera = depth_value

# 输出相机坐标系下的三维坐标
print(f"相机坐标系下的三维坐标: ({X_camera}, {Y_camera}, {Z_camera})")
cur_angle,cur_pos = my_Huilin.get_scara_ZIWEI()
print("cur_angle",cur_angle)
print("cur_pos",cur_pos)
# pose
RZ_deg = input_RZ-108
RZ_rad = np.deg2rad(RZ_deg)
X,Y,Z,RX,RY,RZ = [cur_pos[0],cur_pos[1],coor_z, 0.0, 0.0, RZ_rad]
R_arm2base = euler_angles_to_rotation_matrix(RX,RY,RZ)
t_arm2base = [X,Y,Z]
T_a2b = np.eye(4,4)
T_a2b[:3,:3] = R_arm2base
T_a2b[:3,3] = t_arm2base

# Tsai
# T_eye2arm = np.array([[-0.96933825,0.24532823,0.01405038,-0.04136888],
#                     [0.20219851,0.76382047,0.61293887,-0.14899204],
#                     [0.13963924,0.59698606,-0.7900054,-0.08347833],
#                     [0,0,0,1]])
T_eye2arm = np.array([[-0.9955406,0.09182738,-0.0216019,0.0024927],
                    [0.08717433,0.98304449,0.16132007,-0.04637209],
                    [0.03604922,0.15871755,-0.98666569,-0.05493419],
                    [0,0,0,1]])

pos_eye = np.array([X_camera,Y_camera,Z_camera,1])

pos_world = T_a2b @ T_eye2arm @ pos_eye.T

print(pos_world)

# # 逆运动学
desire_joint,test = my_Huilin.inverse_kinematic(cur_angle,[pos_world[0],pos_world[1]])
print("desired joint:",desire_joint)
cam.stop()

my_Huilin.move_joint(desire_joint,0)
my_Huilin.wait_stop()
_,cur_pos = my_Huilin.get_scara_ZIWEI()
print(cur_pos)
# [ 27.22009831 240.12281662  79.37744026] move joint