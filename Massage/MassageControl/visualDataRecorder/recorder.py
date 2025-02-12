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
L1 = 325
L2 = 275
L_end = 248

# 正运动学接受参数列表：[z q2 q3 q4] q1为移动副，采用z坐标
def FK(params):
    # DH参数表 [theta d a alpha flag]
    DH = np.array([
        [0,0,0,0,1], #关节1
        [0,0,325,0,0], # 2
        [0,0,275,0,0], # 3
        [0,0,248,0,0], # 4
        [0,0,0,0,0] # 5 暂时不设置旋转
    ])
    z,q2,q3,q4 = params
    q2*=np.pi/180
    q3*=np.pi/180
    q4*=np.pi/180

    # 计算转换矩阵T
    T = np.identity(4)
    for i in range(4):
        theta = DH[i,0]+(0 if i==0 else [q2,q3,q4][i-1])
        d = DH[i,1]+(0 if i!=0 else z)
        a = DH[i,2]
        alpha = DH[i,3]

        t = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0,0,0,1]
        ]) # t 表示相邻坐标系的坐标变换
        T = np.dot(T,t)
    return T

# 输出的Q4存在耦合，需要转换为q4
def Q4_2_q4(params):
    Q4 = params[-1]
    q2 = params[1]
    q3 = params[2]
    q4 = Q4-108-q2-q3
    return q4

def q4_2_Q4(params):
    q4 = params[-1]
    q2 = params[1]
    q3 = params[2]
    Q4 = q2+q3+q4+108


## 相机侧
def capture_and_save(cam):
    # 获取最新的RGB和深度图像
    rgb_image, depth_image, camera_intrinsics = cam.get_latest_frame()
    print(camera_intrinsics)

    max_depth = np.max(depth_image)
    # print(np.min(depth_image), np.max(depth_image))
    # print(depth_image[200, 320])
    depth_image = (depth_image / max_depth * 65535).astype(np.uint16)
    # print(np.min(depth_image), np.max(depth_image))

    # 对图像进行水平翻转
    rgb_image = cv2.flip(rgb_image, 1)
    depth_image = cv2.flip(depth_image, 1)

    # 检测ArUco标记
    corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, cv2.aruco.DICT_4X4_50, parameters=cv2.aruco.CORNER_REFINE_SUBPIX)

    if len(corners) > 0:
        # 绘制检测到的标记
        rgb_image = cv2.aruco.drawDetectedMarkers(rgb_image, corners, ids)

        # 获取每个标记的中心点，并结合深度信息
        for i, corner in enumerate(corners):
            # 计算标记的中心位置
            center = np.mean(corner[0], axis=0)
            x, y = int(center[0]), int(center[1])

            # 获取该位置的深度值
            depth_value = depth_image[y, x]  # 深度图像是行列格式

            # 显示深度信息
            print(f"Marker ID: {ids[i]} at ({x}, {y}) has depth value: {depth_value}")
            
            # 可以在图像上标记深度信息
            cv2.putText(rgb_image, f"Depth: {depth_value}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 显示图像
    cam.display_images(rgb_image, depth_image)

## 电机侧
def rotate_motor():
    odrv0 = odrive.find_any()
    odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
    odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
    odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
    #减速比 8：1 输入8旋转360°，负数反转
    odrv0.axis0.controller.input_pos = 0.5  # -4.4 为朝正下方
    return 

if __name__ == '__main__':
    # robot
    my_Huilin = Huilin()
    time.sleep(0.02)
    # cam
    cam = ToolCamera(host='127.0.0.1')
    cam.start()
    time.sleep(0.1)

    # 移动
    my_Huilin.robot.xyz_move(1,100,10)
    my_Huilin.robot.wait_stop()
    time.sleep(0.02)

    # 记录当前正运动学姿态矩阵
    pos_and_deg = my_Huilin.get_position_ZIWEI()
    print(pos_and_deg)
    q4 = Q4_2_q4(pos_and_deg[-4:])
    q = pos_and_deg[2:5]
    q.append(q4)
    T_E2B = FK(q)
    print(T_E2B)

    # capture_and_save()

