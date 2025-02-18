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
    z,q2,q3,q4= params
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
# 逆转换
def q4_2_Q4(params):
    q4 = params[-1]
    q2 = params[1]
    q3 = params[2]
    Q4 = q2+q3+q4+108
    return Q4

def capture_and_save(cam):
    # 加载ArUco字典和检测参数
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    # 获取图像,相机内参矩阵
    rgb_image, depth_image, camera_intrinsics = cam.get_latest_frame()
    
    # print(camera_intrinsics)

    camera_matrix = np.array([
    [camera_intrinsics['fx'], 0, camera_intrinsics['cx']],
    [0, camera_intrinsics['fy'], camera_intrinsics['cy']],
    [0, 0, 1]
    ])

    distortion_coeffs_dict = camera_intrinsics['distortion_coeffs']

    dist_coeffs = np.array([distortion_coeffs_dict['k1'], 
                        distortion_coeffs_dict['k2'], 
                        distortion_coeffs_dict['p1'], 
                        distortion_coeffs_dict['p2'], 
                        distortion_coeffs_dict['k3']],
                        dtype=float)

    # 深度图
    max_depth = np.max(depth_image)
    depth_image = (depth_image / max_depth * 65535).astype(np.uint16)

    # # 对图像进行水平和垂直翻转
    # rgb_image = cv2.flip(rgb_image, 1)  # 水平翻转
    # depth_image = cv2.flip(depth_image, 1)
    # rgb_image = cv2.flip(rgb_image, 0)  # 垂直翻转
    # depth_image = cv2.flip(depth_image, 0)

    # 将RGB图像转为灰度图像
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 角点精细化

    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # 绘制检测到的标记
        rgb_image = cv2.aruco.drawDetectedMarkers(rgb_image, corners, ids)
        
        # 估计每个标记的位姿
        for i in range(len(ids)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.1, camera_matrix, dist_coeffs)  # marker尺寸 0.1m
            
            # 绘制坐标轴
            cv2.drawFrameAxes(rgb_image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)  # 绘制坐标轴
            # 转换单位
            tvec *= 1000
            # 输出标记的位姿
            print(f"Marker ID: {ids[i]}")
            print(f"Rotation Vector: {rvec}")
            print(f"Translation Vector: {tvec}")

    # # 显示图像
    # cam.display_images(rgb_image, depth_image)
    cv2.imshow('out',rgb_image)
    cv2.waitKey()
    filename = "ArUco_6x6_" + str(id) + ".png"
    cv2.imwrite(filename,rgb_image)
    # 使用cv2.Rodrigues()将旋转向量rvec转换为旋转矩阵
    R, _ = cv2.Rodrigues(rvec)

    # 构造齐次变换矩阵
    T = np.eye(4)  # 创建4x4单位矩阵
    T[:3, :3] = R  # 将旋转矩阵填入上三行三列
    T[:3, 3] = tvec.flatten()  # 将平移向量填入最后一列

    print("齐次变换矩阵：")
    print(T)
    save_T_robot_to_file(T,'T_O2C_data.txt')

def rotation_vector_to_euler(rotation_vector):
    # 将旋转向量转换为旋转矩阵
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    
    # 计算欧拉角 (XYZ 旋转顺序)
    sy = np.sqrt(rotation_matrix[0,0]**2 + rotation_matrix[1,0]**2)

    singular = sy < 1e-6  # 判断是否接近奇异情况

    if not singular:
        x_angle = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2])
        y_angle = np.arctan2(-rotation_matrix[2,0], sy)
        z_angle = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
    else:
        x_angle = np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1])
        y_angle = np.arctan2(-rotation_matrix[2,0], sy)
        z_angle = 0  # 奇异情况时，Z 角设为 0

    # 转换为角度
    euler_angles = np.degrees([x_angle, y_angle, z_angle])

    return euler_angles

## 电机侧
def rotate_motor():
    odrv0 = odrive.find_any()
    odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
    odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
    odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
    #减速比 8：1 输入8旋转360°，负数反转
    odrv0.axis0.controller.input_pos = 0.5  # -4.4 为朝正下方
    return 

def save_T_robot_to_file(T, filename):
    """
    将4x4齐次变换矩阵 T 保存到 txt 文件，并支持追加写入。
    :param T: 4x4 变换矩阵 (numpy array)
    :param filename: 存储的文件名，默认为 "T_E2B_data.txt"
    """
    absolute_filename = "/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/" + filename
    if T.shape != (4, 4):
        raise ValueError("输入的 T 必须是 4x4 变换矩阵")
    
    # 将 T 展平成 1 行，方便存储
    T_flat = T.flatten()
    
    # 判断文件是否存在
    if not os.path.exists(absolute_filename):
        # 如果文件不存在，写入新的数据
        np.savetxt(absolute_filename, [T_flat], delimiter=",", fmt="%.6f")
    else:
        # 如果文件已存在，追加数据
        with open(absolute_filename, "a") as f:
            np.savetxt(f, [T_flat], delimiter=",", fmt="%.6f")

    print(f"变换矩阵 T 已成功追加保存至 {absolute_filename}")

if __name__ == '__main__':
    # robot
    my_Huilin = Huilin()
    time.sleep(0.02)
    # cam
    cam = ToolCamera(host='127.0.0.1')
    cam.start()
    time.sleep(0.1)
    # motor
    # odrv0 = odrive.find_any()
    # odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
    # odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
    # odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL

    # 末端轴旋转
    #####################
    input_pos = -4.5
    #####################
    # odrv0.axis0.controller.input_pos = input_pos
    q5_deg = (input_pos+4.5)*(-45)
    print("q5_deg:", q5_deg)
    q5 = (q5_deg)*(np.pi/180)
    time.sleep(1)
    ###################################
    # 移动
    # my_Huilin.robot.xyz_move(1,120,10)
    # my_Huilin.robot.wait_stop()
    # my_Huilin.robot.xyz_move(2,85,10)
    # my_Huilin.robot.wait_stop()
    time.sleep(0.5)
    ###################################

    # 记录当前正运动学姿态矩阵
    pos_and_deg = my_Huilin.get_position_ZIWEI()
    print(pos_and_deg)
    q4 = Q4_2_q4(pos_and_deg[-4:])
    q = pos_and_deg[2:5]
    q.append(q4)
    T_E2B = FK(q)
    # print(T_E2B)
    
    t_1 = np.array([
            [0,0,1,0],
            [0,1,0,0],
            [-1,0,0,0],
            [0,0,0,1]
        ]) # t 表示相邻坐标系的坐标变换
    t_2 = np.array([
            [np.cos(q5), -np.sin(q5), 0, 0],
            [np.sin(q5), np.cos(q5), 0, 0],
            [0, 0, 1, 0],
            [0,0,0,1]])
    
    T_E2B = np.dot(T_E2B,t_1)
    T_E2B = np.dot(T_E2B,t_2)
    print(T_E2B)
    # save_T_robot_to_file(T_E2B,"T_E2B_data.txt")

    # capture_and_save(cam)







    #################################################################################
    # # 给定的旋转向量
    # rotation_vector = np.array([[-0.0911708], [-0.24094092], [0.10618752]])

    # # 计算欧拉角
    # euler_angles = rotation_vector_to_euler(rotation_vector)

    # # 输出欧拉角 (绕 X, Y, Z 轴的旋转角度)
    # print("Euler Angles (degrees):", euler_angles)

    # Rotation Vector:
    #  [[0.19932134]
    #  [0.01549496]
    #  [0.06967173]]
    # Translation Vector:
    #  [[-31.59261508]
    #  [  4.44400312]
    #  [182.41367811]]