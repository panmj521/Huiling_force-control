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
# 逆转换
def q4_2_Q4(params):
    q4 = params[-1]
    q2 = params[1]
    q3 = params[2]
    Q4 = q2+q3+q4+108
    return Q4


# 创建 ArUco 网格码板的函数

def create_grid_board(markers_x, markers_y, marker_length, marker_separation, dictionary, first_marker_id):
    # 生成每个标记的 ID，从 first_marker_id 开始
    ids = np.array([[i] for i in range(first_marker_id, first_marker_id + markers_x * markers_y)])

    # 创建网格码板并设置标记的 ID
    gridboard = cv2.aruco.GridBoard([markers_x, markers_y], marker_length, marker_separation, dictionary)
    
    # 设置网格码板的标记 ID
    # gridboard.ids = ids
    
    return gridboard

def capture_and_save(cam,gridboard):
    # 获取最新的RGB和深度图像
    rgb_image, depth_image, camera_intrinsics = cam.get_latest_frame()
    
    print(camera_intrinsics)

    camera_matrix = np.array([
    [camera_intrinsics['fx'], 0, camera_intrinsics['cx']],
    [0, camera_intrinsics['fy'], camera_intrinsics['cy']],
    [0, 0, 1]
    ])
    
    # [fx, 0, cx],
    # [0, fy, cy],
    # [0, 0, 1]
    
    # 提取畸变系数字典
    distortion_coeffs_dict = camera_intrinsics['distortion_coeffs']

    # 将畸变系数转化为 numpy 数组
    dist_coeffs = np.array([distortion_coeffs_dict['k1'], 
                        distortion_coeffs_dict['k2'], 
                        distortion_coeffs_dict['p1'], 
                        distortion_coeffs_dict['p2'], 
                        distortion_coeffs_dict['k3']])
    # dist_coeffs = np.array([k1, k2, p1, p2, k3])

    max_depth = np.max(depth_image)
    depth_image = (depth_image / max_depth * 65535).astype(np.uint16)

    # # 对图像进行水平和垂直翻转
    # rgb_image = cv2.flip(rgb_image, 1)  # 水平翻转
    # depth_image = cv2.flip(depth_image, 1)
    # rgb_image = cv2.flip(rgb_image, 0)  # 垂直翻转
    # depth_image = cv2.flip(depth_image, 0)

    # 将RGB图像转为灰度图像
    gray_rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 角点精细化

    aruco_dict = cv2.aruco.Dictionary(cv2.aruco.DICT_APRILTAG_36H11,3)

    # 检测ArUco标记
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(rgb_image, aruco_dict, parameters=parameters)
    print("Detected corners:", len(corners))

    # 用于存储有效的标记
    valid_corners = []
    valid_ids = []
    valid_3d_points = []  # 存储三维坐标

    for i, corner in enumerate(corners):
        # 计算标记的中心位置
        center = np.mean(corner[0], axis=0)
        x, y = int(center[0]), int(center[1])

        # 获取该位置的深度值
        depth_value = depth_image[y, x]  # 深度图像是行列格式

        # 过滤深度大于35000和深度小于10的点
        if depth_value > 10 and depth_value < 35000:
            valid_corners.append(corner)  # 保留有效的角点
            valid_ids.append(ids[i])      # 保留有效的标记ID
            # 计算在相机坐标系中的三维坐标
            depth_value /= 100 # convert to millimetre
            X = (x - camera_matrix[0, 2]) * depth_value / camera_matrix[0, 0]
            Y = (y - camera_matrix[1, 2]) * depth_value / camera_matrix[1, 1]
            Z = float(depth_value)
            valid_3d_points.append([X, Y, Z])  # 存储三维坐标

    print("valid corners:", len(valid_corners))
    
    # 将 valid_corners 转换为 numpy 数组, 并确保数据类型为 float32
    valid_corners = np.array(valid_corners, dtype=np.float32)
    print(valid_corners)
    print(type(valid_corners))
    # 将 valid_ids 转换为 numpy 数组, 并确保数据类型为 int32
    valid_ids = np.array(valid_ids, dtype=np.int32)

    # 将 valid_3d_points 转换为 numpy 数组, 并确保数据类型为 float32
    valid_3d_points = np.array(valid_3d_points, dtype=np.float32)
    print(valid_3d_points)

    if len(valid_corners) > 0:
        # 绘制检测到的标记
        rgb_image = cv2.aruco.drawDetectedMarkers(rgb_image, valid_corners, valid_ids)

        # 计算 ArUco 标定板相对于相机的位姿
        # 使用cv2.aruco.estimatePoseBoard来估算标定板的位姿
        rvec = None
        tvec = None

        num, rvec, tvec= cv2.aruco.estimatePoseBoard(valid_3d_points, valid_ids, gridboard, camera_matrix, dist_coeffs, rvec, tvec)
        print(num)
        if num:
            # 绘制标定板的位置和方向
            cv2.drawFrameAxes(rgb_image, camera_matrix, dist_coeffs, rvec, tvec, 40)  # 100 是绘制坐标轴的长度

            # 输出位姿信息
            print("Rotation Vector:\n", rvec)
            print("Translation Vector:\n", tvec)

            # 获取标定板的中心点并结合深度信息
            for i, corner in enumerate(valid_corners):
                # 计算标记的中心位置
                center = np.mean(corner[0], axis=0)
                x, y = int(center[0]), int(center[1])

                # 获取该位置的深度值
                depth_value = depth_image[y, x]/100  # 深度图像是行列格式
                
                # 显示深度信息
                print(f"Marker ID: {ids[i]} at ({x}, {y}) has depth value: {depth_value}")

                # 可以在图像上标记深度信息
                cv2.putText(rgb_image, f"Depth: {depth_value}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 显示图像
    cam.display_images(rgb_image, depth_image)

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

def tsai_lenz(A_list, B_list):
    """
    Tsai-Lenz hand-eye calibration method to solve AX = XB.
    
    A_list: list of A_i (robot transformation matrices)
    B_list: list of B_i (camera transformation matrices)
    """
    assert len(A_list) == len(B_list), "A and B lists must have the same length."
    
    # Initialize matrices
    n = len(A_list)
    A_stack = np.zeros((n * 3, 3))
    B_stack = np.zeros((n * 3, 3))
    b_stack = np.zeros((n * 3, 1))
    
    for i in range(n):
        A = A_list[i][:3, :3]  # Rotation part of A_i
        B = B_list[i][:3, :3]  # Rotation part of B_i
        t = A_list[i][:3, 3]   # Translation part of A_i
        
        # Stack the equations
        A_stack[i * 3: (i + 1) * 3, :] = A - B
        b_stack[i * 3: (i + 1) * 3, :] = t
        
    # Use least squares to solve the linear system
    X = np.linalg.lstsq(A_stack, b_stack, rcond=None)[0]
    
    return X

def save_T_robot_to_file(T, filename="T_E2B_data.txt"):
    """
    将4x4齐次变换矩阵 T 保存到 txt 文件，并支持追加写入。
    :param T: 4x4 变换矩阵 (numpy array)
    :param filename: 存储的文件名，默认为 "T_E2B_data.txt"
    """
    filename = "/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/" + filename
    if T.shape != (4, 4):
        raise ValueError("输入的 T 必须是 4x4 变换矩阵")
    
    # 将 T 展平成 1 行，方便存储
    T_flat = T.flatten()
    
    # 判断文件是否存在
    if not os.path.exists(filename):
        # 如果文件不存在，写入新的数据
        np.savetxt(filename, [T_flat], delimiter=",", fmt="%.6f")
    else:
        # 如果文件已存在，追加数据
        with open(filename, "a") as f:
            np.savetxt(f, [T_flat], delimiter=",", fmt="%.6f")

    print(f"变换矩阵 T 已成功追加保存至 {filename}")

def load_T_robot_from_file(filename="T_E2B_data.txt"):
    """
    从 txt 文件读取所有保存的 4x4 变换矩阵
    :param filename: 存储的文件名，默认为 "T_E2B_data.txt"
    :return: 读取的所有 4x4 变换矩阵列表
    """
    if not os.path.exists(filename):
        print("文件不存在，无法读取。")
        return []

    # 读取数据
    data = np.loadtxt(filename, delimiter=",")
    
    # 处理数据维度
    if data.ndim == 1:
        # 只有一行数据时，reshape 成 4x4
        return [data.reshape(4, 4)]
    else:
        # 多行数据，每 16 个值构成一个 4x4 矩阵
        return [row.reshape(4, 4) for row in data]

if __name__ == '__main__':
    # robot
    # my_Huilin = Huilin()
    # time.sleep(0.02)
    # cam
    cam = ToolCamera(host='127.0.0.1')
    cam.start()
    time.sleep(0.1)

    markers_x = 7
    markers_y = 5
    marker_length = 3  # 3 cm
    marker_separation = 0.6  # 0.6 cm
    first_marker_id = 0

    # 创建网格码板
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    grid_board = create_grid_board(markers_x, markers_y, marker_length, marker_separation, aruco_dict, first_marker_id)

    # 显示网格码板图像（可选）
    img_grid_board = grid_board.generateImage(outSize=[800,600])
    # cv2.imshow('GridBoard', img_grid_board)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # 移动
    # my_Huilin.robot.xyz_move(1,100,10)
    # my_Huilin.robot.wait_stop()
    # my_Huilin.robot.xyz_move(2,50,10)
    # my_Huilin.robot.wait_stop()
    # time.sleep(0.02)

    # 记录当前正运动学姿态矩阵
    # pos_and_deg = my_Huilin.get_position_ZIWEI()
    # print(pos_and_deg)
    # q4 = Q4_2_q4(pos_and_deg[-4:])
    # q = pos_and_deg[2:5]
    # q.append(q4)
    # T_E2B = FK(q)
    # print(T_E2B)
    # save_T_robot_to_file(T_E2B)

    capture_and_save(cam,grid_board)

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