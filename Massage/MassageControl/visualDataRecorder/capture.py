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

def capture_and_save(cam,num):
    # 获取最新的RGB和深度图像
    rgb_image, depth_image, camera_intrinsics = cam.get_latest_frame()
    filename = "./visualDataRecorder/" + str(num)+".jpg"
    print(filename)
    cv2.imshow('img',rgb_image)
    cv2.waitKey()
    cv2.imwrite(filename,rgb_image)
    print(filename + "已保存")

def report_my_pos(robot,Z): # my_Huilin
    robot.robot.get_scara_param()
    print("XYR_OG",robot.robot.x,robot.robot.y,Z,robot.robot.r)
    cur_pos,cur_angle = robot.get_arm_position()
    print("cur_pos_with_extended_arm",cur_pos)

if __name__ == '__main__':

    # input info
    # Z-axis
    angle_Z = [20,225,80]
    # relative movement
    # direction - X
    dis_X = 75
    # direction - Y
    dis_Y = -45
    # X-axis -4.5 为原点 -4 为反转22.5度，-5 为正转22.5度




    input_X = -3.2
    # direction - Z
    dis_Z = 0
    # robot
    my_Huilin = Huilin()
    time.sleep(0.02)
    # cam
    cam = ToolCamera(host='127.0.0.1')
    cam.start()
    time.sleep(0.1)
    # motor
    odrv0 = odrive.find_any()
    odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
    odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
    odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
    ###################################
    # P2P
    my_Huilin.move_pose([249.17990112304688,-108.0781021118164,0],[0,0,118.0],lr=-1)
    my_Huilin.robot.wait_stop()
    ###################################
    # direction Z-motor
    my_Huilin._move_z_axis_p(dis_Z)
    time.sleep(1.5) # 阻塞
    # Z轴先转动
    # def new_movej_angle(self, goal_angle1, goal_angle2, goal_z, goal_r, speed,roughly) deg deg mm deg mm/s deg/s
    # my_Huilin.move_joint(angle_Z,0)
    # time.sleep(0.1)
    # 位置相对移动
    # my_Huilin.robot.xyz_move(1,dis_X,10)
    # my_Huilin.robot.wait_stop()
    # my_Huilin.robot.xyz_move(2,dis_Y,10)
    # my_Huilin.robot.wait_stop()
    # time.sleep(0.5)
    ###################################
    # X轴旋转
    #####################
    # input_pos = -4.2 # -4 为反转22.5度，-5 为正转22.5度
    ###################
    input_pos = input_X
    odrv0.axis0.controller.input_pos = input_X
    time.sleep(4)
    q5_deg = (input_pos+3.5)*(-45)
    print("q5_deg:", q5_deg)
    RX = np.deg2rad(q5_deg)
    # 记录当前正运动学姿态矩阵（不计末端轴旋转的正运动学矩阵，仅获取位置）
    pos_and_deg = my_Huilin.get_position_ZIWEI()
    print(pos_and_deg)
    Q4 = pos_and_deg[-1]-108
    input = pos_and_deg[-4:]
    q4 = Q4_2_q4(input)
    q = pos_and_deg[2:5]
    q.append(q4)
    T_E2B = FK(q)
    RY = 0
    RZ = np.deg2rad(Q4)

    pos = T_E2B[:3,3]
    ori = [RX,RY,RZ] # 记录顺序 X Y Z (rad) 还原时应采用计算顺序Z Y X
    print("pos:",pos)
    print("ori:",ori)
    
    # with open("./visualDataRecorder/pose.txt", "a") as f:
    #     f.write(f"{pos[0]/1000},{pos[1]/1000},{dis_Z/1000},{ori[0]},{ori[1]},{ori[2]}\n")
    
    # capture_and_save(cam,14)

    # test
    rgb_image, depth_image, camera_intrinsics = cam.get_latest_frame()
    cv2.imshow('img',rgb_image)
    cv2.waitKey()

    report_my_pos(my_Huilin,dis_Z)