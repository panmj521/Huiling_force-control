import numpy as np
import cv2

def load_matrix(fileName):
    absolute_filename = "./visualDataRecorder/" + fileName
    data = []
    with open(absolute_filename, 'r') as file:
        for row in file:
            values = list(map(float, row.strip().split(',')))
            matrix = np.array(values).reshape(4, 4)
            data.append(matrix)
    return np.array(data)

def tsai(E2B, O2C):
    n = E2B.shape[0]
    
    # 将旋转矩阵和位移向量转换为列表形式
    RA = [E2B[i, :3, :3] for i in range(n)]
    tA = [E2B[i, :3, 3] for i in range(n)]
    RB = [O2C[i, :3, :3] for i in range(n)]
    tB = [O2C[i, :3, 3] for i in range(n)]
    
    # 检查转换后的形状
    assert all(r.shape == (3,3) for r in RA), "RA 的每个元素应为 3x3"
    assert all(r.shape == (3,) for r in tA), "tA 的每个元素应为 3 元素向量"
    assert all(r.shape == (3,3) for r in RB), "RB 的每个元素应为 3x3"
    assert all(r.shape == (3,) for r in tB), "tB 的每个元素应为 3 元素向量"
    
    # 调用手眼标定函数
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        RA, tA,
        RB, tB
    )

    return R_cam2gripper, t_cam2gripper

if __name__ == '__main__':
    T_E2B = load_matrix("T_E2B_data.txt")
    T_O2C = load_matrix("T_O2C_data.txt")
    
    R_cam2gripper, t_cam2gripper = tsai(T_E2B, T_O2C)
    print("相机与机械臂末端的旋转矩阵：\n", R_cam2gripper)
    print("相机与机械臂末端的平移向量：\n", t_cam2gripper)