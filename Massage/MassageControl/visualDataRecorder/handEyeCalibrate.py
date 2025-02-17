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
    
    # 初始化旋转矩阵和位移向量
    RA = np.zeros((3, 3, n))
    RB = np.zeros((3, 3, n))
    tA = np.zeros((3, n))
    tB = np.zeros((3, n))
    
    for i in range(n):
        A_mat = E2B[i, :, :].reshape(4, 4)
        B_mat = O2C[i, :, :].reshape(4, 4)
        
        RA[:, :, i] = A_mat[:3, :3]
        tA[:, i] = A_mat[:3, 3]
        RB[:, :, i] = B_mat[:3, :3]
        tB[:, i] = B_mat[:3, 3]
    print("tB",tB)
    print("A_mat",A_mat)
    # 计算旋转矩阵 RX
    M = np.zeros((3, 3))
    for i in range(n):
        M += RB[:, :, i] @ RA[:, :, i].T
    U, _, Vt = np.linalg.svd(M)
    RX = U @ Vt
    
    # 计算平移向量 tX
    A = np.zeros((3 * n, 3))
    b = np.zeros((3 * n, 1))
    
    for i in range(n):
        A[3 * i:3 * i + 3, :] = np.eye(3) - RA[:, :, i]
        b[3 * i:3 * i + 3, :] = tA[:, i].reshape(-1, 1) - RX @ tB[:, i].reshape(-1, 1)
    tX = np.linalg.pinv(A) @ b
    print("tX",tX)

    # 输出结果 X
    X = np.eye(4)
    X[:3, :3] = RX
    X[:3, 3] = tX.flatten()

    return X

if __name__ == '__main__':

    T_E2B = load_matrix("T_E2B_data.txt")
    T_O2C = load_matrix("T_O2C_data.txt")

    sol = tsai(T_E2B,T_O2C)
    print("X=",sol)

    vali_1 = T_E2B[0] @ sol @ T_O2C[0]
    print("Validation 1:", vali_1)
    vali_2 = T_E2B[1] @ sol @ T_O2C[1]
    print("Validation 2:", vali_2)

    error = vali_2 - vali_1
    print("Error:", error)

    # 将结果保存到文件
    # np.savetxt("/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/handeye_R.txt", cali_R, delimiter=",")
    # np.savetxt("/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/visualDataRecorder/handeye_t.txt", cali_t, delimiter=",")