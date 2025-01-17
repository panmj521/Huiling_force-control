import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import BSpline
from scipy.interpolate import make_interp_spline
from scipy.signal import savgol_filter

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import copy

def linear_interpolate(positions=None, quaternions=None, time_points=None, time_step=0.1):
    """
    进行位置和/或姿态的线性插值，支持多个点
    :param positions: 位置序列，单位为米 (Nx2 或 Nx3 numpy array, 可选)
    :param quaternions: 姿态序列，四元数 (Nx4 numpy array, 可选)
    :param time_points: 时间点序列 (N个浮点数)
    :param time_step: 离散时间步长，单位为秒 (float)
    :return: 插值序列，包含位置序列、姿态序列或两者兼有
    """
    if positions is None and quaternions is None:
        raise ValueError("至少需要输入位置或姿态的序列")
    if time_points is None or len(time_points) < 2:
        raise ValueError("需要提供至少两个时间点")

    # 由于浮点数精度问题，time_step要乘0.9
    times = np.arange(time_points[0], time_points[-1] + time_step * 0.9, time_step)

    if positions is not None:
        all_positions = np.zeros((len(times), positions.shape[1]))
        segment_durations = np.diff(time_points)
        segment_counts = np.floor(segment_durations / time_step).astype(int)
        current_idx = 0
        for i in range(len(segment_counts)):
            segment_times = np.linspace(0, segment_durations[i], segment_counts[i] + 1)
            if i > 0:
                segment_times = segment_times[1:]
            start_pos = positions[i]
            end_pos = positions[i + 1]
            segment_positions = start_pos + np.outer(segment_times, (end_pos - start_pos) / segment_durations[i])
            all_positions[current_idx:current_idx + len(segment_positions)] = segment_positions
            current_idx += len(segment_positions)

         # 确保最后一个位置被处理
        if current_idx < len(times):
            all_positions[current_idx:] = positions[-1]

    if quaternions is not None:
        slerp = Slerp(time_points, R.from_quat(quaternions))
        all_quaternions = slerp(times).as_quat()

    if positions is not None and quaternions is not None:
        return all_positions, all_quaternions
    elif positions is not None:
        return all_positions
    elif quaternions is not None:
        return all_quaternions

def spline_interpolate(positions=None, quaternions=None, time_points=None, time_step=0.1):
    """
    进行位置和/或姿态的样条插值，支持多个点
    :param positions: 位置序列，单位为米 (Nx2 或 Nx3 numpy array, 可选)
    :param quaternions: 姿态序列，四元数 (Nx4 numpy array, 可选)
    :param time_points: 时间点序列 (N个浮点数)
    :param time_step: 离散时间步长，单位为秒 (float)
    :return: 插值序列，包含位置序列、姿态序列或两者兼有
    """
    if positions is None and quaternions is None:
        raise ValueError("至少需要输入位置或姿态的序列")
    if time_points is None or len(time_points) < 2:
        raise ValueError("需要提供至少两个时间点")

    all_positions = []
    all_quaternions = []
    times = np.arange(time_points[0], time_points[-1] + time_step * 0.9, time_step)

    if positions is not None:
        cs = CubicSpline(time_points, positions, axis=0)
        all_positions = cs(times)

    if quaternions is not None:
        slerp = Slerp(time_points, R.from_quat(quaternions))
        all_quaternions = slerp(times).as_quat()

    if positions is not None and quaternions is not None:
        return np.array(all_positions), np.array(all_quaternions)
    elif positions is not None:
        return np.array(all_positions)
    elif quaternions is not None:
        return np.array(all_quaternions)



def generate_circle_trajectory(center, omega=0.4, radius=0.02, reverse=False, time_points=None, time_step=0.01, start_transition_duration=None, end_transition_duration=None):
    """
    Generate a 3D trajectory of a circular motion from the center to the specified radius.

    Parameters:
    center (list): The center of the circle [x, y, z , r , p , y].
    omega (float): The angular velocity.
    radius (float): The radius of the circle.
    reverse (bool): If True, rotates counterclockwise. If False, rotates clockwise.
    time_points (list or np.ndarray): List or array of time points.
    time_step (float): Time step for generating the trajectory.
    start_transition_duration (float): Duration for the transition at the start.
    end_transition_duration (float): Duration for the transition at the end.

    Returns:
    np.ndarray: Array of positions over time.
    """

    # print(time_points)
    if time_points is None or len(time_points) < 2:
        raise ValueError("需要提供至少两个时间点")
    
    
    if start_transition_duration is None:
        start_transition_duration = 2

    if end_transition_duration is None:
        end_transition_duration = 2
    t_points = time_points.copy()
    t_points[-1] = time_points[-1] + end_transition_duration + start_transition_duration
    t_points[-1] = round(t_points[-1] / time_step) * time_step
    times = np.arange(t_points[0], t_points[-1] + time_step * 0.9, time_step)

    if reverse:
        angles = -omega * times
    else:
        angles = omega * times

    radii = np.ones_like(times) * radius

    start_transition = times < start_transition_duration
    end_transition = times > (times[-1] - end_transition_duration)
    radii[start_transition] = radius * (1 - np.cos(np.pi * times[start_transition] / start_transition_duration)) / 2
    radii[end_transition] = radius * (1 + np.cos(np.pi * (times[end_transition] - (times[-1] - end_transition_duration)) / end_transition_duration)) / 2
    
    x_positions = center[0][0] + radii * np.cos(angles)
    y_positions = center[0][1] + radii * np.sin(angles)
    z_positions = np.full_like(x_positions, center[0][2])  # Z position remains constant

    positions = np.column_stack((x_positions, y_positions, z_positions))
    return positions



# def cloud_point_interpolate(positions=None, quaternions=None, time_points=None, time_step=0.1):
#     """
#     进行输入的点云位置和/或姿态的曲线插值，支持多个点，采用B样条插值法和四元数的Squad插值法。
#     :param positions: 位置序列，单位为米 (Nx2 或 Nx3 numpy array, 可选)
#     :param quaternions: 姿态序列，四元数 (Nx4 numpy array, 可选)
#     :param time_points: 时间点序列 (N个浮点数)
#     :param time_step: 离散时间步长，单位为秒 (float)
#     :return: 插值序列，包含位置序列、姿态序列或两者兼有
#     """
#     if positions is None and quaternions is None:
#         raise ValueError("至少需要输入位置或姿态的序列")
#     if time_points is None or len(time_points) < 2:
#         raise ValueError("需要提供至少两个时间点")

#     # 确保 positions 是至少二维的
#     temp_positions = np.atleast_2d(np.array(positions))

#     # 检查 quaternions 的形状，确保其为 Nx4 的四元数
#     if quaternions is not None:
#         temp_quaternions = np.array(quaternions)
#         if temp_quaternions.shape[-1] == 3:  # 如果是欧拉角 (N, 3)
#             temp_quaternions = R.from_euler('xyz', temp_quaternions).as_quat()  # 转换为四元数 (N, 4)
#         elif temp_quaternions.shape[-1] != 4:
#             raise ValueError(f"Expected `quaternions` to have shape (4,) or (N, 4), got {temp_quaternions.shape}")
#     else:
#         temp_quaternions = None

#     time_points = np.linspace(time_points[0], time_points[-1], len(temp_positions))
#     times = np.arange(time_points[0], time_points[-1] + time_step * 0.9, time_step)

#     temp_positions_smoothed = copy.deepcopy(temp_positions)

#     if temp_quaternions is not None:
#         euler_angles = R.from_quat(temp_quaternions).as_euler('xyz')
#         euler_angles_smoothed = copy.deepcopy(euler_angles)
#         temp_quaternions_smoothed = R.from_euler('xyz', euler_angles_smoothed).as_quat()

#     all_positions, all_quaternions = [], []

#     if temp_positions_smoothed is not None:
#         BS = make_interp_spline(time_points, temp_positions_smoothed, k=3, bc_type='natural')
#         all_positions = BS(times)

#     if temp_quaternions_smoothed is not None:
#         temp_quaternions_smoothed = np.array(temp_quaternions_smoothed)
#         all_quaternions = []

#         for t in times:
#             idx = np.searchsorted(time_points, t) - 1
#             idx = max(0, min(idx, len(time_points) - 2))

#             t1, t2 = time_points[idx], time_points[idx + 1]
#             q1, q2 = temp_quaternions_smoothed[idx], temp_quaternions_smoothed[idx + 1]

#             if t2 != t1:
#                 alpha = (t - t1) / (t2 - t1)
#             else:
#                 alpha = 0.0

#             dot_product = np.dot(q1, q2)
#             if dot_product < 0.0:
#                 q2 = -q2
#                 dot_product = -dot_product

#             if dot_product > 0.9995:
#                 q_interp = (1 - alpha) * q1 + alpha * q2
#             else:
#                 theta_0 = np.arccos(dot_product)
#                 theta = theta_0 * alpha

#                 q2_ortho = q2 - q1 * dot_product
#                 q2_ortho /= np.linalg.norm(q2_ortho)

#                 q_interp = q1 * np.cos(theta) + q2_ortho * np.sin(theta)

#             all_quaternions.append(q_interp)

#     if temp_quaternions is not None:
#         euler_angles = R.from_quat(all_quaternions).as_euler('xyz')

#         if len(euler_angles) >= 20:
#             euler_angles_smoothed = savgol_filter(euler_angles, window_length=5, polyorder=3, axis=0)
#         else:
#             euler_angles_smoothed = euler_angles

#         all_quaternions = R.from_euler('xyz', euler_angles_smoothed).as_quat()

#     if temp_positions is not None and temp_quaternions is not None:
#         return np.array(all_positions), np.array(all_quaternions)
#     elif temp_positions is not None:
#         return np.array(all_positions)
#     elif temp_quaternions is not None:
#         return np.array(all_quaternions)

def cloud_point_interpolate(positions=None, quaternions=None, time_points=None, time_step=0.1):
    """
    进行输入的点云位置和/或姿态的曲线插值，支持多个点，采用B样条插值法
    :param positions: 位置序列，单位为米 (Nx2 或 Nx3 numpy array, 可选)
    :param quaternions: 姿态序列，四元数 (Nx4 numpy array, 可选)
    :param time_points: 时间点序列 (N个浮点数)
    :param time_step: 离散时间步长，单位为秒 (float)
    :return: 插值序列，包含位置序列、姿态序列或两者兼有
    """
    if positions is None and quaternions is None:
        raise ValueError("至少需要输入位置或姿态的序列")
    if time_points is None or len(time_points) < 2:
        raise ValueError("需要提供至少两个时间点")
    

    # print("未过滤positions:")
    # for position in positions:
    #     print("positon:",position)


    # print("未过滤quaternions:")
    # for quaternion in quaternions:
    #     print("quaternion:",quaternion)

    temp_positions = np.array(positions)
    temp_quaternions = np.zeros((len(quaternions), 4))

    # 将RPY角度转换为四元数
    for i in range(len(quaternions)):
        temp_quaternions[i] = R.from_euler('xyz', quaternions[i]).as_quat()

    time_points = np.linspace(time_points[0], time_points[-1], len(temp_positions))
    times = np.arange(time_points[0], time_points[-1] + time_step * 0.9, time_step)

    # # 使用 Savitzky-Golay 滤波器进行平滑，平滑窗口大小为 5，使用 3 次多项式
    # # temp_positions_smoothed = savgol_filter(temp_positions, window_length=5, polyorder=3, axis=0)
    # temp_positions_smoothed = copy.deepcopy(temp_positions)

    # # 如果需要对四元数进行平滑处理，通常用的是欧拉角，而不是直接对四元数进行平滑
    # # 因为直接对四元数进行线性滤波会导致失真，所以建议对欧拉角进行滤波后再转换为四元数
    # euler_angles = R.from_quat(temp_quaternions).as_euler('xyz')
    # #euler_angles_smoothed = savgol_filter(euler_angles, window_length=5, polyorder=3, axis=0)
    # euler_angles_smoothed = copy.deepcopy(euler_angles)
    # temp_quaternions_smoothed = R.from_euler('xyz', euler_angles_smoothed).as_quat()

    temp_positions_smoothed = copy.deepcopy(temp_positions)
    temp_quaternions_smoothed = copy.deepcopy(temp_quaternions)




    all_positions, all_quaternions = [], []

    # 进行B样条插值
    if temp_positions_smoothed is not None:
        BS = make_interp_spline(time_points, temp_positions_smoothed, k=3, bc_type='natural')
        all_positions = BS(times)


    # 姿态插值

    # if temp_quaternions is not None:
    #     if temp_quaternions.ndim == 2 and temp_quaternions.shape[1] == 4:  # 四元数必须为 Nx4
    #         # 使用欧拉角进行插值
    #         euler_angles = R.from_quat(temp_quaternions).as_euler('xyz')
            
    #         # 检查数据点数量
    #         if len(euler_angles) >= 5:
    #             # 数据点数量大于等于3时使用 Savitzky-Golay 滤波器
    #             euler_angles_smoothed = savgol_filter(euler_angles, window_length=5, polyorder=3, axis=0)
    #         else:
    #             # 数据点小于3时直接使用原始数据
    #             euler_angles_smoothed = euler_angles

    #         # 将平滑后的欧拉角转换回四元数
    #         quaternions_smoothed = R.from_euler('xyz', euler_angles_smoothed).as_quat()

    #         # 使用球面线性插值（SLERP）计算中间姿态
    #         slerp = Slerp(time_points, R.from_quat(quaternions_smoothed))
    #         all_quaternions = slerp(times).as_quat()

    #         # slerp = R.from_quat(quaternions_smoothed)
    #         # all_quaternions = slerp.interpolate(times).as_quat()
    #     else:
    #         raise ValueError("四元数序列形状必须为 (Nx4)")

    # 进行四元数LERP插值
    if temp_quaternions_smoothed is not None:
        temp_quaternions_smoothed = np.array(temp_quaternions_smoothed)
        all_quaternions = []
        for t in times:
            # 在时间点序列中找到最近的两个时间点，用于线性插值
            idx = np.searchsorted(time_points, t) - 1
            idx = max(0, min(idx, len(time_points) - 2))
            
            t1, t2 = time_points[idx], time_points[idx + 1]
            q1, q2 = temp_quaternions_smoothed[idx], temp_quaternions_smoothed[idx + 1]
            
            # 计算插值因子 alpha
            alpha = (t - t1) / (t2 - t1)
            
            # 进行四元数LERP插值
            q_interp = (1 - alpha) * q1 + alpha * q2
            q_interp /= np.linalg.norm(q_interp)  # 归一化四元数
            
            all_quaternions.append(q_interp)

    euler_angles = R.from_quat(all_quaternions).as_euler('xyz')
    
    # 检查数据点数量
    if len(euler_angles) >= 5:
        # 数据点数量大于等于3时使用 Savitzky-Golay 滤波器
        euler_angles_smoothed = savgol_filter(euler_angles, window_length=5, polyorder=3, axis=0)
    else:
        # 数据点小于5时直接使用原始数据
        euler_angles_smoothed = euler_angles

    # 将平滑后的欧拉角转换回四元数
    all_quaternions = R.from_euler('xyz', euler_angles_smoothed).as_quat()

    if temp_positions is not None and temp_quaternions is not None:
        return np.array(all_positions), np.array(all_quaternions)
    elif temp_positions is not None:
        return np.array(all_positions)
    elif temp_quaternions is not None:
        return np.array(all_quaternions)


# def cloud_point_interpolate(positions=None, quaternions=None, time_points=None, time_step=0.1):
#     """
#     进行输入的点云位置和/或姿态的曲线插值，支持多个点，采用B样条插值法
#     :param positions: 位置序列，单位为米 (Nx2 或 Nx3 numpy array, 可选)
#     :param quaternions: 姿态序列，四元数 (Nx4 numpy array, 可选)
#     :param time_points: 时间点序列 (N个浮点数)
#     :param time_step: 离散时间步长，单位为秒 (float)
#     :return: 插值序列，包含位置序列、姿态序列或两者兼有
#     """
#     if positions is None and quaternions is None:
#         raise ValueError("至少需要输入位置或姿态的序列")
#     if time_points is None or len(time_points) < 2:
#         raise ValueError("需要提供至少两个时间点")

#     temp_positions = np.array(positions)
#     temp_quaternions = np.zeros((len(quaternions), 4))

#     print("未过滤positions:")
#     for temp_position in temp_positions:
#         print("positon:",temp_position)


#     print("未过滤quaternions:")
#     for temp_quaternions in temp_quaternions:
#         print("quaternion:",temp_quaternions)

#     # 将RPY角度转换为四元数
#     for i in range(len(quaternions)):
#         temp_quaternions[i] = R.from_euler('xyz', quaternions[i]).as_quat()

#     time_points = np.linspace(time_points[0], time_points[-1], len(temp_positions))
#     times = np.arange(time_points[0], time_points[-1] + time_step * 0.9, time_step)

#     # 使用 Savitzky-Golay 滤波器进行平滑，平滑窗口大小为 5，使用 3 次多项式
#     # temp_positions_smoothed = savgol_filter(temp_positions, window_length=5, polyorder=3, axis=0)
#     temp_positions_smoothed = copy.deepcopy(temp_positions)

#     # 如果需要对四元数进行平滑处理，通常用的是欧拉角，而不是直接对四元数进行平滑
#     # 因为直接对四元数进行线性滤波会导致失真，所以建议对欧拉角进行滤波后再转换为四元数
#     euler_angles = R.from_quat(temp_quaternions).as_euler('xyz')
#     #euler_angles_smoothed = savgol_filter(euler_angles, window_length=5, polyorder=3, axis=0)
#     euler_angles_smoothed = copy.deepcopy(euler_angles)
#     temp_quaternions_smoothed = R.from_euler('xyz', euler_angles_smoothed).as_quat()

#     all_positions, all_quaternions = [], []

#     # 进行B样条插值
#     if temp_positions_smoothed is not None:
#         BS = make_interp_spline(time_points, temp_positions_smoothed, k=3, bc_type='natural')
#         all_positions = BS(times)


#     # 进行四元数LERP插值
#     if temp_quaternions_smoothed is not None:
#         temp_quaternions_smoothed = np.array(temp_quaternions_smoothed)
#         all_quaternions = []
#         for t in times:
#             # 在时间点序列中找到最近的两个时间点，用于线性插值
#             idx = np.searchsorted(time_points, t) - 1
#             idx = max(0, min(idx, len(time_points) - 2))
            
#             t1, t2 = time_points[idx], time_points[idx + 1]
#             # q1, q2 = temp_quaternions_smoothed[idx], temp_quaternions_smoothed[idx + 1]

#             if idx + 1 < len(temp_quaternions_smoothed):
#                 q1, q2 = temp_quaternions_smoothed[idx], temp_quaternions_smoothed[idx + 1]
#             else:
#                 # 处理越界情况，使用最后一个四元数
#                 q1 = q2 = temp_quaternions_smoothed[idx]
                    
#             # # 计算插值因子 alpha
#             # alpha = (t - t1) / (t2 - t1)

#             # 计算插值因子 alpha，确保 t2 - t1 不为 0
#             if t2 - t1 > 1e-6:  # 避免除以零
#                 alpha = (t - t1) / (t2 - t1)
#             else:
#                 alpha = 0
            
#             # 进行四元数LERP插值
#             q_interp = (1 - alpha) * q1 + alpha * q2
#             q_interp /= np.linalg.norm(q_interp)  # 归一化四元数
            
#             all_quaternions.append(q_interp)

#     euler_angles = R.from_quat(all_quaternions).as_euler('xyz')
    
#     # 检查数据点数量
#     if len(euler_angles) >= 5:
#         # 数据点数量大于等于3时使用 Savitzky-Golay 滤波器
#         euler_angles_smoothed = savgol_filter(euler_angles, window_length=5, polyorder=3, axis=0)
#     else:
#         # 数据点小于5时直接使用原始数据
#         euler_angles_smoothed = euler_angles

#     # 将平滑后的欧拉角转换回四元数
#     all_quaternions = R.from_euler('xyz', euler_angles_smoothed).as_quat()

#     if temp_positions is not None and temp_quaternions is not None:
#         return np.array(all_positions), np.array(all_quaternions)
#     elif temp_positions is not None:
#         return np.array(all_positions)
#     elif temp_quaternions is not None:
#         return np.array(all_quaternions)



# def cloud_point_interpolate(positions=None, quaternions=None, time_points=None, time_step=0.1):
#     """
#     进行输入的点云位置和/或姿态的曲线插值，支持多个点，采用B样条插值法
#     :param positions: 位置序列，单位为米 (Nx2 或 Nx3 numpy array, 可选)
#     :param quaternions: 姿态序列，四元数 (Nx4 numpy array, 可选)
#     :param time_points: 时间点序列 (N个浮点数)
#     :param time_step: 离散时间步长，单位为秒 (float)
#     :return: 插值序列，包含位置序列、姿态序列或两者兼有
#     """
#     if positions is None and quaternions is None:
#         raise ValueError("至少需要输入位置或姿态的序列")
#     if time_points is None or len(time_points) < 2:
#         raise ValueError("需要提供至少两个时间点")
    

#     #closest_points = np.array(positions)

#     #通过法向量在基坐标系上的投影计算欧拉角
#     temp_positions = np.array(positions)
#     # temp_quaternions = calculate_target_Euler(point = closest_points)
#     temp_quaternions = np.zeros((len(quaternions), 4))


#     # 将RPY角度转换为四元数
#     for i in range(len(quaternions)):
#         temp_quaternions[i] = R.from_euler('xyz', quaternions[i]).as_quat()
#     #print(len(positions_2d))
#     time_points = np.linspace(time_points[0], time_points[-1],len(temp_positions))

#     all_positions = []
#     all_quaternions = []
#     times = np.arange(time_points[0], time_points[-1] + time_step * 0.9, time_step)
 
#     #B样条插值次数
#     degree=3

#     #进行B样条插值
#     if temp_positions is not None:
#         BS = make_interp_spline(time_points, temp_positions, degree, bc_type='natural')
#         all_positions = BS(times)

#     # if quaternions is not None:
#     #     slerp = Slerp(time_points, R.from_quat(quaternions))
#     #     all_quaternions = slerp(times).as_quat()

#     # 进行姿态的LERP插值
#     if temp_quaternions is not None:
#         temp_quaternions = np.array(temp_quaternions)
#         all_quaternions = []
#         for t in times:
#             # 在时间点序列中找到最近的两个时间点，用于线性插值
#             idx = np.searchsorted(time_points, t) - 1
#             idx = max(0, min(idx, len(time_points) - 2))
            
#             t1, t2 = time_points[idx], time_points[idx + 1]
#             q1, q2 = temp_quaternions[idx], temp_quaternions[idx + 1]
            
#             # 计算插值因子 alpha
#             alpha = (t - t1) / (t2 - t1)
            
#             # 进行四元数LERP插值
#             q_interp = (1 - alpha) * q1 + alpha * q2
#             q_interp /= np.linalg.norm(q_interp)  # 归一化四元数
            
#             all_quaternions.append(q_interp)

#     if temp_positions is not None and temp_quaternions is not None:
#         return np.array(all_positions), np.array(all_quaternions)
#     elif temp_positions is not None:
#         return np.array(all_positions)
#     elif temp_quaternions is not None:
#         return np.array(all_quaternions)
    
def circle_trajectory(center, omega=8.0, radius=0.02, reverse=False, time_points=None, time_step=0.01, start_transition_duration=None, end_transition_duration=None):

    # print(time_points)
    if time_points is None or len(time_points) < 2:
        raise ValueError("需要提供至少两个时间点")
    
    
    if start_transition_duration is None:
        start_transition_duration = 2

    if end_transition_duration is None:
        end_transition_duration = 2
    t_points = time_points.copy()
    t_points[-1] = time_points[-1] + end_transition_duration + start_transition_duration
    t_points[-1] = round(t_points[-1] / time_step) * time_step
    times = np.arange(t_points[0], t_points[-1] + time_step * 0.9, time_step)

    if reverse:
        angles = -omega * times
    else:
        angles = omega * times

    radii = np.ones_like(times) * radius

    start_transition = times < start_transition_duration
    end_transition = times > (times[-1] - end_transition_duration)
    radii[start_transition] = radius * (1 - np.cos(np.pi * times[start_transition] / start_transition_duration)) / 2
    radii[end_transition] = radius * (1 + np.cos(np.pi * (times[end_transition] - (times[-1] - end_transition_duration)) / end_transition_duration)) / 2
    
    x_positions = radii * np.cos(angles)
    y_positions = radii * np.sin(angles)
    z_positions = np.full_like(x_positions, 0)  # Z position remains constant



    positions = np.column_stack((x_positions, y_positions, z_positions))
    tempToolRPY = R.from_euler('xyz', center[0][3:], degrees=False).as_matrix()
    for i in range(len(positions)):
        positions[i] = tempToolRPY @  positions[i] + center[0][:3]       # 将RPY角度转换为四元数
        
    print(positions)
    return positions

# def generate_circle_cloud_points(center, start_point, radius, delta_theta = 10*np.pi/180, num_turns = 3):
#     """
#     center: 圆心坐标，形如 (x_c, y_c)
#     start_point: 起始点坐标，形如 (x_0, y_0)
#     radius: 圆的半径
#     delta_theta: 每次插补的角度增量
#     num_turns: 绕圈的次数
#     """
#     # 确定总共需要生成的插补点数
#     num_points = int((2 * np.pi * num_turns) / delta_theta)
    
#     # 圆心
#     x_c, y_c = center
    
#     # 计算起始点的初始角度
#     x_0, y_0 = start_point
#     theta_0 = np.arctan2(y_0 - y_c, x_0 - x_c)
    
#     # 初始化存储插补点的列表
#     circle_points = []
    
#     # 生成插补点
#     for i in range(num_points):
#         # 当前角度
#         theta_i = theta_0 + i * delta_theta
        
#         # 计算插补点的坐标
#         x_i = x_c + radius * np.cos(theta_i)
#         y_i = y_c + radius * np.sin(theta_i)
        
#         # 将点添加到列表中

#         circle_points.append((np.round(x_i).astype(int), np.round(y_i).astype(int)))

#     circle_points.append((np.round(x_0).astype(int), np.round(y_0).astype(int)))
    
#     return circle_points

    

def calculate_target_Euler(point):

    temp_euler = np.zeros((len(point), 3), dtype=np.float64)

    for i in range(len(point)):
        if(point[i][5]<0): 
            temp_euler[i][0] = -math.asin(-point[i][4])
            temp_euler[i][1] =  math.atan2(-point[i][3],-point[i][5])
        else:
            temp_euler[i][0] = -math.asin(point[i][4])
            temp_euler[i][1] =  math.atan2(point[i][3],point[i][5])
                
        temp_euler[i][2] =  0.0

    return temp_euler


if __name__ == "__main__":  
    # import pathlib
    # import sys
    # sys.path.append(str(pathlib.Path.cwd()))
    # from MassageControl.tools.draw_tools import plot_trajectory

    import numpy as np
    import pandas as pd
    import matplotlib.pyplot as plt

    # 示例使用
    center = (80, 90)  # 圆心
    start_point = (70, 0)  # 起点
    radius =  np.linalg.norm(np.array(start_point) - np.array(center))  # 半径
    delta_theta = np.pi / 9  # 每次插补的角度增量
    num_turns = 2  # 绕2圈

    # 生成圆的插补点
    circle_points_with_start = generate_circle_cloud_points(center, start_point, radius, delta_theta, num_turns)

    # 将生成的插补点转换为可视化的DataFrame
    circle_points_with_start_df = pd.DataFrame({
        "x": [point[0] for point in circle_points_with_start],
        "y": [point[1] for point in circle_points_with_start]
    })

    # 打印生成的插补点
    print(circle_points_with_start_df)

    # 绘制插补点的图像
    x_vals = [point[0] for point in circle_points_with_start]
    y_vals = [point[1] for point in circle_points_with_start]

    plt.figure(figsize=(6, 6))
    plt.scatter(x_vals, y_vals, marker='o', linestyle='-', color='b')
    plt.scatter([x_vals[0]], [y_vals[0]], color='r', label='Start Point')  # 标记起点
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title('Circle Interpolation Points with Start Point')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)

    # 保存并显示图片
    plt.show()



#     start1pos = np.array([0.0392,-0.408,0.752])
#     end1pos = np.array([0.0392,0.2,0.752])

#     start2pos = np.array([0.2281194148213992,-0.1320499159555817,0.7499999952316284])
#     end2pos = np.array([0.14268880718774088,-0.13746791895961052,0.7350000095367432])

#     start3pos = np.array([0.23648124242722512,-0.2097320409627573,0.7430000257492065])
#     end3pos = np.array([0.1493414817211018,-0.21703966731273366,0.7340000224113464])

#     start4pos = np.array([0.24389595888042098,-0.30559190060482105,0.7499999952316284])
#     end4pos = np.array([0.15822969083491112,-0.3106326911577041,0.7440000128746033])

#     start5pos = np.array([0.2535787008200847,-0.402571052456421,0.7559999775886536])
#     end5pos = np.array([0.16737854928028986,-0.41016720685793384,0.7580000114440918])


    

    



#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(start2pos - end1pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = end1pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (start2pos - end1pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = start2pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(end2pos - start2pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = start2pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (end2pos - start2pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = end2pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(start3pos - end2pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = end2pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (start3pos - end2pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = start3pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(end3pos - start3pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = start3pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (end3pos - start3pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = end3pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(start4pos - end3pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = end3pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (start4pos - end3pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = start4pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(end4pos - start4pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = start4pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (end4pos - start4pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = end4pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(start5pos - end4pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = end4pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (start5pos - end4pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = start5pos

#     # points = np.vstack((points, temppoints))

#     # #--------------------------
#     # # 计算起始点和结束点之间的总距离
#     # total_distance = np.linalg.norm(end5pos - start5pos)
    
#     # # 根据欧式距离步长为0.005估算所需的点数
#     # num_points = int(total_distance / dis)

#     # # 初始化用于存储点的数组
#     # temppoints = np.zeros((num_points + 1, 3))
#     # temppoints[0] = start5pos

#     # # 生成欧式距离差约为0.01的点
#     # for i in range(1, num_points + 1):
#     #     direction = (end5pos - start5pos) / total_distance  # 单位方向向量
#     #     temppoints[i] = temppoints[i-1] + direction * dis

#     # # 确保最后一个点与endpos完全重合
#     # temppoints[-1] = end5pos

#     # points = np.vstack((points, temppoints))


#     positions_2d = np.vstack([start1pos,end1pos])


#     #print(points)

#     time_points = np.array([0,10])
#     time_step = 0.01
#     #print(time_points)
#     #使用点云B样条插值
#     temppose = np.array([1,1,1,1,0,0])
#     positions_2d_interp = circle_trajectory(center=temppose,radius=0.05,time_points=time_points,time_step=time_step)
#     quaternions_interp = np.tile(R.from_euler('xyz', np.array(temppose[3:])).as_quat(), (positions_2d_interp.shape[0], 1))
#     # positions_2d_interp, quaternions_interp = cloud_point_interpolate(positions=positions_2d, time_points=time_points, time_step=time_step)
#     # print("2D Position Trajectory (Spline):")
#     # print(positions_2d)
    
#     # # 分解轨迹的x, y, z坐标
#     x_trajectory, y_trajectory, z_trajectory = zip(*positions_2d_interp)

#     # 分解散点的x, y, z坐标
#     x_scatter, y_scatter, z_scatter = zip(*positions_2d)

#     # 创建一个3D图形
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     # # 设置字体为 SimHei (黑体)
#     # plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
#     # plt.rcParams['axes.unicode_minus'] = False    # 解决负号无法正常显示的问题

#     # 绘制3D连续轨迹
#     ax.plot(x_trajectory, y_trajectory, z_trajectory, label='B-spline interpolation trajectory', color='b')

#     # 绘制3D散点图
#     ax.scatter(x_scatter, y_scatter, z_scatter, label='cloud points', color='r', marker='o')

#     # 设置轴标签
#     ax.set_xlabel('X Label')
#     ax.set_ylabel('Y Label')
#     ax.set_zlabel('Z Label')

#     # 设置标题
#     ax.set_title('3D Trajectory and Scatter Plot')

#     # 添加图例
#     ax.legend()

#     # 设置轴的比例为相等
#     plt.gca().set_aspect('equal', adjustable='box')
#     # # 显示图形
#     # plt.show()

#     plot_trajectory(positions_2d_interp, quaternions_interp)

#     # for i in range(10):
#     #     tempX = np.linspace(startpos[0], endpos[0], 10)

#    # 示例使用
#     positions_2d = np.array([[0, 0], [1, 1], [1, 2],[2,2]]) #, [1, 1], [2, 1], [1, 1], [1, 1],[0, 0]])
#     positions_3d = np.array([[0, 0, 0], [1, 1, 1], [2, 0, 2], [3, 1, 3]])
#     quaternions = np.array([[0, 0, 0, 1], [0.707, 0, 0, 0.707], [1, 0, 0, 0], [1, 0, 0, 0]])
#     time_points = np.array([0, 1, 3, 4])
#     time_step = 0.5

#     # # 只插值2D位置
#     # positions_2d_interp = linear_interpolate(positions=positions_2d, time_points=time_points, time_step=time_step)
#     # print("2D Position Trajectory:")
#     # print(positions_2d_interp)

#     # # 只插值3D位置
#     # positions_3d_interp = linear_interpolate(positions=positions_3d, time_points=time_points, time_step=time_step)
#     # print("3D Position Trajectory:")
#     # print(positions_3d_interp)

#     # # 只插值姿态
#     # quaternions_interp = linear_interpolate(quaternions=quaternions, time_points=time_points, time_step=time_step)
#     # print("Quaternion Trajectory:")
#     # print(quaternions_interp)

#     # # 同时插值3D位置和姿态
#     # positions_3d_interp, quaternions_interp = linear_interpolate(positions=positions_3d, quaternions=quaternions, time_points=time_points, time_step=time_step)
#     # print("3D Position and Quaternion Trajectory:")
#     # print(positions_3d_interp)
#     # print(quaternions_interp)

#     # # 绘制插值轨迹
#     # plot_trajectory(positions_2d_interp)
#     # plot_trajectory(positions_3d_interp)
#     # plot_trajectory(quaternions_interp)
#     # plot_trajectory(positions_3d_interp, quaternions_interp)

#     # # 使用样条插值
#     # positions_2d_interp = spline_interpolate(positions=positions_2d, time_points=time_points, time_step=time_step)
#     # print("2D Position Trajectory (Spline):")
#     # print(positions_2d_interp)
#     # plot_trajectory(positions_2d_interp)

#     # positions_3d_interp, quaternions_interp = spline_interpolate(positions=positions_3d, quaternions=quaternions, time_points=time_points, time_step=time_step)
#     # print("3D Position and Quaternion Trajectory (Spline):")
#     # print(positions_3d_interp)
#     # print(quaternions_interp)
#     # plot_trajectory(positions_3d_interp, quaternions_interp)

#     # # 使用点云B样条插值
#     # positions_2d_interp = cloud_point_interpolate(positions=positions_2d, time_points=time_points, time_step=time_step)
#     # print("2D Position Trajectory (Spline):")
#     # print(positions_2d_interp)
#     # plot_trajectory(positions_2d_interp)

#     # positions_3d_interp, quaternions_interp = cloud_point_interpolate(positions=positions_3d, quaternions=quaternions, time_points=time_points, time_step=time_step)
#     # print("3D Position and Quaternion Trajectory (Spline):")
#     # print(positions_3d_interp)
#     # print(quaternions_interp)
#     # plot_trajectory(positions_3d_interp, quaternions_interp)