import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation

def plot_trajectory(positions=None, quaternions=None):
    
    fig = plt.figure()
    if positions is not None:
        if positions.shape[1] == 2:
            plt.plot(positions[:, 0], positions[:, 1], '-o')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('2D Position Trajectory')
        elif positions.shape[1] == 3:
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], '-o')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('3D Position Trajectory')
    if quaternions is not None and positions is not None and positions.shape[1] == 3:
        # 初始向量 (朝上的箭头)
        arrow_vec = np.array([0, 0, 1])
        
        for pos, quat in zip(positions, quaternions):
            rotation = R.from_quat(quat)
            rotated_vec = rotation.apply(arrow_vec)
            ax.quiver(pos[0], pos[1], pos[2], rotated_vec[0], rotated_vec[1], rotated_vec[2], length=0.5, color='r')
    plt.show()

def update(frame, arrow_vec, ax, bed_corners_rot):
    global current_position, current_quaternion, y_offset
    ax.clear()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Position Trajectory')

    
    ax.set_xlim([-0.7, 1.5])
    ax.set_ylim([-0.2, 2])
    ax.set_zlim([-2, 0.2])

    # 旋转坐标系，绕 x 轴旋转 180 度
    rotation_matrix = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    # 更新动态坐标系的原点
    dynamic_origin = np.array([0, -y_offset, 0])

    # 将动态坐标系中的点转换为世界坐标系
    pos = np.dot(rotation_matrix, current_position + dynamic_origin) 
    quat = current_quaternion
    rotation = R.from_quat(quat)
    rotated_vec = rotation.apply(arrow_vec)
    rotated_vec = np.dot(rotation_matrix, rotated_vec)

    ax.scatter(pos[0], pos[1], pos[2], color='b', s=50)
    ax.quiver(pos[0], pos[1], pos[2], rotated_vec[0], rotated_vec[1], rotated_vec[2], length=0.6, color='r')

    # 绘制动态坐标系
    x_axis = dynamic_origin + np.array([1, 0, 0])
    y_axis = dynamic_origin + np.array([0, 1, 0])
    z_axis = dynamic_origin + np.array([0, 0, 1])

    dynamic_origin_rot = np.dot(rotation_matrix, dynamic_origin)
    x_axis_rot = np.dot(rotation_matrix, x_axis)
    y_axis_rot = np.dot(rotation_matrix, y_axis)
    z_axis_rot = np.dot(rotation_matrix, z_axis)

    ax.quiver(*dynamic_origin_rot, *(x_axis_rot - dynamic_origin_rot), color='r', length=0.4, normalize=True, arrow_length_ratio=0.2)
    ax.quiver(*dynamic_origin_rot, *(y_axis_rot - dynamic_origin_rot), color='g', length=0.4, normalize=True, arrow_length_ratio=0.2)
    ax.quiver(*dynamic_origin_rot, *(z_axis_rot - dynamic_origin_rot), color='b', length=0.4, normalize=True, arrow_length_ratio=0.2)

    # 绘制床
    ax.plot(bed_corners_rot[:, 0], bed_corners_rot[:, 1], bed_corners_rot[:, 2], color='brown')

def plot_trajectory_animation(interval=50):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    arrow_vec = np.array([0, 0, 1])
    
    # 定义床的角点，床是静态的
    bed_corners = np.array([
        [0, 1.8, -1],  # 左上角
        [0, 0, -1],  # 右上角
        [0.7, 0 , -1],  # 右下角
        [0.7, 1.8, -1],  # 左下角
        [0, 1.8, -1]  # 返回左上角
    ])
   
    ani = FuncAnimation(fig, update, frames=200, fargs=(arrow_vec, ax, bed_corners), interval=interval)
    plt.show()