import numpy as np

class ArmState:
    def __init__(self) -> None:
         # 当前状态
         
        self.arm_position = np.zeros(3,dtype=np.float64)
        self.arm_orientation = np.array([0.0,0.0,0.0,1.0]) # [qx, qy, qz, qw]
        self.external_wrench_base = np.zeros(6,dtype=np.float64)    
        self.external_wrench_tcp = np.zeros(6,dtype=np.float64)

        # 上一个状态
        self.last_arm_position = np.zeros(3,dtype=np.float64)
        self.last_arm_orientation = np.array([0.0,0.0,0.0,1.0]) # [qx, qy, qz, qw]
        self.last_external_wrench_base = np.zeros(6,dtype=np.float64)
        self.last_external_wrench_tcp = np.zeros(6,dtype=np.float64)
        
        # 目标状态
        self.desired_position = np.zeros(3,dtype=np.float64)
        self.desired_orientation = np.array([0.0,0,0,1]) # [qx, qy, qz, qw]
        self.desired_wrench = np.zeros(6,dtype=np.float64)
        self.desired_twist = np.zeros(6,dtype=np.float64)

        # 导纳计算过程变量
        self.arm_desired_twist = np.zeros(6,dtype=np.float64) 
        self.arm_desired_twist_tcp = np.zeros(6,dtype=np.float64)
        self.arm_desired_acc = np.zeros(6,dtype=np.float64)

        # 控制输出
        self.arm_position_command = np.zeros(3,dtype=np.float64)
        self.arm_orientation_command = np.array([0.0,0,0,1]) # [qx, qy, qz, qw]

        # 误差信息
        self.pose_error = np.zeros(6,dtype=np.float64)
        self.twist_error = np.zeros(6,dtype=np.float64)
        self.wrench_error = np.zeros(6,dtype=np.float64)

        # clip项
        self.max_acc_tran = 40
        self.max_acc_rot = 20 * 3.5
        self.max_vel_tran = 0.5 * 2.1
        self.max_vel_rot = 3.1415 * 2 #1.5
        self.max_dx = 0.01 *1.5
        self.max_dr = 0.0087 * 3*1.5

