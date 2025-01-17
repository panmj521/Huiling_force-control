from numpy import *
import numpy as np
import math

'''
Made by 水木皆Ming
重力补偿计算
'''
class GravityCompensation:
    M = np.empty((0, 0))
    F = np.empty((0, 0))
    f = np.empty((0, 0))
    R = np.empty((0, 0))

    x = 0
    y = 0
    z = 0
    k1 = 0
    k2 = 0
    k3 = 0

    U = 0
    V = 0
    g = 0

    F_x0 = 0
    F_y0 = 0
    F_z0 = 0

    M_x0 = 0
    M_y0 = 0
    M_z0 = 0

    F_ex = 0
    F_ey = 0
    F_ez = 0

    M_ex = 0
    M_ey = 0
    M_ez = 0

    def Update_M(self, torque_data):
        M_x = torque_data[0]
        M_y = torque_data[1]
        M_z = torque_data[2]

        if (any(self.M)):
            M_1 = matrix([M_x, M_y, M_z]).transpose()
            self.M = vstack((self.M, M_1))
        else:
            self.M = matrix([M_x, M_y, M_z]).transpose()

    def Update_F(self, force_data):
        F_x = force_data[0]
        F_y = force_data[1]
        F_z = force_data[2]

        if (any(self.F)):
            F_1 = matrix([[0, F_z, -F_y, 1, 0, 0],
                          [-F_z, 0, F_x, 0, 1, 0],
                          [F_y, -F_x, 0, 0, 0, 1]])
            self.F = vstack((self.F, F_1))
        else:
            self.F = matrix([[0, F_z, -F_y, 1, 0, 0],
                             [-F_z, 0, F_x, 0, 1, 0],
                             [F_y, -F_x, 0, 0, 0, 1]])

    def Solve_A(self):
        A = dot(dot(linalg.inv(dot(self.F.transpose(), self.F)), self.F.transpose()), self.M)

        self.x = A[0, 0]
        self.y = A[1, 0]
        self.z = A[2, 0]
        self.k1 = A[3, 0]
        self.k2 = A[4, 0]
        self.k3 = A[5, 0]
        # print("A= \n" , A)
        print("x= ", self.x)
        print("y= ", self.y)
        print("z= ", self.z)
        print("k1= ", self.k1)
        print("k2= ", self.k2)
        print("k3= ", self.k3)

    def Update_f(self, force_data):
        F_x = force_data[0]
        F_y = force_data[1]
        F_z = force_data[2]

        if (any(self.f)):
            f_1 = matrix([F_x, F_y, F_z]).transpose()
            self.f = vstack((self.f, f_1))
        else:
            self.f = matrix([F_x, F_y, F_z]).transpose()

    def Update_R(self, euler_data):
        # 机械臂末端到基坐标的旋转矩阵
        R_array = self.eulerAngles2rotationMat(euler_data)

        alpha = (0) * 180 / np.pi

        # 力传感器到末端的旋转矩阵
        R_alpha = np.array([[math.cos(alpha), -math.sin(alpha), 0],
                            [math.sin(alpha), math.cos(alpha), 0],
                            [0, 0, 1]
                            ])

        R_array = np.dot(R_alpha, R_array.transpose())

        if (any(self.R)):
            R_1 = hstack((R_array, np.eye(3)))
            self.R = vstack((self.R, R_1))
        else:
            self.R = hstack((R_array, np.eye(3)))

    def Solve_B(self):
        B = dot(dot(linalg.inv(dot(self.R.transpose(), self.R)), self.R.transpose()), self.f)

        self.g = math.sqrt(B[0] * B[0] + B[1] * B[1] + B[2] * B[2])
        self.U = math.asin(-B[1] / self.g)
        self.V = math.atan(-B[0] / B[2])

        self.F_x0 = B[3, 0]
        self.F_y0 = B[4, 0]
        self.F_z0 = B[5, 0]

        # print("B= \n" , B)
        print("g= ", self.g / 9.81)
        print("U= ", self.U * 180 / math.pi)
        print("V= ", self.V * 180 / math.pi)
        print("F_x0= ", self.F_x0)
        print("F_y0= ", self.F_y0)
        print("F_z0= ", self.F_z0)

    def Solve_Force(self, force_data, euler_data):
        Force_input = matrix([force_data[0], force_data[1], force_data[2]]).transpose()

        my_f = matrix([cos(self.U)*sin(self.V)*self.g, -sin(self.U)*self.g, -cos(self.U)*cos(self.V)*self.g, self.F_x0, self.F_y0, self.F_z0]).transpose()

        R_array = self.eulerAngles2rotationMat(euler_data)
        R_array = R_array.transpose()
        R_1 = hstack((R_array, np.eye(3)))

        Force_ex = Force_input - dot(R_1, my_f)
        print('接触力：', Force_ex.T)

    def Solve_Torque(self, torque_data, euler_data):
        Torque_input = matrix([torque_data[0], torque_data[1], torque_data[2]]).transpose()
        M_x0 = self.k1 - self.F_y0 * self.z + self.F_z0 * self.y
        M_y0 = self.k2 - self.F_z0 * self.x + self.F_x0 * self.z
        M_z0 = self.k3 - self.F_x0 * self.y + self.F_y0 * self.x

        Torque_zero = matrix([M_x0, M_y0, M_z0]).transpose()

        Gravity_param = matrix([[0, -self.z, self.y],
                                [self.z, 0, -self.x],
                                [-self.y, self.x, 0]])

        Gravity_input = matrix([cos(self.U)*sin(self.V)*self.g, -sin(self.U)*self.g, -cos(self.U)*cos(self.V)*self.g]).transpose()

        R_array = self.eulerAngles2rotationMat(euler_data)
        R_array = R_array.transpose()

        Torque_ex = Torque_input - Torque_zero - dot(dot(Gravity_param, R_array), Gravity_input)

        print('接触力矩：', Torque_ex.T)

    def eulerAngles2rotationMat(self, theta):
        theta = [i * math.pi / 180.0 for i in theta]  # 角度转弧度

        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])

        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])

        # 第一个角为绕X轴旋转，第二个角为绕Y轴旋转，第三个角为绕Z轴旋转
        R = np.dot(R_x, np.dot(R_y, R_z))
        return R


def main():
    #
    force_data = [1.40e+00 , 0.00e+00 , 4.50e+00]
    torque_data= [-1.30e-02 ,1.68e-01 ,3.00e-03]
    euler_data = [2.346 *(180/np.pi),0.14 *(180/np.pi),1.43 *(180/np.pi)]

    force_data1 = [-0.7  ,  2.8  ,  4.1 ]
    torque_data1= [ -0.341 , -0.115 , 0.005]
    euler_data1 = [2.743 *(180/np.pi),-0.271 *(180/np.pi),1.486 *(180/np.pi)]

    force_data2 = [2.9  ,  0.8  ,  4.2]
    torque_data2= [ -0.064 ,-0.36 , -0.01]
    euler_data2 = [2.325 *(180/np.pi),-0.595 *(180/np.pi),1.603 *(180/np.pi)]


    compensation = GravityCompensation()

    compensation.Update_F(force_data)
    compensation.Update_F(force_data1)
    compensation.Update_F(force_data2)

    compensation.Update_M(torque_data)
    compensation.Update_M(torque_data1)
    compensation.Update_M(torque_data2)


    compensation.Solve_A()

    compensation.Update_f(force_data)
    compensation.Update_f(force_data1)
    compensation.Update_f(force_data2)

    compensation.Update_R(euler_data)
    compensation.Update_R(euler_data1)
    compensation.Update_R(euler_data2)

    compensation.Solve_B()


    compensation.Solve_Force(force_data, euler_data)
    compensation.Solve_Torque(torque_data, euler_data)
    compensation.Solve_Force(force_data1, euler_data1)
    compensation.Solve_Torque(torque_data1, euler_data1)
    compensation.Solve_Force(force_data2, euler_data2)
    compensation.Solve_Torque(torque_data2, euler_data2)



if __name__ == '__main__':
    main()
