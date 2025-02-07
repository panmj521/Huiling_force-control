from ctypes import *
import ctypes

# do not edit this python file

class HitbotInterface:
    card_number = 0
    x = 0.0
    y = 0.0
    z = 0.0
    r = 0.0
    angle1 = 0.0
    angle2 = 0.0
    communicate_success = False
    initial_finish = False
    move_flag = False
    efg_distance = 0.0
    efg_type = 0.0
    encoder_x = 0.0
    encoder_y = 0.0
    encoder_z = 0.0
    encoder_r = 0.0
    encoder_angle1 = 0.0
    encoder_angle2 = 0.0

    def __init__(self, card_number):
        self.card_number = card_number
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.r = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.efg_dis = 0.0
        self.efg_type = 0.0

    def net_port_initial(self):
        # self.dll = CDLL('D:\桌面\hitbotsdk\Python_64\small_scara_interface.dll')
        #self.dll = CDLL('.\small_scara_interface.dll')
        self.dll=CDLL('/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl/libsmall_scara_interface.so')
        # self.dll = CDLL('C:\1Work\2-月度工作\01执行器售后工作\V2.0.0.26\Python_64\Python_64\small_scara_interface.dll')
        return self.dll.net_port_initial()

    def close_server(self):
        self.dll.close_server()

    def initial(self, generation, z_trail):
        return self.dll.initial(c_int(self.card_number), c_int(generation), c_float(z_trail))

    def get_scara_param(self):
        c_angle1 = c_float(0)
        c_angle2 = c_float(0)
        c_z = c_float(0)
        c_r = c_float(0)
        c_x = c_float(0)
        c_y = c_float(0)
        c_communicate_success = c_bool(False)
        c_initial_finish = c_bool(False)
        c_move_flag = c_bool(False)
        self.dll.get_scara_param(c_int(self.card_number), byref(c_x), byref(c_y), byref(c_z), byref(c_angle1),
                                       byref(c_angle2), byref(c_r), byref(c_communicate_success),
                                       byref(c_initial_finish),
                                       byref(c_move_flag))

        self.x = c_x.value
        self.y = c_y.value
        self.z = c_z.value
        self.angle1 = c_angle1.value
        self.angle2 = c_angle2.value
        self.r = c_r.value
        self.communicate_success = c_communicate_success.value
        self.initial_finish = c_initial_finish.value
        self.move_flag = c_move_flag.value

    def set_arm_length(self, l1, l2):
        self.dll.set_arm_length(c_int(self.card_number), c_float(l1), c_float(l2))

    def unlock_position(self):
        return self.dll.unlock_position(c_int(self.card_number))

    def is_connect(self):
        return self.dll.is_connected(c_int(self.card_number))

    def get_joint_state(self, joint_num):
        return self.dll.get_joint_state(c_int(self.card_number), c_int(joint_num))

    def set_drag_teach(self, enable):
        return self.dll.set_drag_teach(c_int(self.card_number), c_bool(enable))

    def get_drag_teach(self):
        return self.dll.get_drag_teach(c_int(self.card_number))

    def set_cooperation_fun_state(self, enable):
        return self.dll.set_cooperation_fun_state(c_int(self.card_number), c_bool(enable))

    def get_cooperation_fun_state(self):
        return self.dll.get_cooperation_fun_state(c_int(self.card_number))

    def is_collision(self):
        return self.dll.is_collision(c_int(self.card_number))

    def stop_move(self):
        return self.dll.stop_move(c_int(self.card_number))
    
    def emergency_stop(self):
        return self.dll.emergency_stop(c_int(self.card_number))

    def joint_home(self, joint_num):
        return self.dll.joint_home(c_int(self.card_number), c_int(joint_num))
    
    def get_j5_positon(self, type):
        return self.dll.get_j5_positon(c_int(self.card_number), c_int(type))
    
    def judge_in_range_xyzr(self, x,y,z,r):
        return self.dll.judge_in_range_xyzr(c_int(self.card_number), c_float(x), c_float(y), c_float(z), c_float(r))
    
    def judge_in_range_a12r(self, angle1,angle2,r):
        return self.dll.judge_in_range_a12r(c_int(self.card_number), c_float(angle1), c_float(angle2),  c_float(r))

    def movel_xyz(self, goal_x, goal_y, goal_z, goal_r, speed):
        return self.dll.movel_xyz(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed))

    def movej_xyz(self, goal_x, goal_y, goal_z, goal_r, speed, roughly):
        return self.dll.movej_xyz(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly))

    def movej_angle(self, goal_angle1, goal_angle2, goal_z, goal_r, speed, roughly):
        return self.dll.movej_angle(c_int(self.card_number), c_float(goal_angle1), c_float(goal_angle2), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly))

    def change_attitude(self, speed):
        return self.dll.change_attitude(c_int(self.card_number), c_float(speed))

    def movel_xyz_by_offset(self, x_offset, y_offset, z_offset, r_offset, speed):
        return self.dll.movel_xyz_by_offset(c_int(self.card_number), c_float(x_offset), c_float(y_offset), c_float(z_offset), c_float(r_offset), c_float(speed))

    def wait_stop(self):
        return self.dll.wait_stop(c_int(self.card_number))

    def pause_move(self):
        return self.dll.pause_move(c_int(self.card_number))

    def resume_move(self):
        return self.dll.resume_move(c_int(self.card_number))

    def is_stop(self):
        return self.dll.is_stop(c_int(self.card_number))

    def set_digital_out(self, io_number, io_value):
        return self.dll.set_digital_io_out(c_int(self.card_number), c_int(io_number), c_int(io_value))

    def get_digital_out(self, io_number):
        return self.dll.get_digital_io_out(c_int(self.card_number), c_int(io_number))

    def get_digital_in(self, io_number):
        return self.dll.get_digital_io_in(c_int(self.card_number), c_int(io_number))

    def set_efg_state(self, efg_type: object, efg_distance: object) -> object:
        return self.dll.set_efg_state(c_int(self.card_number), c_int(efg_type), c_float(efg_distance))

    def get_efg_state(self):
        c_efg_type = c_int(0)
        c_efg_distance = c_float(0)
        ret = self.dll.get_efg_state(c_int(self.card_number), byref(c_efg_type), byref(c_efg_distance))
        self.efg_type = c_efg_type
        self.efg_distance = c_efg_distance
        return ret
    
    def new_set_efg_state(self, channel,efg_type, efg_distance):
        return self.dll.new_set_efg_state(c_int(self.card_number), c_int(channel),c_int(efg_type), c_float(efg_distance))
    
    def new_get_efg_state(self,channel,efg_type):
        c_efg_distance = c_float(0)
        ret = self.dll.new_get_efg_state(c_int(self.card_number), c_int(channel),c_int(efg_type), byref(c_efg_distance))
        self.efg_distance = c_efg_distance
        return ret

    def movej_xyz_lr(self, goal_x, goal_y, goal_z, goal_r, speed, roughly, lr):
        return self.dll.movej_xyz_lr(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly), c_int(lr))

    def Tmovej_xyz_lr(self, goal_x, goal_y, goal_z, goal_r, speed, lr):
        return self.dll.Tmovej_xyz_lr(c_int(self.card_number), c_double(goal_x), c_double(goal_y), c_double(goal_z), c_double(goal_r), c_double(speed),  c_int(lr))

    def set_Tmovej_acc(self, acc1_rate, acc2_rate, acc3_rate, acc4_rate):
        return self.dll.set_Tmovej_acc(c_int(self.card_number), c_double(acc1_rate), c_double(acc2_rate), c_double(acc3_rate), c_double(acc4_rate))

    def new_movej_xyz_lr(self, goal_x, goal_y, goal_z, goal_r, speed, roughly, lr):
        return self.dll.new_movej_xyz_lr(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly), c_int(lr))



    def new_set_acc(self, j1_max_acc, j2_max_acc, j3_max_acc, j4_max_acc):
        return self.dll.new_set_acc(c_int(self.card_number), c_float(j1_max_acc), c_float(j2_max_acc), c_float(j3_max_acc), c_float(j4_max_acc))

    def j5_motor_zero(self):
        return self.dll.j5_motor_zero(c_int(self.card_number))

    def set_j5_motor_pos(self, deg, speed):
        self.dll.set_j5_motor_pos.restype = c_float
        return self.dll.set_j5_motor_pos(c_int(self.card_number), c_float(deg), c_float(speed))

    def get_j5_parameter(self):
        self.dll.get_j5_parameter.restype = c_float
        return self.dll.get_j5_parameter(c_int(self.card_number))
    
    def get_j5_motor_state(self):
        self.dll.get_j5_motor_state.restype = c_float
        return self.dll.get_j5_motor_state(c_int(self.card_number))
    
    def get_j5_motor_pos(self):
        self.dll.get_j5_motor_pos.restype = c_float
        return self.dll.get_j5_motor_pos(c_int(self.card_number))

    def movej_j5(self, deg, speed):
        return self.dll.movej_j5(c_int(self.card_number), c_float(deg), c_float(speed))
    
    def single_joint_move(self, axis,distance, speed):
        return self.dll.single_joint_move(c_int(self.card_number), c_int(axis), c_float(distance),c_float(speed))
    
    def xyz_move(self, direction,distance, speed):
        return self.dll.xyz_move(c_int(self.card_number), c_int(direction), c_float(distance),c_float(speed))

    def get_efg_state_dji(self):
        c_efg_type = c_int(0)
        c_efg_distance = c_float(0)
        ret = self.dll.get_efg_state_dji(c_int(self.card_number), byref(c_efg_type), byref(c_efg_distance))
        self.efg_type = c_efg_type
        self.efg_distance = c_efg_distance
        return ret

    def set_efg_state_dji(self, efg_type, efg_distance):
        return self.dll.set_efg_state_dji(c_int(self.card_number), c_int(efg_type), c_float(efg_distance))

    def new_stop_move(self):
        return self.dll.new_stop_move(c_int(self.card_number))
    
    def hi_position_send(self, j1_deg, j2_deg, j3_mm, j4_deg):
        return self.dll.hi_position_send(c_int(self.card_number), c_float(j1_deg), c_float(j2_deg), c_float(j3_mm), c_float(j4_deg))

    def get_encoder_coor(self):
        c_x = c_float(0)
        c_y = c_float(0)
        c_z = c_float(0)
        c_angle1 = c_float(0)
        c_angle2 = c_float(0)
        c_r = c_float(0)
        self.dll.get_encoder_coor(c_int(self.card_number), byref(c_x), byref(c_y), byref(c_z), byref(c_angle1), byref(c_angle2), byref(c_r))
        self.encoder_x = c_x.value
        self.encoder_y = c_y.value
        self.encoder_z = c_z.value
        self.encoder_angle1 = c_angle1.value
        self.encoder_angle2 = c_angle2.value
        self.encoder_r = c_r.value

    def new_movej_angle(self, goal_angle1, goal_angle2, goal_z, goal_r, speed, roughly):
        return self.dll.new_movej_angle(c_int(self.card_number), c_float(goal_angle1), c_float(goal_angle2), c_float(goal_z), c_float(goal_r), c_float(speed), c_float(roughly))

    def new_move_xyz(self,  goal_x, goal_y, goal_z, goal_r, speed, lr,motiontype):
        return self.dll.new_move_xyz(c_int(self.card_number), c_float(goal_x), c_float(goal_y), c_float(goal_z), c_float(goal_r), c_float(speed), c_int(lr),c_int(motiontype))

    #new 屏蔽或开启关节检测，在（initial）初始化函数前调用。
    #joint_num 轴号：1，2，3，4 
    #state 状态： True--开启关节检测   False---屏蔽关节检测
    #返回：True--设置成功，False--轴号输入错误
    def check_joint(self, joint_num, state):
        return self.dll.check_joint(c_int(self.card_number), c_int(joint_num), c_bool(state))
    
    def set_Interpolation_position_buf_catch_count(self, count):
        return self.dll.set_Interpolation_position_buf_catch_count(c_int(self.card_number), c_int(count))
    
    def set_robot_buf_size(self, bufsize):
        return self.dll.set_robot_buf_size(c_int(self.card_number), c_int(bufsize))
    
    def set_robot_joint_torque_value(self, jointnum,value):
        return self.dll.set_robot_joint_torque_value(c_int(self.card_number), c_int(jointnum), c_int(value))
    
    def com485_initial(self, baudRate):
        return self.dll.com485_initial(c_int(self.card_number), c_int(baudRate))

    def com485_send(self, data,len):
        return self.dll.com485_send(c_int(self.card_number), c_char_p(data), c_char(len))

    def com485_send2(self):
        a=[0x01,0x10,0x00,0x02,0x00,0x02,0x04,0x41,0xF0,0x00,0x00,0x66,0x79]
        b = [0x01, 0x10, 0x00, 0x02, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x72, 0x76]
        str_a="".join([chr(x) for x in a])
        str_b = " ".join([chr(x) for x in b])
        print(str_a.encode())
        str_c='0110000200020441F000006679'
        input = ctypes.c_char_p()
        input.value = str_c.encode()
        str_d = b'\x01\x10\x00\x02\x00\x02\x04\x41\xF0\x00\x00\x66\x79'
        print(str_d)
        print(input)
        print(input.value)
        print(a)
        print(str_a)
        print(str_c)
        print(str_c.encode())
        print(str_c.encode("ASCII"))
        print(str_c.encode("UTF-8"))
        print(id(str_c))
        len=0x0D
        char_len=chr(len)
        #ret=self.com485_send(id(str_c),13)
        ret = self.com485_send(str_d, 13)
        print('ret:%d'%ret)
        #return self.dll.com485_send(c_int(self.card_number), c_char_p(data), c_char(len))

    def com485_recv(self, data):
        return self.dll.com485_recv(c_int(self.card_number), c_char_p(data))
    
    def set_stop_io(self, io_number,io_value):
        self.dll.set_stop_io(c_int(self.card_number), c_int(io_number), c_int(io_value))

    def get_hard_emergency_stop_state(self):
        return self.dll.get_hard_emergency_stop_state(c_int(self.card_number))
    
    def clear_hard_emergency_stop(self):
        return self.dll.clear_hard_emergency_stop(c_int(self.card_number))
    
    def is_robot_goto_target(self,target_x,target_y,target_z,target_r):
        return self.dll.is_robot_goto_target(c_int(self.card_number),c_float(target_x),c_float(target_y),c_float(target_z),c_float(target_r))

    # 485旋转夹爪功能
    # 手动初始化
    def com485_init(self):
        return self.dll.com485_init(c_int(self.card_number))

    # 设置旋转速度
    def com485_set_rotation_speed(self, speed):
        return self.dll.com485_set_rotation_speed(c_int(self.card_number), c_float(speed))

    # 设置旋转速度
    def com485_set_clamping_speed(self, speed):
        return self.dll.com485_set_clamping_speed(c_int(self.card_number), c_float(speed))

    # 设置夹持距离
    def com485_set_clamping_distance(self, distance):
        return self.dll.com485_set_clamping_distance(c_int(self.card_number), c_float(distance))

    # 获取夹持距离
    def com485_get_clamping_distance(self):
        self.dll.com485_get_clamping_distance.restype = c_float
        return self.dll.com485_get_clamping_distance(c_int(self.card_number))

    # 设置绝对旋转角度
    def com485_set_rotation_angle(self, angle):
        return self.dll.com485_set_rotation_angle(c_int(self.card_number), c_float(angle))

    # 设置相对旋转角度
    def com485_set_relative_rotation_angle(self, angle):
        return self.dll.com485_set_relative_rotation_angle(c_int(self.card_number), c_float(angle))

    # 获取旋转角度
    def com485_get_rotation_angle(self):
        self.dll.com485_get_rotation_angle.restype = c_float
        return self.dll.com485_get_rotation_angle(c_int(self.card_number))

    # 获取夹持状态
    def com485_get_clamping_state(self):
        return self.dll.com485_get_clamping_state(c_int(self.card_number))

    # 获取旋转状态
    def com485_get_rotation_state(self):
        return self.dll.com485_get_rotation_state(c_int(self.card_number))

    # 设置夹持电流
    def com485_set_clamping_current(self, current):
        return self.dll.com485_set_clamping_current(c_int(self.card_number), c_float(current))

    # 设置旋转电流
    def com485_set_rotation_current(self, current):
        return self.dll.com485_set_rotation_current(c_int(self.card_number), c_float(current))
    
    # 获取夹持电流
    def com485_get_clamping_current(self):
        self.dll.com485_get_clamping_current.restype = c_float
        return self.dll.com485_get_clamping_current(c_int(self.card_number))
    
    # 获取旋转电流
    def com485_get_rotation_current(self):
        self.dll.com485_get_rotation_current.restype = c_float
        return self.dll.com485_get_rotation_current(c_int(self.card_number))
    
    # 吸盘控制 1：吸取 2：释放 3：按真空度设置吸取
    def com485_suction_cup_control(self,type):
        return self.dll.com485_suction_cup_control(c_int(self.card_number), c_int(type))
    
    # 最小真空度设置
    def com485_min_vacuum_degree(self,degree):
        return self.dll.com485_min_vacuum_degree(c_int(self.card_number), c_int(degree))
    
    # 最大真空度设置
    def com485_max_vacuum_degree(self,degree):
        return self.dll.com485_max_vacuum_degree(c_int(self.card_number), c_int(degree))
    
    # 获取吸取状态
    def com485_get_draw_state(self):
        return self.dll.com485_get_draw_state(c_int(self.card_number))
    
    def get_error_code(self):
	    return self.dll.get_error_code(c_int(self.card_number))
