import sys
import os
import time
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, os.pardir))
sys.path.append(project_root)

#测试demo

from Huiling.Hardware.Huiling_Scara import Huilin

def main():
    robot_controller = Huilin()
    pos_1 = [50, 50, 0]
    pos_2 = [100, 50, 0]
    deg = [0,0,0,180]

    #robot_controller.wait_stop()
    # robot_controller.move_pose(pos_1, deg,speed = 20)
    # robot_controller.move_pose(pos_2, deg,speed = 20)

    # robot_controller.wait_stop()
    # robot_controller.move_pose(pos[:2]+[10,10], deg)
    # robot_controller.wait_stop()
    # robot_controller.arms_home(1)
    # robot_controller.move_pose(pos[:2]+[10,10], deg)
    # robot_controller.wait_stop()
    # robot_controller.arms_home(1)

    q_1 = [0,1.57,1.57,1]
    q_2 = [0,0.56,1.57,1]
    robot_controller.move_joint(q_1)
    robot_controller.move_joint(q_2)
    robot_controller.wait_stop()



    



if __name__ == "__main__":
    main()
