import cv2
import os
import numpy as np
import sys
from pathlib import Path
import time
sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append('/home/jsfb/jsfb_ws/MassageRobot_huiling/Massage/MassageControl')
from Hardware.remote_cam import ToolCamera 
# from Hardware.Huiling_Scara import Huilin

# class Capture:

#     def __init__(self,arm:Huilin, config_path:str):
#         self.arm = arm
#         self.config_path = config_path



# 确保有一个目录来保存照片
# save_directory = '/home/jsfb/jsfb_ws/global_config/captured_images'
# if not os.path.exists(save_directory):
#     os.makedirs(save_directory)

print(f'打开摄像头中.....')
# 获取摄像头实例并设置分辨率
cam = ToolCamera(host='127.0.0.1')
cam.start()
time.sleep(3)
print(f'摄像头打开完成')

count = 0 # 用于计数拍摄的照片数量

# # =====================初始化Robot对象=====================
# print(f'===============初始化robot并连接中.....==============')
# arm = Huilin()

# cp = Capture(arm=arm,config_path="/home/jsfb/jsfb_ws/global_config/massage_head/empty_playload.yaml")
# time.sleep(1)
# print(f'===============初始化robot并连接成功==============')


try:
    i = 1
    while True:
# 读取摄像头的帧
        r, d, intrinsics = cam.get_latest_frame()
        # ret, frame = cam.read()
        frame = r

        # if not ret:
        #     print("Error: Could not read frame from video device.")
        #     break

        # 显示摄像头的帧
        cv2.imshow('Camera', frame)

        # 等待按键事件
        key = cv2.waitKey(1) & 0xFF

        # 按下's'键保存照片
        # if key == ord('s'):
        #     img_path = f'photo{i}'
        #     if not os.path.exists(os.path.join(save_directory, img_path)):
        #         os.makedirs(os.path.join(save_directory, img_path))
        #     filename = os.path.join(save_directory, f'{img_path}/rgb_image.png')
        #     cv2.imwrite(filename, frame) # 保存照片
        #     filename1 = os.path.join(save_directory, f'{img_path}/depth_image.png')
        #     cv2.imwrite(filename1, d) # 保存照片

            # pose = cp.arm.robot_rpc_client.getRobotInterface(arm.robot_name).getRobotState().getTcpPose()
            # print("pose = ",pose)

            # filename2 = os.path.join(save_directory, f'{img_path}/pose.txt')
            # np.savetxt(filename2, pose)
            # print(f'Saved {filename}')
            # count += 1 # 增加照片计数
            # i += 1
        # 按下'q'键退出循环
        # elif key == ord('q'):
        #     break
        if key == ord('s'):
            break

except KeyboardInterrupt:
    # 处理Ctrl+C中断
    print("\nCamera session ended by user.")
finally:
    # 释放资源
    cam.stop()
    cv2.destroyAllWindows()

    # 总结拍摄的照片
    print(f'Total number of images captured: {count}')