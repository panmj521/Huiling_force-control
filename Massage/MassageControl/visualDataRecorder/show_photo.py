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

cam = ToolCamera(host='127.0.0.1')
cam.start()
time.sleep(0.1)

# 获取最新的RGB和深度图像
rgb_image, depth_image, _ = cam.get_latest_frame()

cv2.imshow('show',rgb_image)
cv2.waitKey()