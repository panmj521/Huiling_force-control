import cv2

# 打开相机
cap = cv2.VideoCapture(0)

# 获取相机的分辨率
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print(f"Camera Resolution: {width}x{height}")

# 释放相机
cap.release()