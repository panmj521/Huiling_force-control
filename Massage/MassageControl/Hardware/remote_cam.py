import requests
import base64
import cv2
import open3d as o3d
import numpy as np
import paramiko
import time
class ToolCamera:
    def __init__(self, host):
        """
        初始化 CameraClient 类。
        """
        self.host = host
        self.port = 22               # SSH端口号，默认是22
        self.username = "jsfb"      # SSH用户名
        self.password = "jsfb"      # SSH密码
        self.root_password = "jsfb"
        
        # 要执行的命令
        self.start_command = "systemctl restart cam_server"
        self.stop_command = "systemctl stop cam_server"
        
    def start(self):
        self.execute_command_on_remote(
            host=self.host, 
            username=self.username, 
            password=self.password, 
            root_password=self.root_password, 
            command=self.start_command
        )
        time.sleep(2)

    def stop(self):
        self.execute_command_on_remote(
            host=self.host, 
            username=self.username, 
            password=self.password, 
            root_password=self.root_password, 
            command=self.stop_command
        )

    def execute_command_on_remote(self, host, username, password, root_password, command, port=22):
        # 创建SSH客户端
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        try:
            # 连接到远程服务器，指定端口
            ssh.connect(hostname=host, port=port, username=username, password=password)

            # 构建完整的命令字符串，将root密码传递给sudo
            sudo_command = f'echo {root_password} | sudo -S {command}'

            # 获取通道对象
            transport = ssh.get_transport()
            channel = transport.open_session()
            channel.get_pty()  # 获取伪终端
            channel.exec_command(sudo_command)  # 执行命令

            # 检查命令是否在后台执行
            while True:
                # 如果命令在后台运行, channel.exit_status_ready() 将会一直是 False
                if channel.exit_status_ready():
                    break

                # 非阻塞方式读取输出
                if channel.recv_ready():
                    output = channel.recv(1024).decode('utf-8')
                    print(output)

                # 非阻塞方式读取错误输出
                if channel.recv_stderr_ready():
                    error = channel.recv_stderr(1024).decode('utf-8')
                    print(error)

                # 延时，防止CPU占用过高
                time.sleep(0.1)

            return channel.recv_exit_status()
        finally:
            # 关闭SSH连接
            ssh.close()

    def get_latest_frame(self):
        """
        发送请求到服务器以获取最新的RGB和深度图像数据。

        返回:
        tuple: 包含RGB图像和深度图像的元组 (rgb_image, depth_image)，如果请求失败则返回 (None, None)。
        """
        try:
            # 发送GET请求到服务器
            response = requests.get("http://" + self.host + ":8000/get_latest_frame")

            # 检查请求是否成功
            if response.status_code == 200:
                data = response.json()

                # 获取Base64编码的RGB和深度图像数据
                rgb_image_base64 = data['rgb_image']
                depth_image_base64 = data['depth_image']
                camera_intrinsics = data['camera_intrinsics']

                # 解码Base64为二进制数据
                rgb_image_data = base64.b64decode(rgb_image_base64)
                depth_image_data = base64.b64decode(depth_image_base64)

                # 转换为NumPy数组
                rgb_image_np = np.frombuffer(rgb_image_data, np.uint8)
                depth_image_np = np.frombuffer(depth_image_data, np.uint8)

                # 解码为OpenCV图像
                rgb_image = cv2.imdecode(rgb_image_np, cv2.IMREAD_COLOR)
                depth_image = cv2.imdecode(depth_image_np, cv2.IMREAD_UNCHANGED)


                return rgb_image, depth_image, camera_intrinsics
            else:
                print(f"Failed to retrieve data from server, status code: {response.status_code}")
                return None, None, None
        except Exception as e:
            print(f"Exception occurred: {e}")
            return None, None, None

    def display_images(self, rgb_image, depth_image):
        """
        显示RGB和深度图像。

        参数:
        rgb_image (numpy.ndarray): RGB图像。
        depth_image (numpy.ndarray): 深度图像。
        """
        if rgb_image is not None and depth_image is not None:
            cv2.imshow('RGB Image', rgb_image)
            cv2.imshow('Depth Image', depth_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("No image data to display.")

    def display_rgb_point_cloud(self, rgb_image, depth_image, camera_intrinsics):
        """
        显示RGB点云，将RGB图像和深度图像转换为点云并显示。

        参数:
        rgb_image (np.ndarray): RGB图像。
        depth_image (np.ndarray): 深度图像。
        camera_intrinsics (dict): 相机的内参字典，包含 'fx', 'fy', 'cx', 'cy'。

        返回:
        None
        """
        # 获取RGB图像和深度图像的尺寸
        rgb_h, rgb_w = rgb_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]

        # 计算裁剪区域
        start_x = (rgb_w - depth_w) // 2
        start_y = (rgb_h - depth_h) // 2

        # 裁剪RGB图像以匹配深度图像的尺寸
        rgb_image_cropped = rgb_image[start_y:start_y + depth_h, start_x:start_x + depth_w]
        rgb_image_cropped = cv2.cvtColor(rgb_image_cropped, cv2.COLOR_RGB2BGR)

        # 将深度图像转换为浮点型并将单位从毫米转换为米（假设深度图像以毫米为单位）
        depth_image = depth_image.astype(np.float32) / 1000.0

        # 创建点云的空数组
        points = []
        colors = []

        # 相机内参
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['cx']
        cy = camera_intrinsics['cy']

        # 遍历每个像素，将其转换为3D点
        for v in range(depth_h):
            for u in range(depth_w):
                z = depth_image[v, u]
                if z > 0:  # 排除无效深度
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
                    colors.append(rgb_image_cropped[v, u] / 255.0)  # 颜色归一化到[0,1]

        # 将点云和颜色转换为NumPy数组
        points = np.array(points)
        colors = np.array(colors)

        # 创建Open3D点云对象
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)
        point_cloud.colors = o3d.utility.Vector3dVector(colors)

        # 显示点云
        o3d.visualization.draw_geometries([point_cloud])

if __name__ == "__main__":
    import time
    # 初始化客户端并指定服务器地址
    cam = ToolCamera(host='127.0.0.1')
    # cam.stop()
    cam.start()
    time.sleep(1)
    # 获取最新的RGB和深度图像
    rgb_image, depth_image, camera_intrinsics = cam.get_latest_frame()
    print(camera_intrinsics)

    max_depth = np.max(depth_image)
    print(np.min(depth_image),np.max(depth_image))
    # print(depth_image[200, 320])
    # depth_image = (depth_image / max_depth * 65535).astype(np.uint16)
    # print(np.min(depth_image),np.max(depth_image))
    # 对图像进行水平翻转
    # rgb_image = cv2.flip(rgb_image, 1)
    # depth_image = cv2.flip(depth_image, 1)

    # 显示图像
    cam.display_images(rgb_image, depth_image)
    cv2.imwrite("aucpuncture2point/configs/using_img/color.png",rgb_image)
    cv2.imwrite("aucpuncture2point/configs/using_img/depth.png",depth_image)

    # 显示RGB点云
    cam.display_rgb_point_cloud(rgb_image, depth_image, camera_intrinsics)
