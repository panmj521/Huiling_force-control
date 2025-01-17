import time
import serial
import serial.tools.list_ports
import signal
import sys
import yaml

sys.path.append("/home/jsfb/jsfb_ws/MassageRobot_aubo/Massage/MassageControl")

class Shockwave:
    def __init__(self, baudrate=115200, timeout=1):
        self.port = self.find_esp32_port()
        self.baudrate = baudrate
        self.p_level = 7
        self.f_level = 7
        self.p_level_diff = 0
        self.f_level_diff = 0
        self.isOn = False
        if self.port:
            self.ser = serial.Serial(self.port, baudrate, timeout=1)
            print(f"连接到 {self.port}，波特率: {baudrate}")
        else:
            print("无法找到 ESP32 串口。")

    def find_esp32_port(self):
        """寻找对应通用控制板的串口"""

        # 目标 YAML 文件路径
        yaml_file_path = '/home/jsfb/jsfb_ws/global_config/massage_head/shockwave_playload.yaml'

        # 读取现有的 YAML 文件内容
        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)  # 读取并解析 YAML 内容

        # 获取 target_usb_device_path 的值
        target_usb_device_path = data.get('target_usb_device_path')

        print(f"已成功将 target_usb_device_path 添加到 {yaml_file_path}")     

        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(port.usb_device_path)
            if target_usb_device_path == port.usb_device_path:
                print(f"找到 ESP32 在端口: {port.usb_device_path}")
                return port.device
        print("未找到具有指定描述的 ESP32。")
    
    def close(self):
        """关闭串口"""
        if self.ser.is_open:
            self.ser.close()
            print("串口关闭。")
    
    def on(self):
        """开启冲击波"""
        if self.isOn == False:
            command = 'Shockwave On'
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            self.isOn = True
        else:
            print("冲击波已经开启")
    
    def off(self):
        """关闭冲击波"""
        if self.isOn == True:
            command = 'Shockwave Off'
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            self.isOn = False
        else:
            print("冲击波已经关闭")

    def p_plus(self):
        """冲击波压力+1档"""
        command = 'P+'
        if self.ser.isOpen():
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            self.p_level = self.p_level + 1
        else:
            raise ConnectionError("Serial port not open")

    def p_minus(self):
        """冲击波压力-1档"""
        command = 'P-'
        if self.ser.isOpen():
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            self.p_level = self.p_level - 1
        else:
            raise ConnectionError("Serial port not open")

    def p_set(self, level):
        """ 设置冲击波压力到 'level' 档
            level范围:1-27
        """
        command = 'SET P=' + str(level)
        self.p_level_diff = self.p_level - level
        self.p_level = level
        if self.ser.isOpen():
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            time.sleep(abs(self.p_level_diff)*0.3)
        else:
            raise ConnectionError("Serial port not open")

    def f_plus(self):
        """冲击波频率+1档"""
        command = 'F+'
        if self.ser.isOpen():
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            self.f_level = self.f_level + 1
        else:
            raise ConnectionError("Serial port not open")

    def f_minus(self):
        """冲击波频率-1档"""
        command = 'F-'
        if self.ser.isOpen():
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            self.f_level = self.f_level - 1
        else:
            raise ConnectionError("Serial port not open")

    def f_set(self, level):
        """ 设置冲击波频率到 'level' 档
            level范围:1-16
        """
        command = 'SET F=' + str(level)
        self.f_level_diff = self.f_level - level
        self.f_level = level
        if self.ser.isOpen():
            self.ser.write((command + '\n').encode('UTF-8'))  # 发送命令并换行
            response = self.ser.readline().decode()  # 读取ESP32的响应
            print(response)
            time.sleep(abs(self.f_level_diff)*0.3)
        else:
            raise ConnectionError("Serial port not open")

# 使用示例
if __name__ == "__main__":
    # 创建对象
    board = Shockwave()

    def signal_handler(sig, frame):
        print('Received Ctrl+C, exiting gracefully...')
        board.f_set(1)
        board.close()
        sys.exit(0)

    # 开关测试
    board.on()
    board.p_set(1)
    board.f_set(1)
    time.sleep(2)
    board.p_set(8) 
    board.f_set(8)
    time.sleep(2)
    board.p_set(1)
    board.f_set(1)
    board.off()