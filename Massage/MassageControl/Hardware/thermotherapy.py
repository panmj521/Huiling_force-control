import struct
import serial
import numpy as np
import atexit
import time
import subprocess
import psutil
import threading
import signal
import sys
sys.path.append("/home/jsfb/jsfb_ws/MassageRobot_aubo/Massage/MassageControl")
from tools.ssh_tools import execute_command_on_remote
from tools.serial_tools import start_virtual_serial, stop_virtual_serial

class Thermotherapy:
    def __init__(self, port="/tmp/ttyThermotherapy", baudrate=57600, arm_ip='192.168.100.20'):
        self.port = port
        self.baudrate = baudrate
        self.arm_ip = arm_ip
        self.command_interval = 0.2  # 指令间隔200ms
        self.max_attempts = 5  # 最多尝试次数
        self.disconnect_attempts = 0  # 断开计数器
        self.running = False  # 控制线程的运行状态
        self.thread = None  # 线程对象
        username = 'root'
        password = 'bestcobot'
        ssh_port = 8822


        self.level = 10
        self.upper_temp = 50
        self.lower_temp = 30

        # self.level = 4
        # self.upper_temp = 35
        # self.lower_temp = 20

        self.minutes = 15
        self.seconds = 0

        self.print_status = False

        try:
            # 启动ser2net服务
            stop_command = 'systemctl start ser2net'
            output, error = execute_command_on_remote(self.arm_ip, username, password, stop_command, ssh_port)
            print('start ser2net')

            # 启动虚拟串口
            stop_virtual_serial(port)
            start_virtual_serial(self.arm_ip, 5001, port)
            time.sleep(0.1)

            self.ser = serial.Serial(port, baudrate, bytesize=8, stopbits=1, parity=serial.PARITY_NONE, timeout=1)

            atexit.register(self.disconnect)

        except KeyboardInterrupt:
            print("Process interrupted by user")
            self.disconnect()

    def calculate_checksum(self, data):
        """
        计算校验码：从发送方到数据域所有数据相加和，并取低八位，再取反
        """
        checksum = sum(data) & 0xFF  # 取低八位
        checksum = ~checksum & 0xFF  # 取反
        return checksum

    def send_command(self, command_code, data_field):
        """
        发送Hex格式命令的函数并读取返回值
        参数:
        - command_code: 指令码 (1字节)
        - data_field: 数据域 (字节列表)
        """
        HEADER = [0x5A, 0xA5]
        FOOTER = [0xC3, 0x3C]
        sender = 0x01  # 发送方：主机
        receiver = 0x02  # 接收方：从机
        data_length = len(data_field)

        # 构建命令
        command = HEADER + [sender, receiver, command_code, data_length] + data_field
        checksum = self.calculate_checksum(command[2:])  # 从发送方到数据域所有数据
        command.append(checksum)
        command += FOOTER

        # 将指令转换为字节流
        command_bytes = bytes(command)

        try:
            # 发送指令
            self.ser.write(command_bytes)
            # print("发送的命令:", command_bytes.hex())

            # 指令间隔 > 200 ms
            time.sleep(self.command_interval)

            # 读取返回数据
            if self.ser.in_waiting > 0:  # 检查是否有数据返回
                response = self.ser.read(self.ser.in_waiting)  # 读取所有可用的返回数据
                # print("接收到的返回值:", response.hex())
                self.disconnect_attempts = 0  # 重置断开计数器
                return response
            else:
                self.disconnect_attempts += 1
                print("没有接收到返回数据。尝试次数:", self.disconnect_attempts)
                if self.disconnect_attempts >= self.max_attempts:
                    print("与理疗设备断开连接，报警并停止作业。")
                    self.stop_module()
                return None
        except (IOError, serial.SerialException) as e:
            print(f"通信错误: {e}")
            return None
        except Exception as e:
            print(f"发生未知错误: {e}")
            return None


    def parse_device_status(self, response):
        """
        解析设备状态信息
        """
        if len(response) < 8:
            print("无效的设备状态返回数据。")
            return

        data_field = list(response[6:6 + 6])
        current_status = data_field[0]
        module_temp = (data_field[2] * 256 + data_field[1]) / 10.0
        remaining_time_min = data_field[3]
        remaining_time_sec = data_field[4]
        fault_code = data_field[5]

        # 尝试将剩余时间转换为数值，如果不能转换，则设置为无效的值（比如0）
        try:
            remaining_time_min = int(remaining_time_min)
        except (ValueError, TypeError):
            print(f"警告：剩余时间分钟值无效: {remaining_time_min}")
            remaining_time_min = 0

        try:
            remaining_time_sec = int(remaining_time_sec)
        except (ValueError, TypeError):
            print(f"警告：剩余时间秒钟值无效: {remaining_time_sec}")
            remaining_time_sec = 0

        # 构造状态信息字典
        status_info = {
            "当前工作状态": "停止" if current_status == 1 else "工作" if current_status == 2 else "未知状态",
            "模块温度": "{:.1f} ℃".format(module_temp),
            "剩余工作时间": "{} 分钟 {} 秒".format(remaining_time_min, remaining_time_sec),
            "故障码": {
                0: "正常",
                1: "散热器温度过高保护",
                2: "红外热敏异常"
            }.get(fault_code, "错误状态")
        }

        # 如果设备状态非工作状态，或故障码不为0（正常），则打印警告信息
        if current_status != 2 or fault_code != 0:
            print("警告：设备处于非工作状态或存在故障！")
            for key, value in status_info.items():
                print("{}: {}".format(key, value))
            
            # 如果设备处于非工作状态且剩余工作时间有效，则尝试重新设置工作状态
            if current_status != 2 and (remaining_time_min > 0 or remaining_time_sec > 0):
                print("尝试重新设置设备为工作状态...")
                self.set_working_status()
        else:
            # 如果状态正常，且标志位 print_status 为 True，则打印状态信息
            if self.print_status:
                for key, value in status_info.items():
                    print("{}: {}".format(key, value))


    def get_device_status(self):
        """
        获取设备状态信息，并处理任何可能的异常
        """
        try:
            response = self.send_command(0x02, [0x00])
            if response:
                self.parse_device_status(response)
            else:
                print("未收到设备状态数据。")
        except Exception as e:
            print(f"获取设备状态时发生异常: {e}")

    def set_working_status(self, level=None, upper_temp=None, lower_temp=None, status=2, max_retries=3):
        """
        设置工作状态，并解析和打印返回的数据。如果出现通信问题或返回不匹配，将进行重试。
        """
        retry_count = 0
        success = False

        while retry_count < max_retries and not success:
            try:
                # 如果未传入 level 或温度参数，则使用当前实例的默认值
                if level is None:
                    level = self.level
                else:
                    self.level = level

                if upper_temp is None:
                    upper_temp = self.upper_temp
                else:
                    self.upper_temp = upper_temp

                if lower_temp is None:
                    lower_temp = self.lower_temp
                else:
                    self.lower_temp = lower_temp

                # 计算温度的高低八位
                upper_temp_low = upper_temp % 256
                upper_temp_high = upper_temp // 256
                lower_temp_low = lower_temp % 256
                lower_temp_high = lower_temp // 256

                # 组成要发送的数据字段
                data_field = [level, upper_temp_low, upper_temp_high, lower_temp_low, lower_temp_high, status]
                response = self.send_command(0x01, data_field)

                # 检查是否有返回数据
                if response:
                    # 解析返回的数据
                    level_response = response[6]
                    upper_temp_response = response[8] * 256 + response[7]  # 上限温度 = 高八位 * 256 + 低八位
                    lower_temp_response = response[10] * 256 + response[9]  # 下限温度 = 高八位 * 256 + 低八位
                    work_status = response[11]

                    print(f"档位: {level_response}, 温度上限: {upper_temp_response}, 温度下限: {lower_temp_response}, 工作状态: {work_status}")

                    # 检查返回的数据是否与发送的匹配
                    if (level_response == level and
                        response[7] == upper_temp_low and
                        response[8] == upper_temp_high and
                        response[9] == lower_temp_low and
                        response[10] == lower_temp_high and
                        work_status == 0):  # 0表示工作状态设置成功
                        print("工作状态设置成功。")
                        success = True  # 如果成功，退出循环
                    else:
                        print("工作状态设置失败，返回数据不匹配。")
                        retry_count += 1  # 增加重试次数
                else:
                    print("未收到返回数据，正在重试...")
                    retry_count += 1  # 增加重试次数

            except Exception as e:
                print(f"出现异常: {e}，正在重试...")
                retry_count += 1  # 增加重试次数

        if not success:
            print(f"设置工作状态失败，已重试 {max_retries} 次。")



    def set_working_time(self, minutes, seconds, max_retries=3):
        """
        设置工作时间，并解析返回的数据。如果出现通信问题或返回不匹配，将进行重试。
        """
        retry_count = 0
        success = False

        if minutes is None:
            minutes = self.minutes
        else:
            self.minutes = minutes
        
        if seconds is None:
            seconds = self.seconds
        else:
            self.seconds = seconds


        while retry_count < max_retries and not success:
            try:
                # 发送工作时间指令 (0x04) 和数据域 [minutes, seconds]
                response = self.send_command(0x04, [minutes, seconds])

                # 检查是否有返回数据
                if response:
                    response_minutes = response[6]
                    response_seconds = response[7]

                    print(f"工作时间: {response_minutes} 分 {response_seconds} 秒")

                    # 检查返回的时间是否与发送的匹配
                    if response_minutes == minutes and response_seconds == seconds:
                        print("工作时间设置成功。")
                        success = True  # 设置成功，退出重试循环
                    else:
                        print("工作时间设置失败，返回数据不匹配。")
                        retry_count += 1  # 增加重试次数
                else:
                    print("未收到返回数据，正在重试...")
                    retry_count += 1  # 增加重试次数

            except Exception as e:
                print(f"出现异常: {e}，正在重试...")
                retry_count += 1  # 增加重试次数

        if not success:
            print(f"设置工作时间失败，已重试 {max_retries} 次。")


    def stop_module(self):
        """
        停止理疗模块工作
        """
        self.set_working_status(1, 0, 0, 1)  # 停止工作

    def start_module(self, minutes, seconds):
        """
        启动理疗模块
        """
        self.set_working_time(minutes, seconds)
        # self.set_working_status(3, 30, 20, 2)  # 设置档位、温度上下限，启动工作

        self.set_working_status(10, 50, 40, 2)  # 设置档位、温度上下限，启动工作



    # Heartbeat function to continuously get device status
    def heartbeat(self):
        """
        持续获取设备状态，确保线程不会因异常退出
        """
        while self.running:
            try:
                # 获取设备状态
                self.get_device_status()
                time.sleep(0.5)  # 每0.5秒钟获取一次设备状态
            except Exception as e:
                # 捕获并处理所有可能的异常，确保线程不会异常退出
                print(f"获取设备状态时发生异常: {e}")

    # Start the module and the heartbeat thread
    def start_process(self, minutes, seconds):
        """
        启动理疗模块并开始心跳线程
        """
        if not self.running:
            try:
                # 启动理疗模块
                self.start_module(minutes, seconds)
                
                # 启动线程来获取设备状态
                self.running = True
                self.thread = threading.Thread(target=self.heartbeat)
                self.thread.start()
                print("Started module and heartbeat thread.")
            except Exception as e:
                # 捕获启动过程中的异常
                print(f"启动过程中发生异常: {e}")

    # Stop the module and stop the heartbeat thread
    def stop_process(self):
        """
        停止理疗模块和心跳线程
        """
        if self.running:
            try:
                # 停止理疗模块
                self.stop_module()

                # 停止心跳线程
                self.running = False
                if self.thread:
                    self.thread.join()
                print("Stopped module and heartbeat thread.")
            except Exception as e:
                # 捕获停止过程中的异常
                print(f"停止过程中发生异常: {e}")
        else:
            print("模块未在运行。")
        # 保证模块总是被停止
        self.stop_module()

    def disconnect(self):
        """
        断开与设备的连接
        """
        # self.stop_module()
        # 停止本地的socat进程
        stop_virtual_serial(self.port)

        # 停止远程的ser2net服务
        if hasattr(self, 'arm_ip'):
            stop_command = 'systemctl stop ser2net'
            output, error = execute_command_on_remote(self.arm_ip, 'root', 'bestcobot', stop_command, 8822)
            print('stop ser2net')

        if self.ser.is_open:
            self.ser.close()
            print("已断开串口连接")

    def __del__(self):
        self.disconnect()

# if __name__ == '__main__':
#     module = Thermotherapy('/tmp/ttyThermotherapy')
#     def signal_handler(sig, frame):
#         print('Received Ctrl+C, exiting gracefully...')
#         sys.exit(0)
#     # 启动理疗模块，设置工作时间为10秒
#     module.start_module(0, 15)

#     # 持续运行15秒，期间每200ms获取一次设备状态
#     start_time = time.time()
#     while time.time() - start_time < 15:
#         print("-----------")
#         # module.set_working_status(20, 420, 420, 2)  # 设置档位、温度上下限，启动工作
#         module.get_device_status()
#         time.sleep(1)

#     # 停止模块
#     module.stop_module()

#     # 关闭串口连接
#     module.disconnect()

if __name__ == '__main__':
    module = Thermotherapy('/tmp/ttyThermotherapy')

    def signal_handler(sig, frame):
        print('Received Ctrl+C, exiting gracefully...')
        module.stop_process()
        sys.exit(0)

    # 捕获Ctrl+C信号以优雅退出
    signal.signal(signal.SIGINT, signal_handler)

    # 启动理疗模块，设置工作时间为15秒，并启动心跳线程
    module.start_process(0, 60)

    # 等待15秒后停止模块并停止心跳线程
    time.sleep(60)

    # 停止模块
    module.stop_process()

    # 关闭串口连接
    module.disconnect()

