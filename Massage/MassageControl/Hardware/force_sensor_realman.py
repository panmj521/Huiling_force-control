import struct
import serial
import numpy as np
import atexit

import time

import subprocess
import psutil
import sys
sys.path.append("/home/jsfb/jsfb_ws/MassageRobot_aubo01/Massage/MassageControl")
from tools.ssh_tools import execute_command_on_remote
from tools.serial_tools import start_virtual_serial,stop_virtual_serial

from hardware.realman_RM63 import RealmanRM63

import serial
import atexit
import os
import struct
import numpy as np
import time

class XjcSensor:
    def __init__(self, port="/tmp/ttyRobotTool", baudrate=115200, arm_ip='192.168.1.18', rate=250):
        self.port = port
        self.baudrate = baudrate
        self.rate = rate
        self.arm_ip = arm_ip
        
        self.crc16_table = self.generate_crc16_table()
        self.slave_adress = 0x09
        self.ser = None
        
        try:
            self.connect()
            atexit.register(self.disconnect)

        except KeyboardInterrupt:
            print("Process interrupted by user")
            self.disconnect()
        
    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, 'instance'):
            cls.instance = super(XjcSensor, cls).__new__(cls)
        return cls.instance
    
    def connect(self):
        # print(rm63.robot.rm_set_modbus_mode(1,115200,2))
        print(rm63.robot.rm_set_modbustcp_mode("192.168.1.18", 502, 2000))
        # Start socat process and check if the virtual port is created
        stop_virtual_serial(self.port)
        start_virtual_serial(self.arm_ip, 502, self.port)
        
        # Wait for socat to initialize
        for _ in range(10):  # Wait up to 1 second (10 * 0.1s)
            if os.path.exists(self.port):
                break
            time.sleep(0.1)
        else:
            print(f"Error: {self.port} does not exist. Check if socat is running.")
            return -1

        # 在本地后台启动socat命令
        subprocess.call(['pkill', '-f', 'socat'])
        socat_command = f'socat PTY,raw,echo=0,link=/tmp/ttyRobotTool TCP:{self.arm_ip}:5000'
        self.socat_process = subprocess.Popen(socat_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print('socat process started')
        stop_virtual_serial(self.port)
        start_virtual_serial(self.arm_ip,5000,self.port)

        time.sleep(0.1)

        self.ser = serial.Serial(self.port, self.baudrate,timeout=0.1)

        # Open serial port
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"Connected to {self.port}")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return -1

    @staticmethod
    def generate_crc16_table():
        table = []
        polynomial = 0xA001
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ polynomial
                else:
                    crc >>= 1
            table.append(crc)
        return table
    
    def crc16(self, data: bytes):
        crc = 0xFFFF
        for byte in data:
            crc = (crc >> 8) ^ self.crc16_table[(crc ^ byte) & 0xFF]
        return (crc >> 8) & 0xFF, crc & 0xFF

    def set_zero(self):
        try:
            if not self.ser.is_open:
                raise serial.SerialException("Serial port is not open")

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)

            zero_cmd = bytes.fromhex('01 10 46 1C 00 02 04 00 00 00 00 E8 95')
            self.ser.write(zero_cmd)
            response = bytearray(self.ser.read(8))
            print(f"set zero response: {response}")

            Hi_check, Lo_check = self.crc16(response[:-2])
            if response[0] != self.slave_address or response[1] != 0x10 or \
               response[-1] != Hi_check or response[-2] != Lo_check:
                print("set zero failed!")
                return -1

            print("set zero success!")
            return 0

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return -1
        except Exception as e:
            print(f"Unexpected error: {e}")
            return -1

    def read_data_f32(self):
        try:
            if not self.ser.is_open:
                raise serial.SerialException("Serial port is not open")

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)

            command = bytes.fromhex('01 04 00 00 00 0C F0 0F')
            self.ser.write(command)

            response = bytearray(self.ser.read(29))
            print("response:",response)
            if len(response) < 29:
                print("Received response is too short!")
                return -1

            Hi_check, Lo_check = self.crc16(response[:-2])
            if response[0] != self.slave_address or response[1] != 0x04 or \
            response[-1] != Hi_check or response[-2] != Lo_check or response[2] != 24:
                print("read data failed!")
                return -1

            sensor_data = struct.unpack('>ffffff', response[3:27])
            return np.array(sensor_data)

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return -1
        except Exception as e:
            print(f"Unexpected error: {e}")
            return -1

    def read(self):
        """
        Read the sensor's data.
        """
        try:
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            while True:
                if int.from_bytes(self.ser.read(1), byteorder='big') == 0x20:
                    if int.from_bytes(self.ser.read(1), byteorder='big') == 0x4E:
                        break
            response = bytearray([0x20, 0x4E])  # 使用bytearray创建16进制数据的列表
            response.extend(self.ser.read(14))  # 读取12个字节的数据并添加到列表中
            Hi_check, Lo_check = self.crc16(response[:-2])
            if response[-1] != Hi_check or response[-2] != Lo_check:
                print("check failed!")
                return None
            sensor_data = self.parse_data_passive(response)
            return sensor_data
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None    
    
    def enable_active_transmission(self):
        """
        Activate the sensor's active transmission mode
        """
        try:
            # 根据不同的速率设置命令
            if self.rate == 100:
                if self.slave_adress == 0x01:
                    rate_cmd = bytes.fromhex('01 10 01 9A 00 01 02 00 00 AB 6A')  # 传感器内置通信协议--100Hz
                elif self.slave_adress == 0x09:
                    rate_cmd = bytes.fromhex('09 10 01 9A 00 01 02 00 00 CC AA')
                print("Set mode in 100Hz")
            elif self.rate == 250:
                if self.slave_adress == 0x01:
                    rate_cmd = bytes.fromhex('01 10 01 9A 00 01 02 00 01 6A AA')
                elif self.slave_adress == 0x09:
                    rate_cmd = bytes.fromhex('09 10 01 9A 00 01 02 00 01 0D 6A')
                print("Set mode in 250Hz")
            elif self.rate == 500:  
                if self.slave_adress == 0x01:
                    rate_cmd = bytes.fromhex('01 10 01 9A 00 01 02 00 02 2A AB')  # 传感器内置通信协议--500Hz
                elif self.slave_adress == 0x09:
                    rate_cmd = bytes.fromhex('09 10 01 9A 00 01 02 00 02 4D 6B')
                print("Set mode in 500Hz")
            else:
                print("Rate not supported")
                return

            # 检查串口是否打开
            if not self.ser.is_open:
                raise serial.SerialException("Serial port is not open")

            # 重置输入缓冲区
            self.ser.reset_input_buffer()
            print("重置缓冲区")
             
            # 写入指令
            self.ser.write(rate_cmd)
            print("Transmission command sent successfully")
            return 0

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            print(f"Details - Rate: {self.rate}, Address: {self.slave_adress}, Port: {self.ser.port}")
            return -1
        except Exception as e:
            print(f"Unexpected error: {e}")
            return -1
        
    
    def disable_active_transmission(self):
        """
        Disable the sensor's active transmission mode
        """
        try:
            # 禁用传感器活动传输模式的命令
            rate_cmd = bytes.fromhex('FF FF FF FF FF FF FF FF FF FF FF')
            print("Disable the sensor's active transmission mode")

            # 重置输入缓冲区
            if not self.ser.is_open:
                raise serial.SerialException("Serial port is not open")
            self.ser.reset_input_buffer()

            # 发送禁用传输模式的命令
            self.ser.write(rate_cmd)
            print("Transmission disabled successfully")
            return 0

        except serial.SerialException as e:



            print(f"Serial communication error: {e}")
            print(f"Details - Port: {self.ser.port}")
            return -1
        except Exception as e:
            print(f"Unexpected error: {e}")
            return -1

    def disconnect(self):
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
                print("Serial port closed.")
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")

        stop_virtual_serial(self.port)

    def __del__(self):
        self.disconnect()

if __name__ == "__main__":
    # xjc_sensor = XjcSensor()
    # time.sleep(1)
    # xjc_sensor.set_zero()
    # print(xjc_sensor.read_data_f32())
    import sys
    import pathlib
    sys.path.append(str(pathlib.Path(__file__).parent.parent))
    from tools.Rate import Rate
    import signal
    import sys

    # 实例化
    rm63 = RealmanRM63()
    xjc_sensor = XjcSensor(port="/tmp/ttyRobotTool", baudrate=115200, arm_ip='192.168.1.18')
    def signal_handler(sig, frame):
        print('Received Ctrl+C, exiting gracefully...')
        sys.exit(0)
    signal.signal(signal.SIGINT, lambda signum, frame: sys.exit(0))
    
    # atexit.register(xjc_sensor.disconnect)
    
    xjc_sensor.disable_active_transmission()
    # xjc_sensor.disable_active_transmission()
    # xjc_sensor.disable_active_transmission()
    time.sleep(1)
    xjc_sensor.enable_active_transmission()

    print(xjc_sensor.read_data_f32())
    print(xjc_sensor.read_data_f32())
    print(xjc_sensor.read_data_f32())
    # print(xjc_sensor.read())
    time.sleep(1)
    xjc_sensor.set_zero()
    xjc_sensor.set_zero()
    xjc_sensor.set_zero()
    time.sleep(1)
    
    # while True:
    #     sensor_data = xjc_sensor.read()
    #     #sensor_data = xjc_sensor.read_data_f32()
    #     if sensor_data is None:
    #        print('failed to get force sensor data!')
    #     print(sensor_data)
    #     # rate.sleep(False)
    #     # break