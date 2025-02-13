import os
import struct
import serial
import numpy as np
import atexit

import time

class XjcSensor:
    def __init__(self, port, baudrate, rate=250):
        self.port = port
        self.baudrate = baudrate
        self.rate = rate
        self.ser = serial.Serial(port, baudrate,timeout=0.1)
        self.crc16_table = self.generate_crc16_table()

        self.slave_adress = 0x01

        try:
            atexit.register(self.disconnect)

        except KeyboardInterrupt:
            print("Process interrupted by user")
            self.disconnect()
        
    def __new__(cls, *args, **kwargs):
        """
        Making this class singleton
        """
        if not hasattr(cls, 'instance'):
            cls.instance = super(XjcSensor, cls).__new__(cls)
        return cls.instance
    
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
    
    def crc16(self,data: bytes):
        crc = 0xFFFF
        for byte in data:
            crc = (crc >> 8) ^ self.crc16_table[(crc ^ byte) & 0xFF]
        return (crc >> 8) & 0xFF, crc & 0xFF


    def set_zero(self):
        """
        传感器置零，重新上电之后需要置零
        """
        try:
            # 重置输入输出缓冲区
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # 清除串口缓冲区中的数据
            if self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)

            # 根据地址设置置零命令
            if self.slave_adress == 0x01:
                zero_cmd = bytes.fromhex('01 10 46 1C 00 02 04 00 00 00 00 E8 95')  # 传感器内置通信协议--清零传感器数据  
            elif self.slave_adress == 0x09:
                zero_cmd = bytes.fromhex('09 10 46 1C 00 02 04 00 00 00 00 C2 F5')

            # 发送置零命令
            self.ser.write(zero_cmd)
            response = bytearray(self.ser.read(8))
            print(f"set zero response: {response}")

            # CRC 校验
            Hi_check, Lo_check = self.crc16(response[:-2])
            if response[0] != self.slave_adress or response[1] != 0x10 or \
            response[-1] != Hi_check or response[-2] != Lo_check:
                print("set zero failed!")
                return -1

            print("set zero success!")
            return 0

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            print(f"Details - Address: {self.slave_adress}, Port: {self.ser.port}")
            return -1
        except Exception as e:
            print(f"Unexpected error: {e}")
            return -1
   

    # 可能比较慢
    def read_data_f32(self):
        self.ser.reset_input_buffer()   # 清除输入缓冲区
        self.ser.reset_output_buffer()  # 清除输出缓冲区
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        if self.slave_adress == 0x01:
            command = bytes.fromhex('01 04 00 54 00 0C B1 DF')  # 传感器内置通信协议--读取一次六维力数据
        elif self.slave_adress == 0x09:
            command = bytes.fromhex('09 04 00 54 00 0C B0 97')  # 传感器内置通信协议--读取一次六维力数据      
        try:
            self.ser.write(command)
            response = bytearray(self.ser.read(29))
            print(response)
            Hi_check, Lo_check = self.crc16(response[:-2])
            if response[0] != self.slave_adress or response[1] != 0x04 or \
                response[-1] != Hi_check or response[-2] != Lo_check or response[2] != 24:
                print("read data failed!")
                return -1
            sensor_data = struct.unpack('>ffffff', response[3:27])
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return -1
        except Exception as e:
            print(f"Unexpected error: {e}")
            return -1
        return np.array(sensor_data)


    def enable_active_transmission(self):
        """
        Activate the sensor's active transmission mode
        """
        try:
            
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
    
    def disconnect(self):
        """
        Deactivate the sensor's active transmission mode.
        """
        try:
            # 关闭串口通信
            if hasattr(self, 'ser') and self.ser.is_open:
                send_str = "FF " * 50
                send_str = send_str[:-1]
                rate_cmd = bytes.fromhex(send_str)
                self.ser.write(rate_cmd)
                self.ser.close()
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
    

    def parse_data_passive(self, buffer):
        values = [
                int.from_bytes(buffer[i:i+2], byteorder='little', signed=True)
                for i in range(2, 14, 2)
                ]
        Fx, Fy, Fz = np.array(values[:3]) / 10.0
        Mx, My, Mz = np.array(values[3:]) / 1000.0
        return np.array([Fx, Fy, Fz, Mx, My, Mz])
    
    def __del__(self):
        self.disconnect()
    
if __name__ == "__main__":
    import sys
    import pathlib
    sys.path.append(str(pathlib.Path(__file__).parent.parent))
    from Rate import Rate
    
    import signal
    import sys
    rate = Rate(20)

    for i in range(10):
        try:
            sensor_name = "COM" + str(i)
            xjc_sensor = XjcSensor(sensor_name, 115200)
            xjc_sensor.enable_active_transmission()
            print("connect:",sensor_name)
            break
        except:
            print("sensor_name no found")

    def signal_handler(sig, frame):
        print('Received Ctrl+C, exiting gracefully...')
        sys.exit(0)
    signal.signal(signal.SIGINT, lambda signum, frame: sys.exit(0))
    # atexit.register(xjc_sensor.disconnect)
    # time.sleep(1)
    # xjc_sensor.disable_active_transmission()
    # xjc_sensor.disable_active_transmission()
    # xjc_sensor.disable_active_transmission()
    # print(xjc_sensor.read_data_f32())
    # print(xjc_sensor.read_data_f32())
    # print(xjc_sensor.read_data_f32())
    # # print(xjc_sensor.read())
    # time.sleep(1)
    # xjc_sensor.set_zero()
    # xjc_sensor.set_zero()
    # xjc_sensor.set_zero()
    # time.sleep(1)
    
    while True:
        sensor_data = xjc_sensor.read()
        #sensor_data = xjc_sensor.read_data_f32()
        if sensor_data is None:
           print('failed to get force sensor data!')
        print(sensor_data)
 
    #     # rate.sleep(False)
    #     time.sleep(0.1)
    #     # break