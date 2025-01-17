import serial.tools.list_ports as list_ports
import subprocess

def find_serial_by_serial_number(serial_number):
    ports = list_ports.comports()
    for port in ports:
        if port.serial_number == serial_number:
            return port.device
    return None

def find_serial_by_location(location):
    ports = list_ports.comports()
    for port in ports:
        if port.location == location:
            return port.device
    return None

def list_usb_ports_with_details():
    ports = list_ports.comports()
    usb_ports = {}

    for port in ports:
        if "ttyUSB" in port.device:
            usb_ports[port.device] = {
                'serial_number': port.serial_number,
                'vid': port.vid,
                'pid': port.pid,
                'location': port.location,
                'description': port.description,
                'manufacturer': port.manufacturer
            }

    return usb_ports

# 维护 socat 进程的字典，用于跟踪每个虚拟串口的进程
socat_processes = {}

def start_virtual_serial(remote_ip, remote_port, device_name):
    """
    启动一个 socat 进程来创建虚拟串口。
    :param remote_ip: 远程主机 IP 地址
    :param remote_port: 远程主机端口
    :param device_name: 虚拟串口的设备名称
    """
    # 构建 socat 命令
    socat_command = f'socat PTY,raw,echo=0,link={device_name} TCP:{remote_ip}:{remote_port}'
    # 启动 socat 进程
    process = subprocess.Popen(socat_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # 将进程存储到字典中
    socat_processes[device_name] = process
    print(f"Started socat process for {device_name} on {remote_ip}:{remote_port}")

def stop_virtual_serial(device_name):
    """
    停止一个 socat 进程。
    :param device_name: 虚拟串口的设备名称
    """
    # 检查进程是否在字典中
    if device_name in socat_processes:
        # 获取进程对象
        process = socat_processes[device_name]
        # 使用 pkill 终止相应的 socat 进程
        subprocess.call(['pkill', '-f', f'{device_name}'])
        # 终止进程
        process.terminate()
        process.wait()
        # 移除已终止的进程
        del socat_processes[device_name]
        print(f"Stopped socat process for {device_name}")
    else:
        print(f"No running socat process found for {device_name}")

if __name__ == "__main__":
    usb_ports_with_details = list_usb_ports_with_details()

    if usb_ports_with_details:
        print("USB Ports and their Details:")
        for device, details in usb_ports_with_details.items():
            print(f"Device: {device}, Serial Number: {details['serial_number']}, VID: {details['vid']}, PID: {details['pid']}, Location: {details['location']}, Description: {details['description']}, Manufacturer: {details['manufacturer']}")
    else:
        print("No USB ports found.")

    # serial_number = "1234567890"  # Replace with your serial number
    # port = find_serial_by_serial_number(serial_number)
    # if port:
    #     print(f"Serial number {serial_number} found on port {port}")
    # else:
    #     print(f"Serial number {serial_number} not found")

    port = find_serial_by_location("1-8")
    if port:
        print(f"Port found: {port}")
    else:
        print("Port not found")