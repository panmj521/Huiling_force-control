import sys
import time
import pysoem

def initalize_master():
    master = pysoem.Master()
    master.open(f'enp3s0')
    return master

def configure_motor(master):
    # 配置从站
    if not master.config_init():
        raise Exception('Failed to initialize slave configuration')  
    # 映射PDO
    master.config_map()
    # 配置DC（分布式时钟）
    master.config_dc()
    # 获取EtherCAT从站对象
    slave = master.slaves[0]
    # 设置并验证工作模式
    target_mode = 0x01  # 轮廓位置模式
    slave.sdo_write(0x6060, 0x00, int(target_mode).to_bytes(1, 'little'))
    # 读取当前模式
    current_mode = int.from_bytes(
        slave.sdo_read(0x6060, 0x00),
        byteorder='little',
        signed=False
    )
    if current_mode != target_mode:
        raise Exception(f'Failed to set operation mode. Expected: {target_mode}, Actual: {current_mode}')
    
    print(f"Operation mode successfully set to: {current_mode}")

    # 设置目标位置 (用户单位)
    target_position = 1000  # 目标位置，用户单位
    slave.sdo_write(0x607A, 0x00, int(target_position).to_bytes(4, 'little'))  # 0x607Ah 设置目标位置

    # 读取当前位置
    current_target_position = int.from_bytes(
        slave.sdo_read(0x607A, 0x00),
        byteorder='little',
        signed=False
    )
    if current_target_position != target_position:
        raise Exception(f'Failed to set operation mode. Expected: {target_position}, Actual: {current_target_position }')
    
    print(f"Operation mode successfully set to: {current_target_position}")

    # 设置匀速运行速度 (用户单位/s)
    velocity = 100  # 运行速度，用户单位/s
    slave.sdo_write(0x6081, 0x00, int(velocity).to_bytes(4, 'little'))  # 0x6081h 设置匀速运行速度

    # 设置加速度和减速度 (用户单位/s^2)
    acceleration = 10  # 加速度，用户单位/s^2
    deceleration = 10  # 减速度，用户单位/s^2
    slave.sdo_write(0x6083, 0x00, int(acceleration).to_bytes(4, 'little'))  # 0x6083h 设置加速度
    slave.sdo_write(0x6084, 0x00, int(deceleration).to_bytes(4, 'little'))  # 0x6084h 设置减速度

    # 设置控制字（状态机切换）
    control_word = 0x06  # 初始控制字
    slave.sdo_write(0x6040, 0x00, int(control_word).to_bytes(2, 'little'))
    



# 主函数
if __name__ == "__main__":
    master = None
    # 初始化
    master = initalize_master()
    if not master:
        sys.exit(1)           
    # 配置电机
    configure_motor(master)

