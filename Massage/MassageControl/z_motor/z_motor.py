import pysoem
import time

def int_to_bytes(value, length):
    """
    将整数转换为小端字节序的 bytes。
    """
    return value.to_bytes(length, byteorder='little', signed=True)

# def initialize_motor(master, slave_index=0):
#     """
#     初始化电机，包括设置工作模式、使能伺服，并确保状态切换到“操作使能”。
#     """
#     slave = master.slaves[slave_index]

#     slave.sdo_write(0x1C12, 0x01, int_to_bytes(0x1600, 2))

#     control_word = 0x0006  # 示例控制字
#     control_word_bytes = control_word.to_bytes(2, byteorder='little')

#     pdo_data = bytearray(2)  # 假设 RPDO 的缓冲区长度为 2 字节
#     pdo_data[0:2] = control_word.to_bytes(2, 'little')  # 写入控制字

#     # 发送 PDO 数据
#     slave.tx_pdo(pdo_data)
#     print(f"Sent control word: {control_word:#06x}")

def main():
    # 创建 EtherCAT master 对象
    master = pysoem.Master()
    # 设置要使用的网卡名称，比如 'eth0'
    master.open('enp3s0')
    
    # 扫描从站，识别设备数量
    if master.config_init() > 0:
        print(f"Found {len(master.slaves)} slave(s).")
    else:
        print("No slaves found!")
        return
    # 映射 PDO
    master.config_map()
    # 配置 DC（分布式时钟），如果伺服需要同步时钟，可以在这里配置
    master.config_dc()
    slave = master.slaves[0]
    print(slave)
    slave.sdo_write(0x6040, 0x00, (0x0000).to_bytes(2, 'little'))


    slave.sdo_write(0x607A, 0x00, (0x03E8).to_bytes(4, 'little', signed=True))
    time.sleep(1)

    #电机使能1开启0关闭
    slave.sdo_write(0x2100, 0x00, (0x0001).to_bytes(2, 'little'))
    # #电机正反转0正转1反转
    slave.sdo_write(0x2607, 0x00, (0x0000).to_bytes(2, 'little'))

    # [0]位置模式 [1]速度模式 [2]力矩模式 [3]电压模式 [4]电流模式
    target_position = 1000000  # 目标位置，用户单位
    slave.sdo_write(0x2101, 0x00, int(target_position).to_bytes(4, 'little'))
    
    # # 切换到 OPERATIONAL 状态
    # master.state = pysoem.OP_STATE
    # master.write_state()
    # time.sleep(1)
    # if master.slaves[0].state != pysoem.OP_STATE:
    #     print(master.slaves[0].state)
    #     print("Failed to enter OP state.")
    #     return
    
    # print("All slaves are in OP state now.")

main()
