import pysoem
import time

def main():
    # 创建 EtherCAT master 对象
    master = pysoem.Master()
    # 设置要使用的网卡名称，比如 'eth0'
    master.open(r'\Device\NPF_{AD703B2E-06D8-4AC2-8DF8-F227E4903107}')
    
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

    # 从站就一个电机编号从0开始
    slave = master.slaves[0]

    #电机使能1开启0关闭
    slave.sdo_write(0x2100, 0x00, (0x0001).to_bytes(2, 'little'))

    # [0]位置模式 [1]速度模式 [2]力矩模式 [3]电压模式 [4]电流模式
    slave.sdo_write(0x2101, 0x00, (0x0002).to_bytes(2, 'little'))

    #给定目标转矩单位0.1%, 范围-5000 - 5000, 正负控制正反转
    target_torque = 670
    slave.sdo_write(0x2700, 0x00, target_torque.to_bytes(4, 'little', signed=True))
    
    # time.sleep(2)
    
    # start_time = time.time()
    # for i in range(50):
    #     target_torque = -1
    #     slave.sdo_write(0x2700, 0x00, target_torque.to_bytes(4, 'little', signed=True))
    #     time.sleep(0.001)
    #     target_torque = 660
    #     slave.sdo_write(0x2700, 0x00, target_torque.to_bytes(4, 'little', signed=True))
    #     time.sleep(0.001)
    #     print(i)
    # end_time = time.time()
    # print(end_time - start_time)

    # # 平滑处理, 逐步减小力矩到 0
    # for torque in range(target_torque, 0, 10):  # 从 -50 到 0，步进为 10
    #     slave.sdo_write(0x2700, 0x00, torque.to_bytes(4, 'little', signed=True))
    #     time.sleep(0.1)  # 每次调整间隔
    #     torque_value = slave.sdo_read(0x6077, 0x00) #读取实际力矩

    #     torque_value = int.from_bytes(torque_value, byteorder='little', signed=True)
    #     print("实际力矩 = ", torque_value)

    # #电机使能1开启0关闭
    # slave.sdo_write(0x2100, 0x00, (0x0000).to_bytes(2, 'little'))

if __name__ == "__main__":
    main()
