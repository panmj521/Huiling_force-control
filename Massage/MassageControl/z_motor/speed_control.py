import pysoem
import time

def main():
    # 创建 EtherCAT master 对象
    master = pysoem.Master()
    # 设置要使用的网卡名称，比如 'eth0'
    master.open(r'enp3s0')
    
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

    #故障恢复
    slave.sdo_write(0x230A,0x00,(0x01).to_bytes(2, 'little'))

    #电机正反转0正转1反转
    slave.sdo_write(0x2607, 0x00, (0x00).to_bytes(2, 'little'))

    # [0]位置模式 [1]速度模式 [2]力矩模式 [3]电压模式 [4]电流模式
    slave.sdo_write(0x2101, 0x00, (0x00).to_bytes(2, 'little'))


    target_position = 486993  # 目标位置，用户单位
    slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))

    pp_mode = 4
    slave.sdo_write(0x2402, 0x00, int(pp_mode).to_bytes(2, 'little'))

    gain_position = 1500
    slave.sdo_write(0x2401, 0x00, int(gain_position).to_bytes(2, 'little'))



    # slave.sdo_write(0x2404, 0x00,(0x00).to_bytes(2, 'little'))
    # #电机使能1开启0关闭
    # slave.sdo_write(0x2100, 0x00, (0x0001).to_bytes(2, 'little'))

    # #给定目标速度单位0.1rpm, 范围-200000 - 200000, 正负控制正反转
    # target_speed = 2000
    # slave.sdo_write(0x2600, 0x00, target_speed.to_bytes(4, 'little', signed=True))

    # time.sleep(10)

    # for i in range(300):
    #     target_speed = 2000
    #     slave.sdo_write(0x2600, 0x00, target_speed.to_bytes(4, 'little', signed=True))
    #     time.sleep(0.1)
    #     target_speed = -2000
    #     slave.sdo_write(0x2600, 0x00, target_speed.to_bytes(4, 'little', signed=True))
    #     time.sleep(0.1)
    #     print(i)

    # target_speed = 0
    # slave.sdo_write(0x2600, 0x00, target_speed.to_bytes(4, 'little', signed=True))

    position_value = slave.sdo_read(0x6064,0x00)
    position_value = int.from_bytes(position_value, byteorder='little', signed=True)
    print("实际位置 = ", position_value)

    #  #电机使能1开启0关闭
    # slave.sdo_write(0x2100, 0x00, (0x0000).to_bytes(2, 'little'))

if __name__ == "__main__":
    main()
