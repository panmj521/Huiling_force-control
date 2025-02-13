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
    mode = 0
    slave.sdo_write(0x2101, 0x00, int(mode).to_bytes(2, 'little'))
    #位置模式指令选择
    #[0]增量模式 [1]绝对值模式 用于区分PP模式下位置指令是增量值还是绝对值
    slave.sdo_write(0x2404, 0x00,(0x01).to_bytes(2, 'little'))

    # 读取当前模式，是否正确
    current_position = int.from_bytes(
        slave.sdo_read(0x2101, 0x00),
        byteorder='little',
        signed=False
    )
    if current_position != mode:
        raise Exception(f'Failed to set operation mode. Expected: {mode}, Actual: {current_position}')
    print(f"Operation mode successfully set to: {current_position}")

    # [0]CSP ethercat总线输入，位置命令由ethercat总线周期发送，伺服不对位置轨迹做规划，只监控位置指令是否存在异常
    # [1]JOG模式，轨迹由内部产生，可用于测试，辨识负载惯量比
    # [2]内部测试信号发生器，用于产生周期性位置信号，测试位置环控制效果
    # [3]脉冲输入，位置命令由光耦脉冲输入
    # [4]PP点到点模式ECAT，实现Cia402点位控制模式，轨迹由伺服自己产生，主站提供目标位置，并约束加速度，和峰值速度
    # [5]PP点到点模式USER，用户自定义点位命令模式，可以不遵循Cia402规范    
    #设置当前具体位置模式
    pp_mode = 5
    slave.sdo_write(0x2402, 0x00, int(pp_mode).to_bytes(2, 'little'))
    #当前具体位置模式是否设置正确
    current_pp_mode = int.from_bytes(
        slave.sdo_read(0x2402, 0x00),
        byteorder='little',
        signed=False
    )
    if current_pp_mode != pp_mode:
        raise Exception(f'Failed to set operation mode. Expected: {pp_mode}, Actual: {current_pp_mode}')
    print(f"Operation mode successfully set to: {current_pp_mode}")


    # target_speed = 2000
    # slave.sdo_write(0x2600, 0x00, target_speed.to_bytes(4, 'little', signed=True))
    # time.sleep(0.1)

    position_value = slave.sdo_read(0x6064, 0x00)
    position_value = int.from_bytes(position_value, byteorder='little', signed=True)
    print("实际位置 = ", position_value)
    # # 读取当前电机是否使能
    # current_enable = int.from_bytes(
    #     slave.sdo_read(0x2100, 0x00,),
    #     byteorder='little',
    #     signed=False
    # )
    # if current_enable != enable:
    #     raise Exception(f'Failed to set operation mode. Expected: {enable}, Actual: {current_enable}')
    # print(f"Operation enable successfully set to: {current_enable}")
    enable = 1
    slave.sdo_write(0x2100, 0x00, int(enable).to_bytes(2, 'little'))

    target_position = 5000000# 目标位置，用户单位
    slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))
    current_position = int.from_bytes(
        slave.sdo_read(0x2400, 0x00),
        byteorder='little',
        signed=True
    )
    time.sleep(0.01)

    # slave.sdo_write(0x2607, 0x00, (0x00).to_bytes(2, 'little'))

    #读取实际位置
    # position_value = slave.sdo_read(0x6064, 0x00)
    # position_value = int.from_bytes(position_value, byteorder='little', signed=True)
    # print("实际位置 = ", position_value)
    # # 正转，写入0x00
    # slave.sdo_write(0x2607, 0x00, (0x00).to_bytes(2, 'little'))
    # if current_position != target_position:
    #     raise Exception(f'Failed to set operation mode. Expected: {target_position}, Actual: {current_position}') 
    # print(f"Operation mode successfully set to: {current_position}")
    # 电机使能1开启0关闭



    # time.sleep(1)
    # target_position = 500000# 目标位置，用户单位
    # slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))
    # current_position = int.from_bytes(
    #     slave.sdo_read(0x2400, 0x00),
    #     byteorder='little',
    #     signed=True
    # )
    # time.sleep(0.01)
    # target_position = 500000# 目标位置，用户单位
    # slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))
    # current_position = int.from_bytes(
    #     slave.sdo_read(0x2400, 0x00),
    #     byteorder='little',
    #     signed=True
    # )

    # for i in range(15):
    #     target_position = 1690633 + i*100000
    #     slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))
    #     time.sleep(0.01)
    # time.sleep(10)
    # time.sleep(10)
    # # 读取实际位置
    # position_value = slave.sdo_read(0x6064, 0x00)
    # position_value = int.from_bytes(position_value, byteorder='little', signed=True)
    # print("实际位置 = ", position_value)

    # position_value = slave.sdo_read(0x6062, 0x00)
    # position_value = int.from_bytes(position_value, byteorder='little', signed=True)
    # print("驱动器内部当前生效的目标位置指令值 = ", position_value)



    # #总线目标位置命令，与6062h作用相同
    # for i in range(2000):
    #     # 电机正反转控制，偶数正转，奇数反转
    #     if i % 2 == 0:
    #         # 正转，写入0x00
    #         slave.sdo_write(0x2607, 0x00, (0x00).to_bytes(2, 'little'))
    #     else:
    #         # 反转，写入0x01
    #         slave.sdo_write(0x2607, 0x00, (0x01).to_bytes(2, 'little'))
        
    #     target_position = 10000   # 目标位置，用户单位
    #     # 设置目标位置并发送
    #     slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))
    #     time.sleep(0.05)
    #     position_value = slave.sdo_read(0x6064, 0x00)
    #     position_value = int.from_bytes(position_value, byteorder='little', signed=True)
    #     print("实际位置 = ", position_value)


    # target_position = 308692  # 目标位置，用户单位
    # slave.sdo_write(0x2400, 0x00, int(target_position).to_bytes(4, 'little'))
    # 读取当前位置，是否正确



    # gain_position = 500
    # slave.sdo_write(0x2401, 0x00, int(gain_position).to_bytes(2, 'little'))


    # # # 电机使能1开启0关闭
    # enable = 1
    # slave.sdo_write(0x2100, 0x00, int(enable).to_bytes(2, 'little'))

    # #给定目标速度单位0.1rpm, 范围-200000 - 200000, 正负控制正反转
    # target_speed = -2000
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



    #  #电机使能1开启0关闭
    # slave.sdo_write(0x2100, 0x00, (0x0000).to_bytes(2, 'little'))

if __name__ == "__main__":
    main()
