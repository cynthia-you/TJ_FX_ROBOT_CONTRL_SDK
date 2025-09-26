from fx_robot import Marvin_Robot
import time
import logging

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''#################################################################
该DEMO 为末端模组485控制案列
使用逻辑
    1 初始化机器人接口
    2 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 设置位置模式和速度保障连接：听上私服声音
    6 发送数据前，先清缓存
    7 发送HEX数据到CAN
    8 每0.2秒接收CAN的HEX数据
'''#################################################################
'''初始化机器人接口'''
robot=Marvin_Robot()
robot.connect('192.168.1.190')
'''开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志
'''清错'''
robot.clear_set()
robot.clear_error('A')
robot.send_cmd()
time.sleep(1)


'''设置位置模式和速度保障连接：听上私服声音'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.5)

''' 发送数据前，先清缓存'''
time.sleep(0.5)
robot.clear_485_cache('A')
time.sleep(0.5)


'''发送HEX数据到CAN
注意看模组协议 CANID 如果是32位的发送数据要发对，比如协议CANID为0x01, 按HEX发送为：01 00 00 00 00
64位协议CANID为0x01, 按HEX发送为：01 00
'''
hex_data = "06 01 00 00 77 03 e8 03 00 00 00 00 00 00 02 02" # 0 1 CANID,  2 3 BYTE1, 4 5 BYTE2,  5 6 BYTE3 以此类推
success, sdk_return = robot.set_485_data('A',hex_data, len(hex_data), 1)
logger.info(f"设置结果: {'成功' if success else '失败'}")

while 1:
    '''接收CAN的HEX数据'''
    tag,receive_hex_data=robot.get_485_data('A',1)
    time.sleep(0.2)
    if tag>=1:
        logger.info(f"接收信号: {tag}, 接收的HEX数据：{receive_hex_data}")




