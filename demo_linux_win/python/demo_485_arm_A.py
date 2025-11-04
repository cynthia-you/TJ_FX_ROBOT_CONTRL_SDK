from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
'''#################################################################
该DEMO 为末端模组485控制案列
使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置位置模式和速度保障连接：听上始能声音
    7 发送数据前，先清缓存
    8 发送HEX数据到com1串口
    9 每0.2秒接收com1串口的HEX数据
    10 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()


'''查验连接是否成功'''
init = robot.connect('192.168.1.190')
if init==-1:
    logger.error('failed:端口占用，连接失败!')
    exit(0)
else:
    '''防总线通信异常,先清错'''
    time.sleep(0.5)
    robot.clear_set()
    robot.clear_error('A')
    robot.clear_error('B')
    robot.send_cmd()
    time.sleep(0.5)

    motion_tag = 0
    frame_update = None
    for i in range(5):
        sub_data = robot.subscribe(dcss)
        print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
        if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
            motion_tag += 1
            frame_update = sub_data['outputs'][0]['frame_serial']
        time.sleep(0.1)
    if motion_tag > 0:
        logger.info('success:机器人连接成功!')
    else:
        logger.error('failed:机器人连接失败!')
        exit(0)

'''开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志

'''清错'''
robot.clear_set()
robot.clear_error('A')
robot.send_cmd()
time.sleep(1)


'''设置位置模式和速度保障连接：听上始能声音'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.5)

''' 发送数据前，先清缓存'''
time.sleep(0.5)
robot.clear_485_cache('A')
time.sleep(0.5)

'''发送HEX数据到com1串口'''
hex_data = "01 06 00 00 00 01 48 0A"
success, sdk_return = robot.set_485_data('A',hex_data, len(hex_data), 2)
logger.info(f"设置结果: {'成功' if success else '失败'}")

'''接收com1串口的HEX数据'''
i=0
while 1:
    tag,receive_hex_data=robot.get_485_data('A',2)
    time.sleep(0.2)
    if tag >= 1:
        logger.info(f"接收信号: {tag}, 接收的HEX数据：{receive_hex_data}")
        break
    i+=1
    if i>100:
        break

'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=1位置模式
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()








