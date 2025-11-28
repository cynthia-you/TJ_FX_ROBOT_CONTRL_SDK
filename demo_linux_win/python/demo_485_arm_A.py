from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
import threading
import queue
'''#################################################################
该DEMO 为末端模组485控制案列
使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 选择端口和左右臂
    4 查验连接是否成功。失败程序直接退出；成功：清CAN缓存，开启读CAN回复数据的线程
    5 关日志以便检查
    6 发送HEX数据到com1串口
    7 接收线程收到的回复
    8 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''创建队列'''
data_queue = queue.Queue()

def read_data(robot_id,com):
    '''接收CAN的HEX数据'''
    while True:
        try:
            tag, receive_hex_data = robot.get_485_data(robot_id, com)
            if tag >= 1:
                logger.info(f"接收的HEX数据：{receive_hex_data}")
                data_queue.put(receive_hex_data)
            else:
                time.sleep(0.001)
        except Exception as e:
            logger.error(f"读取数据错误: {e}")
            time.sleep(0.001)


def get_received_data():
    '''获取接收到的数据并计数'''
    received_count = 0
    received_data_list = []

    while True:
        try:
            data = data_queue.get_nowait()
            received_count += 1
            received_data_list.append(data)
        except queue.Empty:
            break

    return received_count, received_data_list

'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()

'''选择模式和手臂'''
robot_id='A'
com=2 #com1


'''查验连接是否成功'''
init = robot.connect('192.168.1.190')
if init==0:
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
        ''' 发送数据前，先清缓存'''
        time.sleep(0.5)
        robot.clear_485_cache(robot_id)
        time.sleep(0.5)
        thread = threading.Thread(target=read_data, args=(robot_id,com),daemon=True)
        thread.start()
        logger.info('读CAN回复线程开启')
    else:
        logger.error('failed:机器人连接失败!')
        exit(0)

'''关日志'''
robot.log_switch('0') #全局日志开关
robot.local_log_switch('0') # 主要日志


'''发送HEX数据到com1串口'''
hex_data = "01 06 00 00 00 01 48 0A"
success, sdk_return = robot.set_485_data(robot_id,hex_data, len(hex_data), com)
logger.info(f"设置结果: {'成功' if success else '失败'}")

'''接收com1串口的HEX数据'''
received_count, received_data = get_received_data()
if received_count>0:
    print(f'thread接收的数据信息， 帧数：{received_count},  接收的数据:\n{received_data}')


'''释放机器人内存'''
robot.release_robot()








