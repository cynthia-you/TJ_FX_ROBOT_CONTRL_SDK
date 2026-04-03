import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
import time
import logging
import threading
import queue
'''#################################################################
该DEMO 为末端模组485控制案列
使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    选择端口和左右臂
    查验连接是否成功。失败程序直接退出；成功：清CAN缓存，开启读CAN回复数据的线程
    关日志以便检查
    发送HEX数据到com1串口
    接收收到的回复
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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


'''选择模式和手臂'''
robot_id='A'
com=2 #com1

'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()

'''查验连接是否成功'''
init = robot.connect('192.168.1.190')
if init==0:
    logger.error('failed to connect to the robot, port is occupied')
    exit(0)

'''检查机械臂和伺服当前是否存错误，有错误清错'''
robot.check_error_and_clear(dcss)

'''通过确认freame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）'''
motion_tag = 0
frame_update = None
for i in range(5):
    sub_data = robot.subscribe(dcss)
    print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
    if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
        motion_tag += 1
        frame_update = sub_data['outputs'][0]['frame_serial']
    time.sleep(0.01)
if motion_tag > 0:
    logger.info('success:robot connected')
else:
    logger.error('failed:robot connection failed')
    exit(0)

'''开启日志以便检查'''
robot.log_switch('1') #全局日志开："1", 关："0"
robot.local_log_switch('1') # 主要日志开："1", 关："0"


'''发送HEX数据到com1串口'''
hex_data = "01 06 00 00 00 01 48 0A"
success, sdk_return = robot.set_485_data(robot_id,hex_data, len(hex_data), com)
logger.info(f"设置结果: {'成功' if success else '失败'}")

'''接收com1串口的HEX数据'''
tag, receive_hex_data = robot.get_485_data(robot_id, com)
if tag >= 1:
    logger.info(f"接收的HEX数据：{receive_hex_data}")


'''释放机器人内存'''
robot.release_robot()








