from fx_robot import Marvin_Robot
from structure_data import DCSS
import time
import logging
import threading
import queue

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''#################################################################
该DEMO 为末端模组CANFD控制案列
使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出,成功清CAN缓存，开启读CAN回复数据的线程
    4 发送HEX数据到CAN
    5 任务完成释放内存使别的程序或者用户可以连接机器人
'''#################################################################
# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''创建队列'''
data_queue = queue.Queue()

def read_data():
    '''接收CAN的HEX数据'''
    while True:
        try:
            tag, receive_hex_data = robot.get_485_data('A', 1)
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
        robot.clear_485_cache('A')
        time.sleep(0.2)

        thread = threading.Thread(target=read_data, daemon=True)
        thread.start()
        logger.info('读CAN回复线程开启')
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


'''发送HEX数据到CAN
注意看模组协议 CANID 如果是32位的发送数据要发对，比如协议CANID为0x01, 按HEX发送为：01 00 00 00 00
64位协议CANID为0x01, 按HEX发送为：01 00
'''
hex_data = "06 01 00 00 77 03 e8 03 00 00 00 00 00 00 02 02" # 0 1 CANID,  2 3 BYTE1, 4 5 BYTE2,  5 6 BYTE3 以此类推
success, sdk_return = robot.set_485_data('A',hex_data, len(hex_data), 1)
logger.info(f"设置结果: {'成功' if success else '失败'}")
received_count, received_data = get_received_data()
print(f'thread接收的数据信息， 帧数：{received_count},  接收的数据:\n{received_data}')


'''释放机器人内存'''
robot.release_robot()





