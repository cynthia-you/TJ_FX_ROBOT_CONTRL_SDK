from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS

'''#################################################################
该DEMO 为电机内外编码器清零和编码器清错参数案列

#注意控制器需升级到1003_34方可解锁编码器清零和清错的功能。

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 确保下始能
    7 第一轴/关节电机内编清零
    8 第一轴/关节电机外编清零
    9 第一轴/关节电机编码器清错
    10 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''初始化订阅数据的结构体'''
dcss = DCSS()

'''初始化机器人接口'''
robot = Marvin_Robot()

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

'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.set_state(arm='B',state=0)
robot.send_cmd()
time.sleep(0.5)


'''订阅数据确保下使能后第一轴/关节的电机内编值清零'''
axis=0 #第一轴清0
sub_data=robot.subscribe(dcss)
logger.info(f'A arm current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'B arm current state{sub_data["states"][1]["cur_state"]}')
if sub_data["states"][0]["cur_state"]==0 and sub_data["states"][0]["cur_state"]==0:
    robot.set_param(type='int', paraName="RESETMOTENC0", value=axis) #A arm
    robot.set_param(type='int', paraName="RESETMOTENC1", value=axis) #B arm
else:
    logger.error('failed:机器人未下使能，不能将电机编码器值清零!')
    exit(0)

time.sleep(1)

'''订阅数据确保下使能后第一轴/关节的电机外编值清零'''
axis=0 #第一轴清0
sub_data=robot.subscribe(dcss)
logger.info(f'A arm current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'B arm current state{sub_data["states"][1]["cur_state"]}')
if sub_data["states"][0]["cur_state"]==0 and sub_data["states"][0]["cur_state"]==0:
    robot.set_param(type='int', paraName="RESETEXTENC0", value=axis)#A arm
    robot.set_param(type='int', paraName="RESETEXTENC1", value=axis)#B arm
else:
    logger.error('failed:机器人未下使能，不能将电机外部编码器值清零!')
    exit(0)

time.sleep(1)


'''订阅数据确保下使能后第一轴/关节的电机编码器清错'''
axis=0 #第一轴清0
sub_data=robot.subscribe(dcss)
logger.info(f'A arm current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'B arm current state{sub_data["states"][1]["cur_state"]}')
if sub_data["states"][0]["cur_state"]==0 and sub_data["states"][0]["cur_state"]==0:
    robot.set_param(type='int', paraName="CLEARMOTENC0", value=axis)#A arm
    robot.set_param(type='int', paraName="CLEARMOTENC1", value=axis)#B arm
else:
    logger.error('failed:机器人未下使能，不能将电机外部编码器值清零!')
    exit(0)


'''释放机器人内存'''
robot.release_robot()