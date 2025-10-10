from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS

'''#################################################################
该DEMO 为获取,参数案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 设置位置模式和速度加速度百分比
    6 获取整形参数
    7 获取浮点形参数
    8 任务完成，下伺服 释放连接
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
robot.connect('192.168.1.190')
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

'''read'''
int_param1=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param1}')

float_param1=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param1}')


'''任务完成，下伺服 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()