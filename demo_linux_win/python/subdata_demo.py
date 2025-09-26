from fx_robot import Marvin_Robot
from structure_data import DCSS
import time

'''#################################################################
该DEMO 为订阅机器人双臂数据的案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 订阅全部数据
'''#################################################################
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
'''订阅数据结构体'''
sub_data=robot.subscribe(dcss)
print(sub_data)


