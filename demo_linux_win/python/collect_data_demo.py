from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级
'''#################################################################
该DEMO 为关节跟随模式下保存机器人数据案例

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 设置位置模式和速度加速度百分比
    6 机器人运动前开始设置保存数据参数并开始保存数据
    7 下发运动点位
    8 停止数据采集
    9 保存数据
    10 任务完成，下伺服 释放连接
'''#################################################################

'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()
robot.connect('192.168.1.190')
'''开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志


'''设置位置模式和速度加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.5)

'''机器人运动前开始设置保存数据'''
cols=7
idx=[0,1,2,3,4,5,6,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0]
rows=1000
robot.clear_set()
robot.collect_data(targetNum=cols,targetID=idx,recordNum=rows)
robot.send_cmd()
time.sleep(0.5)

'''运动'''
robot.clear_set()
joint_cmd_1=[0.,0.,0.,0.,0.,0.,5.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()

time.sleep(0.5)# 模拟运动时长

'''停止采集'''
robot.stop_collect_data()

'''保存采集数据'''

'''linux'''
path='aa.txt'

robot.save_collected_data_to_path(path)



robot.clear_set()
'''释放机器人内存'''
robot.release_robot()

'''下伺服'''
robot.set_state(arm='A',state=0)#state=1位置模式
robot.send_cmd()

