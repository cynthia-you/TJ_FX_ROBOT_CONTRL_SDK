
from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
'''#################################################################
该DEMO 为跑PVT轨迹并保存数据的案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 设置位置模式和速度加速度百分比
    6 订阅查看设置是否成功
    7 设置PVT 轨迹本机路径 和PVT号
    8 机器人运动前开始设置保存数据并开始采集数据
    9 设置运行的PVT号并立即执行PVT轨迹
    10 停止采集
    11 任保存采集数据
    12 任务完成，下伺服 释放连接
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


'''设置位置模式和速度'''
robot.clear_set()
robot.set_state(arm='A',state=2)#PVT， 自己的速度和加速度，不受外部控制。
robot.send_cmd()
time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'cmd state:{sub_data["states"][0]["cmd_state"]}')
logger.info(f'error code:{sub_data["states"][0]["err_code"]}')

'''设置PVT 轨迹本机路径 和PVT号'''
#linux
pvt_file='DEMO_SRS_Left.fmv'
robot.send_pvt_file('A',pvt_file, 2)
time.sleep(1)

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


'''设置运行的PVT号'''
robot.clear_set()
robot.set_pvt_id('A',2)
robot.send_cmd()
time.sleep(0.5)

time.sleep(10)#模拟跑轨迹时间


'''停止采集'''
robot.stop_collect_data()

'''保存采集数据'''

'''linux'''
path='aa.txt'
robot.save_collected_data_to_path(path)
time.sleep(0.5)


'''任务完成，下伺服 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()



