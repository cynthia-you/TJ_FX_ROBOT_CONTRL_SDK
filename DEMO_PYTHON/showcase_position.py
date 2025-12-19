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

'''#################################################################
该DEMO 为关节位置跟随控制案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置位置模式和速度加速度百分比
    7 订阅查看设置是否成功
    8 下发运动点位1
    9 订阅查看是否运动到位
    10 下发运动点位2
    11 订阅查看是否运动到位
    12 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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
    for i in range(10):
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
robot.clear_error('B')
robot.send_cmd()
time.sleep(1)


'''设置位置模式和速度 加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.set_state(arm='B',state=1)#state=1位置模式
robot.set_vel_acc(arm='B',velRatio=10, AccRatio=10)
robot.send_cmd()


'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info('-----------\nA arm:')
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][0]["err_code"]}')
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
logger.info('-----------\nB arm:')
logger.info(f'current state{sub_data["states"][1]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][1]["err_code"]}')
logger.info(f'set vel={sub_data["inputs"][1]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')


'''点位1'''
robot.clear_set()
joint_cmd_1=[0.,0.,0.,0.,0.,0.,5.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.set_joint_cmd_pose(arm='B',joints=joint_cmd_1)
robot.send_cmd()
time.sleep(3) #预留运动时间


'''订阅数据查看是否到位'''
sub_data=robot.subscribe(dcss)
logger.info('-----------\nA arm:')
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')
logger.info('-----------\nB arm:')
logger.info(f'set joint={sub_data["inputs"][1]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][1]["fb_joint_pos"]}')

'''点位2'''
robot.clear_set()
joint_cmd_2=[0.,0.,0.,0.,0.,10.,50.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_2)
robot.set_joint_cmd_pose(arm='B',joints=joint_cmd_2)
robot.send_cmd()
time.sleep(3) #预留运动时间


'''订阅数据查看是否到位'''
sub_data=robot.subscribe(dcss)
logger.info('-----------\nA arm:')
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')
logger.info('-----------\nB arm:')
logger.info(f'set joint={sub_data["inputs"][1]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][1]["fb_joint_pos"]}')


'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.set_state(arm='B',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()









