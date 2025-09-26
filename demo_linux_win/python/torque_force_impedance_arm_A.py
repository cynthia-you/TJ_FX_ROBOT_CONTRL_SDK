from fx_robot import Marvin_Robot
import time
from structure_data import DCSS
import math
import logging
'''#################################################################
该DEMO 为力控案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 设置扭矩模式和速度加速度百分比
    6 设置阻抗参数
    7 选择阻抗模式
    8 订阅数据查看是否设置
    9 下发运动点位1
    10 订阅查看是否运动到位
    11 任务完成，下伺服 释放连接
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


'''设置扭矩模式  速度加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.5)

'''阻抗参数'''
robot.clear_set()

# 这两条指令搭配使用才有力控的效果
# 设置是在Y轴方向有个2斤的力一直拽着手臂提起5厘米， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=5.)
robot.set_force_cmd(arm='A',f=10)
time.sleep(0.5)

'''选择阻抗模式'''
robot.clear_set()
robot.set_impedance_type(arm='A',type=3) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f"current state{sub_data['states'][0]['cur_state']}")
logger.info(f"cmd state:{sub_data['states'][0]['cmd_state']}")
logger.info(f"error code:{sub_data['states'][0]['err_code']}")
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
# logger.info(f'set card k={sub_data["inputs"][0]["cart_k"][:]}, d={sub_data["inputs"][0]["cart_k"][:]}')
# logger.info(f'set joint k={sub_data["inputs"][0]["joint_k"][:]}, d={sub_data["inputs"][0]["joint_d"][:]}')
logger.info(f'set force fcType={sub_data["inputs"][0]["force_type"]}, '
             f'fxDirection={sub_data["inputs"][0]["force_dir"][:]}, '
             f'fcCtrlpara={sub_data["inputs"][0]["force_pidul"][:]}, '
             f'fcAdjLmt={sub_data["inputs"][0]["force_adj_lmt"]}')
logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')

'''点位1'''
robot.clear_set()
joint_cmd_1=[10.,20.,30.,40.,50.,60.,70.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()

time.sleep(5)
'''订阅数据查看是否到位'''
sub_data=robot.subscribe(dcss)
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')


'''任务完成，下伺服 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()









