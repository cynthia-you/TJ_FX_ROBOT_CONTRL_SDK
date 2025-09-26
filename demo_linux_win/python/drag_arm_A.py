from fx_robot import Marvin_Robot
import logging
from structure_data import DCSS
import time

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


'''#################################################################
该DEMO 为拖动控制案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 开启日志以便检查
    4 为了防止伺服有错，先清错
    5 设置拖动模式和参数
    6 订阅查看设置是否成功
    7 拖动
    8 任务完成，下伺服 释放连接
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



'''设置拖动模式和参数'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=1)
# dgType
# 0 退出拖动模式
# 1 关节空间拖动
# 2 笛卡尔空间x方向拖动
# 3 笛卡尔空间y方向拖动
# 4 笛卡尔空间z方向拖动
# 5 笛卡尔空间旋转方向拖动
robot.send_cmd()

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)

logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'cmd state:{sub_data["states"][0]["cmd_state"]}')
logger.info(f'error code:{sub_data["states"][0]["err_code"]}')

logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
logger.info(f'set drag space type={sub_data["inputs"][0]["drag_sp_type"]}')



#手拖动#
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')


'''拖动任务完成，下伺服 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()









