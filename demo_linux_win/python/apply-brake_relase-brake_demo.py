from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
'''#################################################################
该DEMO 为强制抱闸和强制松闸案例,应对手臂飞车或者撞机急停后扭到一团无法上使能情况,先松闸调整,调整完毕后抱闸再切换成想要的控制模式.

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 左臂强制松闸
    7 调整完毕,左臂强制抱闸
    8 右臂强制抱闸
    9 调整完毕,右臂强制松闸
    10 任务完成,释放内存使别的程序或者用户可以连接机器人
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
if init==-1:
    logger.error('failed:端口占用，连接失败!')
    exit(0)
else:
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
robot.clear_error('B')
robot.send_cmd()
time.sleep(1)


'''左臂强制松闸'''
robot.set_param('int','BRAK0',2)
time.sleep(30) #预留时间去调整手臂的姿态


'''调整完毕,左臂强制抱闸'''
robot.set_param('int','BRAK0',1)
time.sleep(3)


'''右臂强制松闸'''
robot.set_param('int','BRAK1',2)
time.sleep(30)#预留时间去调整手臂的姿态


'''调整完毕,,右臂强制抱闸'''
robot.set_param('int','BRAK1',1)
time.sleep(3)


'''释放机器人内存'''
robot.release_robot()