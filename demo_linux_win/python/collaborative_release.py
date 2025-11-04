from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
'''#################################################################
该DEMO 为机器人进入协作释放案列(即关节不为抱死状态,有重力补偿,可以手轻松的扭/拽/拖机器人,用于紧急情况:如机器人卡死抱在一起的姿态,把手臂扭开)

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置协作释放模式
    7 复位以取消协作释放模式
    8 任务完成，释放内存使别的程序或者用户可以连接机器人
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


'''设置协作释放模式'''
robot.clear_set()
robot.set_state(arm='A',state=4)#state=4 CR
robot.send_cmd()
time.sleep(30) # 预留手拖动调整手臂构型时间


'''复位以取消协作释放模式'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=4 CR
robot.send_cmd()
time.sleep(1)

'''释放机器人内存'''
robot.release_robot()












