from fx_robot import Marvin_Robot
import logging
from structure_data import DCSS
import time

'''#################################################################
该DEMO 为坐标拖动控制案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 先进坐标阻抗再设置坐标拖动模式
    7 订阅查看设置是否成功
    8 拖动
    9 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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
robot.send_cmd()
time.sleep(1)

'''进拖动前先切换坐标阻抗模式'''
robot.clear_set()
robot.set_impedance_type(arm='A',type=2) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)


'''设置坐标X方向拖动'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=2)
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
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')



'''
手拖动x方向,拖动结束需要且其他坐标方向需要先退出拖动再选要进入的模式
'''


'''设置坐标y方向拖动'''
'''先退出X拖动'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=0)
robot.send_cmd()
time.sleep(0.5)

'''设置坐标Y方向拖动'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=3)
# dgType
# 0 退出拖动模式
# 1 关节空间拖动
# 2 笛卡尔空间x方向拖动
# 3 笛卡尔空间y方向拖动
# 4 笛卡尔空间z方向拖动
# 5 笛卡尔空间旋转方向拖动
robot.send_cmd()



'''
手拖动y方向
'''


'''拖动任务完成，退出拖动下使能'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=0)
robot.set_state(arm='A',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()










