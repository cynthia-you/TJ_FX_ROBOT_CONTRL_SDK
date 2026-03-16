import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
import time
import logging

'''#################################################################
该DEMO 为关节位置下使用规划点位执行消除指令/通讯抖动,中断规划的案例

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置速度加速度百分比，位置模式，并订阅查看设置是否成功
    7 直接指令走到零位
    8 完成多次次循环：规划点位下发并中断
    9 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''################################################################


# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


def check_velocity_stopped(fb_vel, threshold=0.5):
    if fb_vel is None or len(fb_vel) != 7:
        return False
    for i, vel in enumerate(fb_vel):
        if abs(vel) >= threshold:
            logger.debug(f'Joint {i + 1} velocity {vel} exceeds threshold {threshold}')
            return False
    return True


def check_joints_accuracy_with_tolerance(joint1, joint2, tolerance=0.01):
    if not (joint1 and joint2 and len(joint1) == 7 and len(joint2) == 7):
        return False
    return all(abs(j1 - j2) < tolerance for j1, j2 in zip(joint1, joint2))

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



'''设置速度 加速度百分比'''
robot.clear_set()
robot.set_vel_acc(arm='A',velRatio=100, AccRatio=100)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set vel&acc, 100ms 内测到的执行的响应延迟是 ：{timeout} ms')

'''设置位置模式'''
robot.clear_set()
robot.set_state(arm='A',state=1)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set position mode, 100ms 内测到的执行的响应延迟是 ：{timeout} ms')


'''订阅数据查看是否设置'''
time.sleep(0.2)
sub_data=robot.subscribe(dcss)
logger.info('-----------\nA arm:')
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][0]["err_code"]}')
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')



'''设置循环的规划点位'''
joint_cmd_1 = [69.02, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]
joint_cmd_2 = [9.22, -40.58, -43.89, -102.09, 128.44, 17.55, -28.35]


'''设置初始位置'''
initial_pos=joint_cmd_1
robot.clear_set()
robot.set_joint_cmd_pose(arm='A',joints=initial_pos)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set joint cmd, 100ms 内测到的执行的响应延迟是 ：{timeout} ms')
time.sleep(3)


'''初始化规划器'''
ret=robot.pln_init(config_path='ccs_m6_40.MvKDCfg')
if not ret:
    logger.info('load config failed')
else:
    logger.info(f'load cfg success')

'''定义规划器的速度和加速度比例：范围0~1.'''
vel_ratio=0.2
acc_ratio=0.2


'''多次次循环：用规划关节'''
for i in range(10):
    logger.info(f'iter:{i}')
    '''等待控制器无规划轨迹信号'''
    while True:
        data = robot.subscribe(dcss)
        time.sleep(0.001)
        if data['outputs'][0]['traj_state'] ==  b'\x00':
            break

    pln_re=robot.setPln_joint(arm='A',start_joints=joint_cmd_1,target_joints=joint_cmd_2,velRatio=vel_ratio,accRatio=acc_ratio)
    if not pln_re:
        logger.info('pln fail')
        exit(-1)
    time.sleep(0.2)

    '''规划中断'''
    robot.stopRunPln_joint(arm='A')
    time.sleep(0.2)

    data = robot.subscribe(dcss)
    logger.info(f'break at:{data["outputs"][0]["fb_joint_pos"]}')


'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()
time.sleep(0.5)

'''释放机器人内存'''
robot.release_robot()









