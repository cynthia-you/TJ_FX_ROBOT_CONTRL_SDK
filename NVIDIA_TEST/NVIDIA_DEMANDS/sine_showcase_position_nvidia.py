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
import math
import ctypes
import atexit

'''#################################################################
该DEMO 为关节位置跟随控制案列

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    设置位置模式和速度加速度百分比
    订阅查看设置是否成功
    下发运动点位1
    订阅查看是否运动到位
    下发运动点位2
    订阅查看是否运动到位
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################


# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


def sleep_until(deadline, spin_threshold=0.0005):
    """Sleep until an absolute perf_counter deadline with sub-ms trim."""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > spin_threshold:
            time.sleep(remaining - spin_threshold)
        else:
            while time.perf_counter() < deadline:
                pass
            return


winmm = None
high_res_timer_enabled = False
if os.name == 'nt':
    try:
        winmm = ctypes.WinDLL('winmm')
        if winmm.timeBeginPeriod(1) == 0:
            high_res_timer_enabled = True
            logger.info('enabled Windows high-resolution timer: 1ms')
        else:
            logger.warning('failed to enable Windows high-resolution timer')
    except Exception as exc:
        logger.warning(f'failed to load winmm timer APIs: {exc}')


def _release_high_res_timer():
    if high_res_timer_enabled and winmm is not None:
        winmm.timeEndPeriod(1)


atexit.register(_release_high_res_timer)

'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()

'''查验连接是否成功'''
init = robot.connect('192.168.1.190')
if init==0:
    logger.error('failed to connect to the robot, port is occupied')
    exit(0)

'''检查机械臂和伺服当前是否存错误，有错误清错'''
robot.check_error_and_clear(dcss)

'''通过确认freame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）'''
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
    logger.info('success:robot connected')
else:
    logger.error('failed:robot connection failed')
    exit(0)

'''开启日志以便检查'''
robot.log_switch('1') #全局日志开："1", 关："0"
robot.local_log_switch('1') # 主要日志开："1", 关："0"

'''设置位置模式和速度 加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=1位置模式
robot.set_state(arm='B',state=0)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.set_vel_acc(arm='B',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(1.0)

robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_state(arm='B',state=1)#state=1位置模式
robot.send_cmd()
time.sleep(1.0)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info('-----------\nA arm:')
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][0]["err_code"]}')
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
logger.info('-----------\nB arm:')
logger.info(f'current state{sub_data["states"][1]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][1]["err_code"]}')
logger.info(f'set vel={sub_data["inputs"][1]["joint_vel_ratio"]}, acc={sub_data["inputs"][1]["joint_acc_ratio"]}')


'''点位1'''
count = 0
time_max = 60
dt = 0.01

left_default_pos = [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
right_default_pos = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]

robot.clear_set()
robot.set_joint_cmd_pose(arm='A',joints=left_default_pos)
robot.set_joint_cmd_pose(arm='B',joints=right_default_pos)
robot.send_cmd()
time.sleep(10) #预留运动时间

next_deadline = time.perf_counter()
max_count = time_max / dt

while True:
    sin_term = math.sin(2 * math.pi * count * dt * 0.5)
    left_pos = left_default_pos.copy()
    left_pos[0] += 10.0 * sin_term
    left_pos[3] += 10.0 * sin_term
    left_pos[5] += 30.0 * sin_term
    right_pos = right_default_pos.copy()
    right_pos[0] += 10.0 * sin_term
    right_pos[3] += 10.0 * sin_term
    right_pos[5] += 30.0 * sin_term

    robot.clear_set()
    robot.set_joint_cmd_pose(arm='A',joints=left_pos)
    robot.set_joint_cmd_pose(arm='B',joints=right_pos)
    robot.send_cmd()

    '''订阅数据查看是否到位'''
    sub_data=robot.subscribe(dcss)
    logger.info('-----------\nA arm:')
    logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
    logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')
    logger.info('-----------\nB arm:')
    logger.info(f'set joint={sub_data["inputs"][1]["joint_cmd_pos"]}')
    logger.info(f'current joint={sub_data["outputs"][1]["fb_joint_pos"]}')

    count += 1
    if count > max_count:
        break
    next_deadline += dt
    now = time.perf_counter()
    if next_deadline < now:
        next_deadline = now
    sleep_until(next_deadline)


'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.set_state(arm='B',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()









