import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS,check_joints_accuracy_with_tolerance
import time
import logging

'''#################################################################
该DEMO 为双臂协作关节空间同步规划运动演示

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    设置双臂速度加速度百分比，位置模式
    双臂走到初始位置
    初始化规划器
    双臂同时执行关节空间规划运动（setPln_joint_AB）
    等待双臂规划轨迹执行结束
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''################################################################


logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)
logger.setLevel(logging.DEBUG)

dcss = DCSS()

robot = Marvin_Robot()

init = robot.connect('192.168.1.190')
if init == 0:
    logger.error('failed to connect to the robot, port is occupied')
    exit(0)

robot.check_error_and_clear(dcss)

motion_tag = 0
frame_update = None
for i in range(5):
    sub_data = robot.subscribe(dcss)
    print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
    if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
        motion_tag += 1
        frame_update = sub_data['outputs'][0]['frame_serial']
    time.sleep(0.01)
if motion_tag > 0:
    logger.info('success:robot connected')
else:
    logger.error('failed:robot connection failed')
    exit(0)

robot.log_switch('1')
robot.local_log_switch('1')

robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=100, AccRatio=100)
robot.set_vel_acc(arm='B', velRatio=100, AccRatio=100)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set A&B arm vel&acc, 100ms timeout: {timeout} ms')

robot.clear_set()
robot.set_state(arm='A', state=1)
robot.set_state(arm='B', state=1)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set A&B arm position mode, 100ms timeout: {timeout} ms')

time.sleep(0.2)
sub_data = robot.subscribe(dcss)
logger.info('-----------\nA arm:')
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][0]["err_code"]}')
logger.info('-----------\nB arm:')
logger.info(f'current state{sub_data["states"][1]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][1]["err_code"]}')

start_joints_A = [0]*7
start_joints_B = [0]*7

robot.clear_set()
robot.set_joint_cmd_pose(arm='A', joints=start_joints_A)
robot.set_joint_cmd_pose(arm='B', joints=start_joints_B)
robot.send_cmd()
logger.info(f'set A&B arm initial pos')

i=0
while i<1000:
    i+=0
    time.sleep(0.2)
    data = robot.subscribe(dcss)
    if check_joints_accuracy_with_tolerance(data['outputs'][0]['fb_joint_pos'], start_joints_A) and check_joints_accuracy_with_tolerance(data['outputs'][1]['fb_joint_pos'], start_joints_B):
        break

_cfg_files=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg')
if not os.path.exists(_cfg_files):
    print("Failed!", f"no {_cfg_files} files found.")
    exit(-1)
ret = robot.pln_init(config_path=_cfg_files)
if not ret:
    logger.error('load calculate config failed')
    exit(-1)
else:
    logger.info('load cfg success')

vel_ratio = 0.2
acc_ratio = 0.2

target_joints_A = [17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134]
target_joints_B = [-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134]

i=0
while i<1000:
    i+=1
    time.sleep(0.001)
    data = robot.subscribe(dcss)
    if data['outputs'][0]['traj_state'] == b'\x00' and data['outputs'][1]['traj_state'] == b'\x00':
        break

ret = robot.setPln_joint_AB(start_joints_A, target_joints_A,
                             start_joints_B, target_joints_B,
                             vel_ratio, acc_ratio)
if not ret:
    logger.error('Planning of left arm and right arm failed, please check data')
    exit(-1)
logger.info('Dual-arm joint planning started')

i=0
while i<1000:
    i+=1
    time.sleep(0.2)
    data = robot.subscribe(dcss)
    if data['outputs'][0]['traj_state'] == b'\x00' and data['outputs'][1]['traj_state'] == b'\x00':
        break
logger.info('Dual-arm joint planning completed')

time.sleep(0.5)
robot.clear_set()
robot.set_state(arm='A', state=0)
robot.set_state(arm='B', state=0)
robot.send_cmd()
time.sleep(0.5)

robot.release_robot()
