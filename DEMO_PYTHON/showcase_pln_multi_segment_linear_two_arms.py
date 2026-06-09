import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara, convert_to_8x8_matrix
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
import time
import logging

'''#################################################################
该DEMO 为双臂协作多点直线规划同步运动演示

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    设置双臂速度加速度百分比，位置模式
    双臂走到初始位置
    实例化双臂计算接口，分别加载左臂和右臂配置
    使用multi_movL_set_start + multi_movL_next_point + multi_movL_get_points
    分别对双臂进行多点直线规划
    通过setPln_Cart_AB同时下发双臂规划点位
    等待双臂规划轨迹执行结束
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人

    说明：多点直线规划允许设置多个途经点，机器人将依次经过各途经点，
         每段之间自动进行过渡规划。适用于需要走矩形、折线等复杂路径的场景。
         zsp_type和zsp_params用于控制臂角（第7轴）的调整策略。
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

start_joints_A = [17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086]
start_joints_B = [-17.970, -35.197, -11.414, -73.344, 9.154, -17.035, -7.086]

robot.clear_set()
robot.set_joint_cmd_pose(arm='A', joints=start_joints_A)
robot.set_joint_cmd_pose(arm='B', joints=start_joints_B)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set A&B arm initial pos, 100ms timeout: {timeout} ms')
time.sleep(3)

kk1 = Marvin_Kine()
kk2 = Marvin_Kine()

kk1.log_switch(0)
kk2.log_switch(0)

ini_result1 = kk1.load_config(arm_type=0, config_path=os.path.join(current_path, 'ccs_m6_40.MvKDCfg'))
ini_result2 = kk2.load_config(arm_type=1, config_path=os.path.join(current_path, 'ccs_m6_40.MvKDCfg'))

initial_kine_tag1 = kk1.initial_kine(
    robot_type=ini_result1['TYPE'][0],
    dh=ini_result1['DH'][0],
    pnva=ini_result1['PNVA'][0],
    j67=ini_result1['BD'][0])

initial_kine_tag2 = kk2.initial_kine(
    robot_type=ini_result1['TYPE'][1],
    dh=ini_result1['DH'][1],
    pnva=ini_result1['PNVA'][1],
    j67=ini_result1['BD'][1])

points_arm0 = [
    [509.731, 233.614, 265.949, -169.144, 55.011, -146.752],
    [509.731, 233.614, 65.949, -169.144, 55.011, -146.752],
    [509.731, 33.614, 65.949, -169.144, 55.011, -146.752],
    [509.731, 33.614, 265.949, -169.144, 55.011, -146.752]
]

points_arm1 = [
    [509.731, -233.614, 265.949, 169.144, 55.011, 146.752],
    [509.731, -233.614, 65.949, 169.144, 55.011, 146.752],
    [509.731, -33.614, 65.949, 169.144, 55.011, 146.752],
    [509.731, -33.614, 265.949, 169.144, 55.011, 146.752]
]

allow_range = 5
zsp_type = 1
zsp_params = [0, 0, -1, 0, 0, 0]
vel = 100
acc = 100
freq = 50

tag_multi_start = kk1.multi_movL_set_start(start_joints_A, points_arm0[0], points_arm0[1],
                                             allow_range, zsp_type, zsp_params, vel, acc, freq)
if not tag_multi_start:
    logger.error('Left arm multi-segment planning: set start failed')
    exit(-1)

for next_one in points_arm0[2:]:
    ret1 = kk1.multi_movL_next_point(next_one, allow_range, zsp_type, zsp_params, vel, acc)
    if not ret1:
        logger.error('Left arm multi-segment planning: set next failed')
        exit(-1)

data_A, pset1 = kk1.multi_movL_get_points()
if pset1 is None:
    logger.error('Left arm planning failed.')
    exit(-1)
logger.info(f'Left arm multi-segment planning success, got {len(data_A)} points')

tag_multi_start = kk2.multi_movL_set_start(start_joints_B, points_arm1[0], points_arm1[1],
                                             allow_range, zsp_type, zsp_params, vel, acc, freq)
if not tag_multi_start:
    logger.error('Right arm multi-segment planning: set start failed')
    exit(-1)

for next_one in points_arm1[2:]:
    ret1 = kk2.multi_movL_next_point(next_one, allow_range, zsp_type, zsp_params, vel, acc)
    if not ret1:
        logger.error('Right arm multi-segment planning: set next failed')
        exit(-1)

data_B, pset2 = kk2.multi_movL_get_points()
if pset2 is None:
    logger.error('Right arm planning failed.')
    exit(-1)
logger.info(f'Right arm multi-segment planning success, got {len(data_B)} points')

ret = robot.setPln_Cart_AB(pset1, pset2)
if not ret:
    logger.error('Set planning of left arm and right arm failed, please check data')
    exit(-1)
logger.info('Dual-arm multi-segment planning started')

i=0
while i<1000:
    i+=1
    time.sleep(0.001)
    data = robot.subscribe(dcss)
    if data['outputs'][0]['traj_state'] == b'\x00' and data['outputs'][1]['traj_state'] == b'\x00':
        break
logger.info('Dual-arm multi-segment planning completed')

time.sleep(0.5)
robot.clear_set()
robot.set_state(arm='A', state=0)
robot.set_state(arm='B', state=0)
robot.send_cmd()
time.sleep(0.5)

robot.release_robot()
