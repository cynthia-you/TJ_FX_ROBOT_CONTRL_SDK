import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara, convert_to_8x8_matrix
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS,check_joints_accuracy_with_tolerance
import time
import logging

'''#################################################################
该DEMO 为双臂协作笛卡尔空间直线规划同步运动演示

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    设置双臂速度加速度百分比，位置模式
    双臂走到初始位置
    实例化双臂计算接口，分别加载左臂和右臂配置
    使用movLA分别对双臂进行笛卡尔空间直线规划
    通过setPln_Cart_AB同时下发双臂规划点位
    等待双臂规划轨迹执行结束
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人

    说明：movLA根据提供的起始目标笛卡尔位姿和终止目标笛卡尔位姿规划一段直线路径点，
         该接口不约束到达终点时的机器人构型。
         需要提供参考关节角度（ref_joints）作为逆解参考。
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

ref_joints_A = [19.597, -32.480, 10.050, -58.939, -8.863, -33.821, 4.772]
ref_joints_B = [-19.597, -32.480, -10.050, -58.939, 8.863, -33.821, -4.772]

robot.clear_set()
robot.set_joint_cmd_pose(arm='A', joints=ref_joints_A)
robot.set_joint_cmd_pose(arm='B', joints=ref_joints_B)
robot.send_cmd()
logger.info(f'set A&B arm initial pos')
i=0
while i<1000:
    i+=0
    time.sleep(0.2)
    data = robot.subscribe(dcss)
    if check_joints_accuracy_with_tolerance(data['outputs'][0]['fb_joint_pos'], ref_joints_A) and check_joints_accuracy_with_tolerance(data['outputs'][1]['fb_joint_pos'], ref_joints_B):
        break

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

start_xyzabc_A = [509.734, 233.609, 365.948, -169.144, 55.011, -146.752]
end_xyzabc_A = [509.734, 233.609, 265.948, -169.144, 55.011, -146.752]
start_xyzabc_B = [509.734, -233.609, 365.948, 169.144, 55.011, 146.752]
end_xyzabc_B = [509.734, -233.609, 265.948, 169.144, 55.011, 146.752]

vel = 100
acc = 100
freq = 50

points_A, pset1 = kk1.movLA(start_xyzabc_A, end_xyzabc_A, ref_joints_A, vel, acc, freq)
if pset1 is None:
    logger.error('Left arm planning failed.')
    exit(-1)
logger.info(f'Left arm planning success, got {len(points_A)} points')

points_B, pset2 = kk2.movLA(start_xyzabc_B, end_xyzabc_B, ref_joints_B, vel, acc, freq)
if pset2 is None:
    logger.error('Right arm planning failed.')
    exit(-1)
logger.info(f'Right arm planning success, got {len(points_B)} points')

ret = robot.setPln_Cart_AB(pset1, pset2)
if not ret:
    logger.error('Set planning of left arm and right arm failed, please check data')
    exit(-1)
logger.info('Dual-arm movLA planning started')

i=0
while i<1000:
    i+=1
    time.sleep(0.2)
    data = robot.subscribe(dcss)
    if data['outputs'][0]['traj_state'] == b'\x00' and data['outputs'][1]['traj_state'] == b'\x00':
        break
logger.info('Dual-arm movLA planning completed')

time.sleep(0.5)
robot.clear_set()
robot.set_state(arm='A', state=0)
robot.set_state(arm='B', state=0)
robot.send_cmd()
time.sleep(0.5)

robot.release_robot()
