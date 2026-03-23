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
该DEMO 为位置模式下，使用规划点位执行消除指令/通讯抖动问题

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    设置速度加速度百分比，位置模式，并订阅查看设置是否成功
    直接指令走到零位
    完成YZ平面的矩形框
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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
    time.sleep(0.01)
if motion_tag > 0:
    logger.info('success:robot connected')
else:
    logger.error('failed:robot connection failed')
    exit(0)

'''开启日志以便检查'''
robot.log_switch('1') #全局日志开："1", 关："0"
robot.local_log_switch('1') # 主要日志开："1", 关："0"



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



'''设置初始位置'''
initial_pos=[44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1]
robot.clear_set()
robot.set_joint_cmd_pose(arm='A',joints=initial_pos)
timeout = robot.send_cmd_wait_response(100)
logger.info(f'set joint cmd, 100ms 内测到的执行的响应延迟是 ：{timeout} ms')
time.sleep(3)



'''实列化计算'''
kk = Marvin_Kine()
'''关闭日志'''
kk.log_switch(0)  # 0 off, 1 on
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result = kk.load_config(arm_type=0, config_path=os.path.join(current_path, 'ccs_m6_40.MvKDCfg'))
print(ini_result)

'''
初始化动力学
'''
initial_kine_tag = kk.initial_kine(
    robot_type=ini_result['TYPE'][0],
    dh=ini_result['DH'][0],
    pnva=ini_result['PNVA'][0],
    j67=ini_result['BD'][0])

'''
    将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    起点的末端位置姿态矩阵转为XYZABC
    定义直线结束点的XYZABC，showcase是规划了末端Z方向移动100毫米
'''
fk_mat = kk.fk(joints=initial_pos)
if fk_mat:
    print(f'fk matrix:{fk_mat}')

pose_6d_start = kk.mat4x4_to_xyzabc(pose_mat=fk_mat)
print(f'pose_6d_start:{pose_6d_start}')


'''矩形框yz平面'''

'''z+200'''
pose_6d_end = pose_6d_start.copy()
pose_6d_end[2] += 200  # Z方向移动200mm
'''直线规划（MOVLA）'''
points,pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                  ref_joints=initial_pos, vel=1000, acc=1000,freq_hz=50)
if pset:
    '''规划下发并执行'''
    robot.setPln_Cart(arm='A',pset=pset)

'''等待规划轨迹执行结束'''
while True:
    data = robot.subscribe(dcss)
    time.sleep(0.001)
    if data['outputs'][0]['traj_state'] ==  b'\x00':
        break

'''y-200'''
pose_6d_start = pose_6d_end.copy()
pose_6d_end = pose_6d_start.copy()
pose_6d_end[1] -= 200  # Y方向移动200mm
'''直线规划（MOVLA）'''
points,pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                  ref_joints=initial_pos, vel=1000, acc=1000,freq_hz=50)
if pset:
    '''规划下发并执行'''
    robot.setPln_Cart(arm='A',pset=pset)
'''等待规划轨迹执行结束'''
while True:
    data = robot.subscribe(dcss)
    time.sleep(0.001)
    if data['outputs'][0]['traj_state'] ==  b'\x00':
        break


'''z-200'''
pose_6d_start = pose_6d_end.copy()
pose_6d_end = pose_6d_start.copy()
pose_6d_end[2] -= 200  # z方向移动200mm
'''直线规划（MOVLA）'''
points,pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                  ref_joints=initial_pos, vel=1000, acc=1000,freq_hz=50)
if pset:
    '''规划下发并执行'''
    robot.setPln_Cart(arm='A',pset=pset)
'''等待规划轨迹执行结束'''
while True:
    data = robot.subscribe(dcss)
    time.sleep(0.001)
    if data['outputs'][0]['traj_state'] ==  b'\x00':
        break


'''y+200'''
pose_6d_start = pose_6d_end.copy()
pose_6d_end = pose_6d_start.copy()
pose_6d_end[1] += 200  # Y方向移动200mm
'''直线规划（MOVLA）'''
points,pset = kk.movLA(start_xyzabc=pose_6d_start, end_xyzabc=pose_6d_end,
                  ref_joints=initial_pos, vel=1000, acc=1000,freq_hz=50)
if pset:
    '''规划下发并执行'''
    robot.setPln_Cart(arm='A',pset=pset)
'''等待规划轨迹执行结束'''
while True:
    data = robot.subscribe(dcss)
    time.sleep(0.001)
    if data['outputs'][0]['traj_state'] ==  b'\x00':
        break

'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()
time.sleep(0.5)

'''释放机器人内存'''
robot.release_robot()









