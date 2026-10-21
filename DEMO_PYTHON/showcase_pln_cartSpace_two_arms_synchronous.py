import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine, ArmsSynchronousPlanningParams
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS, check_joints_accuracy_with_tolerance
import time
import logging

'''#################################################################
该DEMO 为双臂同步直线规划并在位置模式下执行演示
对应 C++ 示例: DEMO_C++/showcase_pln_cartSpace_two_arms_synchronous.cpp

使用逻辑
    阶段一：连接与双臂状态检查
        初始化订阅数据的结构体
        初始化机器人接口, 查验连接是否成功, 失败程序直接退出
        读取双臂状态并清除机械臂错误/伺服错误
        检查帧序号更新, 确认 UDP 数据通道正常
        开启日志以便检查
    阶段二：双臂控制模式配置与运动学初始化
        设置双臂速度加速度百分比，位置模式
        加载机型配置并初始化双臂运动学计算接口
        双臂走到初始位置（左右臂镜像对称）
        通过正运动学计算双臂起点末端位姿并转换为 XYZABC
    阶段三：双臂同步直线规划
        填写双臂同步规划参数结构体 ArmsSynchronousPlanningParams
        调用 movL_synchronous 生成双臂同步轨迹
    阶段四：轨迹执行与资源释放
        等待双臂上一段轨迹结束
        通过 setPln_Cart_AB 同时下发双臂规划点位并开始运动
        等待双臂规划轨迹执行结束
        任务完成, 下使能, 释放连接

    说明：
    1. movL_synchronously 以较长一侧手臂的路径长度为主轴，
       双臂按相同的时间比例同步运动，运动过程中躯干（机体）保持固定。
    2. World_Co_Flag 为 False 时，双臂位姿分别基于各自机械臂基坐标系给定；
       为 True 时，双臂位姿基于机体坐标系给定，需同时提供 Skye_Body_Start_PRR。
    3. 本示例的 libKine 库需为包含 FX_Robot_PLN_MOVL_Arms_Synchronous 接口的
       新编译版本，详见 readme.md 中 2.1 编译方法。
'''################################################################


logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)
logger.setLevel(logging.DEBUG)

dcss = DCSS()

robot = Marvin_Robot()

# [阶段一｜步骤 1] 初始化状态数据结构并连接机器人
init = robot.connect('192.168.1.190')
if init == 0:
    logger.error('failed to connect to the robot, port is occupied')
    exit(0)

# [阶段一｜步骤 2] 读取双臂状态并清除机械臂错误/伺服错误
robot.check_error_and_clear(dcss)

# [阶段一｜步骤 3] 检查帧序号更新，确认 UDP 数据通道正常
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
    robot.release_robot()
    exit(0)

# [阶段一｜步骤 4] 开启控制日志
robot.log_switch('1')
robot.local_log_switch('1')

# [阶段二｜步骤 5] 设置双臂速度、加速度百分比并切换为位置模式
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=10, AccRatio=10)
robot.set_vel_acc(arm='B', velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.1)

robot.clear_set()
robot.set_state(arm='A', state=1)
robot.set_state(arm='B', state=1)
robot.send_cmd()
time.sleep(1)

# [阶段二｜步骤 6] 加载机型配置并初始化双臂运动学计算接口
kk1 = Marvin_Kine()
kk2 = Marvin_Kine()

kk1.log_switch(0)
kk2.log_switch(0)

'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
    ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
    ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
    srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错
一定要确认arm_type是左臂0 还是右臂1
'''

ini_result1 = kk1.load_config(arm_type=0, config_path=os.path.join(parent_dir, 'CommonConfig/ccs_m6_40.MvKDCfg'))
ini_result2 = kk2.load_config(arm_type=1, config_path=os.path.join(parent_dir, 'CommonConfig/ccs_m6_40.MvKDCfg'))
if ini_result1 is None or ini_result2 is None:
    logger.error('Load CFG Error')
    robot.release_robot()
    exit(-1)

initial_kine_tag1 = kk1.initial_kine(
    robot_type=ini_result1['TYPE'][0],
    dh=ini_result1['DH'][0],
    pnva=ini_result1['PNVA'][0],
    j67=ini_result1['BD'][0])
if not initial_kine_tag1:
    logger.error('Robot Init Error of arm A')
    robot.release_robot()
    exit(-1)

initial_kine_tag2 = kk2.initial_kine(
    robot_type=ini_result1['TYPE'][1],
    dh=ini_result1['DH'][1],
    pnva=ini_result1['PNVA'][1],
    j67=ini_result1['BD'][1])
if not initial_kine_tag2:
    logger.error('Robot Init Error of arm B')
    robot.release_robot()
    exit(-1)

# [阶段二｜步骤 7] 下发初始关节角，使双臂运动至规划起点（左右臂镜像对称）
joints_A = [17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134]
joints_B = [-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134]

robot.clear_set()
robot.set_joint_cmd_pose(arm='A', joints=joints_A)
robot.set_joint_cmd_pose(arm='B', joints=joints_B)
robot.send_cmd()
logger.info('set A&B arm initial pos')

# 等待双臂运动到起始关节角，带超时保护
wait_ms = 0
reached = False
while wait_ms < 60000:
    time.sleep(0.001)
    wait_ms += 1
    data = robot.subscribe(dcss)
    reached = check_joints_accuracy_with_tolerance(data['outputs'][0]['fb_joint_pos'], joints_A, 0.05) and \
              check_joints_accuracy_with_tolerance(data['outputs'][1]['fb_joint_pos'], joints_B, 0.05)
    if reached:
        break
if not reached:
    logger.error('wait arms reach start joints timeout')
    robot.release_robot()
    exit(-1)

logger.info(f"current joints of arm A={[round(v, 2) for v in data['outputs'][0]['fb_joint_pos']]}")
logger.info(f"current joints of arm B={[round(v, 2) for v in data['outputs'][1]['fb_joint_pos']]}")

# [阶段二｜步骤 8] 通过正运动学计算双臂起点末端位姿并转换为 XYZABC
fk_mat_A = kk1.fk(joints_A)
if not fk_mat_A:
    logger.error('Forward Kinematics Error of arm A')
    robot.release_robot()
    exit(-1)
xyzabc_A = kk1.mat4x4_to_xyzabc(fk_mat_A)
if not xyzabc_A:
    logger.error('matrix to xyzabc failed of arm A.')
    robot.release_robot()
    exit(-1)

fk_mat_B = kk2.fk(joints_B)
if not fk_mat_B:
    logger.error('Forward Kinematics Error of arm B')
    robot.release_robot()
    exit(-1)
xyzabc_B = kk2.mat4x4_to_xyzabc(fk_mat_B)
if not xyzabc_B:
    logger.error('matrix to xyzabc failed of arm B.')
    robot.release_robot()
    exit(-1)

logger.info(f'start XYZABC of arm A={[round(v, 2) for v in xyzabc_A]}')
logger.info(f'start XYZABC of arm B={[round(v, 2) for v in xyzabc_B]}')

# [阶段三｜步骤 9] 填写双臂同步规划参数结构体
# 目标：双臂末端各沿自身基坐标系 Z 轴正方向下探 100 mm
aspp = ArmsSynchronousPlanningParams()

# World_Co_Flag 为 False 时，位姿基于各自机械臂基坐标系给定，Skye_Body_Start_PRR 不生效；
# 若需基于机体坐标系给定双臂位姿，请将 World_Co_Flag 置 True(1)，
# 并填写 Skye_Body_Start_PRR（机体俯仰、横滚及高度，单位：度、度、毫米）。
aspp.World_Co_Flag = 0

# Arm0（左臂 A 臂）
for i in range(6):
    aspp.Arm0_Start_XYZABC[i] = xyzabc_A[i]
    aspp.Arm0_End_XYZABC[i] = xyzabc_A[i]
aspp.Arm0_End_XYZABC[2] += 100
for i in range(7):
    aspp.Arm0_Ref_Joints[i] = joints_A[i]
aspp.Arm0_ZSP_Type = 0  # 逆解零空间约束类型：0 与参考关节角欧式距离最近

# Arm1（右臂 B 臂）
for i in range(6):
    aspp.Arm1_Start_XYZABC[i] = xyzabc_B[i]
    aspp.Arm1_End_XYZABC[i] = xyzabc_B[i]
aspp.Arm1_End_XYZABC[2] += 100
for i in range(7):
    aspp.Arm1_Ref_Joints[i] = joints_B[i]
aspp.Arm1_ZSP_Type = 0

# 公共参数
aspp.Freq = 50   # 规划频率 50 Hz，与 setPln_Cart_AB 下发频率匹配
aspp.Vel = 50.0  # 直线规划速度，单位 mm/s（范围 0.1 ~ 500）
aspp.Acc = 100.0  # 直线规划加速度，单位 mm/s^2（范围 0.1 ~ 500）

# [阶段三｜步骤 10] 生成双臂同步直线轨迹
# movL_synchronous 不区分机械臂序号（内部使用双臂已初始化的运动学参数）
points_arm0, pset_arm0, points_arm1, pset_arm1 = kk1.movL_synchronous(aspp, dimension=7)
if not points_arm0 or not points_arm1:
    logger.error('movL_synchronous Error')
    robot.release_robot()
    exit(-1)

logger.info(f'pln points of arm A:{len(points_arm0)}, arm B:{len(points_arm1)}')

# 保存 arm0 规划点，对应 C++ 示例中 pset_arm0.OnSave("arm0.txt")
save_path = os.path.join(current_path, 'arm0.txt')
with open(save_path, 'w') as f:
    for point in points_arm0:
        f.write(','.join(f'{v:.6f}' for v in point) + '\n')
logger.info(f'arm0 planning points saved as :{save_path}')

# [阶段四｜步骤 11] 同时下发双臂规划点并开始运动
# 等待双臂上一段轨迹结束（双臂都结束才继续），带超时保护
wait_ms = 0
finished = False
while wait_ms < 60000:
    time.sleep(0.001)
    wait_ms += 1
    data = robot.subscribe(dcss)
    if data['outputs'][0]['traj_state'] == b'\x00' and data['outputs'][1]['traj_state'] == b'\x00':
        finished = True
        break
if not finished:
    logger.error('wait previous dual arm traj finish timeout')
    kk1.destroy_point_set(pset_arm0)
    kk2.destroy_point_set(pset_arm1)
    robot.release_robot()
    exit(-1)

ret = robot.setPln_Cart_AB(pset_arm0, pset_arm1)
if not ret:
    logger.error('Failed to run dual arm plan')
    kk1.destroy_point_set(pset_arm0)
    kk2.destroy_point_set(pset_arm1)
    robot.release_robot()
    exit(-1)
logger.info('Dual-arm synchronous planning started')

# [阶段四｜步骤 12] 等待双臂轨迹执行完成，带超时保护
time.sleep(0.2)
wait_ms = 0
finished = False
while wait_ms < 60000:
    time.sleep(0.001)
    wait_ms += 1
    data = robot.subscribe(dcss)
    if data['outputs'][0]['traj_state'] == b'\x00' and data['outputs'][1]['traj_state'] == b'\x00':
        finished = True
        break
if not finished:
    # 超时不得直接下使能（避免运动中断使能），释放连接并退出
    logger.error('wait dual arm traj finish timeout')
    kk1.destroy_point_set(pset_arm0)
    kk2.destroy_point_set(pset_arm1)
    robot.release_robot()
    exit(-1)
logger.info('Dual-arm synchronous planning completed')

data = robot.subscribe(dcss)
logger.info(f"final joints of arm A={[round(v, 2) for v in data['outputs'][0]['fb_joint_pos']]}")
logger.info(f"final joints of arm B={[round(v, 2) for v in data['outputs'][1]['fb_joint_pos']]}")

# [阶段四｜步骤 13] 任务结束：下使能并释放连接
kk1.destroy_point_set(pset_arm0)
kk2.destroy_point_set(pset_arm1)

time.sleep(0.05)
robot.clear_set()
robot.set_state(arm='A', state=0)
robot.set_state(arm='B', state=0)
robot.send_cmd()
time.sleep(0.05)

robot.release_robot()
