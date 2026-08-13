import sys
import os
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)

from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
from SDK_PYTHON.fx_interference import Marvin_Interference, FUNC_RET_SUCCESS, FUNC_RET_ISInterference
import logging

'''#################################################################
该DEMO 为: 双臂进入关节拖动模式, 获取当前关节做碰撞检测
(对应 DEMO_C++/showcase_drag_interference.cpp 的 python 版本)

使用逻辑
   连接机器人
   清错
   初始化碰撞检测,必须根据机型选择配置文件
   设置关节阻抗KD参数
   进入关节阻抗模式(扭矩模式 + 关节阻抗类型)
   进入关节拖动模式(进拖动前必须先进关节阻抗)
   循环约30s: 持续获取左右臂关节角 -> 碰撞检测, 检测到碰撞只打印不退出
   退出拖动, 释放机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
# logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

# 碰撞检测阈值(单位mm), 对应15个碰撞对
interf_threshold = [10, 10, 10, 10, 10,
                    10, 10, 10, 10, 10,
                    10, 10, 10, 10, 10]

# 碰撞检测配置文件路径(相对仓库根目录),必须根据机型选择配置文件
interf_cfg_dir = os.path.join(parent_dir, 'interferenceCheck', 'InterferenceCfg')
cord_def   = os.path.join(interf_cfg_dir, 'CordDef_CCSM6Lite.Cord')
cal_link   = os.path.join(interf_cfg_dir, 'CalLinkDef.Links')
input_def  = os.path.join(interf_cfg_dir, 'CalInputDef.Joints')
convex_def = os.path.join(interf_cfg_dir, 'ConvexDef_CCSM6Lite.Convex')
ic_def     = os.path.join(interf_cfg_dir, 'InterfDef.Interf')

# ===== 初始化订阅数据的结构体 =====
dcss = DCSS()

# ===== 初始化机器人接口 =====
robot = Marvin_Robot()

# ===== 1. 连接机器人 =====
init = robot.connect('192.168.1.190')
if init == 0:
    logger.error('failed to connect to the robot, port is occupied')
    exit(0)
time.sleep(0.2)

# ===== 2. 清错 =====
robot.check_error_and_clear(dcss)

# 通过确认 frame 数据的刷新, 确认 UDP 数据通道连接成功
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

# ===== 3. 初始化碰撞检测 =====
interf = Marvin_Interference()
interf.create()
interf_ret = interf.init(cord_def, cal_link, input_def, convex_def, ic_def)
if interf_ret != FUNC_RET_SUCCESS:
    logger.error(f'interference init failed: {interf_ret}')
    robot.release_robot()
    exit(0)
logger.info('interference init success')

# ===== 4. 设置关节阻抗KD参数(K非负, D取值0~1) =====
joint_stiffness_K = [3, 3, 3, 3, 2, 2, 2]
joint_damping_D   = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

robot.clear_set()
robot.set_joint_kd_params(arm='A', K=joint_stiffness_K, D=joint_damping_D)
robot.set_joint_kd_params(arm='B', K=joint_stiffness_K, D=joint_damping_D)
robot.send_cmd()
time.sleep(0.2)

# ===== 5. 进入关节阻抗模式(扭矩模式 + 关节阻抗类型) =====
robot.clear_set()
robot.set_state(arm='A', state=3)  # 3=扭矩模式
robot.set_impedance_type(arm='A', type=1)  # 1=关节阻抗
robot.set_state(arm='B', state=3)
robot.set_impedance_type(arm='B', type=1)
robot.send_cmd()
time.sleep(0.5)

# ===== 6. 进入关节拖动模式(进拖动前必须先进关节阻抗) =====
robot.clear_set()
robot.set_drag_space(arm='A', dgType=1)  # 1=关节拖动
robot.set_drag_space(arm='B', dgType=1)
robot.send_cmd()
time.sleep(0.5)

# ===== 7. 循环约30s: 持续获取左右臂关节角并做碰撞检测 =====
logger.info('enter drag + interference check loop')
run_seconds = 30
loop_start = time.time()
detect_count = 0

while True:
    elapsed = time.time() - loop_start
    if elapsed >= run_seconds:
        break

    # 订阅数据, 获取左右臂实时关节角, 拼成14维
    sub_data = robot.subscribe(dcss)
    joint_a = sub_data['outputs'][0]['fb_joint_pos']  # 左臂7个关节
    joint_b = sub_data['outputs'][1]['fb_joint_pos']  # 右臂7个关节
    joint_angles = list(joint_a) + list(joint_b)      # 14维

    # 更新构型 -> 更新包络体 -> 计算碰撞距离
    interf.update_cord(14, joint_angles)
    interf.update_convex()
    interf.calc_interf_distance()

    # 判断是否碰撞: 返回 FUNC_RET_ISInterference 表示发生碰撞
    interf_state = interf.on_is_interf(interf_threshold)
    if interf_state == FUNC_RET_ISInterference:
        detect_count += 1
        print(f"[{elapsed:.2f}s] interference detected! count={detect_count}")

    time.sleep(0.01)  # 控制循环频率, 约100Hz

logger.info(f'loop finished, total interference detected: {detect_count}')
time.sleep(0.5)

# ===== 8. 退出拖动并释放资源 =====
robot.clear_set()
robot.set_state(arm='A', state=0)
robot.set_state(arm='B', state=0)
robot.send_cmd()
time.sleep(0.5)

interf.destroy()
robot.release_robot()
