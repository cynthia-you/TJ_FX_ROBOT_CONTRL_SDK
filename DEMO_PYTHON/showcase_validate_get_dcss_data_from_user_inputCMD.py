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

'''#################################################################
该 DEMO 演示 通过 OnSetUserSpcfData 接口切换用户自定义数据通道，
将不同的机器人内部数据（六维力、重力、陀螺仪等）映射到 DCSS 的固定字段中，
从而在不改变结构体定义的前提下读取多样的内部状态。

使用逻辑
     连接机器人 + 清错 + 验证数据通道
     切换数据类别 → 订阅读取 → 打印对应的 DCSS 字段
     支持 arm='A' / 'B' / 'AB'(双臂)
     释放机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)   # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

# ── 用户自定义数据类别常量 ──
DCSS_CMD_ARM0_GET_DATA_6FT     = 116   # A臂: 六维力数据
DCSS_CMD_ARM0_GET_DATA_GRAVITY = 117   # A臂: 重力数据
DCSS_CMD_GET_DATA_GYRO_ACC     = 300   # 双臂: 陀螺仪加速度
DCSS_CMD_GET_DATA_GYRO_ANGLE   = 301   # 双臂: 陀螺仪角度
DCSS_CMD_GET_DATA_GYRO_OMEG    = 302   # 双臂: 陀螺仪角速度

'''初始化订阅数据的结构体'''
dcss = DCSS()

'''初始化机器人接口'''
robot = Marvin_Robot()

'''查验连接是否成功'''
init = robot.connect('192.168.1.190')
if init == 0:
    logger.error('failed to connect to the robot, port is occupied')
    exit(0)

time.sleep(0.2)

'''检查伺服和手臂是否有错，有错误清错'''
robot.check_error_and_clear(dcss)

'''通过确认frame数据的刷新，确认UDP数据通道连接成功'''
motion_tag = 0
frame_update = None
for i in range(5):
    sub_data = robot.subscribe(dcss)
    print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
    if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
        motion_tag += 1
        frame_update = sub_data['outputs'][0]['frame_serial']
    time.sleep(0.001)

if motion_tag > 0:
    logger.info('success:robot connected')
else:
    logger.error('failed:robot connection failed')
    robot.release_robot()
    exit(0)

'''控制日志开'''
robot.log_switch('1')
robot.local_log_switch('1')

# ═══════════════════════════════════════════════════════════════════
# 1. DCSS_CMD_ARM0_GET_DATA_6FT — A臂读取六维力数据
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('A', DCSS_CMD_ARM0_GET_DATA_6FT)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    print(f"DCSS_CMD_ARM0_GET_DATA_6FT {out0['est_joint_firc_dot'][:6]}")
    print(f"current specify Data = {out0['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 2. DCSS_CMD_ARM0_GET_DATA_GRAVITY — A臂读取重力数据
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('A', DCSS_CMD_ARM0_GET_DATA_GRAVITY)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    print(f"DCSS_CMD_ARM0_GET_DATA_GRAVITY {out0['est_joint_firc_dot'][:7]}")
    print(f"current specify Data = {out0['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 3. DCSS_CMD_GET_DATA_GYRO_ACC — 双臂读取陀螺仪加速度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('C', DCSS_CMD_GET_DATA_GYRO_ACC)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_GET_DATA_GYRO_ACC {out0['est_cart_fn'][:3]}")
    print(f"current specify Data A = {out0['est_joint_firc'][1]}")
    print(f"current specify Data B = {out1['est_joint_firc'][1]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 4. DCSS_CMD_GET_DATA_GYRO_ANGLE — 双臂读取陀螺仪角度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('AB', DCSS_CMD_GET_DATA_GYRO_ANGLE)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_GET_DATA_GYRO_ANGLE {out0['est_cart_fn'][:3]}")
    print(f"current specify Data A = {out0['est_joint_firc'][1]}")
    print(f"current specify Data B = {out1['est_joint_firc'][1]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 5. DCSS_CMD_GET_DATA_GYRO_OMEG — 双臂读取陀螺仪角速度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('AB', DCSS_CMD_GET_DATA_GYRO_OMEG)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_GET_DATA_GYRO_OMEG {out0['est_cart_fn'][:3]}")
    print(f"current specify Data A = {out0['est_joint_firc'][1]}")
    print(f"current specify Data B = {out1['est_joint_firc'][1]}")
    time.sleep(0.5)

'''释放机器人内存'''
robot.release_robot()
