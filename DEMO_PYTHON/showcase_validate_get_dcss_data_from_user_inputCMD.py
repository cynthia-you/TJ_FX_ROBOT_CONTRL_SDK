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
     覆盖全部 13 个自定义数据类别（116~120、216~220、300~302）
     释放机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)   # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

# ── 用户自定义数据类别常量 ──
#以下数据的获取使用通讯数据结构DCSS中RT_OUT组别下的关键字m_EST_Joint_Firc_Dot：
DCSS_CMD_ARM0_GET_DATA_6FT     = 116   # A臂: 前6个为六维力数据，第7个无实义
DCSS_CMD_ARM0_GET_DATA_GRAVITY = 117   # A臂: 7个轴的重力补偿数据
DCSS_CMD_ARM0_GET_DATA_TOOLDYN = 118   # A臂: 工具动力学7个参数：m mx my mz ixx iyy izz
DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF = 119  # A臂: 关节阻抗下7个关节刚度
DCSS_CMD_ARM0_GET_DATA_CARTSTIFF = 120  # A臂: 前6个为笛卡尔阻抗下XYZABC刚度，第7个无实义

DCSS_CMD_ARM1_GET_DATA_6FT     = 216   # B臂: 前6个为六维力数据，第7个无实义
DCSS_CMD_ARM1_GET_DATA_GRAVITY = 217   # B臂: 7个轴的重力补偿数据
DCSS_CMD_ARM1_GET_DATA_TOOLDYN = 218   # B臂: 工具动力学7个参数：m mx my mz ixx iyy izz
DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF = 219  # B臂: 关节阻抗下7个关节刚度
DCSS_CMD_ARM1_GET_DATA_CARTSTIFF = 220  # B臂: 前6个为笛卡尔阻抗下XYZABC刚度, 第7个无实义

#以下数据的获取使用通讯数据结构DCSS中RT_OUT组别下的关键字m_EST_Cart_FN：
DCSS_CMD_GET_DATA_GYRO_ACC     = 300   # 双臂: 陀螺仪三轴的加速度
DCSS_CMD_GET_DATA_GYRO_ANGLE   = 301   # 双臂: 陀螺仪三轴的欧拉角，俯仰/横滚/航向
DCSS_CMD_GET_DATA_GYRO_OMEG    = 302   # 双臂: 陀螺仪三轴的角速度

#获取用户最新下发的自定义数据类型使用通讯数据结构DCSS中RT_OUT组别下的关键字m_EST_Joint_Firc

# (类别, 名称, 机械臂, 数据字段, 取值个数)
#   m_EST_Joint_Firc_Dot : 116~120 / 216~220 的数据写入该字段（每臂7个）
#   m_EST_Cart_FN        : 300~302 的数据写入该字段（每臂6个，陀螺仪只用前3个）
USER_SPECIFIED_DATA_CATEGORIES = [
    # ── A臂（arm='A'）──
    (DCSS_CMD_ARM0_GET_DATA_6FT,        'DCSS_CMD_ARM0_GET_DATA_6FT',        'A', 'est_joint_firc_dot', 6),
    (DCSS_CMD_ARM0_GET_DATA_GRAVITY,    'DCSS_CMD_ARM0_GET_DATA_GRAVITY',    'A', 'est_joint_firc_dot', 7),
    (DCSS_CMD_ARM0_GET_DATA_TOOLDYN,    'DCSS_CMD_ARM0_GET_DATA_TOOLDYN',    'A', 'est_joint_firc_dot', 7),
    (DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF, 'DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF', 'A', 'est_joint_firc_dot', 7),
    (DCSS_CMD_ARM0_GET_DATA_CARTSTIFF,  'DCSS_CMD_ARM0_GET_DATA_CARTSTIFF',  'A', 'est_joint_firc_dot', 6),
    # ── B臂（arm='B'）──
    (DCSS_CMD_ARM1_GET_DATA_6FT,        'DCSS_CMD_ARM1_GET_DATA_6FT',        'B', 'est_joint_firc_dot', 6),
    (DCSS_CMD_ARM1_GET_DATA_GRAVITY,    'DCSS_CMD_ARM1_GET_DATA_GRAVITY',    'B', 'est_joint_firc_dot', 7),
    (DCSS_CMD_ARM1_GET_DATA_TOOLDYN,    'DCSS_CMD_ARM1_GET_DATA_TOOLDYN',    'B', 'est_joint_firc_dot', 7),
    (DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF, 'DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF', 'B', 'est_joint_firc_dot', 7),
    (DCSS_CMD_ARM1_GET_DATA_CARTSTIFF,  'DCSS_CMD_ARM1_GET_DATA_CARTSTIFF',  'B', 'est_joint_firc_dot', 6),
    # ── 双臂（arm='AB'）──
    (DCSS_CMD_GET_DATA_GYRO_ACC,        'DCSS_CMD_GET_DATA_GYRO_ACC',        'AB', 'est_cart_fn', 3),
    (DCSS_CMD_GET_DATA_GYRO_ANGLE,      'DCSS_CMD_GET_DATA_GYRO_ANGLE',      'AB', 'est_cart_fn', 3),
    (DCSS_CMD_GET_DATA_GYRO_OMEG,       'DCSS_CMD_GET_DATA_GYRO_OMEG',       'AB', 'est_cart_fn', 3),
]

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
# 3. DCSS_CMD_ARM0_GET_DATA_TOOLDYN — A臂读取工具动力学参数（m mx my mz ixx iyy izz）
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('A', DCSS_CMD_ARM0_GET_DATA_TOOLDYN)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    m, mx, my, mz, ixx, iyy, izz = out0['est_joint_firc_dot'][:7]
    print(f"DCSS_CMD_ARM0_GET_DATA_TOOLDYN m={m:.6f} mx={mx:.6f} my={my:.6f} mz={mz:.6f} "
          f"ixx={ixx:.6f} iyy={iyy:.6f} izz={izz:.6f}")
    print(f"current specify Data = {out0['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 4. DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF — A臂读取关节阻抗下的7个关节刚度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('A', DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    print(f"DCSS_CMD_ARM0_GET_DATA_JOINTSTIFF {out0['est_joint_firc_dot'][:7]}")
    print(f"current specify Data = {out0['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 5. DCSS_CMD_ARM0_GET_DATA_CARTSTIFF — A臂读取笛卡尔阻抗下的XYZABC刚度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('A', DCSS_CMD_ARM0_GET_DATA_CARTSTIFF)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out0 = sub_data['outputs'][0]
    print(f"DCSS_CMD_ARM0_GET_DATA_CARTSTIFF {out0['est_joint_firc_dot'][:6]}")
    print(f"current specify Data = {out0['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 6. DCSS_CMD_ARM1_GET_DATA_6FT — B臂读取六维力数据
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('B', DCSS_CMD_ARM1_GET_DATA_6FT)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_ARM1_GET_DATA_6FT {out1['est_joint_firc_dot'][:6]}")
    print(f"current specify Data = {out1['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 7. DCSS_CMD_ARM1_GET_DATA_GRAVITY — B臂读取重力数据
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('B', DCSS_CMD_ARM1_GET_DATA_GRAVITY)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_ARM1_GET_DATA_GRAVITY {out1['est_joint_firc_dot'][:7]}")
    print(f"current specify Data = {out1['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 8. DCSS_CMD_ARM1_GET_DATA_TOOLDYN — B臂读取工具动力学参数（m mx my mz ixx iyy izz）
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('B', DCSS_CMD_ARM1_GET_DATA_TOOLDYN)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out1 = sub_data['outputs'][1]
    m, mx, my, mz, ixx, iyy, izz = out1['est_joint_firc_dot'][:7]
    print(f"DCSS_CMD_ARM1_GET_DATA_TOOLDYN m={m:.6f} mx={mx:.6f} my={my:.6f} mz={mz:.6f} "
          f"ixx={ixx:.6f} iyy={iyy:.6f} izz={izz:.6f}")
    print(f"current specify Data = {out1['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 9. DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF — B臂读取关节阻抗下的7个关节刚度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('B', DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_ARM1_GET_DATA_JOINTSTIFF {out1['est_joint_firc_dot'][:7]}")
    print(f"current specify Data = {out1['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 10. DCSS_CMD_ARM1_GET_DATA_CARTSTIFF — B臂读取笛卡尔阻抗下的XYZABC刚度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('B', DCSS_CMD_ARM1_GET_DATA_CARTSTIFF)
i = 5
while i > 0:
    i -= 1
    sub_data = robot.subscribe(dcss)
    out1 = sub_data['outputs'][1]
    print(f"DCSS_CMD_ARM1_GET_DATA_CARTSTIFF {out1['est_joint_firc_dot'][:6]}")
    print(f"current specify Data = {out1['est_joint_firc'][0]}")
    time.sleep(0.5)

# ═══════════════════════════════════════════════════════════════════
# 11. DCSS_CMD_GET_DATA_GYRO_ACC — 双臂读取陀螺仪加速度
# ═══════════════════════════════════════════════════════════════════
robot.set_user_specified_data('AB', DCSS_CMD_GET_DATA_GYRO_ACC)
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
# 12. DCSS_CMD_GET_DATA_GYRO_ANGLE — 双臂读取陀螺仪角度（俯仰/横滚/航向）
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
# 13. DCSS_CMD_GET_DATA_GYRO_OMEG — 双臂读取陀螺仪角速度
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

# ═══════════════════════════════════════════════════════════════════
# 14. 遍历全部自定义数据类别 — 统一切换并打印
#     说明：同一通道重复下发相同类别时 m_EST_Joint_Firc 不刷新，此处每类只下发一次
# ═══════════════════════════════════════════════════════════════════
for idx, (category, name, arm, data_key, dim) in enumerate(USER_SPECIFIED_DATA_CATEGORIES, start=1):
    robot.set_user_specified_data(arm, category)
    time.sleep(0.3)
    sub_data = robot.subscribe(dcss)
    if sub_data is None or not sub_data.get('outputs'):
        print(f"[{idx:02d}] {name} (arm={arm}, category={category}) no data")
        continue
    out = sub_data['outputs'][0] if arm == 'A' else sub_data['outputs'][1]
    print(f"[{idx:02d}] {name} (arm={arm}, category={category}) {data_key} = {out[data_key][:dim]}")

'''释放机器人内存'''
robot.release_robot()
