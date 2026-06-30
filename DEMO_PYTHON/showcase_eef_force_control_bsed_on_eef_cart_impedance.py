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
该 DEMO 为 A臂 设置笛卡尔阻抗下力控参数 + 切换扭矩模式 + 读回校验的示例

目的：
    1) 下发 A 臂笛卡尔刚度阻尼参数（K/D）与控制类型
    2) 下发末端笛卡尔旋转方向参数
    3) 设置关节速度/加速度比例限制
    4) 切换 A 臂目标状态到扭矩相关模式，并设置阻抗类型为力控
    5) 设置末端 Z 方向力控参数
    6) 通过 subscribe 读取 DCSS，打印当前状态/指令状态/错误码，以及下发参数是否生效

使用逻辑
     连接机器人 + 清错 + 验证数据通道
     位置模式移动到目标关节位置
     切换到笛卡尔阻抗下力控模式，设置 Z 方向力控
     读回校验 → 打印 DCSS 参数
     任务完成, 下使能, 释放机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)   # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

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

# ── 参数定义 ──
K = [6000, 6000, 6000, 100, 100, 100, 20]   # 笛卡尔阻抗刚度
D = [0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 1]       # 笛卡尔阻抗阻尼
type_val = 3                                   # fcType=3，与末端力控接口一起使用
Dir = [0, 0, 0, 0, 0, 0, 0]                   # 笛卡尔方向: fcType=3 时全填 0
target_joint = [9, 50, 10, -115, 0, 0, 0]     # 目标关节角度

# 力控参数
fxDir = [0, 0, 1, 0, 0, 0]   # Z 方向力控
fcCtrlPara = [0, 0, 0, 0, 0, 0, 0]  # 控制参数(全0)
fcAdjLmt = 50.0               # 允许调节范围 mm
force = 10.0                  # 目标力 N

# ── 安全确认 ──
print(f"The robotic arm A will move to: {target_joint}")
print("Please ensure environmental safety!")
user_input = input("Should it be continued? Y or N: ").strip()
if user_input not in ('Y', 'y'):
    print("program end!")
    robot.release_robot()
    exit(0)
print("Robot is moving!")

'''设置速度和加速度限制'''
robot.clear_set()
robot.set_vel_acc('A', 20, 20)
robot.send_cmd()
time.sleep(0.2)

'''切换到位置模式（ARM_STATE_POSITION = 1）'''
robot.clear_set()
robot.set_state('A', 1)  # 位置模式
robot.send_cmd()
time.sleep(0.2)

'''位置模式下运动到目标关节位置'''
robot.clear_set()
robot.set_joint_cmd_pose('A', target_joint)
robot.send_cmd()
time.sleep(3)

'''设置末端笛卡尔阻抗参数 (fcType=3: 末端阻抗下力控)'''
robot.clear_set()
robot.set_cart_kd_params('A', K, D)
robot.set_EefCart_control_params('A', 3, Dir)
robot.send_cmd()
time.sleep(0.2)

'''切换到扭矩模式 + 力控阻抗类型'''
robot.clear_set()
robot.set_state('A', 3)          # ARM_STATE_TORQ = 3
robot.set_impedance_type('A', 3)  # Type=3 力控
robot.send_cmd()
time.sleep(0.2)

'''设置末端力控参数: Z 方向力控, 10N'''
robot.clear_set()
robot.set_force_control_params('A', 3, fxDir, fcCtrlPara, fcAdjLmt)
robot.set_force_cmd('A', force)
robot.send_cmd()
time.sleep(0.2)

'''── 读回校验：获取 A 臂状态信息并打印 ──'''
sub_data = robot.subscribe(dcss)
state = sub_data['states'][0]
in0 = sub_data['inputs'][0]

print(f"current state of A arm   : {state['cur_state']}")
print(f"cmd state of A arm       : {state['cmd_state']}")
print(f"error code of A arm      : {state['err_code']}")
print(f"CMD of impedance type    : {in0['imp_type']}")
print(f"CMD of vel and acc       : {in0['joint_vel_ratio']} {in0['joint_acc_ratio']}")
print(f"CMD of cart K            : {in0['cart_k']} (KN={in0['cart_kn']})")
print(f"CMD of cart D            : {in0['cart_d']} (DN={in0['cart_dn']})")
print(f"CMD of cart type         : {in0['cart_kd_type']}")
print(f"CMD of force PIDUL       : {in0['force_pidul']}")

# 再次订阅获取关节指令和实际关节位置
sub_data = robot.subscribe(dcss)
in0 = sub_data['inputs'][0]
out0 = sub_data['outputs'][0]

print(f"CMD joints of arm A      : {in0['joint_cmd_pos']}")
print(f"current joints of arm A  : {out0['fb_joint_pos']}")

time.sleep(3)

'''下使能'''
robot.clear_set()
robot.set_state('A', 0)  
robot.send_cmd()
time.sleep(0.2)

'''释放机器人内存'''
robot.release_robot()
