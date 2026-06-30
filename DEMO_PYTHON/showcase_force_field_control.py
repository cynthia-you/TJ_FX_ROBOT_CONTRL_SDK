import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS, FTCmd
import time
import logging

'''#################################################################
该DEMO 让机械臂末端以给定的力和扭矩运动到给定的位置距离和姿态距离。
可实时触发调整力的方向和大小。针对机器人拉门和拉抽屉任务，
需要实时调整力方向和大小的场景十分适用。 此功能无法进行力位混合控制  但可以实时切换控制模式
注：此时零空间刚度为0,力场模式下，限制零空间运动可以增大零空间阻尼

使用逻辑
     初始化订阅数据的结构体
     查验连接是否成功,失败程序直接退出
     设置关节阻抗模式
     关节阻抗模式运动到给定位置
     切换笛卡尔力场模式进行力控
     任务完成,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
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
# 订阅最新数据获取机械臂的错误和状态
robot.check_error_and_clear(dcss)

'''通过确认frame数据的刷新，确认UDP数据通道连接成功（防火墙等可能不能正常收到数据）'''
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
robot.log_switch('1')  # 全局日志开："1", 关："0"
robot.local_log_switch('1')  # 主要日志开："1", 关："0"

# 设置刚度和阻尼参数
K = [5000, 5000, 5000, 10, 10, 10, 50]  # 笛卡尔阻抗刚度
D = [0.5, 0.5, 0.5, 0.3, 0.3, 0.3, 1]  # 笛卡尔阻抗阻尼
k = [10, 10, 10, 8, 5, 4, 4]              # 关节阻抗刚度
d = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]  # 关节阻抗阻尼
run_vel = 20  # 速度百分比
run_acc = 20  # 加速度百分比

'''设置关节阻抗模式'''
robot.clear_set()
robot.set_vel_acc('B', run_vel, run_acc)
robot.set_cart_kd_params('B', K, D)
robot.set_joint_kd_params('B', k, d)
robot.set_state('B', 3)          # ARM_STATE_TORQ = 3, 扭矩模式
robot.set_impedance_type('B', 1)  # Type=1 关节阻抗
robot.send_cmd()
time.sleep(1)

'''关节阻抗模式运动到给定位置'''
joints_b = [90, 80, -90, -90, 0, 0, 0]
robot.clear_set()
robot.set_joint_cmd_pose('B', joints_b)
robot.send_cmd()
time.sleep(5)  # 预留运动时间

time.sleep(0.5)

'''切换笛卡尔力场模式进行力控'''
cmd = FTCmd()
cmd.fxDir[0] = 1     # X方向
cmd.fxDir[1] = 0     # Y方向
cmd.fxDir[2] = 0     # Z方向
cmd.fxDir[3] = 0     # RX方向 (相对末端坐标系)
cmd.fxDir[4] = 0     # RY方向 (相对末端坐标系)
cmd.fxDir[5] = -1    # RZ方向 (相对末端坐标系)
cmd.F = 10            # 目标力 10N
cmd.K = 6000          # 位置方向刚度
cmd.Dis = 50          # 沿给定方向的运动距离 50mm
cmd.FreeDis = 1       # 位置方向无力区间 1mm
cmd.NFreeDis = 1      # 姿态方向无力区间 1度
cmd.Tn = 2            # 姿态方向扭矩 2Nm
cmd.Ndis = 100        # 姿态方向运动距离 100度
cmd.Kn = 80           # 姿态方向刚度

robot.clear_set()
robot.ft_arm_control('B', cmd)
robot.send_cmd()
time.sleep(8)

'''任务完成, 下使能'''
robot.clear_set()
robot.set_state('B', 0) 
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()
