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
该DEMO 为坐标拖动控制案列

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    先进坐标阻抗再设置坐标拖动模式X拖动方向
    订阅查看设置是否成功
    按住末端按钮X方向拖动,刷新订阅数据
    切换Y拖动
    按住末端按钮Y方向拖动,刷新订阅数据
    任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

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

'''进拖动前先切换坐标阻抗模式'''
robot.clear_set()
robot.set_state(arm='A',state=3) #torque mode
robot.set_impedance_type(arm='A',type=2) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)


'''设置坐标X方向拖动'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=2)
# dgType
# 0 退出拖动模式
# 1 关节空间拖动
# 2 笛卡尔空间x方向拖动
# 3 笛卡尔空间y方向拖动
# 4 笛卡尔空间z方向拖动
# 5 笛卡尔空间旋转方向拖动
robot.send_cmd()

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'arm error code:{sub_data["states"][0]["err_code"]}')
logger.info(f'set drag space type={sub_data["inputs"][0]["drag_sp_type"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')



'''
手拖动x方向,拖动结束需要且其他坐标方向需要先退出拖动再选要进入的模式
'''

'''30秒拖动时间预留,10HZ频率刷新关节位置数据'''
for i in range(300):
    time.sleep(0.1)
    sub_data = robot.subscribe(dcss)
    logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')


'''设置坐标y方向拖动'''
'''先退出X拖动'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=0)
robot.send_cmd()
time.sleep(0.5)

'''设置坐标Y方向拖动'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=3)
# dgType
# 0 退出拖动模式
# 1 关节空间拖动
# 2 笛卡尔空间x方向拖动
# 3 笛卡尔空间y方向拖动
# 4 笛卡尔空间z方向拖动
# 5 笛卡尔空间旋转方向拖动
robot.send_cmd()
time.sleep(0.5)


'''
手拖动y方向
'''
'''30秒拖动时间预留,10HZ频率刷新关节位置数据'''
for i in range(300):
    time.sleep(0.1)
    sub_data = robot.subscribe(dcss)
    logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')



'''拖动任务完成，退出拖动下使能'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=0)
robot.send_cmd()
time.sleep(0.5)

robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()
time.sleep(0.5)

'''释放机器人内存'''
robot.release_robot()










