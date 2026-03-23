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
该DEMO 为力控案列

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    设置扭矩模式 力控模式
    设置力控参数
    订阅数据查看是否设置
    设置力控指令
    订阅查看是否设置
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

'''设置扭矩模式 力控模式 '''
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.set_impedance_type(arm='A',type=3) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)

'''设置力控参数'''
robot.clear_set()
# 设置是在Y轴方向有5厘米的调节范围
robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=5.)
time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f"current state{sub_data['states'][0]['cur_state']}")
logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')
logger.info(f"arm error code:{sub_data['states'][0]['err_code']}")
logger.info(f'set force fcType={sub_data["inputs"][0]["force_type"]}, '
             f'fxDirection={sub_data["inputs"][0]["force_dir"][:]}, '
             f'fcCtrlpara={sub_data["inputs"][0]["force_pidul"][:]}, '
             f'fcAdjLmt={sub_data["inputs"][0]["force_adj_lmt"]}')



'''设置力控指令'''
robot.clear_set()
#根据前面设置的力控参数，这里力控的效果是：
#在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
robot.set_force_cmd(arm='A',f=10)
time.sleep(0.5)

'''订阅数据查看力控指令是否设置成功'''
sub_data=robot.subscribe(dcss)
logger.info(f'set force cmd={sub_data["inputs"][0]["force_cmd"]}')


time.sleep(30)#预留时间拖拽时间：上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺

'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()









