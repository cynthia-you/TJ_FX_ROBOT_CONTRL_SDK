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
该DEMO 为获取和设置参数案列

使用逻辑
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功,失败程序直接退出
    开启日志以便检查
    获取整形参数&浮点形参数
    设置整形参数&浮点形参数 
    再次获取整形参数&浮点形参数确认修改
    验证完毕.改回原来的的整形参数&浮点形参数 
    确认改回是否成功:获取整形参数&浮点形参数
    保存参数
    任务完成,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
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

'''read'''
re_flag,int_param1=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param1}')

re_flag,float_param1=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param1}')


'''set'''
robot.set_param('int','R.A1.L0.BASIC.TorqueMax',int_param1+1)
robot.set_param('float','R.A1.L0.BASIC.SensorK',float_param1+0.1)
time.sleep(1)


'''read'''
re_flag,int_param2=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param2}')

re_flag,float_param2=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param2}')


'''reset'''
robot.set_param('int','R.A1.L0.BASIC.TorqueMax',int_param1)
robot.set_param('float','R.A1.L0.BASIC.SensorK',float_param1)
time.sleep(1)


'''re-read'''
re_flag,int_param_final=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param_final}')

re_flag,float_param_final=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param_final}')

'''save'''
robot.save_para_file()


'''释放机器人内存'''
robot.release_robot()
