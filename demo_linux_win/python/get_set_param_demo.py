from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS

'''#################################################################
该DEMO 为获取和设置参数案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置位置模式和速度加速度百分比,听上始能声音
    7 获取整形参数&浮点形参数
    8 设置整形参数&浮点形参数 
    9 再次获取整形参数&浮点形参数确认修改
    10 验证完毕.改回原来的的整形参数&浮点形参数 
    11 确认改回是否成功:获取整形参数&浮点形参数
    12 保存参数
    13 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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
if init==-1:
    logger.error('failed:端口占用，连接失败!')
    exit(0)
else:
    motion_tag = 0
    frame_update = None
    for i in range(5):
        sub_data = robot.subscribe(dcss)
        print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
        if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
            motion_tag += 1
            frame_update = sub_data['outputs'][0]['frame_serial']
        time.sleep(0.1)
    if motion_tag > 0:
        logger.info('success:机器人连接成功!')
    else:
        logger.error('failed:机器人连接失败!')
        exit(0)


'''开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志

'''清错'''
robot.clear_set()
robot.clear_error('A')
robot.send_cmd()
time.sleep(1)

'''设置位置模式和速度保障连接：听上始能声音'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.5)

'''read'''
int_param1=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param1}')

float_param1=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param1}')


'''set'''
robot.set_param('int','R.A1.L0.BASIC.TorqueMax',int_param1+1)
robot.set_param('float','R.A1.L0.BASIC.SensorK',float_param1+0.1)
time.sleep(1)


'''read'''
int_param2=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param2}')

float_param2=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param2}')


'''reset'''
robot.set_param('int','R.A1.L0.BASIC.TorqueMax',int_param1)
robot.set_param('float','R.A1.L0.BASIC.SensorK',float_param1)
time.sleep(1)


'''re-read'''
int_param_final=robot.get_param('int','R.A1.L0.BASIC.TorqueMax')
logger.info(f'R.A1.L0.BASIC.TorqueMax:{int_param_final}')

float_param_final=robot.get_param('float','R.A1.L0.BASIC.SensorK')
logger.info(f'R.A1.L0.BASIC.SensorK:{float_param_final}')

'''save'''
robot.save_para_file()


'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()