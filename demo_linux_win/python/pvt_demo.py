
from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
'''#################################################################
该DEMO 为跑PVT轨迹并保存数据的案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置PVT模式
    7 订阅查看设置是否成功
    8 设置PVT 轨迹本机路径 和PVT号
    9 机器人运动前开始设置保存数据并开始采集数据
    10 设置运行的PVT号并立即执行PVT轨迹
    11 停止采集
    12 保存采集数据
    13 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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
if init==-1:
    logger.error('failed:端口占用，连接失败!')
    exit(0)
else:
    '''防总线通信异常,先清错'''
    time.sleep(0.5)
    robot.clear_set()
    robot.clear_error('A')
    robot.clear_error('B')
    robot.send_cmd()
    time.sleep(0.5)

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


'''设置位置模式和速度'''
robot.clear_set()
robot.set_state(arm='A',state=2)#PVT， 自己的速度和加速度，不受外部控制。
robot.send_cmd()
time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
logger.info(f'cmd state:{sub_data["states"][0]["cmd_state"]}')
logger.info(f'error code:{sub_data["states"][0]["err_code"]}')

'''设置PVT 轨迹本机路径 和PVT号'''
#linux
pvt_file='DEMO_SRS_Left.fmv'
robot.send_pvt_file('A',pvt_file, 2)
time.sleep(1)

'''机器人运动前开始设置保存数据'''
cols=7
idx=[0,1,2,3,4,5,6,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0,
     0,0,0,0,0,0,0]
rows=1000
robot.clear_set()
robot.collect_data(targetNum=cols,targetID=idx,recordNum=rows)
robot.send_cmd()
time.sleep(0.5)


'''设置运行的PVT号'''
robot.clear_set()
robot.set_pvt_id('A',2)
robot.send_cmd()
time.sleep(0.5)

time.sleep(10)#模拟跑轨迹时间


'''停止采集'''
robot.clear_set()
robot.stop_collect_data()
robot.send_cmd()
time.sleep(0.5)


'''保存采集数据'''

'''linux'''
path='aaa.txt'
robot.save_collected_data_to_path(path)
time.sleep(0.5)


'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()




