from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
'''#################################################################
该DEMO 为关节跟随模式下保存机器人数据为CSV案例

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置位置模式和速度加速度百分比
    7 机器人运动前开始设置保存数据参数并开始保存数据
    8 下发运动点位
    9 停止数据采集
    10 保存数据为CSV
    11 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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


'''设置位置模式和速度加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()
time.sleep(0.5)

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

'''运动'''
robot.clear_set()
joint_cmd_1=[0.,0.,0.,0.,0.,0.,5.]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()

time.sleep(0.5)# 模拟运动时长

'''停止采集'''
robot.clear_set()
robot.stop_collect_data()
robot.send_cmd()
time.sleep(0.5)

'''保存采集数据'''
path='aa.csv'
robot.save_collected_data_as_csv_to_path(path)


'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()
