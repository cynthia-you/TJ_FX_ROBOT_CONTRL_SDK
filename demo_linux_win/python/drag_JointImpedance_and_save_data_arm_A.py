from fx_robot import Marvin_Robot
import logging
from structure_data import DCSS
import time

'''#################################################################
该DEMO 为在关节阻抗模式下,进去关节拖动,拖动并保存数据的控制案列

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 进拖动前先切换扭矩模式和切到关节阻抗模式
    7 设置拖动类型
    8 订阅查看是否进入扭矩模式-->是否为关节阻抗模式-->拖动类型是否为关节拖动-->检测是否按下拖动按钮
    9  第8步条件满足,设置保存关节轨迹并开始保存
    10 拖动完成松开按钮即可,程序自动检测是否松开按钮,松开停止采集数据并保存数据到指定文件
    11 拖动任务完成，退出拖动下使能
    12 释放内存使别的程序或者用户可以连接机器人
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

'''进拖动前先切换扭矩模式和切到关节阻抗模式'''
robot.clear_set()
robot.set_state(arm='A',state=3) #torque mode
robot.send_cmd()
time.sleep(0.5)
robot.clear_set()
robot.set_impedance_type(arm='A',type=1) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)


'''设置拖动类型'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=1)
# dgType
# 0 退出拖动模式
# 1 关节空间拖动
# 2 笛卡尔空间x方向拖动
# 3 笛卡尔空间y方向拖动
# 4 笛卡尔空间z方向拖动
# 5 笛卡尔空间旋转方向拖动
robot.send_cmd()


stage1=1
stage2=0
'''是否进入扭矩模式-->是否为关节阻抗模式-->拖动类型是否为关节拖动-->检测是否按下拖动按钮, 满足条件: 设置保存数据参数并开始保存数据'''
while stage1==1:
    '''订阅数据查看是否设置'''
    sub_data=robot.subscribe(dcss)
    logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
    logger.info(f'set impedance type{sub_data["inputs"][0]["imp_type"]}')
    logger.info(f'set drag space type={sub_data["inputs"][0]["drag_sp_type"]}')

    if sub_data["states"][0]["cur_state"]==3 :
        if sub_data["inputs"][0]["imp_type"]==1 :
            if sub_data["inputs"][0]["drag_sp_type"]==1:
                if sub_data['outputs'][0]['tip_di'][0]==1:
                    cols = 7
                    idx = [0, 1, 2, 3, 4, 5, 6,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0,
                           0, 0, 0, 0, 0, 0, 0]
                    rows = 1000000
                    robot.clear_set()
                    robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
                    robot.send_cmd()
                    time.sleep(0.5)
                    stage2 = 1
                    stage1 = 0
    time.sleep(0.01)



'''检测是否松开拖动按钮,松开停止数据采集'''
while stage2==1:
    '''订阅数据查看是否设置'''
    sub_data=robot.subscribe(dcss)
    if sub_data['outputs'][0]['tip_di'][0]!=1:
        robot.clear_set()
        robot.stop_collect_data()
        robot.send_cmd()
        time.sleep(0.5)
        stage2=0
        break


'''保存采集数据'''
path='drag_joint.txt'
robot.save_collected_data_to_path(path)
time.sleep(5)#睡5秒再退拖动和下使能,不然吓到拖动的人噜


'''拖动任务完成，退出拖动下使能'''
robot.clear_set()
robot.set_drag_space(arm='A',dgType=0)
robot.send_cmd()
time.sleep(0.5)

robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()
time.sleep(0.1)


'''释放机器人内存'''
robot.release_robot()







