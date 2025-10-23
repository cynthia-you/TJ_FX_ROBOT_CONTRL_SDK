import os
from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
from fx_kine import Marvin_Kine

def collect_identy_data(robot_id, pvt_file, pvt_id, save_path):
    '''清错'''
    robot.clear_set()
    robot.clear_error('A')
    robot.send_cmd()
    time.sleep(1)

    '''设置位置模式和速度'''
    robot.clear_set()
    robot.set_state(arm='A', state=2)  # PVT， 自己的速度和加速度，不受外部控制。
    robot.send_cmd()
    time.sleep(0.5)

    '''订阅数据查看是否设置'''
    sub_data = robot.subscribe(dcss)
    logger.info(f'current state{sub_data["states"][0]["cur_state"]}')
    logger.info(f'cmd state:{sub_data["states"][0]["cmd_state"]}')
    logger.info(f'error code:{sub_data["states"][0]["err_code"]}')

    '''设置PVT'''
    robot.send_pvt_file('A', pvt_file, pvt_id)
    logger.info(f'set pvt trajectory file: {pvt_file}, pvt id: {pvt_id}')
    time.sleep(0.5)


    '''机器人运动前开始设置保存数据'''
    cols = 15
    if robot_id == 'A':
        idx = [0, 1, 2, 3, 4, 5, 6,
               50, 51, 52, 53, 54, 55, 56,
               76, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0]
    elif robot_id == 'B':
        idx = [100, 101, 102, 103, 104, 105, 106,
               150, 151, 152, 153, 154, 155, 156,
               176, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0]
    else:
        raise ValueError('wrong robot_id')
    rows = 1000000
    robot.clear_set()
    robot.collect_data(targetNum=cols, targetID=idx, recordNum=rows)
    robot.send_cmd()
    logger.info(f'start collect identification data')
    time.sleep(0.5)

    '''设置运行的PVT 号'''
    robot.clear_set()
    robot.set_pvt_id('A', pvt_id)
    robot.send_cmd()
    logger.info(f'start run pvt trajectory')

    time.sleep(60)  # 模拟跑轨迹时间

    '''停止采集'''
    robot.stop_collect_data()
    time.sleep(0.5)

    '''保存采集数据'''
    robot.save_collected_data_to_path(save_path)

    time.sleep(1)

    '''数据预处理'''
    processed_data=[]
    with open(save_path, 'r') as file:
        lines = file.readlines()
        # 删除首行
    lines = lines[1:]
    for i, line in enumerate(lines):
        # 移除行末的换行符并按'$'分割
        parts = line.strip().split('$')
        # 提取每个字段的数字部分（去掉非数字前缀）
        numbers = []
        for part in parts:
            if part:  # 忽略空字符串
                # 找到最后一个空格后的数字部分
                num_str = part.split()[-1]
                numbers.append(num_str)

        # 删除前两列（索引0和1），保留剩余列
        if len(numbers) >= 2:
            numbers = numbers[2:]
        processed_data.append(numbers)
    time.sleep(0.5)
    os.remove(save_path)
    time.sleep(0.5)
    with open(save_path, 'w') as out_file:
        for row in processed_data:
            out_file.write(','.join(row) + '\n')

    logger.info(f'data saved as {save_path} ')

if __name__=="__main__":

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
    if init == -1:
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

    robot.log_switch('1') #全局日志开关
    robot.local_log_switch('1') # 主要日志
    time.sleep(0.5)

    '''
    attention:!!!!!!
    DEMO 演示的是SRS机型下 左臂 工具动力学辨识流程。 
    如果是CCS的机型，请修改collect_identy_data中pvt_file传入值， 以及kk.identify_tool_dyn(robot_type='ccs',ipath='./LoadData')
    
    
    以下三步要依次反注释运行，一共运行三遍!!!
    '''

    # '''step1 采集左臂带载数据'''
    # collect_identy_data(robot_id='A',
    #                     pvt_file="./LoadData/IdenTraj/LoadIdenTraj_MarvinSRS_Left.fmv",
    #                     pvt_id=3,
    #                     save_path='./LoadData/LoadData.csv')

    # '''step2 采集左臂空载数据'''
    # collect_identy_data(robot_id='A',
    #                     pvt_file="./LoadData/IdenTraj/LoadIdenTraj_MarvinSRS_Left.fmv",
    #                     pvt_id=3,
    #                     save_path='./LoadData/NoLoadData.csv')


    # '''step3 算法辨识'''
    # kk=Marvin_Kine()
    # tool_dynamic_parameters=kk.identify_tool_dyn(robot_type='srs',ipath='./LoadData')
    #
    # robot.release_robot()
