from fx_robot import Marvin_Robot
import time
import logging
from structure_data import DCSS
from fx_robot import update_text_file_simple

'''#################################################################
该DEMO 为保存工具参数和更新工具参数案例

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    5 为了防止伺服有错，先清错
    6 设置位置模式和速度加速度百分比
    7 确认控制器是否有保存工具信息，如果有：加载保存的数据并生效； 如果无：初始化一个工具文本，并更新工具参数和设置生效
    8 更新工具和设置生效  
    9 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
'''#################################################################

#配置日志系统
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

'''3 开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志


'''4 清错'''
robot.clear_set()
robot.clear_error('A')
robot.send_cmd()
time.sleep(1)

'''5 设置位置模式和速度 加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=1)#state=1位置模式
robot.set_vel_acc(arm='A',velRatio=10, AccRatio=10)
robot.send_cmd()


'''6 确认控制器是否有保存工具信息，如果有：加载保存的数据并生效； 如果无：初始化一个工具文本，并更新工具参数和设置生效'''
tool_result=robot.get_tool_info()
#无工具
if tool_result == 0:
    print('warning', '机器人未设置工具信息，如果带工具，请设置工具信息')

    #初始化工具保存文件
    lines = ['0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n',
             '0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n']
    with open('tool_dyn_kine.txt', 'w', encoding='utf-8') as file:
        file.writelines(lines)
    file.close()

    # tool_left_dynamic工具动力学信息,长度为10  m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz
    # m， 质量 单位千克
    # mcp_x,mcp_y,mcp_z 工具的质心坐标，相对于法兰的偏移， 单位毫米
    # ixx,ixy,ixz,iyy,iyz,izz 转动惯量， 可以不填。
    tool_left_dynamic = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # 工具运动学信息 长度为6 xyzabc， 工具相对末端法兰的位置偏移和姿态旋转
    tool_left_kinematics = [0, 0, 0, 0, 0, 0]
    full_tool_left = tool_left_dynamic + tool_left_kinematics

    robot_id = 'A'

    # 更新修改控制器内的工具信息
    update_text_file_simple(robot_id, full_tool_left, 'tool_dyn_kine.txt')

    # 设置工具
    robot.set_tool(arm='A', dynamicParams=tool_left_dynamic, kineParams=tool_left_kinematics)

# 有工具
else:
    print(f"成功读取控制器已保存的工具信息: {tool_result}")
    if isinstance(tool_result[0], list):
        print(f"左臂工具: {tool_result[0]}")
        print(f"右臂工具: {tool_result[1]}")

        # 从控制器加载工具信息
        robot.set_tool(arm='A', dynamicParams=tool_result[0][:10], kineParams=tool_result[0][10:])
        robot.set_tool(arm='B', dynamicParams=tool_result[1][:10], kineParams=tool_result[1][10:])



'''7 更新工具和设置生效

工具信息不变情况，步骤6可全局使用； 如果工具信息改变需要用步骤7更改工具信息， 然后继续使用步骤6的操作。
'''
tool_result1=robot.get_tool_info()
#无工具
if tool_result1 == 0:
    print('null tool')
else:
    # tool_left_dynamic工具动力学信息,长度为10  m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz
    # m， 质量 单位千克
    # mcp_x,mcp_y,mcp_z 工具的质心坐标，相对于法兰的偏移， 单位毫米
    # ixx,ixy,ixz,iyy,iyz,izz 转动惯量， 可以不填。
    tool_left_dynamic = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # 工具运动学信息 长度为6 xyzabc， 工具相对末端法兰的位置偏移和姿态旋转
    tool_left_kinematics = [0, 0, 0, 0, 0, 0]
    full_tool_left = tool_left_dynamic + tool_left_kinematics

    robot_id = 'A'

    # 更新修改控制器内的工具信息
    update_text_file_simple(robot_id, full_tool_left, 'tool_dyn_kine.txt')

    # 设置工具
    robot.set_tool(arm='A', dynamicParams=tool_left_dynamic, kineParams=tool_left_kinematics)




'''8 任务完成，下使能 释放连接'''
robot.clear_set()
robot.set_state(arm='A',state=0)#state=0 下伺服
robot.send_cmd()
robot.release_robot()