import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS, update_text_file_simple
from SDK_PYTHON.fx_kine import Marvin_Kine
import time
import logging


'''#################################################################
该DEMO 为保存工具参数和更新工具参数案例

使用逻辑
    1 初始化订阅数据的结构体
    2 初始化机器人接口
    3 查验连接是否成功,失败程序直接退出
    4 开启日志以便检查
    4 确认控制器是否有保存工具信息，如果有：加载保存的数据并生效； 如果无：初始化一个工具文本，并更新工具参数和设置生效
    5 更新工具和设置生效  
    6 任务完成,下使能,释放内存使别的程序或者用户可以连接机器人
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

'''初始化运动计算接口'''
kk1 = Marvin_Kine()
ini_result1 = kk1.load_config(arm_type=0, config_path='ccs_m6.MvKDCfg')
initial_kine_tag1 = kk1.initial_kine(robot_type=ini_result1['TYPE'][0],
                                     dh=ini_result1['DH'][0],
                                     pnva=ini_result1['PNVA'][0],
                                     j67=ini_result1['BD'][0])

kk2 = Marvin_Kine()
ini_result2 = kk2.load_config(arm_type=1, config_path='ccs_m6.MvKDCfg')
initial_kine_tag2 = kk2.initial_kine(robot_type=ini_result2['TYPE'][0],
                                         dh=ini_result2['DH'][0],
                                         pnva=ini_result2['PNVA'][0],
                                         j67=ini_result2['BD'][0])

if not ini_result1 or not ini_result2:
    logger.error('error, *.MvKDCfg 出错，请检查文件内容或路径')
    exit(0)


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

'''3 开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志


'''4 确认控制器是否有保存工具信息，如果有：加载保存的数据并生效； 如果无：初始化一个工具文本，并更新工具参数和设置生效'''
tool_result=robot.get_tool_info()
print(f'tool_result:{tool_result}')
if tool_result == 0:
    logger.info('success, 机器人未设置工具信息，如果带工具，请设置工具信息')
    # 初始化工具保存文件
    lines = ['0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n',
             '0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n']
    with open('tool_dyn_kine.txt', 'w', encoding='utf-8') as file:
        file.writelines(lines)
    file.close()

    '''设置或者修改左臂的动力学信息和运动学信息'''
    # tool_left_dynamic工具动力学信息,长度为10  m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz
    # m， 质量 单位千克
    # mcp_x,mcp_y,mcp_z 工具的质心坐标，相对于法兰的偏移， 单位毫米
    # ixx,ixy,ixz,iyy,iyz,izz 转动惯量， 可以不填。
    tool_left_dynamic = [0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # 工具运动学信息 长度为6 xyzabc， 工具相对末端法兰的位置偏移和姿态旋转
    tool_left_kinematics = [0, 0, 0.1, 0, 0, 0]
    full_tool_left = tool_left_dynamic + tool_left_kinematics
    # 更新修改控制器内的工具信息
    update_text_file_simple('A', full_tool_left, 'tool_dyn_kine.txt')
    time.sleep(0.5)
    # 设置工具
    robot.set_tool(arm='A', dynamicParams=tool_left_dynamic, kineParams=tool_left_kinematics)

    '''设置或者修改右臂的动力学信息和运动学信息'''
    # tool_right_dynamic工具动力学信息,长度为10  m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz
    # m， 质量 单位千克
    # mcp_x,mcp_y,mcp_z 工具的质心坐标，相对于法兰的偏移， 单位毫米
    # ixx,ixy,ixz,iyy,iyz,izz 转动惯量， 可以不填。
    tool_right_dynamic = [0.11, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # 工具运动学信息 长度为6 xyzabc， 工具相对末端法兰的位置偏移和姿态旋转
    tool_right_kinematics = [0, 0, 0.11, 0, 0, 0]
    full_tool_right = tool_right_dynamic + tool_right_kinematics

    # 更新修改控制器内的工具信息
    update_text_file_simple('B', full_tool_right, 'tool_dyn_kine.txt')
    time.sleep(0.5)
    # 设置工具
    robot.set_tool(arm='B', dynamicParams=tool_right_dynamic, kineParams=tool_right_kinematics)

    # 保存两个臂的运动学和动力学信息到控制器下，
    robot.send_file('tool_dyn_kine.txt', os.path.join('/home/fusion/', 'tool_dyn_kine.txt'))
    time.sleep(1)


elif tool_result == -1:
    logger.error('failed,机器人连接成功. 工具信息文件tool_dyn_kine.txt有误。')

elif type(tool_result) == tuple:
    arm_side = tool_result[0]
    if arm_side == 'line1':
        logger.info('success,机器人已设置左臂工具信息，右臂未设置.')
        tool_dyn_l = ""
        tool_kine_l = ""
        for i in range(10):
            tool_dyn_l += f"{tool_result[1][i]:.3f},"
            if i < 6:
                tool_kine_l += f"{tool_result[1][10 + i]:.3f},"
        tool_dyn_l = tool_dyn_l.rstrip(", ")
        tool_kine_l = tool_kine_l.rstrip(", ")
        #控制器设置生效
        robot.set_tool(arm='A', dynamicParams=tool_result[1][:10], kineParams=tool_result[1][10:])
        #计算解算设置生效
        tool_mat = kk1.xyzabc_to_mat4x4(tool_result[1][10:])
        kk1.set_tool_kine(tool_mat=tool_mat)

    elif arm_side == 'line2':
        logger.info('success, 机器人已设置右臂工具信息，左臂未设置.')
        tool_dyn_r = ""
        tool_kine_r = ""
        for i in range(10):
            tool_dyn_r += f"{tool_result[1][i]:.3f},"
            if i < 6:
                tool_kine_r += f"{tool_result[1][10 + i]:.3f},"
        tool_dyn_r = tool_dyn_r.rstrip(", ")
        tool_kine_r = tool_kine_r.rstrip(", ")
        #控制器设置生效
        robot.set_tool(arm='B', dynamicParams=tool_result[1][:10], kineParams=tool_result[1][10:])
        #计算解算设置生效
        tool_mat1 = kk2.xyzabc_to_mat4x4(tool_result[1][10:])
        kk2.set_tool_kine(tool_mat=tool_mat1)
else:
    logger.info('success, 机器人已设置左右臂的工具信息.')
    if isinstance(tool_result[0], list):
        tool_dyn_l = ""
        tool_dyn_r = ""
        tool_kine_l = ""
        tool_kine_r = ""
        for i in range(10):
            tool_dyn_l += f"{tool_result[0][i]:.3f},"
            tool_dyn_r += f"{tool_result[1][i]:.3f},"
            if i < 6:
                tool_kine_l += f"{tool_result[0][10 + i]:.3f},"
                tool_kine_r += f"{tool_result[1][10 + i]:.3f},"
        tool_dyn_l = tool_dyn_l.rstrip(", ")
        tool_dyn_r = tool_dyn_r.rstrip(", ")
        tool_kine_l = tool_kine_l.rstrip(", ")
        tool_kine_r = tool_kine_r.rstrip(", ")

        # 从控制器加载的工具信息
        robot.set_tool(arm='A', dynamicParams=tool_result[0][:10], kineParams=tool_result[0][10:])
        robot.set_tool(arm='B', dynamicParams=tool_result[1][:10], kineParams=tool_result[1][10:])

        tool_mat = kk1.xyzabc_to_mat4x4(tool_result[0][10:])
        tool_mat1 = kk2.xyzabc_to_mat4x4(tool_result[1][10:])
        kk1.set_tool_kine(tool_mat=tool_mat)
        kk2.set_tool_kine(tool_mat=tool_mat1)

'''5 更新工具和设置生效
如果工具信息需要改变。使用该步骤，设置生效。
'''
#先获取控制器缓存文件到本地
tool_result=robot.get_tool_info()
print(f'tool_result:{tool_result}')
#再修改左右臂的数据
'''设置或者修改左臂的动力学信息和运动学信息'''
# tool_left_dynamic工具动力学信息,长度为10  m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz
# m， 质量 单位千克
# mcp_x,mcp_y,mcp_z 工具的质心坐标，相对于法兰的偏移， 单位毫米
# ixx,ixy,ixz,iyy,iyz,izz 转动惯量， 可以不填。
tool_left_dynamic = [0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 工具运动学信息 长度为6 xyzabc， 工具相对末端法兰的位置偏移和姿态旋转
tool_left_kinematics = [0, 0, 0.2, 0, 0, 0]
full_tool_left = tool_left_dynamic + tool_left_kinematics
# 更新修改控制器内的工具信息
update_text_file_simple('A', full_tool_left, 'tool_dyn_kine.txt')
time.sleep(0.5)
# 设置工具
robot.set_tool(arm='A', dynamicParams=tool_left_dynamic, kineParams=tool_left_kinematics)
'''设置或者修改右臂的动力学信息和运动学信息'''
# tool_right_dynamic工具动力学信息,长度为10  m,mcp_x,mcp_y,mcp_z,ixx,ixy,ixz,iyy,iyz,izz
# m， 质量 单位千克
# mcp_x,mcp_y,mcp_z 工具的质心坐标，相对于法兰的偏移， 单位毫米
# ixx,ixy,ixz,iyy,iyz,izz 转动惯量， 可以不填。
tool_right_dynamic = [0.21, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# 工具运动学信息 长度为6 xyzabc， 工具相对末端法兰的位置偏移和姿态旋转
tool_right_kinematics = [0, 0, 0.21, 0, 0, 0]
full_tool_right = tool_right_dynamic + tool_right_kinematics
# 更新修改控制器内的工具信息
update_text_file_simple('B', full_tool_right, 'tool_dyn_kine.txt')
time.sleep(0.5)
# 设置工具
robot.set_tool(arm='B', dynamicParams=tool_right_dynamic, kineParams=tool_right_kinematics)
# 保存两个臂的运动学和动力学信息到控制器下，
robot.send_file('tool_dyn_kine.txt', os.path.join('/home/fusion/', 'tool_dyn_kine.txt'))
time.sleep(1)



'''6 任务完成 释放连接'''
robot.release_robot()
