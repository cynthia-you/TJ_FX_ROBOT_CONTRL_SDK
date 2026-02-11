import sys
import os
import time
import logging

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine, FX_InvKineSolvePara, convert_to_8x8_matrix
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS

# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)  # 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''#################################################################
该DEMO 为机器人以笛卡尔阻抗50HZ频率执行在线规划的完整演示脚本

MOVLA在线直线规划步骤：
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功
    开启日志以便检查
    设置阻抗参数
    设置扭矩模式,笛卡尔阻抗模式,速度加速度百分比
    订阅数据查看是否设置
    运行到起始点位
    实列化计算并导入机型DH数据并初始化计算接口
    将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    起点的末端位置姿态矩阵转为XYZABC
    定义直线结束点的XYZABC，showcase是规划了末端Z方向移动100毫米
    运行在线规划,规划点位为500HZ， 下采样为50HZ执行


'''  #################################################################

'''初始化订阅数据的结构体'''
dcss = DCSS()

'''初始化机器人接口'''
robot = Marvin_Robot()

'''查验连接是否成功'''
init = robot.connect('192.168.1.190')
if init == 0:
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
robot.log_switch('1')  # 全局日志开关
robot.local_log_switch('1')  # 主要日志

'''设置阻抗参数'''
robot.clear_set()
robot.set_cart_kd_params(arm='A', K=[4000, 6000, 6000, 200, 200, 200, 20], D=[0.5, 0.5, 0.5, 0.3, 0.3, 0.3, 1],
                         type=2)
robot.send_cmd()
time.sleep(0.5)

'''设置扭矩模式,笛卡尔阻抗模式'''
robot.clear_set()
robot.set_state(arm='A', state=3)  # state=3扭矩模式
robot.set_impedance_type(arm='A', type=2)  # type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)


# 走大动作用小速度
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=20, AccRatio=20)
robot.send_cmd()
time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data = robot.subscribe(dcss)
logger.info(f"current state{sub_data['states'][0]['cur_state']}")
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
logger.info(f'set cart k={sub_data["inputs"][0]["cart_k"][:]}, d={sub_data["inputs"][0]["cart_d"][:]}')
logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')


'''运行到起始点位'''
robot.clear_set()
joint_cmd_1 = [0] * 7
robot.set_joint_cmd_pose(arm='A', joints=joint_cmd_1)
robot.send_cmd()
time.sleep(4)  # 预留运动时间

'''运行到起始点位'''
robot.clear_set()
joint_cmd_1 = [21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]
robot.set_joint_cmd_pose(arm='A', joints=joint_cmd_1)
robot.send_cmd()
time.sleep(4)  # 预留运动时间

'''订阅数据查看是否到位'''
sub_data = robot.subscribe(dcss)
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')

'''实列化计算'''
kk = Marvin_Kine()
'''关闭日志'''
kk.log_switch(0)  # 0 off, 1 on
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result = kk.load_config(arm_type=0, config_path=os.path.join(current_path, 'ccs_m6_40.MvKDCfg'))
print(ini_result)

'''
初始化动力学
'''
initial_kine_tag = kk.initial_kine(
    robot_type=ini_result['TYPE'][0],
    dh=ini_result['DH'][0],
    pnva=ini_result['PNVA'][0],
    j67=ini_result['BD'][0])

'''
    将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    起点的末端位置姿态矩阵转为XYZABC
    定义直线结束点的XYZABC，showcase是规划了末端Z方向移动100毫米
'''
fk_mat = kk.fk(joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
if fk_mat:
    print(f'fk matrix:{fk_mat}')

pose_6d_1 = kk.mat4x4_to_xyzabc(pose_mat=fk_mat)  # 用关节[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]正解的姿态转XYZABC
print(f'6d_pose_1:{pose_6d_1}')



'''
指定直线规划结束点位姿态
这是一个非常灵活的接口：
位置的平移：基于起始点末端的XYZ的偏移
姿态的旋转：可以基于基座改变末端的旋转，也可以根据末端当前方向改变旋转
• 旋转类型大致分为三种：
            1.FX_ROT_NULL：不进行姿态变换
            2.FX_ROT_EULER_xxx:欧拉角变换，基于末端坐标系旋转
            3.FX_ROT_FIXED_xxx:固定角变换，基于基坐标系旋转
            enum FX_ROTATION_TYPE
            {
                FX_ROT_NULL = 11,

                FX_ROT_EULER_XYZ = 101,
                FX_ROT_EULER_XZY = 102,
                FX_ROT_EULER_YXZ = 103,
                FX_ROT_EULER_YZX = 104,
                FX_ROT_EULER_ZXY = 105,
                FX_ROT_EULER_ZYX = 106,

                FX_ROT_EULER_XYX = 107,
                FX_ROT_EULER_XZX = 108,
                FX_ROT_EULER_YXY = 109,
                FX_ROT_EULER_YZY = 110,
                FX_ROT_EULER_ZXZ = 111,
                FX_ROT_EULER_ZYZ = 112,

                FX_ROT_FIXED_XYZ = 201,
                FX_ROT_FIXED_XZY = 202,
                FX_ROT_FIXED_YXZ = 203,
                FX_ROT_FIXED_YZX = 204,
                FX_ROT_FIXED_ZXY = 205,
                FX_ROT_FIXED_ZYX = 206,

                FX_ROT_FIXED_XYX = 207,
                FX_ROT_FIXED_XZX = 208,
                FX_ROT_FIXED_YXY = 209,
                FX_ROT_FIXED_YZY = 210,
                FX_ROT_FIXED_ZXZ = 211,
                FX_ROT_FIXED_ZYZ = 212,
            };
'''
#在这个showcase中，指定偏移基于当前的末端位置的X负方向移动25mm;指定旋转基于基坐标系，Rx旋转45度.
end_xyzabc=kk.calculate_end_xyzabc(start_xyzabc=pose_6d_1,pose_offset=[-100,0,0], rot_type=206, angle_param=[45,0,0])


'''直线规划（MOVLA）
    运行在线规划,规划点位为500HZ， 下采样为50HZ执行
'''
# ONLINE
points = kk.movLA(start_xyzabc=pose_6d_1, end_xyzabc=end_xyzabc,
                  ref_joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68], vel=200, acc=200)
print(f"Got {len(points)} planning points")
# 走小动作用大速度
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=50, AccRatio=50)
robot.send_cmd()
time.sleep(0.5)
if points:
    # 500hz-->200hz
    for i in range(0, len(points), 1):
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A', joints=points[i])
        robot.send_cmd()
        time.sleep(0.01)  # 500hz:sleep 2ms,  50hz:sleep 20ms， 200hz:sleep 5ms, 100hz:sleep 10ms,



# 走大动作用小速度
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=20, AccRatio=20)
robot.send_cmd()

time.sleep(0.5)
'''运行到起始点位'''
robot.clear_set()
joint_cmd_1 = [21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]
robot.set_joint_cmd_pose(arm='A', joints=joint_cmd_1)
robot.send_cmd()
time.sleep(3)  # 预留运动时间


#在这个showcase中，指定偏移基于当前的末端位置的X负方向移动100mm;指定旋转基于末端坐标系，Rx旋转45度.
end_xyzabc=kk.calculate_end_xyzabc(start_xyzabc=pose_6d_1,pose_offset=[-100,0,0], rot_type=106, angle_param=[45,0,0])


'''直线规划（MOVLA）
    运行在线规划,规划点位为500HZ， 下采样为50HZ执行
'''
# ONLINE
points = kk.movLA(start_xyzabc=pose_6d_1, end_xyzabc=end_xyzabc,
                  ref_joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68], vel=200, acc=200)
print(f"Got {len(points)} planning points")

# 走小动作用大速度
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=50, AccRatio=50)
robot.send_cmd()

if points:
    # 500hz-->200hz
    for i in range(0, len(points), 1):
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A', joints=points[i])
        robot.send_cmd()
        time.sleep(0.01)  # 500hz:sleep 2ms,  50hz:sleep 20ms， 200hz:sleep 5ms,



# 走大动作用小速度
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=20, AccRatio=20)
robot.send_cmd()
time.sleep(0.5)


'''运行到起始点位'''
robot.clear_set()
joint_cmd_1 = [21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]
robot.set_joint_cmd_pose(arm='A', joints=joint_cmd_1)
robot.send_cmd()
time.sleep(3)  # 预留运动时间


#在这个showcase中，指定偏移基于当前的末端位置的X负方向移动100mm
end_xyzabc=kk.calculate_end_xyzabc(start_xyzabc=pose_6d_1,pose_offset=[-100,0,0], rot_type=11, angle_param=[0,0,0])

'''直线规划（MOVLA）
    运行在线规划,规划点位为500HZ， 下采样为50HZ执行
'''
# ONLINE
points = kk.movLA(start_xyzabc=pose_6d_1, end_xyzabc=end_xyzabc,
                  ref_joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68], vel=200, acc=200)
print(f"Got {len(points)} planning points")

# 走小动作用大速度
robot.clear_set()
robot.set_vel_acc(arm='A', velRatio=50, AccRatio=50)
robot.send_cmd()

if points:
    # 500hz-->200hz
    for i in range(0, len(points), 1):
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A', joints=points[i])
        robot.send_cmd()
        time.sleep(0.01)  # 500hz:sleep 2ms,  50hz:sleep 20ms， 200hz:sleep 5ms,

'''下使能'''
robot.clear_set()
robot.set_state(arm='A', state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()