import sys
import os
import time
import logging
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine,FX_InvKineSolvePara,convert_to_8x8_matrix
from SDK_PYTHON.fx_robot import Marvin_Robot, DCSS
# 配置日志系统
logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

'''#################################################################
该DEMO 为机器人以关节阻抗50HZ频率执行在线规划的完整演示脚本

MOVLA在线直线规划步骤：
    初始化订阅数据的结构体
    初始化机器人接口
    查验连接是否成功
    开启日志以便检查
    设置阻抗参数
    设置扭矩模式,关节阻抗模式,速度加速度百分比
    订阅数据查看是否设置
    运行到起始点位
    实列化计算并导入机型DH数据并初始化计算接口
    将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    起点的末端位置姿态矩阵转为XYZABC
    定义直线结束点的XYZABC，showcase是规划了末端Z方向移动100毫米
    运行在线规划,规划点位为500HZ， 下采样为50HZ执行

     
'''#################################################################

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


'''设置阻抗参数'''
robot.clear_set()
robot.set_joint_kd_params(arm='A',K=[2,2,2,1.6, 1, 1, 1], D=[0.4,0.4,0.17,0.23,0.23,0.2,0.2])#预设参考。
robot.send_cmd()
time.sleep(0.5)

'''设置扭矩模式,关节阻抗模式,速度加速度百分比'''
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.set_impedance_type(arm='A',type=1) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.set_vel_acc(arm='A',velRatio=100, AccRatio=100)
robot.send_cmd()
time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f"current state{sub_data['states'][0]['cur_state']}")
logger.info(f'set vel={sub_data["inputs"][0]["joint_vel_ratio"]}, acc={sub_data["inputs"][0]["joint_acc_ratio"]}')
logger.info(f'set joint k={sub_data["inputs"][0]["joint_k"][:]}, d={sub_data["inputs"][0]["joint_d"][:]}')
logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')


'''运行到起始点位'''
robot.clear_set()
joint_cmd_1=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]
robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
robot.send_cmd()
time.sleep(3) #预留运动时间

'''订阅数据查看是否到位'''
sub_data=robot.subscribe(dcss)
logger.info(f'set joint={sub_data["inputs"][0]["joint_cmd_pos"]}')
logger.info(f'current joint={sub_data["outputs"][0]["fb_joint_pos"]}')

'''实列化计算'''
kk=Marvin_Kine()
'''关闭日志'''
kk.log_switch(0)#0 off, 1 on
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result=kk.load_config(arm_type=0,config_path=os.path.join(current_path,'ccs_m6.MvKDCfg'))
print(ini_result)

'''
初始化动力学
'''
initial_kine_tag=kk.initial_kine(
                                 robot_type=ini_result['TYPE'][0],
                                 dh=ini_result['DH'][0],
                                 pnva=ini_result['PNVA'][0],
                                 j67=ini_result['BD'][0])


'''
    将起点的关节角度通过正运动学得到起点的末端位置姿态矩阵
    起点的末端位置姿态矩阵转为XYZABC
    定义直线结束点的XYZABC，showcase是规划了末端Z方向移动100毫米
'''
fk_mat=kk.fk(joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
if fk_mat:
    print(f'fk matrix:{fk_mat}')

pose_6d_1=kk.mat4x4_to_xyzabc(pose_mat=fk_mat) #用关节[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]正解的姿态转XYZABC
print(f'6d_pose_1:{pose_6d_1}')
pose_6d_2=pose_6d_1.copy()
pose_6d_2[2]+=100# Z方向移动100mm

print(f'6d_pose_2:{pose_6d_2}')

'''直线规划（MOVLA）
    运行在线规划,规划点位为500HZ， 下采样为50HZ执行
'''
#ONLINE
points=kk.movLA(start_xyzabc=pose_6d_1,end_xyzabc=pose_6d_2,ref_joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68],vel=100,acc=100)
print(f"Got {len(points)} planning points")
if points:
    #500hz-->50hz
    for i in range(0,len(points),10):
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A', joints=points[i])
        robot.send_cmd()
        time.sleep(0.002)

'''下使能'''
robot.clear_set()
robot.set_state(arm='A',state=0)
robot.send_cmd()

'''释放机器人内存'''
robot.release_robot()






