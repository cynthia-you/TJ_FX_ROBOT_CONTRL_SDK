
from fx_kine import Marvin_Kine
import os
import random
import logging
import time
'''#################################################################
该DEMO 为关节超限和优化办法

描述：给定一个当前关节[32,-64,72,59,-107,-30,58]，不确定是否超限，想要在这组关节下，末端朝Z的负向方向移动100毫米。

使用逻辑
    1 初始化机器人计算接口
    2 配置导入
    3 初始化动力学
    4 目标关节正解到末端位姿矩阵
    5 末端位姿矩阵逆解到关节角度，以判断是否逆解有异常
    6 如果有超限，不改变目标末端位姿情况下，迭代改变臂角，直到逆解的关节正常
    7 新的关节正解后转到XYZABC，设置想要的变化，转回位姿矩阵，求逆解，同步骤6 直到逆解的关节正常。
验证代码查看eval.pp
'''#################################################################

logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级

current_path=os.getcwd()

'''初始化机器人计算接口'''
kk = Marvin_Kine()

'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
'''
ini_result=kk.load_config(config_path='ccs.MvKDCfg')
time.sleep(0.5)

'''
初始化动力学
#一定要确认robot_serial是左臂0 还是右臂1
本DEMO中仅示例了单臂的计算
如果人形，选择右臂， 则为：
initial_kine_tag=kk.initial_kine(robot_serial=1,
                                 robot_type=ini_result['TYPE'][1],
                                 dh=ini_result['DH'][1],
                                 pnva=ini_result['PNVA'][1],
                                 j67=ini_result['BD'][1])
其余有robot_serial参数作为输入的都要确认双臂还是单臂，不再赘述。
'''
initial_kine_tag=kk.initial_kine(robot_serial=0,
                                 robot_type=ini_result['TYPE'][0],
                                 dh=ini_result['DH'][0],
                                 pnva=ini_result['PNVA'][0],
                                 j67=ini_result['BD'][0])


time.sleep(0.5)
print('-'*50)

'''正解'''
joints=[32,-64,72,59,-107,-30,58]
fk_mat=kk.fk(robot_serial=0,joints=joints)
xyzabc1=kk.mat4x4_to_xyzabc(pose_mat=fk_mat)
ik_joints=[0]*7
'''逆解'''
ik_result_structure=kk.ik(robot_serial=0,pose_mat=fk_mat,ref_joints=joints)
print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')

if ik_result_structure.m_Output_IsDeg[:]:
    a=0
    # 计算末端位姿不变、改变零空间（臂角方向）的逆运动学
    nsp_angle=0
    while a==0:

        nsp_angle+=1
        logger.info(f'----iter1:{nsp_angle}---')
        ik_nsp_result_structure = kk.ik_nsp(robot_serial=0, pose_mat=fk_mat, ref_joints=joints,
                                            zsp_type=0, zsp_para=[0, 0, 0, 0, 0, 0], zsp_angle=nsp_angle, dgr=[0.05, 0.05])
        print(f'ik_nsp joints:{ik_nsp_result_structure.m_Output_RetJoint.to_list()}')
        print(f'ik_nsp 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsOutRange}')
        print(f'ik_nsp 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_nsp_result_structure.m_Output_IsDeg[:]}')
        print(f'ik_nsp 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsJntExd}')
        print(f'ik_nsp 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_JntExdTags[:]}')

        if not True in ik_nsp_result_structure.m_Output_JntExdTags[:]:
            iter_joints1=ik_nsp_result_structure.m_Output_RetJoint.to_list()
            logger.info(f'find nsp joints:{iter_joints1}')
            a=1
        if a>100:
            logger.error(f'error, not find valid nsp angle')

new_mat=kk.fk(robot_serial=0,joints=iter_joints1)
xyzabc=kk.mat4x4_to_xyzabc(pose_mat=new_mat)
logger.info(f'current xyzabc:{xyzabc}')

target_xyzabc=xyzabc.copy()
target_xyzabc[2]-=100
logger.info(f'taget xyzabc:{target_xyzabc}')
target_mat4x4=kk.xyzabc_to_mat4x4(xyzabc=target_xyzabc)
logger.info(f'target mat:{target_mat4x4}')



ik_result_structure=kk.ik(robot_serial=0,pose_mat=target_mat4x4,ref_joints=iter_joints1)
print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')

if ik_result_structure.m_Output_IsDeg[:]:

    a=0
    # 计算末端位姿不变、改变零空间（臂角方向）的逆运动学
    nsp_angle=0
    while a==0:
        nsp_angle+=1
        logger.info(f'----iter2:{nsp_angle}---')
        ik_nsp_result_structure = kk.ik_nsp(robot_serial=0, pose_mat=new_mat, ref_joints=iter_joints1,
                                            zsp_type=0, zsp_para=[0, 0, 0, 0, 0, 0], zsp_angle=nsp_angle, dgr=[0.05, 0.05])
        print(f'ik_nsp joints:{ik_nsp_result_structure.m_Output_RetJoint.to_list()}')
        print(f'ik_nsp 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsOutRange}')
        print(f'ik_nsp 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_nsp_result_structure.m_Output_IsDeg[:]}')
        print(f'ik_nsp 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsJntExd}')
        print(f'ik_nsp 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_JntExdTags[:]}')

        if not True in ik_nsp_result_structure.m_Output_JntExdTags[:]:
            iter_joints1=ik_nsp_result_structure.m_Output_RetJoint.to_list()
            logger.info(f'find nsp joints:{iter_joints1}')
            a=1
        if a>100:
            logger.error(f'error, not find valid nsp angle')







