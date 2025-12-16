from SDK_PYTHON.fx_kine import Marvin_Kine
import time
import os

current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)

'''#################################################################
该DEMO 为机器人计算逆解失败总结
'''#################################################################

'''实列化计算'''
kk=Marvin_Kine()

'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result=kk.load_config(arm_type=0,config_path=os.path.join(current_path,'ccs_m6.MvKDCfg'))
print(ini_result)
print('-'*50)

'''
初始化动力学
'''
initial_kine_tag=kk.initial_kine(
                                 robot_type=ini_result['TYPE'][0],
                                 dh=ini_result['DH'][0],
                                 pnva=ini_result['PNVA'][0],
                                 j67=ini_result['BD'][0])
print('-'*50)


'''
#逆向解失败情况1: 四关节为0
'''
fk_mat = kk.fk(robot_serial=0, joints=[10, 10, 10, 0, 10, 10, 10])
ik_result_structure = kk.ik(robot_serial=0, pose_mat=fk_mat, ref_joints=[10, 10, 10, 0, 10, 10, 10])
if ik_result_structure:
    print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
    print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
    print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
    print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
    print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')
else:
    print('NO ik results')


'''
#逆向解失败情况2: 超可达空间
'''
pose_6d=[1000,500,300,0,0,0]
mat=kk.xyzabc_to_mat4x4(pose_6d)
ik_result_structure = kk.ik(robot_serial=0, pose_mat=mat, ref_joints=[10, 10, 10, 10, 10, 10, 10])
if ik_result_structure:
    print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
    print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
    print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
    print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
    print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')
else:
    print('NO ik results')
