import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine,FX_InvKineSolvePara,convert_to_8x8_matrix

'''
#################################################################
该DEMO 为机器人在约束工作构型下的逆解和调整构型的演示脚本   
#################################################################
'''
print('-'*50)
print('**** A arm ****')
print('-'*50)
'''实列化计算'''
kk=Marvin_Kine() # LEFT ARM
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result1=kk.load_config(arm_type=0,config_path=os.path.join(current_path,'ccs_m6.MvKDCfg'))
print(ini_result1)

'''
初始化动力学
'''
initial_kine_tag=kk.initial_kine(
                                 robot_type=ini_result1['TYPE'][0],
                                 dh=ini_result1['DH'][0],
                                 pnva=ini_result1['PNVA'][0],
                                 j67=ini_result1['BD'][0])
'''
逆解到预期构型
1. 确定目标点位下的位置和姿态矩阵(案例里没有加工具,用正解得到末端法兰的位姿)
2. 设置工作参考构型(提前拖动到一个满意的构型,以该构型的臂角矩阵的X方向信息作为所有逆解的臂角坐标引导)
3. 设置逆解基础参数(末端位置姿态和参考位置姿态)以及零空间参数(将步骤2的nsp_mat1的第一列作为逆解的了零空间方向信息)
4. 逆解
5. 优化,这个胳膊肘我不太满意,想要胳膊肘往上抬,左臂的臂角的Z向量是从手腕到肩的,X向量是从胳膊肘部垂直指向Z向量,根据右手法则,我想胳膊肘往上翘,顺时针角度要加,反之逆时针角度要减
'''
# 1. 确定目标点位的位置和姿态矩阵
fk_mat=kk.fk(joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
if fk_mat:
    print(f'fk matrix:{fk_mat}')
# 2. 设置工作参考构型
fk_mat22,nsp_mat=kk.fk_nsp(joints=[44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1]) #该点位模拟左手在桌面工作的构型
if fk_mat22:
    print(f'nsp matrix:{nsp_mat}')
#3. 设置逆解基础参数以及零空间参数
sp=FX_InvKineSolvePara()
mat16=kk.mat4x4_to_mat1x16(fk_mat)
sp.set_input_ik_target_tcp(mat16)
sp.set_input_ik_ref_joint([44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1])
sp.set_input_ik_zsp_type(1)
sp.set_input_ik_zsp_para([nsp_mat[0][0],nsp_mat[1][0],nsp_mat[2][0],0,0,0])#将步骤2的nsp_mat的第一列作为逆解的了零空间方向信息
#4. 逆解
ik_result_structure=kk.ik(structure_data=sp)
if ik_result_structure:
    print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
    print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
    print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
    print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
    print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')
    print(f'ik 各关节正限制: {ik_result_structure.m_Output_RunLmtP.to_list()}')
    print(f'ik 各关节负限制: {ik_result_structure.m_Output_RunLmtN.to_list()}')
    print(f'number of ik results:{ik_result_structure.m_OutPut_Result_Num}')
    print(f'all ik results:{convert_to_8x8_matrix(ik_result_structure.m_OutPut_AllJoint.to_list())}')
else:
    print('NO ik results')
#5. 优化
sp.set_input_zsp_angle(15)#胳膊往上翘15度
sp.set_dgr1(0.05)
sp.set_dgr2(0.05)
ik_nsp_result_structure=kk.ik_nsp(sturcture_data=sp)
if ik_nsp_result_structure:
    print(f'ik_nsp joints:{ik_nsp_result_structure.m_Output_RetJoint.to_list()}')
    print(f'ik_nsp 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsOutRange}')
    print(f'ik_nsp 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_nsp_result_structure.m_Output_IsDeg[:]}')
    print(f'ik_nsp 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsJntExd}')
    print(f'ik_nsp 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_JntExdTags[:]}')
    print(f'number of ik_nsp results:{ik_nsp_result_structure.m_OutPut_Result_Num}')
    print(f'all ik_nsp results:{convert_to_8x8_matrix(ik_nsp_result_structure.m_OutPut_AllJoint.to_list())}')
else:
    print('NO ik_nsp results')

'''##################################################################################################################'''
print('-'*50)
print('**** B arm ****')
print('-'*50)
kk1=Marvin_Kine() # RIGHT ARM
'''
配置导入
!!! 非常重要！！！
使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
一定要确认arm_type是左臂0 还是右臂1
'''
ini_result1=kk1.load_config(arm_type=1,config_path=os.path.join(current_path,'ccs_m6.MvKDCfg'))
print(ini_result1)

'''
初始化动力学
'''
initial_kine_tag=kk1.initial_kine(
                                 robot_type=ini_result1['TYPE'][1],
                                 dh=ini_result1['DH'][1],
                                 pnva=ini_result1['PNVA'][1],
                                 j67=ini_result1['BD'][1])
'''
逆解到预期构型
1. 确定目标点位下的位置和姿态矩阵(案例里没有加工具,用正解得到末端法兰的位姿)
2. 设置工作参考构型(提前拖动到一个满意的构型,以该构型的臂角矩阵的X方向信息作为所有逆解的臂角坐标引导)
3. 设置逆解基础参数(末端位置姿态和参考位置姿态)以及零空间参数(将步骤2的nsp_mat1的第一列作为逆解的了零空间方向信息)
4. 逆解
5. 优化,这个胳膊肘我不太满意,想要胳膊肘往上抬,右臂的臂角的Z向量是从手腕到肩的,X向量是从胳膊肘部垂直指向Z向量,根据右手法则,我想胳膊肘往上翘,逆时针角度要减,反之顺时针角度要加
'''
# 1. 确定目标点位的位置和姿态矩阵
fk_mat1=kk1.fk(joints=[-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
if fk_mat1:
    print(f'fk matrix:{fk_mat1}')
# 2. 设置工作参考构型
fk_mat2,nsp_mat1=kk1.fk_nsp(joints=[-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1]) #该点位模拟右手在桌面工作的构型
if fk_mat2:
    print(f'nsp matrix:{nsp_mat1}')
#3. 设置逆解基础参数以及零空间参数
sp=FX_InvKineSolvePara()
mat16=kk1.mat4x4_to_mat1x16(fk_mat1)
sp.set_input_ik_target_tcp(mat16)
sp.set_input_ik_ref_joint([-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1])
sp.set_input_ik_zsp_type(1)
sp.set_input_ik_zsp_para([nsp_mat1[0][0],nsp_mat1[1][0],nsp_mat1[2][0],0,0,0])#将步骤2的nsp_mat1的第一列作为逆解的了零空间方向信息
#4. 逆解
ik_result_structure1=kk1.ik(structure_data=sp)
if ik_result_structure1:
    print(f'ik joints:{ik_result_structure1.m_Output_RetJoint.to_list()}')
    print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsOutRange}')
    print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure1.m_Output_IsDeg[:]}')
    print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsJntExd}')
    print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_JntExdTags[:]}')
    print(f'ik 各关节正限制: {ik_result_structure1.m_Output_RunLmtP.to_list()}')
    print(f'ik 各关节负限制: {ik_result_structure1.m_Output_RunLmtN.to_list()}')
    print(f'number of ik results:{ik_result_structure1.m_OutPut_Result_Num}')
    print(f'all ik results:{convert_to_8x8_matrix(ik_result_structure1.m_OutPut_AllJoint.to_list())}')
else:
    print('NO ik results')
#5. 优化
sp.set_input_zsp_angle(-15)#胳膊往上翘15度
sp.set_dgr1(0.05)
sp.set_dgr2(0.05)
ik_nsp_result_structure1=kk1.ik_nsp(sturcture_data=sp)
if ik_nsp_result_structure1:
    print(f'ik_nsp joints:{ik_nsp_result_structure1.m_Output_RetJoint.to_list()}')
    print(f'ik_nsp 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_nsp_result_structure1.m_Output_IsOutRange}')
    print(f'ik_nsp 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_nsp_result_structure1.m_Output_IsDeg[:]}')
    print(f'ik_nsp 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure1.m_Output_IsJntExd}')
    print(f'ik_nsp 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure1.m_Output_JntExdTags[:]}')
    print(f'number of ik_nsp results:{ik_nsp_result_structure1.m_OutPut_Result_Num}')
    print(f'all ik_nsp results:{convert_to_8x8_matrix(ik_nsp_result_structure1.m_OutPut_AllJoint.to_list())}')
else:
    print('NO ik_nsp results')

'''
下面是一串轨迹来验证使用fk_nsp约束的点位
#左臂
#设定了一个工作的基准参考
[44.04, -62.57, -8.92, -57.21, 1.45, -4.39, 2.1]
#回零
[0,0,0,0,0,0,0]
#目标位置姿态逆解到约束构型
[26.132192947209525, -41.4299312921566, -12.382793333582738, -63.669999968333876, 15.34616386472852, 14.135899381262774, 5.014661729429383]
#调整臂角度
[13.651106491615714, -41.22561001606551, 9.55389260549187, -63.669999968333876, 0.4169924723007425, 15.17290106829154, 12.745643235928943]

#右臂
#设定了一个工作的基准参考
[-44.04, -62.57, 8.92, -57.21, -1.45, -4.39, -2.1]
#回零
[0,0,0,0,0,0,0]
#目标位置姿态逆解到约束构型
[-26.126455465388723, -41.42961256959308, 12.382716991277716, -63.669999968336434, -15.339360903337091, 14.136820316614646, -5.018123071079321]
#调整臂角度
[-13.645291371687428, -41.22530974382568, -9.554071963583754, -63.669999968336434, -0.41009102022740107, 15.172926189678035, -12.749249430475857]
'''



