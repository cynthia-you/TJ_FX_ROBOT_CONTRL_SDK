from SDK_PYTHON.fx_kine import Marvin_Kine
import os
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)


'''#################################################################
该DEMO 为机器人计算SDK 功能模块完整演示脚本

使用可以全部运行。
    注意 
    实列化计算
    配置导入
    初始化动力学
    是前置操作，其余所有接口调用前必须这三个接口先调用以初始化信息到缓存
     
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
末端工具运动学设置与删除
'''
#设置
tool=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]#改成工具真实参数
tag1=kk.set_tool_kine(tool_mat=tool)
#删除
tag2=kk.remove_tool_kine()
print('-'*50)

'''
正解与逆解
    可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
    关节正解到末端在基坐标下的位置和姿态
'''
#正解
fk_mat=kk.fk(joints=[10,10,10,10,10,10,10])
if fk_mat:
    print(f'fk matrix:{fk_mat}')
print('-'*50)
#逆解
ik_result_structure=kk.ik(pose_mat=fk_mat,ref_joints=[10,10,10,10,10,10,10])
if ik_result_structure:
    print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
    print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
    print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
    print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
    print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')
else:
    print('NO ik results')
print('-'*50)

#逆解优化
#计算末端位姿不变、改变零空间（臂角方向）的逆运动学
ik_nsp_result_structure=kk.ik_nsp(pose_mat=fk_mat,ref_joints=[10,10,10,10,10,10,10],
                                  zsp_type=1,zsp_para=[0,1,0,0,0,0],zsp_angle=10,dgr=[0.05,0.05])
if ik_nsp_result_structure:
    print(f'ik_nsp joints:{ik_nsp_result_structure.m_Output_RetJoint.to_list()}')
    print(f'ik_nsp 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsOutRange}')
    print(f'ik_nsp 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_nsp_result_structure.m_Output_IsDeg[:]}')
    print(f'ik_nsp 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsJntExd}')
    print(f'ik_nsp 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_JntExdTags[:]}')
else:
    print('NO ik_nsp results')
print('-'*50)

'''
计算雅可比矩阵
'''
#计算雅可比矩阵
jts2jacb_result=kk.joints2JacobMatrix(joints=[10,10,10,10,10,10,10])
if jts2jacb_result:
    print(f'jacobin matrix:{jts2jacb_result}')
print('-'*50)


'''
直线规划（MOVL）
    特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
'''
pose_6d_1=kk.mat4x4_to_xyzabc(pose_mat=fk_mat) #用关节[10,20,30,40,50,60,70]正解的姿态转XYZABC
print(f'6d_pose_1:{pose_6d_1}')
pose_6d_2=pose_6d_1.copy()
pose_6d_2[0]+=10# X方向移动10mm
print(f'6d_pose_2:{pose_6d_2}')
tag_movl=kk.movL(start_xyzabc=pose_6d_1,end_xyzabc=pose_6d_2,ref_joints=[10,10,10,10,10,10,10],vel=100,acc=100,save_path='test.txt')
if tag_movl:
    print('movL success')
print('-'*50)

'''
直线规划（MOVL KeepJ）
    特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
'''
fk_mat=kk.fk(joints=[-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995])
tag_movlkj=kk.movL_KeepJ(start_joints=[-5.918, -35.767, 49.494, -68.112, -90.699, 49.211, -23.995],
                       end_joints=[-26.908 ,-91.109, 74.502 ,-88.083, -93.599 ,17.151, -13.602],vel=20,save_path='testkj.txt')
if tag_movlkj:
    print('movL_KeepJ success')
print('-'*50)

'''
工具动力学参数辨识
    #一定确认robot_type（int）参数代表的机型： 1:CCS机型，2:SRS机型
    ！！！目前仅支持横装方式的辨识！！！
    #检查数据是否有问题！
'''
dyn_para = kk.identify_tool_dyn(robot_type=1, ipath='LoadData_ccs_right/LoadData/')
if type(dyn_para)==str:
    print('error:',dyn_para)
else:
    print(f'mass(KG):{dyn_para[0]}')
    print(f'mcp(x,y,z) mm:{dyn_para[1:4]}')
    print(f'I(ixx,ixy,ixz,iyy,iyz,izz):{dyn_para[4:]}')
print('-'*50)

