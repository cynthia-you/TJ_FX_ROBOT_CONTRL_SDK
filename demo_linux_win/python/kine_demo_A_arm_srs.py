from fx_kine import Marvin_Kine
import time

'''#################################################################
该DEMO 为机器人计算全接口 

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
使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
'''
ini_result=kk.load_config(config_path='srs.MvKDCfg')
time.sleep(0.5)
# print(ini_result)
# print(type(ini_result['TYPE'][0]))
# print(type(ini_result['DH'][0]))
# print(type(ini_result['PNVA'][0]))
# print(type(ini_result['BD'][0]))


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





'''
正解与逆解
    可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
    关节正解到末端在基坐标下的位置和姿态
    #一定要确认robot_serial是左臂0 还是右臂1
'''
#正解
fk_mat=kk.fk(robot_serial=0,joints=[10,20,30,40,50,60,70])
#逆解
ik_result_structure=kk.ik(robot_serial=0,pose_mat=fk_mat,ref_joints=[10,20,30,40,50,60,70])
print(f'ik joints:{ik_result_structure.m_Output_RetJoint.to_list()}')
print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure.m_Output_IsOutRange}')
print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure.m_Output_IsDeg[:]}')
print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_IsJntExd}')
print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure.m_Output_JntExdTags[:]}')
time.sleep(0.5)
#逆解优化
#计算末端位姿不变、改变零空间（臂角方向）的逆运动学
ik_nsp_result_structure=kk.ik_nsp(robot_serial=0,pose_mat=fk_mat,ref_joints=[10,20,30,40,50,60,70],
                                  zsp_type=0,zsp_para=[0,0,0,0,0,0],zsp_angle=1,dgr=[0.05,0.05])
print(f'ik_nsp joints:{ik_nsp_result_structure.m_Output_RetJoint.to_list()}')
print(f'ik_nsp 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsOutRange}')
print(f'ik_nsp 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_nsp_result_structure.m_Output_IsDeg[:]}')
print(f'ik_nsp 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_IsJntExd}')
print(f'ik_nsp 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_nsp_result_structure.m_Output_JntExdTags[:]}')
time.sleep(0.5)
print('-'*50)



'''
直线规划（MOVL）
    #一定要确认robot_serial是左臂0 还是右臂1
'''
# start = [-313.420276,-298.597470,582.633149,169.521628,28.403171,-95.504266]
# end = [-303.420276,-298.597470,582.633149,169.521628,28.403171,-95.504266 ]
# tag_movl=kk.movL(robot_serial=0,start_xyzabc=start,end_xyzabc=end,ref_joints=[10,20,30,40,50,60,70],vel=100,acc=100,save_path='test.txt')
pose_6d_1=kk.mat4x4_to_xyzabc(pose_mat=fk_mat) #用关节[10,20,30,40,50,60,70]正解的姿态转XYZABC
pose_6d_2=pose_6d_1.copy()
pose_6d_2[0]+=10# X方向移动10厘米
tag_movl=kk.movL(robot_serial=0,start_xyzabc=pose_6d_1,end_xyzabc=pose_6d_2,ref_joints=[10,20,30,40,50,60,70],vel=100,acc=100,save_path='test11.txt')
time.sleep(0.5)
print('-'*50)


