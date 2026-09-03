import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
current_file_path = os.path.abspath(__file__)
current_path = os.path.dirname(current_file_path)
from SDK_PYTHON.fx_kine import Marvin_Kine,FX_InvKineSolvePara,convert_to_8x8_matrix


'''#################################################################
该DEMO 为机器人计算SDK 两个手臂同时计算正解和逆解的演示脚本

demo展示三个使用场景:
    场景一：不设置工具坐标系及用户坐标系，正运动学求解是法兰末端基于机械臂基座的位姿矩阵，逆运动学同理；
        1. 实例化两个运动类
        2. 加载各自配置参数
        3. 初始化
        4. 正向运动学和逆向运动学验证


    场景二：只设置工具坐标系，不设置用户坐标系，正运动学求解是工具末端基于机械臂基座的位姿矩阵，逆运动学同理；
        1. 实例化两个运动类
        2. 加载各自配置参数
        3. 初始化
        4. 设置两条臂所带的工具的信息
        5. 正向运动学和逆向运动学验证


    场景三：同时设置工具坐标系及用户坐标系，正运动学求解是工具末端基于用户坐标系的位姿矩阵，逆运动学同理。
        1. 实例化两个运动类
        2. 加载各自配置参数
        3. 初始化
        4. 设置两条臂所带的工具信息和设置用户坐标系
        5. 正向运动学和逆向运动学验证
   
'''#################################################################


def fk_ik_situation_1():
    print("*** 场景一：不设置工具坐标系及用户坐标系")
    '''实列化计算'''
    kk1=Marvin_Kine() # LEFT ARM
    kk2=Marvin_Kine() # RIGHT ARM

    '''
    配置导入
    !!! 非常重要！！！
    使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
        ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
        ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
        srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错
    一定要确认arm_type是左臂0 还是右臂1
    '''
    ini_result1=kk1.load_config(arm_type=0,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
    print(ini_result1)

    ini_result2=kk2.load_config(arm_type=1,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
    print(ini_result2)
    print('-'*50)

    '''
    初始化运动学
    '''
    initial_kine_tag=kk1.initial_kine(
                                    robot_type=ini_result1['TYPE'][0],
                                    dh=ini_result1['DH'][0],
                                    pnva=ini_result1['PNVA'][0],
                                    j67=ini_result1['BD'][0])

    initial_kine_tag=kk2.initial_kine(
                                    robot_type=ini_result1['TYPE'][1],
                                    dh=ini_result1['DH'][1],
                                    pnva=ini_result1['PNVA'][1],
                                    j67=ini_result1['BD'][1])
    print('-'*50)


    '''
    正解与逆解
        可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
        关节正解到末端在基坐标下的位置和姿态
    '''
    #正解
    print('left arm')
    fk_mat1=kk1.fk(joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
    if fk_mat1:
        print(f'fk matrix:{fk_mat1}')

    print('right arm')
    fk_mat2=kk2.fk(joints=[-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
    if fk_mat2:
        print(f'fk matrix:{fk_mat2}')
    print('-'*50)

    #逆解
    print('left arm')
    sp1=FX_InvKineSolvePara()
    mat161=kk1.mat4x4_to_mat1x16(fk_mat1)
    sp1.set_input_ik_target_tcp(mat161)
    sp1.set_input_ik_ref_joint([21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
    ik_result_structure1=kk1.ik(structure_data=sp1)
    if ik_result_structure1:
        print(f'ik joints:{ik_result_structure1.m_Output_RetJoint.to_list()}')
        print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsOutRange}')
        print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure1.m_Output_IsDeg[:]}')
        print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsJntExd}')
        print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_JntExdTags[:]}')
    else:
        print('NO ik results')

    print('right arm')
    sp2=FX_InvKineSolvePara()
    mat162=kk1.mat4x4_to_mat1x16(fk_mat2)
    sp2.set_input_ik_target_tcp(mat162)
    sp2.set_input_ik_ref_joint([-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
    ik_result_structure2=kk2.ik(structure_data=sp2)
    if ik_result_structure2:
        print(f'ik joints:{ik_result_structure2.m_Output_RetJoint.to_list()}')
        print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure2.m_Output_IsOutRange}')
        print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure2.m_Output_IsDeg[:]}')
        print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure2.m_Output_IsJntExd}')
        print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure2.m_Output_JntExdTags[:]}')
    else:
        print('NO ik results')
    print('-'*50)

def fk_ik_situation_2():
    print("*** 场景二：只设置工具坐标系")
    '''实列化计算'''
    kk1=Marvin_Kine() # LEFT ARM
    kk2=Marvin_Kine() # RIGHT ARM

    '''
    配置导入
    !!! 非常重要！！！
    使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
        ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
        ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
        srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错
    一定要确认arm_type是左臂0 还是右臂1
    '''
    ini_result1=kk1.load_config(arm_type=0,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
    print(ini_result1)

    ini_result2=kk2.load_config(arm_type=1,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
    print(ini_result2)
    print('-'*50)

    '''
    初始化运动学
    '''
    initial_kine_tag=kk1.initial_kine(
                                    robot_type=ini_result1['TYPE'][0],
                                    dh=ini_result1['DH'][0],
                                    pnva=ini_result1['PNVA'][0],
                                    j67=ini_result1['BD'][0])

    initial_kine_tag=kk2.initial_kine(
                                    robot_type=ini_result1['TYPE'][1],
                                    dh=ini_result1['DH'][1],
                                    pnva=ini_result1['PNVA'][1],
                                    j67=ini_result1['BD'][1])
    print('-'*50)

    
    '''
    设置工具坐标系：两个手臂末端工具运动学
    这里假设：工具与法兰间没有旋转，只有Z轴的偏移100毫米
    '''

    tool_left_arm=[[1,0,0,0],[0,1,0,0],[0,0,1,100],[0,0,0,1]]
    tag1=kk1.set_tool_kine(tool_mat=tool_left_arm)

    tool_right_arm=[[1,0,0,0],[0,1,0,0],[0,0,1,100],[0,0,0,1]]
    tag2=kk2.set_tool_kine(tool_mat=tool_right_arm)
    if not tag1 and not tag2:
        return
    print('-'*50)


    '''
    正解与逆解
        可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
        关节正解到末端在基坐标下的位置和姿态
    '''
    #正解
    print('left arm')
    fk_mat1=kk1.fk(joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
    if fk_mat1:
        print(f'fk matrix:{fk_mat1}')

    print('right arm')
    fk_mat2=kk2.fk(joints=[-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
    if fk_mat2:
        print(f'fk matrix:{fk_mat2}')
    print('-'*50)

    #逆解
    print('left arm')
    sp1=FX_InvKineSolvePara()
    mat161=kk1.mat4x4_to_mat1x16(fk_mat1)
    sp1.set_input_ik_target_tcp(mat161)
    sp1.set_input_ik_ref_joint([21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
    ik_result_structure1=kk1.ik(structure_data=sp1)
    if ik_result_structure1:
        print(f'ik joints:{ik_result_structure1.m_Output_RetJoint.to_list()}')
        print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsOutRange}')
        print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure1.m_Output_IsDeg[:]}')
        print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsJntExd}')
        print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_JntExdTags[:]}')
    else:
        print('NO ik results')

    print('right arm')
    sp2=FX_InvKineSolvePara()
    mat162=kk1.mat4x4_to_mat1x16(fk_mat2)
    sp2.set_input_ik_target_tcp(mat162)
    sp2.set_input_ik_ref_joint([-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
    ik_result_structure2=kk2.ik(structure_data=sp2)
    if ik_result_structure2:
        print(f'ik joints:{ik_result_structure2.m_Output_RetJoint.to_list()}')
        print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure2.m_Output_IsOutRange}')
        print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure2.m_Output_IsDeg[:]}')
        print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure2.m_Output_IsJntExd}')
        print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure2.m_Output_JntExdTags[:]}')
    else:
        print('NO ik results')
    print('-'*50)

def fk_ik_situation_3():
    print("*** 场景三：设置工具坐标系及用户坐标系")
    '''实列化计算'''
    kk1=Marvin_Kine() # LEFT ARM
    kk2=Marvin_Kine() # RIGHT ARM

    '''
    配置导入
    !!! 非常重要！！！
    使用前，请一定确认机型，导入正确的配置文件config_path，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！
        ccs 6公斤的机型的有两个版本: 3.1(计算配置文件为ccs_m6_31.MvKDCfg), 4.0(计算配置文件为ccs_m6_40.MvKDCfg)，两个版本的参数不一样请确认版本后选择参数.
        ccs 3公斤的机型的计算配置文件为ccs_m3.MvKDCfg；
        srs机型为srs.MvKDCfg. 多个*.MvKDCfg会解析出错
    一定要确认arm_type是左臂0 还是右臂1
    '''
    ini_result1=kk1.load_config(arm_type=0,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
    print(ini_result1)

    ini_result2=kk2.load_config(arm_type=1,config_path=os.path.join(parent_dir,'CommonConfig/ccs_m6_40.MvKDCfg'))
    print(ini_result2)
    print('-'*50)

    '''
    初始化运动学
    '''
    initial_kine_tag=kk1.initial_kine(
                                    robot_type=ini_result1['TYPE'][0],
                                    dh=ini_result1['DH'][0],
                                    pnva=ini_result1['PNVA'][0],
                                    j67=ini_result1['BD'][0])

    initial_kine_tag=kk2.initial_kine(
                                    robot_type=ini_result1['TYPE'][1],
                                    dh=ini_result1['DH'][1],
                                    pnva=ini_result1['PNVA'][1],
                                    j67=ini_result1['BD'][1])
    print('-'*50)

    '''
    设置工具坐标系：两个手臂末端工具运动学
    这里假设：工具与法兰间没有旋转，只有Z轴的偏移100毫米
    '''

    tool_left_arm=[[1,0,0,0],[0,1,0,0],[0,0,1,100],[0,0,0,1]]
    tag1=kk1.set_tool_kine(tool_mat=tool_left_arm)

    tool_right_arm=[[1,0,0,0],[0,1,0,0],[0,0,1,100],[0,0,0,1]]
    tag2=kk2.set_tool_kine(tool_mat=tool_right_arm)
    if not tag1 and not tag2:
        return
    print('-'*50)


    '''
    设置用户坐标系坐标系：用户坐标系相对于机器人两个手臂基座坐标系的齐次变换矩阵
    这里假设：用户坐标系是双臂基座顶上的一个视觉系统。只有xyZ轴的偏移各100毫米，没有旋转。
    '''

    user_left_arm=[[1,0,0,100],[0,1,0,100],[0,0,1,100],[0,0,0,1]]
    tag1=kk1.set_user_frame_kine(user_mat=user_left_arm)

    user_right_arm=[[1,0,0,100],[0,1,0,100],[0,0,1,100],[0,0,0,1]]
    tag2=kk2.set_user_frame_kine(user_mat=user_right_arm)
    if not tag1 and not tag2:
        return
    print('-'*50)


    '''
    正解与逆解
        可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
        关节正解到末端在基坐标下的位置和姿态
    '''
    #正解
    print('left arm')
    fk_mat1=kk1.fk(joints=[21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
    if fk_mat1:
        print(f'fk matrix:{fk_mat1}')

    print('right arm')
    fk_mat2=kk2.fk(joints=[-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
    if fk_mat2:
        print(f'fk matrix:{fk_mat2}')
    print('-'*50)

    #逆解
    print('left arm')
    sp1=FX_InvKineSolvePara()
    mat161=kk1.mat4x4_to_mat1x16(fk_mat1)
    sp1.set_input_ik_target_tcp(mat161)
    sp1.set_input_ik_ref_joint([21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68])
    ik_result_structure1=kk1.ik(structure_data=sp1)
    if ik_result_structure1:
        print(f'ik joints:{ik_result_structure1.m_Output_RetJoint.to_list()}')
        print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsOutRange}')
        print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure1.m_Output_IsDeg[:]}')
        print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_IsJntExd}')
        print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure1.m_Output_JntExdTags[:]}')
    else:
        print('NO ik results')

    print('right arm')
    sp2=FX_InvKineSolvePara()
    mat162=kk1.mat4x4_to_mat1x16(fk_mat2)
    sp2.set_input_ik_target_tcp(mat162)
    sp2.set_input_ik_ref_joint([-21.8, -41.0, 4.75, -63.67, -10.15, 14.72, -7.68])
    ik_result_structure2=kk2.ik(structure_data=sp2)
    if ik_result_structure2:
        print(f'ik joints:{ik_result_structure2.m_Output_RetJoint.to_list()}')
        print(f'ik 当前位姿是否超出位置可达空间（False：未超出；True：超出）: {ik_result_structure2.m_Output_IsOutRange}')
        print(f'ik 各关节是否发生奇异（False：未奇异；True：奇异）: {ik_result_structure2.m_Output_IsDeg[:]}')
        print(f'ik 是否有关节超出位置正负限制（False：未超出；True：超出）: {ik_result_structure2.m_Output_IsJntExd}')
        print(f'ik 各关节是否超出位置正负限制（False：未超出；True：超出）: {ik_result_structure2.m_Output_JntExdTags[:]}')
    else:
        print('NO ik results')
    print('-'*50)

if __name__=="__main__":
    
    '''### 场景一：不设置工具坐标系及用户坐标系, 正运动学求解是法兰末端基于机械臂基座的位姿矩阵，逆运动学同理 ###'''
    fk_ik_situation_1()

    '''### 场景二：只设置工具坐标系，不设置用户坐标系，正运动学求解是工具末端基于机械臂基座的位姿矩阵，逆运动学同理 ###'''
    fk_ik_situation_2()

    '''### 场景三：同时设置工具坐标系及用户坐标系，正运动学求解是工具末端基于用户坐标系的位姿矩阵，逆运动学同理 ###'''
    fk_ik_situation_3()


