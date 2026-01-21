# 天机-孚晞 MARVIN机器人计算SDK
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1004
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2025-12


### 工具包主要提供运动学相关功能。

## 一、接口介绍 [SDK_PYTHON/fx_kine.py]
## 接口快速全览： 


    可用方法:

    机器人关节角度正解到末端位置和姿态4*4矩阵
  - fk(joints: list)
  - 
    机器人关节角度正解到末端位置和姿态4*4矩阵，并得出零空间平面参数矩阵
  - - fk_nsp(joints: list)
    
    工具动力学辨识
  - identify_tool_dyn(robot_type: int, ipath: str)

    机器人末端位姿矩阵逆解到7个关节的角度
  - ik(structure_data)

    逆解零空间
  - ik_nsp( structure_data)

    初始化动力学参数
  - initial_kine( robot_type: int, dh: list, pnva: list, j67: list)

    关节角度转雅可比矩阵
  - joints2JacobMatrix(joints: list)

    加载配置文件
  - load_config(arm_type:int, config_path: str)

    末端位姿矩阵转XYZABC表示
  - mat4x4_to_xyzabc(pose_mat: list)

    位姿矩阵展开表示
  - mat4x4_to_mat1x16(pose_mat)

    直线插值规划
  - movL(start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float, save_path)
    
    直线插值规划，约束起始结束关节构型
  - movL_KeepJ(start_joints:list, end_joints:list,vel:float,acc: float,save_path)

    在线直线插值规划
  - movLA(start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float)

        
    在线直线插值规划，约束起始结束关节构型
  - movL_KeepJA(start_joints:list, end_joints:list,vel:float,acc: float)

    移除工具设置
  - remove_tool_kine()

    设置末端工具参数
  - set_tool_kine(tool_mat: list)

    末端XYZABC转位姿矩阵表示
  - xyzabc_to_mat4x4(xyzabc: list)

## 二、 接口详解 

    一定要确认robot_serial是左臂0 还是右臂1
    在DEMO中仅示例了单臂（左臂）的计算
    如果人形，则左右臂都要计算，两个手臂实例化两个机器人。
    使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！

### 逆解结构体参数介绍

    class FX_InvKineSolvePara(ctypes.Structure):
        _fields_ = [
            # 输入部分
            ("m_Input_IK_TargetTCP", Matrix4), #末端位置姿态4x4列表，可通过正解接口获取或者指定末端的位置和旋转
            ("m_Input_IK_RefJoint", Vect7), #参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零
            ("m_Input_IK_ZSPType", FX_INT32L), #零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
            ("m_Input_IK_ZSPPara", FX_DOUBLE * 6), #若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
            ("m_Input_ZSP_Angle", FX_DOUBLE), #末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
            ("m_DGR1", FX_DOUBLE), #(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
            ("m_DGR2", FX_DOUBLE), #(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
            ("m_DGR3", FX_DOUBLE), #预留接口
            # 输出部分
            ("m_Output_RetJoint", Vect7), #逆运动学解出的关节角度（单位：度）
            ("m_OutPut_AllJoint", Matrix8), #逆运动学的全部解（每一行代表一组解, 分别存放1 - 7关节的角度值）（单位：度）
            ("m_OutPut_Result_Num", FX_INT32L), #逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
            ("m_Output_IsOutRange", FX_BOOL), #当前位姿是否超出位置可达空间（False：未超出；True：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
            ("m_Output_IsDeg", FX_BOOL * 7), #各关节是否发生奇异（False：未奇异；True：奇异）
            ("m_Output_JntExdTags", FX_BOOL * 7), #各关节是否超出位置正负限制（False：未超出；True：超出）
            ("m_Output_JntExdABS", FX_DOUBLE), #所有关节中超出限位的最大角度的绝对值，比如解出一组关节角度，7关节超限，的值为-95，已知软限位为-90度，m_Output_JntExdABS=5.
            ("m_Output_IsJntExd", FX_BOOL), #是否有关节超出位置正负限制（False：未超出；True：超出）
            ("m_Output_RunLmtP", Vect7), #各个关节运行的正限位, 可作为计算六七关节的干涉参考最大限制。
            ("m_Output_RunLmtN", Vect7) #各个关节运行的负限位，可作为计算六七关节的干涉参考最大限制。                                                                                mm
        ]

###    2.0 日志关闭
    def log_switch(switch:int):
        '''
        :param switch: 打印日志开：1；打印日志关：0
        '''

    
###    2.1 导入运动学相关参数
load_config(arm_type: int, config_path: str)

        :param srm_type: 选择左臂还是右臂, 左臂:0, 右臂:1
        :param config_path: 本地机械臂配置文件srs.MvKDCfg/ccs.MvKDCfg(请确认机型和DH参数是否对应), 可相对路径.
        • srs.MvKDCfg/ccs.MvKDCfg 文件中包含与运动学计算相关的双臂参数，进行计算之前需要导入机械臂配置相关文件
        • TYPE=1007，Pilot-SRS机型（双臂为MARVIN）；TYPE=1017，Pilot-CCS机型双臂为MARVIN）！
        • GRV参数为双臂重力方向，如[0.000,9.810,0.000];
        • DH参数为双臂MDH参数，包含各关节MDH参数及法兰MDH参数；
        • PNVA参数为双臂各关节正负限制位置以及所允许的正负最大加速度及加加速度；
        • BD参数为Pilot-CCS机型特定参数，为六七关节自干涉允许范围的拟合二阶多项式曲线，其他机型中该参数均为0；
        • Mass参数为双臂各关节质量；MCP参数为双臂各关节质心；I参数为双臂各关节惯量
        • MDH参数单位为度和毫米（mm），速度加速度单位为度/秒，关节质量、关节质心、关节惯量单位均为国际标准单位



###    2.2 初始化运动学相关参数
initial_kine(robot_type: int, dh: list, pnva: list, j67: list)

        '''初始化运动学相关参数
        • 运动学相关计算前，需要按照该顺序调用初始化函数，将配置中导入的参数进行初始化
        :param type: int.机器人机型代号。
        :param dh: list(8,4), 每个轴DH：alpha, a d,theta.
        :param pnva: list(7,4), 每个轴:关节上界p,关节下界n，最大速度v,最大加速度a.
        :param j67: list(4,3),仅CCS机型生效， 67关节干涉限制。
        :return:
            bool
        '''

###    2.3 工具设置
设置工具的运动学参数
set_tool_dyn(dyn: list)
移除工具的运动学参数
remove_tool_kine()


    • 若末端带有负载，对各关节参数初始化后，需要对工具进行设置


    '''工具运动学设置
        :param tool_mat: list(4,4) 工具的运动学信息，齐次变换矩阵，相对末端法兰的旋转和平移，请确认法兰坐标系。
        :return:bool
        '''

    '''移除工具动力学设置
        :return:bool
        '''




###    2.4 计算正运动学
fk(joints: list)

    '''关节角度正解到末端TCP位置和姿态4*4
        :param joints: list(7,1). 正解的输入关节角度，单位：度。
        :return:
            4x4的位姿矩阵，list(4,4)， 旋转矩阵单位为角度，位置向量单位是毫米
        '''

    '''
    正解与逆解
        可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
        关节正解到末端在基坐标下的位置和姿态
    '''



###    2.5 计算正运动学并得到该构型下的零空间参数矩阵
fk_nsp(joints: list)

        '''关节角度正解到末端TCP位置和姿态4*4，并得到基于该角度下的零空间参数
        :param joints: list(7,1). 角度值，单位：度
        :return:
            末端4x4位姿矩阵，list(4,4)
            零空间参数矩阵 array(3,3), 其中第一列可以作为逆解结构体里面m_Input_IK_ZSPPara的x y z的输入值。
        '''

###    2.6 计算逆运动学
ik(structure_data):
        '''末端位置和姿态逆解到关节值
        :param 结构体数据
            输入参数：
                m_Input_IK_TargetTCP：末端位置姿态4x4列表，可通过正解接口获取或者指定末端的位置和旋转
                m_Input_IK_RefJoint：参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零
                m_Input_IK_ZSPType：零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
                m_Input_IK_ZSPPara：若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
                m_Input_ZSP_Angle：末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
                m_DGR1：(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR2：(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR3：预留接口

            结构体的输出参数：
                m_Output_RetJoint      :逆运动学解出的关节角度（单位：度）
                m_OutPut_AllJoint      :逆运动学的全部解（每一行代表一组解,分别存放1-7关节的角度值）（单位：度）
                m_Output_IsOutRange    :当前位姿是否超出位置可达空间（False：未超出；True：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
                m_OutPut_Result_Num    :逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
                m_Output_IsDeg[7]      :各关节是否发生奇异（False：未奇异；True：奇异）
                m_Output_IsJntExd      :是否有关节超出位置正负限制（False：未超出；True：超出）
                m_Output_JntExdTags[7] :各关节是否超出位置正负限制（False：未超出；True：超出）
                m_Output_RunLmtP       :各个关节运行的正限位, 可作为计算六七关节的干涉参考最大限制
                m_Output_RunLmtN       :各个关节运行的负限位，可作为计算六七关节的干涉参考最大限制

         输出：
            成功：True/1; 失败：False/0
            失败情况:
                    1. 输入矩阵超出机器人可达关节空间
                    2. 第四关节为0, 奇异

        • 特别提示:
                结构体以下输出项的TAG仅绑定对m_Output_RetJoint输出的关节描述
                    • m_Output_IsOutRange     :用于判断当前位姿是否超出位置可达空间（0：未超出；1：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
                    • m_Output_IsDeg[7]       :用于判断各关节是否发生奇异（0：未奇异；1：奇异）
                    • m_Output_JntExdABS      :各关节超限绝对值总和(FX_Robot_PLN_MOVL_KeepJ使用)
                    • m_Output_IsJntExd       :用于判断是否有关节超出位置正负限制（0：未超出；1：超出）
                    • m_Output_JntExdTags[7]  :用于判断各关节是否超出位置正负限制（0：未超出；1：超出）

                如果选用用多组解m_OutPut_AllJoint. 请自行对选的解做判断,符合以下三个条件才能控制机械臂正常驱动:
                    1. 第二关节的角度不在正负0.05度范围内(在此范围将奇异)
                    2. 对输出的各个关节做软限位判定:
                        调用接口ini_result=kk.load_config(config_path=os.path.join(current_path,'ccs_m6.MvKDCfg'))后,
                        ini_result['PNVA'][:]矩阵里的请两列对应各个关节的正负限位
                        选取的解的每个关节都满足在限位置内
                    3. 如果条件1和2都满足,还要做六七关节干涉判定:
                        判定方法:
                            调用接口ini_result=kk.load_config(config_path=os.path.join(current_path,'ccs_m6.MvKDCfg'))后,
                            ini_result['BD'][:]矩阵里依次为++, -+,  --, +- 四个象限的干涉参数
                            以CCS为例:
                                如果选的解的六七关节都为正, 则选用在++象限里的参数:[0.018004, -2.3205, 108.0],三个参数分别视为a0,a1,a2,
                                第6关节的值为j6,此时使用公式j7=(a0^2)*j6+ a1*j6+a2  将得到第7个关节的最大限制位置
                                如果选取的解里面的第7关节小于j7, 则不发生干涉, 本组解可被驱动到达.
        '''



###    2.7 计算末端位姿不变、改变零空间（臂角方向）的逆运动学
ik_nsp(sturcture_data):
        '''逆解优化：可调整方向,不能单独使用，ik得到的逆运动学解的臂角不满足当前选解需求时使用。
            输入参数：
                m_Input_IK_TargetTCP：末端位置姿态4x4列表，可通过正解接口获取或者指定末端的位置和旋转
                m_Input_IK_RefJoint：参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。该构型的肩、肘、腕组成初始臂角平面，以肩到腕方向为Z向量，参考角第四关节不能为零
                m_Input_IK_ZSPType：零空间约束类型（0：使求解结果与参考关节角的欧式距离最小适用于一般冗余优化；1：与参考臂角平面最近，需要额外提供平面参数zsp_para）
                m_Input_IK_ZSPPara：若选择零空间约束类型zsp_type为1，则需额外输入参考角平面参数，目前仅支持平移方向的参数约束，即[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
                m_Input_ZSP_Angle：末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节. 在ref_joints为初始臂角平面情况下，使用右手法则，绕Z向量正向旋转为臂角增加方向，绕Z向量负向旋转为臂角减少方向
                m_DGR1：(仅在IK_NSP接口中设置起效)判断第二关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR2：(仅在IK_NSP接口中设置起效)判断第六关节发生奇异的角度范围，数值范围为0.05-10(单位：度)，不设置情况下默认0.05度
                m_DGR3：预留接口

            结构体的输出参数：
                m_Output_RetJoint      :逆运动学解出的关节角度（单位：度）
                m_OutPut_AllJoint      :逆运动学的全部解（每一行代表一组解,分别存放1-7关节的角度值）（单位：度）
                m_Output_IsOutRange    :当前位姿是否超出位置可达空间（False：未超出；True：超出）,如果超出可达空间,则需调整参考角度(参考角度和目标点位相差过大).
                m_OutPut_Result_Num    :逆运动学全部解的组数（七自由度CCS构型最多四组解，SRS最多八组解）
                m_Output_IsDeg[7]      :各关节是否发生奇异（False：未奇异；True：奇异）
                m_Output_IsJntExd      :是否有关节超出位置正负限制（False：未超出；True：超出）
                m_Output_JntExdTags[7] :各关节是否超出位置正负限制（False：未超出；True：超出）
                m_Output_RunLmtP       :各个关节运行的正限位, 可作为计算六七关节的干涉参考最大限制
                m_Output_RunLmtN       :各个关节运行的负限位，可作为计算六七关节的干涉参考最大限制
        输出：
            成功：True/1; 失败：False/0
        '''

###    2.8 计算雅可比矩阵
joints2JacobMatrix(joints: list)

    • 输入关节角度及RobotSerial（参数含义参考初始化参数部分），输出为6*7的雅可比矩阵
    '''当前关节角度转成雅可比矩阵
            :param joints: list(7,1), 当前关节角度
            :return: 雅可比矩阵6*7矩阵
            '''

###    2.9 直线规划（MOVL）
movL(start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float, save_path)

    • 输出点位频率为500Hz，即每20ms执行一行

        '''直线规划，规划文件的频率500Hz，即每2ms执行一行
        :param start_xyzabc:起始点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param end_xyzabc:结束点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param ref_joints:参考关节构型，也是规划文件的起始点位。
        :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
        :param save_path:保存的规划文件的路径
        :return: bool
        特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
                2 输出规划文件的频率为500Hz
                3 movL的特点在于根据提供的起始目标笛卡尔位姿和终止目标笛卡尔位姿规划一段直线路径点，该接口不约束到达终点时的机器人构型。
        '''

            
###    2.10 直线插值规划，约束起始结束关节构型（movL_KeepJ）
movL_KeepJ(start_joints:list, end_joints:list,vel:float,save_path)

    • 输出点位频率为50Hz，即每2ms执行一行
        '''直线规划保持关节构型, 规划文件的点位频率50Hz，即每20ms执行一行
        :param start_joints:起始点各个关节位置（单位：角度）
        :param end_joints:终点各个关节位置（单位：角度）
        :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
        :param save_path:规划文件的保存路径
        :return: bool
        特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
                2 输出点位频率为500Hz
                3 该接口是不同于MOVL的规划接口，movL_KeepJ根据起始关节和结束关节规划一条直线路径。
        '''



###    2.11 在线直线规划（MOVL）
movLA(start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float)

    • 输出点位频率为500Hz，即每20ms执行一行

         '''直线规划，执行MOVLA规划并返回点集数据(频率500Hz)
        :param start_xyzabc:起始点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param end_xyzabc:结束点末端的位置和姿态：xyz平移单位：mm， abc旋转单位：度。
        :param ref_joints:参考关节构型，也是规划文件的起始点位。
        :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
        :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
        :return: 规划得到的点集列表
        特别提示:1 需要读函数返回值,如果关节超限,返回为false.
                2 输出规划文件的频率为500Hz
                3 movL的特点在于根据提供的起始目标笛卡尔位姿和终止目标笛卡尔位姿规划一段直线路径点，该接口不约束到达终点时的机器人构型。
        '''

            
###    2.12 在线直线插值规划，约束起始结束关节构型（movL_KeepJ）
movL_KeepJA(start_joints:list, end_joints:list,vel:float,save_path)

    • 输出点位频率为50Hz，即每2ms执行一行
        '''直线规划，执行movL_KeepJA规划并返回点集数据(频率500Hz)
    
       :param start_joints:起始点各个关节位置（单位：角度）
       :param end_joints:终点各个关节位置（单位：角度）
       :param vel:约束了输出的规划文件的速度。单位毫米/秒， 最小为0.1mm/s， 最大为1000 mm/s
       :param acc:约束了输出的规划文件的加速度。单位毫米/平方秒， 最小为0.1mm/s^2， 最大为10000 mm/s^2
       :return: 规划得到的点集列表
       特别提示:1 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的点集.
               2 输出点位频率为500Hz
               3 该接口是不同于MOVLA的规划接口，movL_KeepJA根据起始关节和结束关节规划一条直线路径。
       '''


                

###    2.13 工具动力学参数辨识
identify_tool_dyn(robot_type: int, ipath: str)

        '''工具动力学参数辨识
        :param robot_type: int. 1:CCS机型，2:SRS机型
        :param ipath: sting, 相对路径导入工具辨识轨迹数据。
        :return:
            辨识成功，返回一个长度为10的list:
                        m,mcp*3,i*6
            辨识失败，返回错误类型：
                    ret=1, 计算错误，需重新采集数据计算； 
                    ret=2,打开采集数据文件错误，须检查采样文件； 
                    ret=3,配置文件被修改； 
                    ret=4, 采集时间不够，缺少有效数据
        '''


###     2.14 位置姿态4×4矩阵转XYZABC
mat4x4_to_xyzabc(pose_mat:list)

    • 输入为4*4的法兰末端位姿矩阵
    • 输出位姿信息XYZ及欧拉角ABC（单位：mm/度）

        '''末端位置和姿态转XYZABC
        :param pose_mat: list(4,4), 位置姿态4x4list.
        :return:
                （6,1）位姿信息XYZ及欧拉角ABC（单位：mm/度）
        '''
    
###     2.15 XYZABC转位置姿态4×4矩阵
xyzabc_to_mat4x4(xyzabc:list)

    • 输入为位姿信息XYZ及欧拉角ABC（单位：mm/度）
    • 输出4*4的法兰末端位姿矩阵

        '''末端XYZABC转位置和姿态矩阵
        param xyzabc: list(6,),
        return:
            mat4x4  list(4,4)

        '''
        
###     2.16 位姿矩阵展开表示
mat4x4_to_mat1x16(self,pose_mat):
        matrix_data=[]
        for i in range(4):
            for j in range(4):
                matrix_data.append(pose_mat[i][j])
        return matrix_data


# 三、案例脚本
请注意：案例仅为参考使用，实地生产和业务逻辑需要您加油写~~~

   见DEMO_PYTHON/readme.md










