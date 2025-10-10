# 天机-孚晞 MARVIN机器人计算SDK
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1004
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2025-09


### 工具包主要提供运动学相关功能。

## 一、接口介绍 [demo_linux_win/python/fx_kine.py]
## 接口快速全览： 


    可用方法:

    机器人关节角度正解到末端位置和姿态4*4矩阵
  - fk(robot_serial: int, joints: list)
    
    工具动力学辨识
  - identify_tool_dyn(robot_type: str, ipath: str)

    机器人末端位姿矩阵逆解到7个关节的角度
  - ik(robot_serial: int, pose_mat: list, ref_joints: list)

    逆解零空间
  - ik_nsp(robot_serial: int, pose_mat: list, ref_joints: list, zsp_type: int, zsp_para: list, zsp_angle: float, dgr: list)

    初始化动力学参数
  - initial_kine(robot_serial: int, robot_type: int, dh: list, pnva: list, j67: list)

    关节角度转雅可比矩阵
  - joints2JacobMatrix(robot_serial: int, joints: list)

    加载配置文件
  - load_config(config_path: str)

    末端位姿矩阵转XYZABC表示
  - mat4x4_to_xyzabc(pose_mat: list)

    直线插值规划
  - movL(robot_serial: int, start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float, save_path)
    
    移除工具设置
  - remove_tool_kine(robot_serial: int)

    设置末端工具参数
  - set_tool_kine(robot_serial: int, tool_mat: list)

    末端XYZABC转位姿矩阵表示
  - xyzabc_to_mat4x4(xyzabc: list)

## 二、 接口详解 

    一定要确认robot_serial是左臂0 还是右臂1
    在DEMO中仅示例了单臂（左臂）的计算
    如果人形，则左右臂都要计算。
    使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！

###    2.1 导入运动学相关参数
load_config(config_path: str)

        :param config_path: 本地机械臂配置文件srs.MvKDCfg/ccs.MvKDCfg(请确认机型和DH参数是否对应), 可相对路径.
        • srs.MvKDCfg/ccs.MvKDCfg 文件中包含与运动学计算相关的双臂参数，进行计算之前需要导入机械臂配置相关文件
        • TYPE=1006，DL机型；TYPE=1007，Pilot-SRS机型（双臂为MARVIN）；TYPE=1017，Pilot-CCS机型双臂为MARVIN）！
        • GRV参数为双臂重力方向，如[0.000,9.810,0.000];
        • DH参数为双臂MDH参数，包含各关节MDH参数及法兰MDH参数；
        • PNVA参数为双臂各关节正负限制位置以及所允许的正负最大加速度及加加速度；
        • BD参数为Pilot-CCS机型特定参数，为六七关节自干涉允许范围的拟合二阶多项式曲线，其他机型中该参数均为0；
        • Mass参数为双臂各关节质量；MCP参数为双臂各关节质心；I参数为双臂各关节惯量
        • MDH参数单位为度和毫米（mm），速度加速度单位为度/秒，关节质量、关节质心、关节惯量单位均为国际标准单位



###    2.2 初始化运动学相关参数
initial_kine(robot_serial: int, robot_type: int, dh: list, pnva: list, j67: list)

        '''初始化运动学相关参数
        • 运动学相关计算前，需要按照该顺序调用初始化函数，将配置中导入的参数进行初始化
        • RobotSerial=0，左臂；RobotSerial=1，右臂
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param type: int.机器人机型代号。
        :param dh: list(8,4), 每个轴DH：alpha, a d,theta.
        :param pnva: list(7,4), 每个轴:关节上界p,关节下界n，最大速度v,最大加速度a.
        :param j67: list(4,3),仅CCS机型生效， 67关节干涉限制。
        :return:
            bool
        '''

###    2.3 工具设置
设置工具的运动学参数
set_tool_dyn(robot_serial: int, dyn: list)
移除工具的运动学参数
remove_tool_kine(robot_serial: int)


    • 若末端带有负载，对各关节参数初始化后，需要对工具进行设置


    '''工具运动学设置
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param tool_mat: list(4,4) 工具的运动学信息，齐次变换矩阵，相对末端法兰的旋转和平移，请确认法兰坐标系。
        :return:bool
        '''

    '''移除工具动力学设置
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :return:bool
        '''




###    2.4 计算正运动学
fk(robot_serial: int, joints: list)

    '''关节角度正解到末端TCP位置和姿态4*4
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param joints: list(7,1). 角度值
        :return:
            4x4的位姿矩阵，list(4,4)， 旋转矩阵单位为角度，位置向量单位是毫米
        '''

    '''
    正解与逆解
        可相互验证:正解的输入得到的4×4作为输入传递给逆解会得到和正解输入的关节位置一致。
        关节正解到末端在基坐标下的位置和姿态
    '''

###    2.5 计算逆运动学
ik(robot_serial: int, pose_mat: list, ref_joints: list)


    '''末端位置和姿态逆解到关节值
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param pose_mat: list(4,4), 位置姿态4x4list，旋转矩阵单位为角度，位置向量单位是毫米
         :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :return:
            结构体，以下几项最相关：
                m_Output_RetJoint      ：逆运动学解出的关节角度（单位：度）
                m_Output_IsOutRange    ：当前位姿是否超出位置可达空间（False：未超出；True：超出）
                m_Output_IsDeg[7]      ：各关节是否发生奇异（False：未奇异；True：奇异）:
                m_Output_IsJntExd      : 是否有关节超出位置正负限制（False：未超出；True：超出）:
                m_Output_JntExdTags[7] ：各关节是否超出位置正负限制（False：未超出；True：超出）:
        '''

###    2.6 计算末端位姿不变、改变零空间（臂角方向）的逆运动学
ik_nsp(robot_serial: int, pose_mat: list, ref_joints: list, zsp_type: int, zsp_para: list, zsp_angle: float, dgr: list)

    '''逆解优化：可调整方向,不能单独使用，ik得到的逆运动学解的臂角不满足当前选解需求时使用。
        :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param pose_mat: list(4,4), 位置姿态4x4list.
        :param ref_joints: list(7,1),参考输入角度，约束构想接近参考解读，防止解出来的构型跳变。
        :param zsp_type: int, 零空间约束类型（0：与参考角欧式距离最小；1：与参考臂角平面最近）
        :param zsp_para: list(6,), 若选择零空间约束类型为1，则需额外输入参考角平面参数,[x,y,z,a,b,c]=[0,0,0,0,0,0],可选择x,y,z其中一个方向调整
        :param zsp_angle: float, 末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）,可正向调节也可逆向调节.
        :param dgr: list(2,), 选择123关节和567关节发生奇异允许的角度范围，如无额外要求无需输入，默认值为0.05（单位：度）
        :return:
            结构体，以下几项最相关：
                m_Output_RetJoint      ：逆运动学解出的关节角度（单位：度）
                m_Output_IsOutRange    ：当前位姿是否超出位置可达空间（False：未超出；True：超出）
                m_Output_IsDeg[7]      ：各关节是否发生奇异（False：未奇异；True：奇异）:
                m_Output_IsJntExd      : 是否有关节超出位置正负限制（False：未超出；True：超出）:
                m_Output_JntExdTags[7] ：各关节是否超出位置正负限制（False：未超出；True：超出）:
        '''

###    2.7 计算雅可比矩阵
joints2JacobMatrix(robot_serial: int, joints: list)

    • 输入关节角度及RobotSerial（参数含义参考初始化参数部分），输出为6*7的雅可比矩阵
    '''当前关节角度转成雅可比矩阵
            :param robot_serial:  int, RobotSerial=0，左臂；RobotSerial=1，右臂
            :param joints: list(7,1), 当前关节
            :return: 雅可比矩阵6*7矩阵
            '''

###    2.8 直线规划（MOVL）
movL(robot_serial: int, start_xyzabc: list, end_xyzabc: list, ref_joints: list, vel: float, acc: float, save_path)

    • 输出点位频率为500Hz

        '''直线规划（MOVL）

        :param robot_serial: int, RobotSerial=0，左臂；RobotSerial=1，右臂
        :param start_xyzabc:
        :param end_xyzabc:
        :param ref_joints:
        :param vel:
        :param acc:
        :param save_path:
        :return: bool
        '''
    特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.

###    2.9 工具动力学参数辨识
identify_tool_dyn(robot_type: str, ipath: str)

        '''工具动力学参数辨识
        :param robot_type: string ,机型
        :param ipath: sting, 相对路径导入工具辨识轨迹数据。
        :return:
            m,mcp,i
        '''


###  2.10 位置姿态4×4矩阵转XYZABC
mat4x4_to_xyzabc(pose_mat:list)

    • 输入为4*4的法兰末端位姿矩阵
    • 输出位姿信息XYZ及欧拉角ABC（单位：mm/度）

        '''末端位置和姿态转XYZABC
        :param pose_mat: list(4,4), 位置姿态4x4list.
        :return:
                （6,1）位姿信息XYZ及欧拉角ABC（单位：mm/度）
        '''
    
###     2.11 XYZABC转位置姿态4×4矩阵
xyzabc_to_mat4x4(xyzabc:list)

    • 输入为位姿信息XYZ及欧拉角ABC（单位：mm/度）
    • 输出4*4的法兰末端位姿矩阵

        '''末端XYZABC转位置和姿态矩阵
        param xyzabc: list(6,),
        return:
            mat4x4  list(4,4)

        '''




# 三、案例脚本
请注意：案例仅为参考使用，实地生产和业务逻辑需要您加油写~~~
## 3.1综合接口案例脚本:[kine_demo_A_arm.py](python/kine_demo_A_arm.py)[kine_demo_a_arm.py](python/kine_demo_A_arm.py)

## 3.2工具辨识案例：[identy_tool_dynamic_SRS_A_demo.py](python/identy_tool_dynamic_SRS_A_demo.py)

## 3.3十字交叉机型67关节干涉解决案列：

    案列的情况和解决方案描述：
    给定一个当前关节[32,-64,72,59,-107,-30,58]，不确定是否超限，想要在这组关节下，末端朝Z的负向方向移动100毫米。

    [ccs_67_interference.py](python/ccs_67_interference.py)
    首先脚本判断该关节值里7关节超限了，先保持末端位姿不变，解到[24.710327454579385, -34.626590037831534, 129.573376956529, 58.999999819257596, -154.5774252821101, -9.250648598649414, 76.73495447348635]， 
    然后这个关节基础上，Z调-100毫米， 再逆解到不超限的[30.441992064349336, -41.66004132484181, 124.19196073906699, 73.1034294701139, -147.2634786470432, 5.414599152769855, 71.29893883226201]。

    解决方案末端验证：[dd_eval.py](python/dd_eval.py) 验证解出来的关节是否保持末端位姿不变以及正确在Z方向-100毫米
    








