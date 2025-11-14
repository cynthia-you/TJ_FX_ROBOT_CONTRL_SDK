# 天机-孚晞 MARVIN机器人计算SDK
## 机器人型号： MARVIN人形双臂, 单臂
## 版本： 1004
## 支持平台： LINUX 及 WINDOWS
## LINUX支持： ubuntu18.04 - ubuntu24.04
## 更新日期：2025-09



## 一、接口介绍
### 接口快速全览见[kinematicsSDK/FxRobot.h]

### 注意
    一定要确认RobotSerial是左臂0 还是右臂1
    在DEMO中仅示例了单臂（左臂）的计算
    如果人形，则左右臂都要计算。
    使用前，请一定确认机型，导入正确的配置文件，文件导错，计算会错误啊啊啊,甚至看起来运行正常，但是值错误！！！



###    1. 导入运动学相关参数
FX_BOOL  LOADMvCfg(FX_CHAR* path, FX_INT32L TYPE[2], FX_DOUBLE GRV[2][3], FX_DOUBLE DH[2][8][4], FX_DOUBLE PNVA[2][7][4], FX_DOUBLE BD[2][4][3],FX_DOUBLE Mass[2][7], FX_DOUBLE MCP[2][7][3], FX_DOUBLE I[2][7][6])

    • Eg.:LOADMvCfg((char *)"xxx.MvKDCfg", TYPE, GRV, DH, PNVA, BD, Mass, MCP, I)
    • xxx.MvKDCfg文件为本地机械臂配置文件srs.MvKDCfg/ccs.MvKDCfg(请确认机型和DH参数是否对应), 可相对路径.
    • srs.MvKDCfg/ccs.MvKDCfg 文件中包含与运动学计算相关的双臂参数，进行计算之前需要导入机械臂配置相关文件
    • TYPE=1006，DL机型；TYPE=1007，Pilot-SRS机型（双臂为MARVIN）；TYPE=1017，Pilot-CCS机型双臂为MARVIN）！
    • GRV参数为双臂重力方向，如[0.000,9.810,0.000];
    • DH参数为双臂MDH参数，包含各关节MDH参数及法兰MDH参数；
    • PNVA参数为双臂各关节正负限制位置以及所允许的正负最大加速度及加加速度；
    • BD参数为Pilot-CCS机型特定参数，为六七关节自干涉允许范围的拟合二阶多项式曲线，其他机型中该参数均为0；
    • Mass参数为双臂各关节质量；MCP参数为双臂各关节质心；I参数为双臂各关节惯量
    • MDH参数单位为度和毫米（mm），速度加速度单位为度/秒，关节质量、关节质心、关节惯量单位均为国际标准单位

###    2. 初始化运动学相关参数
FX_BOOL  FX_Robot_Init_Type(FX_INT32L RobotSerial, FX_INT32L RobotType)

FX_BOOL  FX_Robot_Init_Kine(FX_INT32L RobotSerial, FX_DOUBLE DH[8][4])

FX_BOOL  FX_Robot_Init_Lmt(FX_INT32L RobotSerial, FX_DOUBLE PNVA[7][4], FX_DOUBLE J67[4][3])

    • 运动学相关计算前，需要按照该顺序调用初始化函数，将配置中导入的参数进行初始化
    • RobotSerial=0，左臂；RobotSerial=1，右臂

###    3. 工具设置
FX_BOOL  FX_Robot_Tool_Set(FX_INT32L RobotSerial, Matrix4 tool)

FX_BOOL  FX_Robot_Tool_Rmv(FX_INT32L RobotSerial)

    • 若末端带有负载，对各关节参数初始化后，需要对工具进行设置

###    4. 计算正运动学
FX_BOOL  FX_Robot_Kine_FK(FX_INT32L RobotSerial, FX_DOUBLE joints[7], Matrix4 pgos)

    • 输入七关节角度及RobotSerial（参数含义参考初始化参数部分），输出为4*4的法兰末端位姿矩阵

###    5. 计算逆运动学
FX_BOOL  FX_Robot_Kine_IK(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)

    • 输入RobotSerial（参数含义参考初始化参数部分）及solve_para结构体，输出包含在solve_para中
    • solve_para结构体中，包含以下内容：
        • 输入项
            • Matrix4 m_Input_IK_TargetTCP ：4*4的目标点末端的位姿矩阵
            • Vect7   m_Input_IK_RefJoint  ：逆运动学的各关节参考角（单位：度）
        • 输出项
            • Vect7   m_Output_RetJoint      ：逆运动学解出的关节角度（单位：度）
            • FX_BOOL m_Output_IsOutRange    ：用于判断当前位姿是否超出位置可达空间（0：未超出；1：超出）
            • FX_BOOL m_Output_IsDeg[7]      ：用于判断各关节是否发生奇异（0：未奇异；1：奇异）
            • FX_BOOL m_Output_IsJntExd      : 用于判断是否有关节超出位置正负限制（0：未超出；1：超出）
            • FX_BOOL m_Output_JntExdTags[7] ：用于判断各关节是否超出位置正负限制（0：未超出；1：超出）

###    6. 计算末端位姿不变、改变零空间（臂角方向）的逆运动学
FX_BOOL  FX_Robot_Kine_IK_NSP(FX_INT32L RobotSerial, FX_InvKineSolvePara *solve_para)

    • 输入RobotSerial（参数含义参考初始化参数部分）及solve_para结构体，输出包含在solve_para中
    • 特别的，在该函数的solve_para结构体中，额外包含以下内容：
        • 输入项
            • Matrix4    m_Input_IK_TargetTCP ：4*4的目标点末端的位姿矩阵
            • Vect7      m_Input_IK_RefJoint  ：逆运动学的各关节参考角（单位：度）
            • FX_INT32L	 m_Input_IK_ZSPType   ：零空间约束类型（0：与参考角欧式距离最小；1：与参考臂角平面最近）
            •*FX_DOUBLE	 m_Input_IK_ZSPPara[6]：若选择零空间约束类型为1，则需额外输入参考角平面参数
            • FX_DOUBLE	 m_Input_ZSP_Angle    ：末端位姿不变的情况下，零空间臂角相对于参考平面的旋转角度（单位：度）
            •*FX_DOUBLE  m_DGR1,m_DGR2        ：选择123关节和567关节发生奇异允许的角度范围，如无额外要求无需输入，默认值为0.05（单位：度）
        • 输出项
            • 参考计算逆运动学函数接口输出项

###    7. 计算雅可比矩阵
FX_BOOL  FX_Robot_Kine_Jacb(FX_INT32L RobotSerial, FX_DOUBLE joints[7], FX_Jacobi* jcb)

    • 输入关节角度及RobotSerial（参数含义参考初始化参数部分），输出为6*7的雅可比矩阵

###    8. 直线规划（MOVL）
FX_BOOL  FX_Robot_PLN_MOVL(FX_INT32L RobotSerial, Vect6 Start_XYZABC, Vect6 End_XYZABC, Vect7 Ref_Joints, FX_DOUBLE Vel, FX_DOUBLE ACC, FX_INT8* OutPutPath)

    • 输入RobotSerial（参数含义参考初始化参数部分）、起始点位姿、结束点位姿、当前位置参考关节角度、直线规划速度及直线规划加速度，输出为包含该段规划的关节点位文件
    • 位姿信息XYZ及欧拉角（单位：mm/度）
    • 输出点位频率为500Hz
    • 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.

    特别提示:直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.

###    9.直线规划，约束机器人气势和结束的各个关节角度（MOVLJ）
FX_BOOL  FX_Robot_PLN_MOVL_KeepJ(FX_INT32L RobotSerial, Vect7 startjoints, Vect7 stopjoints, FX_DOUBLE vel, FX_CHAR* OutPutPath);

    • 输入RobotSerial（参数含义参考初始化参数部分）、起始点位姿、结束点位姿、当前位置参考关节角度、直线规划速度及直线规划加速度，输出为包含该段规划的关节点位文件
    • startjoints:起始点各个关节位置（单位：角度）
    • stopjointss:终点各个关节位置（单位：角度）
    • vel：规划速度，百分比，值范围0-100
    • OutPutPath：规划文件的保存路径

     特别提示:1 直线规划前,需要将起始关节位置调正解接口,将数据更新到起始关节.
            2 需要读函数返回值,如果关节超限,返回为false,并且不会保存规划的PVT文件.
    

###    10. 工具动力学参数辨识
FX_INT32  FX_Robot_Iden_LoadDyn(FX_INT32 Type,FX_CHAR* path,FX_DOUBLE* mass, Vect3 mr, Vect6 I);

    • 输入当前机型Type(获取机型Type参考导入运动学参数部分)及文件存放路径，输出为工具相对于法兰端的质量、质心及惯量
    • Type:  1:CCS机型，2:SRS机型
    • path:工具辨识轨迹数据, 指定到文件目录LoadData即可（LoadData文件夹内包含参数辨识的所需文件），只需要输入该文件夹存放的绝对/相对路径，如："/xxxx/xxxx/LoadData"；
    • 函数返回有具体的辨识结果说明：
        typedef enum {
        LOAD_IDEN_NoErr = 0, // 成功
        LOAD_IDEN_CalErr = 1, //  计算错误，需重新采集数据计算
        LOAD_IDEN_OpenSmpDateFieErr = 2, // 打开采集数据文件错误，须检查采样文件
        LOAD_IDEN_OpenCfgFileErr = 3, //  配置文件被修改
        LOAD_IDEN_DataSmpErr = 4 // 采集时间不够，缺少有效数据
        }LoadIdenErrCode;
            
    • 其中 NoLoadData.csv 文件为无负载下采集的数据，在无负载情况下采集；LoadData.csv 文件需要在更换末端携带负载后重新采集（注意左右臂不可同时辨识，需要一根一根的逐一采集空载和带载辨识）

###    11. 位置姿态4×4矩阵转XYZABC
FX_BOOL FX_Matrix42XYZABCDEG(FX_DOUBLE m[4][4],FX_DOUBLE xyzabc[6])

    • 输入为4*4的法兰末端位姿矩阵
    • 输出位姿信息XYZ及欧拉角ABC（单位：mm/度）
    
###     12. XYZABC转位置姿态4×4矩阵
FX_VOID FX_XYZABC2Matrix4DEG(FX_DOUBLE xyzabc[6], FX_DOUBLE m[4][4])

    • 输入为位姿信息XYZ及欧拉角ABC（单位：mm/度）
    • 输出4*4的法兰末端位姿矩阵

# 二、案例脚本
## C++开发的使用编译见：demo_linu_win/c++linux/API_USAGE_KINMATICS.txt
##DEMO: [my_dd.cpp](c%2B%2B_win/my_dd.cpp)






